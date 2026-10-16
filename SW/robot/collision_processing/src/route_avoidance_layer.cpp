// Short-lived route exclusions. These belong only to global planning and do
// not change the authoritative 3D obstacle scene or velocity protection.
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace robocup_collision_processing {
class RouteAvoidanceLayer : public nav2_costmap_2d::Layer {
  struct Gate {double x,y,yaw,expires;};
  struct Bounds {
    double x0=1e30,y0=1e30,x1=-1e30,y1=-1e30;
    void add(double x,double y) {
      x0=std::min(x0,x);y0=std::min(y0,y);x1=std::max(x1,x);y1=std::max(y1,y);
    }
    void extend(double *a,double *b,double *c,double *d) const {
      if(x0>x1)return;
      *a=std::min(*a,x0);*b=std::min(*b,y0);*c=std::max(*c,x1);*d=std::max(*d,y1);
    }
  };
 public:
  void onInitialize() override {
    const auto node=node_.lock();if(!node)throw std::runtime_error("missing_costmap_node");
    declareParameter("enabled",rclcpp::ParameterValue(true));
    declareParameter("topic",rclcpp::ParameterValue(std::string("/navigation/blocked_routes")));
    std::string topic;node->get_parameter(name_+".enabled",enabled_);node->get_parameter(name_+".topic",topic);
    current_=true;
    subscription_=node->create_subscription<std_msgs::msg::String>(topic,rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        try {
          const auto value=nlohmann::json::parse(msg->data);
          if(value.at("frame")!="odom" || !value.at("zones").is_array() || value.at("zones").size()>6)
            throw std::runtime_error("invalid_blocked_routes");
          std::vector<Gate> next;const double now=clock_->now().seconds();
          for(const auto &zone:value.at("zones")) {
            const auto &pose=zone.at("pose");
            if(!pose.is_array() || pose.size()!=3)throw std::runtime_error("invalid_route_pose");
            Gate g{pose[0].get<double>(),pose[1].get<double>(),pose[2].get<double>(),zone.at("expires").get<double>()};
            if(!std::isfinite(g.x+g.y+g.yaw+g.expires) || g.expires>now+31.)
              throw std::runtime_error("invalid_route_expiry");
            if(g.expires>now)next.push_back(g);
          }
          std::lock_guard<std::mutex> lock(mutex_);zones_=std::move(next);
        } catch(const std::exception &e) {
          RCLCPP_WARN_THROTTLE(logger_,*clock_,2000,"Route exclusion: %s",e.what());
        }
      });
  }
  void reset() override {current_=true;} // A costmap clear must not retry the failed entrance.
  bool isClearable() override {return false;}
  void updateBounds(double,double,double,double *x0,double *y0,double *x1,double *y1) override {
    // Include previous bounds so expiry and cancellation restore the other
    // layers' costs instead of leaving a permanent virtual obstacle behind.
    bounds_.extend(x0,y0,x1,y1);
    if(!enabled_) {gates_.clear();bounds_=Bounds{};current_=true;return;}
    std::vector<Gate> zones;
    const double now=clock_->now().seconds();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      zones_.erase(std::remove_if(zones_.begin(),zones_.end(),[&](const Gate &g){return g.expires<=now;}),zones_.end());
      zones=zones_;
    }
    try {
      Eigen::Isometry3d transform=Eigen::Isometry3d::Identity();
      const auto frame=layered_costmap_->getGlobalFrameID();
      if(!zones.empty() && frame!="odom") {
        const auto tf=tf_->lookupTransform(frame,"odom",tf2::TimePointZero).transform;
        Eigen::Quaterniond q(tf.rotation.w,tf.rotation.x,tf.rotation.y,tf.rotation.z);
        if(!q.coeffs().allFinite() || q.norm()<1e-9)throw std::runtime_error("invalid_route_transform");
        transform.linear()=q.normalized().toRotationMatrix();
        transform.translation()=Eigen::Vector3d(tf.translation.x,tf.translation.y,tf.translation.z);
        if(!transform.matrix().allFinite())throw std::runtime_error("invalid_route_transform");
      }
      const double rotation=std::atan2(transform.linear()(1,0),transform.linear()(0,0));
      Bounds bounds;std::vector<Gate> next;
      for(const auto &zone:zones) {
        // The gate starts 15 cm ahead of the failed base pose. The stopped
        // robot (including a failed reverse) keeps a free planning start cell.
        const auto center=(transform*Eigen::Vector3d(zone.x+.35*std::cos(zone.yaw),zone.y+.35*std::sin(zone.yaw),0.)).eval();
        Gate g{center.x(),center.y(),zone.yaw+rotation,zone.expires};next.push_back(g);
        for(int a:{-1,1})for(int b:{-1,1})
          bounds.add(g.x+a*.20*std::cos(g.yaw)-b*.30*std::sin(g.yaw),
                     g.y+a*.20*std::sin(g.yaw)+b*.30*std::cos(g.yaw));
      }
      gates_.swap(next);bounds_=bounds;current_=true;
    } catch(const std::exception &e) {
      current_=false;RCLCPP_WARN_THROTTLE(logger_,*clock_,2000,"Route exclusion transform: %s",e.what());
    }
    bounds_.extend(x0,y0,x1,y1);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &master,int min_i,int min_j,int max_i,int max_j) override {
    if(!enabled_)return;
    const double half_cell=.5*master.getResolution(),now=clock_->now().seconds();
    for(const auto &g:gates_) {
      if(g.expires<=now)continue;
      const double c=std::cos(g.yaw),s=std::sin(g.yaw);
      const double hx=.20*std::abs(c)+.30*std::abs(s),hy=.20*std::abs(s)+.30*std::abs(c);
      int left,bottom,right,top;
      master.worldToMapEnforceBounds(g.x-hx,g.y-hy,left,bottom);
      master.worldToMapEnforceBounds(g.x+hx,g.y+hy,right,top);
      for(int y=std::max(bottom,min_j);y<=std::min(top,max_j-1);++y)
        for(int x=std::max(left,min_i);x<=std::min(right,max_i-1);++x) {
          double wx,wy;master.mapToWorld(x,y,wx,wy);wx-=g.x;wy-=g.y;
          if(std::abs(c*wx+s*wy)<=.20+half_cell && std::abs(-s*wx+c*wy)<=.30+half_cell)
            master.setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
        }
    }
  }
 private:
  std::mutex mutex_;std::vector<Gate> zones_,gates_;Bounds bounds_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
} // namespace robocup_collision_processing
PLUGINLIB_EXPORT_CLASS(robocup_collision_processing::RouteAvoidanceLayer,nav2_costmap_2d::Layer)
