// Project the guard's authoritative scene into Nav2. Each scene replaces this
// layer's previous contribution, so clearing does not create a second memory.
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace robocup_collision_processing {
class SceneLayer : public nav2_costmap_2d::Layer {
  using Cloud=sensor_msgs::msg::PointCloud2;
  using Clock=std::chrono::steady_clock;
  struct Point {double x,y,hx,hy;};
  struct Bounds {
    double min_x=1e30,min_y=1e30,max_x=-1e30,max_y=-1e30;
    void include(const Point &p) {
      min_x=std::min(min_x,p.x-p.hx);min_y=std::min(min_y,p.y-p.hy);
      max_x=std::max(max_x,p.x+p.hx);max_y=std::max(max_y,p.y+p.hy);
    }
    void extend(double *x0,double *y0,double *x1,double *y1) const {
      if(min_x>max_x)return;
      *x0=std::min(*x0,min_x);*y0=std::min(*y0,min_y);
      *x1=std::max(*x1,max_x);*y1=std::max(*y1,max_y);
    }
  };
 public:
  void onInitialize() override {
    const auto node=node_.lock();if(!node)throw std::runtime_error("missing_costmap_node");
    declareParameter("enabled",rclcpp::ParameterValue(true));
    declareParameter("topic",rclcpp::ParameterValue(std::string("/collision/points")));
    declareParameter("min_height_above_floor",rclcpp::ParameterValue(.05));
    declareParameter("max_obstacle_height",rclcpp::ParameterValue(2.1));
    declareParameter("voxel_size",rclcpp::ParameterValue(.04));
    declareParameter("use_source_points",rclcpp::ParameterValue(true));
    declareParameter("cloud_timeout",rclcpp::ParameterValue(.4));
    declareParameter("maximum_cells",rclcpp::ParameterValue(250000));
    // A table top over the low chassis (the chassis is under it after a close approach) is not a 2D obstacle
    // for the robot's current pose: skip cells above overhang_min_height that lie inside the body outline now.
    // Cells the body has not reached are still projected, and the 3D guard keeps checking the real geometry.
    declareParameter("overhang_clearing",rclcpp::ParameterValue(true));
    declareParameter("overhang_min_height",rclcpp::ParameterValue(.25));
    declareParameter("overhang_footprint",rclcpp::ParameterValue(std::string("")));
    std::string topic;
    node->get_parameter(name_+".enabled",enabled_);node->get_parameter(name_+".topic",topic);
    node->get_parameter(name_+".min_height_above_floor",min_height_);
    node->get_parameter(name_+".max_obstacle_height",max_height_);
    node->get_parameter(name_+".voxel_size",voxel_);node->get_parameter(name_+".cloud_timeout",timeout_);
    node->get_parameter(name_+".use_source_points",use_sources_);
    node->get_parameter(name_+".maximum_cells",maximum_cells_);
    node->get_parameter(name_+".overhang_clearing",overhang_clearing_);
    node->get_parameter(name_+".overhang_min_height",overhang_min_height_);
    std::string outline;node->get_parameter(name_+".overhang_footprint",outline);
    if(!outline.empty() && (!nav2_costmap_2d::makeFootprintFromString(outline,overhang_footprint_) || overhang_footprint_.size()<3))
      throw std::runtime_error("invalid_overhang_footprint");
    if(!std::isfinite(overhang_min_height_) || overhang_min_height_<0.)throw std::runtime_error("invalid_scene_layer_configuration");
    if(!std::isfinite(min_height_+max_height_+voxel_+timeout_) || min_height_<0. ||
        voxel_<=0. || timeout_<=0. || maximum_cells_<=0)throw std::runtime_error("invalid_scene_layer_configuration");
    rclcpp::SubscriptionOptions options;options.callback_group=callback_group_;
    subscription_=node->create_subscription<Cloud>(topic,rclcpp::QoS(1).reliable(),
      [this](Cloud::ConstSharedPtr cloud) {
        std::lock_guard<std::mutex> lock(mutex_);latest_=std::move(cloud);received_=Clock::now();
      },options);
    current_=false;
  }
  void reset() override {current_=false;}
  bool isClearable() override {return false;}
  static bool inside(const std::vector<std::pair<double,double>> &polygon,double x,double y) {
    bool in=false;
    for(size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
      const auto &a=polygon[i],&b=polygon[j];
      if((a.second>y)!=(b.second>y) && x<(b.first-a.first)*(y-a.second)/(b.second-a.second)+a.first)in=!in;
    }
    return in;
  }
  void updateBounds(double robot_x,double robot_y,double robot_yaw,double *min_x,double *min_y,double *max_x,double *max_y) override {
    // Revisit old bounds so the master's reset + other layers restore cells
    // that were removed from this scene, including after map->odom changes.
    bounds_.extend(min_x,min_y,max_x,max_y);
    if(!enabled_) {points_.clear();bounds_=Bounds{};current_=true;return;}
    Cloud::ConstSharedPtr cloud;Clock::time_point received;
    {std::lock_guard<std::mutex> lock(mutex_);cloud=latest_;received=received_;}
    try {
      if(!cloud || cloud->header.frame_id.empty())throw std::runtime_error("waiting_for_collision_scene");
      const double source_age=(clock_->now()-rclcpp::Time(cloud->header.stamp,clock_->get_clock_type())).seconds();
      if(source_age<-.05 || source_age>timeout_ || std::chrono::duration<double>(Clock::now()-received).count()>timeout_)
        throw std::runtime_error("stale_collision_scene");
      if(size_t(cloud->width)*cloud->height>size_t(maximum_cells_) || !cloud->point_step ||
          cloud->row_step<size_t(cloud->width)*cloud->point_step || cloud->data.size()<size_t(cloud->height)*cloud->row_step)
        throw std::runtime_error("invalid_collision_scene");
      std::array<uint32_t,4> offsets{};
      const std::array<std::string,4> names=use_sources_?
        std::array<std::string,4>{"source_x","source_y","source_z","height_above_floor"}:
        std::array<std::string,4>{"x","y","z","height_above_floor"};
      for(size_t i=0;i<names.size();++i) {
        const auto f=std::find_if(cloud->fields.begin(),cloud->fields.end(),[&](const auto &field){return field.name==names[i];});
        if(f==cloud->fields.end() || f->datatype!=sensor_msgs::msg::PointField::FLOAT32 || f->count!=1 || f->offset+4>cloud->point_step)
          throw std::runtime_error("invalid_collision_scene_fields");
        offsets[i]=f->offset;
      }
      Eigen::Isometry3d transform=Eigen::Isometry3d::Identity();
      const auto frame=layered_costmap_->getGlobalFrameID();
      if(frame!=cloud->header.frame_id) {
        const auto tf=tf_->lookupTransform(frame,cloud->header.frame_id,tf2::TimePointZero).transform;
        Eigen::Quaterniond q(tf.rotation.w,tf.rotation.x,tf.rotation.y,tf.rotation.z);
        if(!q.coeffs().allFinite() || q.norm()<1e-9)throw std::runtime_error("invalid_scene_transform");
        transform.linear()=q.normalized().toRotationMatrix();
        transform.translation()=Eigen::Vector3d(tf.translation.x,tf.translation.y,tf.translation.z);
        if(!transform.matrix().allFinite())throw std::runtime_error("invalid_scene_transform");
      }
      const uint16_t endian=1;const bool swap=cloud->is_bigendian!=(*reinterpret_cast<const uint8_t *>(&endian)==0);
      const auto number=[&](const uint8_t *data,uint32_t offset) {
        std::array<uint8_t,4> bytes;std::copy_n(data+offset,4,bytes.data());if(swap)std::reverse(bytes.begin(),bytes.end());
        float value;std::memcpy(&value,bytes.data(),4);return value;
      };
      // Cover the whole XY extent of a voxel, even across costmap cell edges.
      const double hx=use_sources_?0.:.5*voxel_*(std::abs(transform.linear()(0,0))+std::abs(transform.linear()(0,1))+std::abs(transform.linear()(0,2)));
      const double hy=use_sources_?0.:.5*voxel_*(std::abs(transform.linear()(1,0))+std::abs(transform.linear()(1,1))+std::abs(transform.linear()(1,2)));
      std::vector<std::pair<double,double>> body;  // body outline at the current pose, costmap frame
      if(overhang_clearing_) {
        const auto &outline=overhang_footprint_.empty()?layered_costmap_->getFootprint():overhang_footprint_;
        const double c=std::cos(robot_yaw),s=std::sin(robot_yaw);
        for(const auto &v:outline)body.emplace_back(robot_x+c*v.x-s*v.y,robot_y+s*v.x+c*v.y);
        if(body.size()<3)body.clear();
      }
      std::vector<Point> next;next.reserve(size_t(cloud->width)*cloud->height);Bounds bounds;
      for(size_t row=0;row<cloud->height;++row)for(size_t col=0;col<cloud->width;++col) {
        const auto *data=cloud->data.data()+row*cloud->row_step+col*cloud->point_step;
        const double height=number(data,offsets[3]);
        Eigen::Vector3d p(number(data,offsets[0]),number(data,offsets[1]),number(data,offsets[2]));
        if(!p.allFinite() || !std::isfinite(height) || height<=min_height_ || p.z()>max_height_)continue;
        p=transform*p;
        if(!body.empty() && height>overhang_min_height_ && inside(body,p.x(),p.y()))continue;
        Point point{p.x(),p.y(),hx,hy};next.push_back(point);bounds.include(point);
      }
      points_.swap(next);bounds_=bounds;current_=true;
    } catch(const std::exception &error) {
      // Preserve the last contribution, but make Nav2 wait for fresh input.
      current_=false;RCLCPP_WARN_THROTTLE(logger_,*clock_,2000,"Shared collision scene: %s",error.what());
    }
    bounds_.extend(min_x,min_y,max_x,max_y);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &master,int min_i,int min_j,int max_i,int max_j) override {
    if(!enabled_)return;
    const double x0=master.getOriginX(),y0=master.getOriginY();
    const double x1=x0+master.getSizeInCellsX()*master.getResolution(),y1=y0+master.getSizeInCellsY()*master.getResolution();
    for(const auto &p:points_) {
      if(p.x+p.hx<x0 || p.x-p.hx>=x1 || p.y+p.hy<y0 || p.y-p.hy>=y1)continue;
      int left,bottom,right,top;
      master.worldToMapEnforceBounds(p.x-p.hx,p.y-p.hy,left,bottom);
      master.worldToMapEnforceBounds(p.x+p.hx,p.y+p.hy,right,top);
      for(int y=std::max(bottom,min_j);y<=std::min(top,max_j-1);++y)
        for(int x=std::max(left,min_i);x<=std::min(right,max_i-1);++x)
          master.setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
 private:
  std::mutex mutex_;Cloud::ConstSharedPtr latest_;Clock::time_point received_{};
  rclcpp::Subscription<Cloud>::SharedPtr subscription_;
  std::vector<Point> points_;Bounds bounds_;
  double min_height_=0.,max_height_=0.,voxel_=0.,timeout_=0.;int maximum_cells_=0;
  bool use_sources_=true,overhang_clearing_=true;double overhang_min_height_=.25;
  std::vector<geometry_msgs::msg::Point> overhang_footprint_;
};
} // namespace robocup_collision_processing
PLUGINLIB_EXPORT_CLASS(robocup_collision_processing::SceneLayer,nav2_costmap_2d::Layer)
