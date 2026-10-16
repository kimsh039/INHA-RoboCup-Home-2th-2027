// Input callbacks stay short; immutable scenes are built by one latest-only
// worker. The control group never overlaps itself and has a separate deadman.
#include "guard_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nlohmann/json.hpp>
#include <array>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>

namespace {
using namespace collision_guard;
using Json=nlohmann::json;
using Cloud=sensor_msgs::msg::PointCloud2;
constexpr double NS=1e9;
int64_t stamp(const builtin_interfaces::msg::Time &value) {return int64_t(value.sec)*1000000000LL+value.nanosec;}
double age(Clock::time_point time) {return std::chrono::duration<double>(Clock::now()-time).count();}
double ms(Clock::time_point start) {return age(start)*1000.;}
double yaw(const geometry_msgs::msg::Quaternion &q) {return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
bool valid(const geometry_msgs::msg::Quaternion &q) {
  return std::isfinite(q.x+q.y+q.z+q.w) && q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w>1e-12;
}
Json point(const V3 &p) {return {p.x(),p.y(),p.z()};}
Json rectangle(const std::vector<Box> &boxes,double padding) {
  V3 low=V3::Constant(std::numeric_limits<double>::infinity()),high=-low;
  for(const auto &box : boxes)for(int x : {-1,1})for(int y : {-1,1})for(int z : {-1,1}) {
    const V3 p=box.pose*box.half.cwiseProduct(V3(x,y,z));low=low.cwiseMin(p);high=high.cwiseMax(p);
  }
  return {{high.x()+padding,high.y()+padding},{high.x()+padding,low.y()-padding},
          {low.x()-padding,low.y()-padding},{low.x()-padding,high.y()+padding}};
}
Json convex_footprint(const std::vector<Box> &boxes) {
  using XY=std::array<double,2>;
  std::vector<XY> points;
  for(const auto &box : boxes)for(int x : {-1,1})for(int y : {-1,1})for(int z : {-1,1}) {
    const V3 p=box.pose*box.half.cwiseProduct(V3(x,y,z));points.push_back({p.x(),p.y()});
  }
  std::sort(points.begin(),points.end());
  points.erase(std::unique(points.begin(),points.end()),points.end());
  if(points.size()<3)throw std::runtime_error("invalid_physical_footprint");
  auto half=[](const std::vector<XY> &sequence) {
    std::vector<XY> out;
    for(const auto &p : sequence) {
      while(out.size()>=2) {
        const auto &a=out[out.size()-2],&b=out.back();
        if((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])>0.)break;
        out.pop_back();
      }
      out.push_back(p);
    }
    out.pop_back();return out;
  };
  auto hull=half(points);std::reverse(points.begin(),points.end());
  const auto upper=half(points);hull.insert(hull.end(),upper.begin(),upper.end());
  Json out=Json::array();for(const auto &p : hull)out.push_back({p[0],p[1]});return out;
}
struct Observation {
  std::shared_ptr<const Scene> scene;
  int64_t stamp=0;uint64_t epoch=0;
  Clock::time_point received;
  double parse_ms=0.,build_ms=0.;
};
struct Health {Json value;Clock::time_point received;};
struct State {
  Observation scene;Health health{{{"ready",false}},Clock::time_point{}};
  Health latest_health{{{"ready",false}},Clock::time_point{}};
  V3 pose=V3::Zero();V2 measured=V2::Zero(),command=V2::Zero();std::vector<double> joints;
  int64_t odom_stamp=0,joint_stamp=0;
  Clock::time_point odom_received{},joint_received{},command_received{},fault_until{},arm_moved{};
  std::string command_error,route_error="waiting_for_protected_tracer";
  uint64_t epoch=0;
};
struct CloudJob {Cloud::ConstSharedPtr cloud;Clock::time_point received;uint64_t epoch;};

class Guard : public rclcpp::Node {
 public:
  Guard():Node("collision_guard") {
    auto text=[this](const char *name,const char *value){return declare_parameter<std::string>(name,value);};
    auto real=[this](const char *name,double value){return declare_parameter<double>(name,value);};
    base_=text("base_frame","base_link");odom_=text("odom_frame","odom");
    approved_=text("approved_topic","/collision/cmd_vel_approved");protected_=text("protected_topic","/tracer/cmd_vel_safe");
    command_topic_=text("command_topic","/cmd_vel");
    cfg_.body_only=declare_parameter<bool>("body_only",true);
    cfg_.clearance=real("clearance",0.);cfg_.uncertainty=real("uncertainty_margin",0.);cfg_.voxel=real("safety_voxel",.04);
    cfg_.reaction=real("reaction_time",.3);cfg_.linear_deceleration=real("linear_deceleration",.5);
    cfg_.angular_deceleration=real("angular_deceleration",1.);cfg_.step=real("prediction_step",.04);
    cfg_.minimum_scale=real("minimum_speed_scale",.1);
    budget_=real("computation_budget",.04);frequency_=real("control_frequency",20.);
    cloud_timeout_=real("cloud_timeout",.4);odom_timeout_=real("odom_timeout",.2);joint_timeout_=real("joint_timeout",.5);
    command_timeout_=real("command_timeout",.25);arm_speed_=real("arm_motion_limit",.08);settle_=real("arm_settle_sec",.5);
    max_forward_=real("max_linear_velocity",.25);max_reverse_=real("max_reverse_velocity",.15);max_angular_=real("max_angular_velocity",.6);
    costmap_resolution_=real("costmap_resolution",.05);require_arm_=declare_parameter<bool>("require_arm_feedback",true);
    max_cells_=declare_parameter<int>("maximum_cells",250000);
    for(double value : {cfg_.voxel,cfg_.linear_deceleration,cfg_.angular_deceleration,cfg_.step,
         budget_,frequency_,cloud_timeout_,odom_timeout_,joint_timeout_,command_timeout_,arm_speed_,settle_,
         max_forward_,max_reverse_,max_angular_,costmap_resolution_})
      if(!std::isfinite(value) || value<=0.)throw std::runtime_error("invalid_collision_guard_configuration");
    if(!std::isfinite(cfg_.clearance+cfg_.uncertainty+cfg_.reaction+cfg_.minimum_scale) || cfg_.clearance<0 || cfg_.uncertainty<0 || cfg_.reaction<0 ||
       cfg_.minimum_scale<=0 || cfg_.minimum_scale>1 || max_cells_<=0 || approved_==protected_ ||
       approved_==command_topic_ || protected_==command_topic_)throw std::runtime_error("invalid_collision_guard_configuration");
    model_=std::make_unique<collision_model::Geometry>(text("geometry_file",""));
    if(model_->tracer.empty())throw std::runtime_error("missing_tracer_geometry");
    static_shape_=std::make_shared<Shape>(model_->fixed);
    pub_=create_publisher<geometry_msgs::msg::TwistStamped>(approved_,1);
    status_pub_=create_publisher<std_msgs::msg::String>(text("status_topic","/collision/status"),10);
    local_footprint_=create_publisher<geometry_msgs::msg::Polygon>("/local_costmap/footprint",1);
    global_footprint_=create_publisher<geometry_msgs::msg::Polygon>("/global_costmap/footprint",1);
    tf_=std::make_unique<tf2_ros::Buffer>(get_clock());listener_=std::make_unique<tf2_ros::TransformListener>(*tf_);
    input_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;options.callback_group=input_group_;
    auto latest=rclcpp::SensorDataQoS().keep_last(1);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(text("odom_topic","/odom_ekf"),latest,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){receive_odom(*m);},options);
    joint_sub_=create_subscription<sensor_msgs::msg::JointState>(text("joint_topic","/robot/joint_states"),latest,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){receive_joints(*m);},options);
    command_sub_=create_subscription<geometry_msgs::msg::Twist>(command_topic_,rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m){receive_command(*m);},options);
    scene_sub_=create_subscription<Cloud>(text("safety_topic","/collision/points"),rclcpp::QoS(1).reliable(),
      [this](Cloud::ConstSharedPtr m){receive_scene(std::move(m));},options);
    health_sub_=create_subscription<std_msgs::msg::String>(text("cloud_status_topic","/collision/cloud_status"),10,
      [this](std_msgs::msg::String::ConstSharedPtr m){receive_health(m->data);},options);
    control_timer_=create_wall_timer(std::chrono::duration<double>(1./frequency_),[this]{control();},control_group_);
    status_timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{status();},control_group_);
    routes_timer_=create_wall_timer(std::chrono::milliseconds(500),[this]{routes();},control_group_);
    worker_=std::thread([this]{work();});
    RCLCPP_INFO(get_logger(),"C++ 3D velocity guard active; clearance %.2f m, budget %.0f ms",cfg_.clearance,budget_*1000);
  }
  ~Guard() override {
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}changed_.notify_all();if(worker_.joinable())worker_.join();
  }
 private:
  Config cfg_;std::string base_,odom_,approved_,protected_,command_topic_;
  double budget_,frequency_,cloud_timeout_,odom_timeout_,joint_timeout_,command_timeout_,arm_speed_,settle_;
  double max_forward_,max_reverse_,max_angular_,costmap_resolution_;bool require_arm_,stopping_=false;int max_cells_;
  std::mutex mutex_;std::condition_variable changed_;std::thread worker_;State state_;
  std::string generation_;std::optional<CloudJob> pending_;
  std::map<int64_t,Observation> scenes_;std::map<int64_t,Health> healths_;uint64_t dropped_=0;
  std::unique_ptr<collision_model::Geometry> model_;std::shared_ptr<const Shape> static_shape_,arm_shape_;
  std::vector<double> cached_q_;Json result_={{"state","blocked"},{"ready",false},{"reason","waiting_for_mid360"}};
  std::unique_ptr<tf2_ros::Buffer> tf_;std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::CallbackGroup::SharedPtr input_group_,control_group_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr local_footprint_,global_footprint_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr scene_sub_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_,status_timer_,routes_timer_;

  State snapshot() {std::lock_guard<std::mutex> lock(mutex_);return state_;}
  bool fresh(int64_t value,double timeout) const {
    const double elapsed=(now().nanoseconds()-value)/NS;return value>0 && elapsed>=-.05 && elapsed<=timeout;
  }
  void receive_command(const geometry_msgs::msg::Twist &m) {
    const std::array<double,6> values{m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z};
    bool ok=true;for(double v : values)ok=ok && std::isfinite(v);
    for(int i : {1,2,3,4})ok=ok && std::abs(values[i])<=1e-6;
    std::lock_guard<std::mutex> lock(mutex_);state_.command_received=Clock::now();
    state_.command_error=ok?"":"invalid_velocity_command";
    state_.command=ok?V2(std::clamp(m.linear.x,-max_reverse_,max_forward_),std::clamp(m.angular.z,-max_angular_,max_angular_)):V2::Zero();
  }
  void receive_odom(const nav_msgs::msg::Odometry &m) {
    const auto &p=m.pose.pose.position;const auto &q=m.pose.pose.orientation;const auto &v=m.twist.twist;
    const auto t=stamp(m.header.stamp);std::lock_guard<std::mutex> lock(mutex_);
    if(m.header.frame_id!=odom_ || m.child_frame_id!=base_ || t<=0 || !valid(q) ||
        !std::isfinite(p.x+p.y+v.linear.x+v.angular.z)) {state_.fault_until=Clock::now()+std::chrono::seconds(1);return;}
    const V3 pose(p.x,p.y,yaw(q));
    if(state_.odom_stamp && (t<=state_.odom_stamp || (pose.head<2>()-state_.pose.head<2>()).norm()>.15 ||
        std::abs(std::atan2(std::sin(pose.z()-state_.pose.z()),std::cos(pose.z()-state_.pose.z())))>.15))
      state_.fault_until=Clock::now()+std::chrono::seconds(1);
    state_.pose=pose;state_.measured=V2(v.linear.x,v.angular.z);state_.odom_stamp=t;state_.odom_received=Clock::now();
  }
  void receive_joints(const sensor_msgs::msg::JointState &m) {
    if(m.name.size()!=m.position.size())return;
    std::vector<double> values;
    for(const auto &name : model_->required) {
      const auto at=std::find(m.name.begin(),m.name.end(),name);
      if(at==m.name.end())return;
      const double value=m.position[std::distance(m.name.begin(),at)];
      if(!std::isfinite(value))return;
      values.push_back(value);
    }
    const auto t=stamp(m.header.stamp);std::lock_guard<std::mutex> lock(mutex_);
    if(t<=0 || t<=state_.joint_stamp)return;
    if(state_.joint_stamp)for(size_t i=0;i<values.size();++i)
      if(std::abs(values[i]-state_.joints[i])/((t-state_.joint_stamp)/NS)>arm_speed_)state_.arm_moved=Clock::now();
    state_.joints=std::move(values);state_.joint_stamp=t;state_.joint_received=Clock::now();
  }
  void receive_scene(Cloud::ConstSharedPtr m) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(m->header.frame_id!=odom_ || stamp(m->header.stamp)<=0) {state_.scene={};return;}
    if(pending_)++dropped_;
    pending_=CloudJob{std::move(m),Clock::now(),state_.epoch};changed_.notify_one();
  }
  void receive_health(const std::string &data) {
    try {
      Json h=Json::parse(data);
      if(!h.is_object() || !h.contains("session_id") || !h.contains("generation") || !h["ready"].is_boolean())
        throw std::runtime_error("invalid_cloud_health");
      const std::string generation=h["session_id"].dump()+":"+h["generation"].dump();
      std::lock_guard<std::mutex> lock(mutex_);
      if(generation!=generation_) {
        generation_=generation;++state_.epoch;state_.scene={};state_.health={{ {"ready",false},{"reason","waiting_for_mid360"}},Clock::now()};
        scenes_.clear();healths_.clear();pending_.reset();
      }
      state_.latest_health={h,Clock::now()};
      if(!h["ready"].get<bool>() || !h["reference_stamp_ns"].is_number_integer() || h["reference_stamp_ns"].get<int64_t>()<=0) {
        h["ready"]=false;state_.health={std::move(h),Clock::now()};state_.latest_health=state_.health;return;
      }
      const int64_t t=h["reference_stamp_ns"].get<int64_t>();healths_[t]={std::move(h),Clock::now()};
      while(healths_.size()>4)healths_.erase(healths_.begin());
      pair_locked(t);
    } catch(const std::exception &) {
      std::lock_guard<std::mutex> lock(mutex_);state_.health={{{"ready",false},{"reason","invalid_cloud_health"}},Clock::now()};
      state_.latest_health=state_.health;
    }
  }
  void pair_locked(int64_t t) {
    const auto h=healths_.find(t);if(h==healths_.end())return;
    if(state_.scene.stamp==t){state_.health=h->second;return;}
    const auto s=scenes_.find(t);
    if(s==scenes_.end() || s->second.epoch!=state_.epoch || t<state_.scene.stamp)return;
    state_.scene=s->second;state_.health=h->second;
    scenes_.erase(scenes_.begin(),scenes_.lower_bound(t));healths_.erase(healths_.begin(),healths_.lower_bound(t));
  }
  static float packed_float(const uint8_t *data,bool swap) {
    std::array<uint8_t,4> bytes;std::copy_n(data,4,bytes.data());if(swap)std::reverse(bytes.begin(),bytes.end());
    float value;std::memcpy(&value,bytes.data(),4);return value;
  }
  Observation decode(const CloudJob &job) {
    const auto start=Clock::now();const auto &m=*job.cloud;
    if(size_t(m.width)*m.height>size_t(max_cells_) || !m.point_step || m.row_step<size_t(m.width)*m.point_step ||
        m.data.size()<size_t(m.height)*m.row_step)throw std::runtime_error("invalid_retained_scene");
    std::array<uint32_t,6> offsets;
    const std::array<std::string,6> names{"x","y","z","source_x","source_y","source_z"};
    for(size_t i=0;i<names.size();++i) {
      const auto f=std::find_if(m.fields.begin(),m.fields.end(),[&](const auto &field){return field.name==names[i];});
      if(f==m.fields.end() || f->count!=1 || f->datatype!=sensor_msgs::msg::PointField::FLOAT32 || f->offset+4>m.point_step)
        throw std::runtime_error("invalid_retained_scene");
      offsets[i]=f->offset;
    }
    std::array<std::optional<uint32_t>,2> extra_offsets;
    const std::array<std::string,2> extra_names{"last_seen_age","height_above_floor"};
    for(size_t i=0;i<extra_names.size();++i) {
      const auto f=std::find_if(m.fields.begin(),m.fields.end(),[&](const auto &field){return field.name==extra_names[i];});
      if(f==m.fields.end())continue;
      if(f->count!=1 || f->datatype!=sensor_msgs::msg::PointField::FLOAT32 || f->offset+4>m.point_step)
        throw std::runtime_error("invalid_retained_scene");
      extra_offsets[i]=f->offset;
    }
    const uint16_t endian=1;const bool swap=m.is_bigendian!=(*reinterpret_cast<const uint8_t *>(&endian)==0);
    std::vector<V3> points,sources;points.reserve(size_t(m.width)*m.height);sources.reserve(points.capacity());
    std::vector<double> observation_ages,floor_heights;
    if(extra_offsets[0])observation_ages.reserve(points.capacity());
    if(extra_offsets[1])floor_heights.reserve(points.capacity());
    for(size_t r=0;r<m.height;++r)for(size_t c=0;c<m.width;++c) {
      const auto *p=m.data.data()+r*m.row_step+c*m.point_step;V3 point,source;
      for(size_t a=0;a<3;++a){point[a]=packed_float(p+offsets[a],swap);source[a]=packed_float(p+offsets[a+3],swap);}
      if(!point.allFinite() || !source.allFinite())throw std::runtime_error("invalid_retained_scene");
      points.push_back(cfg_.body_only?source:point);sources.push_back(source);
      if(extra_offsets[0]) {
        const double value=packed_float(p+*extra_offsets[0],swap);
        if(!std::isfinite(value) || value<0)throw std::runtime_error("invalid_retained_scene");
        observation_ages.push_back(value);
      }
      if(extra_offsets[1]) {
        const double value=packed_float(p+*extra_offsets[1],swap);
        if(!std::isfinite(value))throw std::runtime_error("invalid_retained_scene");
        floor_heights.push_back(value);
      }
    }
    const double parse=ms(start);const auto tree_start=Clock::now();
    auto scene=std::make_shared<Scene>(std::move(points),std::move(sources),std::move(observation_ages),std::move(floor_heights));
    return {std::move(scene),stamp(m.header.stamp),job.epoch,job.received,parse,ms(tree_start)};
  }
  void work() {
    while(true) {
      CloudJob job;
      {std::unique_lock<std::mutex> lock(mutex_);changed_.wait(lock,[this]{return stopping_ || pending_.has_value();});
       if(stopping_)return;
       job=std::move(*pending_);pending_.reset();}
      try {
        auto scene=decode(job);std::lock_guard<std::mutex> lock(mutex_);
        if(stopping_ || job.epoch!=state_.epoch)continue;
        if(!fresh(scene.stamp,cloud_timeout_))continue;
        const auto t=scene.stamp;scenes_[t]=std::move(scene);while(scenes_.size()>4)scenes_.erase(scenes_.begin());pair_locked(t);
      } catch(const std::exception &e) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(job.epoch==state_.epoch)state_.health={{{"ready",false},{"reason",e.what()}},Clock::now()};
      }
    }
  }
  void routes() {
    std::string error;
    try {
      const auto approvals=get_publishers_info_by_topic(approved_),outputs=get_publishers_info_by_topic(protected_);
      const auto consumers=get_subscriptions_info_by_topic(protected_),bypass=get_subscriptions_info_by_topic(command_topic_);
      auto tracer=[](const auto &info){return info.node_name().find("tracer_base")!=std::string::npos;};
      if(approvals.size()!=1 || approvals.front().node_name()!=get_name())error="duplicate_velocity_guard";
      else if(outputs.size()!=1 || outputs.front().node_name()!="tracer_safety_watchdog")error="missing_actuator_watchdog";
      else if(std::any_of(bypass.begin(),bypass.end(),tracer))error="tracer_requires_protected_remap";
      else if(!std::any_of(consumers.begin(),consumers.end(),tracer))error="waiting_for_protected_tracer";
    } catch(const std::exception &){error="velocity_route_unavailable";}
    std::lock_guard<std::mutex> lock(mutex_);state_.route_error=std::move(error);
  }
  std::string healthy(const State &s) {
    if(!s.route_error.empty())return s.route_error;
    // Distinguish fresh preprocessing with a lost scene from a stale sensor.
    // Matching health is still required before using any scene.
    if(age(s.latest_health.received)>cloud_timeout_)return "missing_cloud_health";
    if(!s.latest_health.value.value("ready",false))return s.latest_health.value.value("reason",std::string("invalid_mid360"));
    if(!fresh(s.latest_health.value.value("reference_stamp_ns",int64_t(0)),cloud_timeout_))return "stale_mid360";
    if(!s.scene.scene || s.scene.epoch!=s.epoch || age(s.scene.received)>cloud_timeout_ || !fresh(s.scene.stamp,cloud_timeout_))return "stale_collision_scene";
    if(!s.health.value.value("ready",false))return s.health.value.value("reason",std::string("invalid_mid360"));
    if(age(s.odom_received)>odom_timeout_ || !fresh(s.odom_stamp,odom_timeout_))return "stale_odometry";
    if(Clock::now()<s.fault_until)return "odometry_discontinuity";
    if(require_arm_) {
      if(s.joints.size()!=model_->required.size() || age(s.joint_received)>joint_timeout_ || !fresh(s.joint_stamp,joint_timeout_))return "stale_arm_feedback";
      if(age(s.arm_moved)<settle_)return "arm_is_moving";
    }
    try {
      const auto m=tf_->lookupTransform(odom_,base_,tf2::TimePointZero);
      const auto &p=m.transform.translation;const auto &q=m.transform.rotation;
      const double delta=yaw(q)-s.pose.z();
      if(!fresh(stamp(m.header.stamp),odom_timeout_) || !valid(q) || !std::isfinite(p.x+p.y) ||
          std::hypot(p.x-s.pose.x(),p.y-s.pose.y())>.05 || std::abs(std::atan2(std::sin(delta),std::cos(delta)))>.08)
        return "stale_or_inconsistent_tf";
    } catch(const tf2::TransformException &){return "missing_odometry_tf";}
    return "";
  }
  std::shared_ptr<const Shape> shape(const State &s,bool measured) {
    if(!measured)return static_shape_;
    if(!arm_shape_ || cached_q_!=s.joints) {
      arm_shape_=std::make_shared<Shape>(model_->at(s.joints,base_));cached_q_=s.joints;
    }
    return arm_shape_;
  }
  void control() {
    const auto began=Clock::now(),deadline=began+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(budget_));
    const auto input=snapshot();Result r;bool ready=false;std::string reason;
    try {
      reason=healthy(input);ready=reason.empty();
      if(ready)r=evaluate(*input.scene.scene,*shape(input,require_arm_),input.pose,input.measured,input.command,
          age(input.command_received)<=command_timeout_,input.command_error,(now().nanoseconds()-input.scene.stamp)/NS,cfg_,deadline);
      if(ready)reason=r.reason;
    } catch(const Expired &){ready=false;reason="computation_timeout";}
      catch(const std::exception &e){ready=false;reason="guard_error";RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"3D guard: %s",e.what());}
    geometry_msgs::msg::TwistStamped approved;approved.header.frame_id=base_;
    // Inputs may have changed on the other executor thread during prediction.
    // Recheck both the exact observation used and the current health, then stamp
    // and publish under the same lock. No new timestamp revives old data.
    {std::lock_guard<std::mutex> lock(mutex_);
      if(ready) {
        auto expired=healthy(state_);
        if(expired.empty() && (!fresh(input.scene.stamp,cloud_timeout_) || age(input.scene.received)>cloud_timeout_))expired="stale_collision_scene";
        if(expired.empty() && state_.epoch!=input.epoch)expired="stale_collision_scene";
        if(expired.empty() && (state_.command-input.command).norm()>1e-9)expired="command_changed";
        if(expired.empty() && (state_.command_error!=input.command_error || (age(state_.command_received)>command_timeout_ && r.output.norm()>0)))expired="command_timeout";
        const double heading=state_.pose.z()-input.pose.z();
        if(expired.empty() && ((state_.pose.head<2>()-input.pose.head<2>()).norm()>.05 || std::abs(std::atan2(std::sin(heading),std::cos(heading)))>.08))expired="odometry_discontinuity";
        if(expired.empty() && require_arm_ && state_.joints.size()==input.joints.size()) {
          double movement=0.;for(size_t i=0;i<input.joints.size();++i)movement+=std::abs(state_.joints[i]-input.joints[i])*model_->joint_reach[i];
          if(movement>.005)expired="arm_is_moving";
        }
        if(Clock::now()>deadline)expired="computation_timeout";
        if(!expired.empty()){ready=false;reason=expired;}
      }
      if(!ready){r.output.setZero();r.scale=0.;r.state="blocked";}
      approved.header.stamp=now();approved.twist.linear.x=r.output.x();approved.twist.angular.z=r.output.y();pub_->publish(approved);
    }
    result_={{"ready",ready},{"state",r.state},{"reason",reason},{"speed_scale",r.scale},
      {"command",{input.command.x(),input.command.y()}},{"output",{r.output.x(),r.output.y()}},
      {"guard_processing_ms",ms(began)},{"guard_budget_ms",budget_*1000.},{"implementation","cpp"},
      {"robot_pose_odom",point(input.pose)},
      {"nearest_point",nullptr},{"nearest_source_point",nullptr},{"nearest_link",nullptr},{"minimum_distance",nullptr},
      {"nearest_source_age",nullptr},{"nearest_height_above_floor",nullptr},{"nearest_source_point_base",nullptr},{"nearest_last_seen_stamp",nullptr},
      {"blocking_point",nullptr},{"blocking_source_point",nullptr},{"blocking_link",nullptr},
      {"blocking_source_age",nullptr},{"blocking_height_above_floor",nullptr},{"blocking_source_point_base",nullptr},{"blocking_last_seen_stamp",nullptr}};
    const auto source_in_base=[&](size_t i)->V3 {
      return Eigen::AngleAxisd(-input.pose.z(),V3::UnitZ())*
        (input.scene.scene->sources[i]-V3(input.pose.x(),input.pose.y(),0.));
    };
    if(r.current.found && input.scene.scene) {
      const auto geometry=shape(input,require_arm_);
      result_["minimum_distance"]=std::max(0.,r.current.value-(cfg_.body_only?0.:std::sqrt(3.)*cfg_.voxel/2.));
      result_["nearest_point"]=point(input.scene.scene->points[r.current.point]);
      result_["nearest_source_point"]=point(input.scene.scene->sources[r.current.point]);result_["nearest_link"]=geometry->boxes[r.current.box].name;
      result_["nearest_source_point_base"]=point(source_in_base(r.current.point));
      if(!input.scene.scene->observation_ages.empty()) {
        result_["nearest_source_age"]=input.scene.scene->observation_ages[r.current.point]+std::max(0.,(now().nanoseconds()-input.scene.stamp)/NS);
        result_["nearest_last_seen_stamp"]=input.scene.stamp/NS-input.scene.scene->observation_ages[r.current.point];
      }
      if(!input.scene.scene->floor_heights.empty())result_["nearest_height_above_floor"]=input.scene.scene->floor_heights[r.current.point];
    }
    if(ready && r.obstruction.found && input.scene.scene && (r.state=="slowed" || reason=="predicted_clearance_violation")) {
      const auto &scene=*input.scene.scene;const size_t i=r.obstruction.point;
      result_["blocking_point"]=point(scene.points[i]);result_["blocking_source_point"]=point(scene.sources[i]);
      result_["blocking_link"]=shape(input,require_arm_)->boxes[r.obstruction.box].name;
      result_["blocking_source_point_base"]=point(source_in_base(i));
      if(!scene.observation_ages.empty()) {
        result_["blocking_source_age"]=scene.observation_ages[i]+std::max(0.,(now().nanoseconds()-input.scene.stamp)/NS);
        result_["blocking_last_seen_stamp"]=input.scene.stamp/NS-scene.observation_ages[i];
      }
      if(!scene.floor_heights.empty())result_["blocking_height_above_floor"]=scene.floor_heights[i];
    }
    if(r.state=="blocked" && age(input.command_received)<=command_timeout_ && input.command.cwiseAbs().maxCoeff()>1e-5)
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"3D command blocked: %s; requested=(%.3f, %.3f); guard=%.2f ms",reason.c_str(),input.command.x(),input.command.y(),ms(began));
  }
  void status() {
    try {
      const auto s=snapshot();const auto geometry=shape(s,require_arm_ && s.joints.size()==model_->required.size());
      // Clearance belongs to the direction-aware prediction, not to an
      // expanded footprint that prevents departing from a rear obstacle.
      // Keep all physical vertices without the rectangle's empty corners.
      const Json footprint=convex_footprint(geometry->boxes);geometry_msgs::msg::Polygon polygon;
      for(const auto &p : footprint){geometry_msgs::msg::Point32 value;value.x=p[0].get<float>();value.y=p[1].get<float>();polygon.points.push_back(value);}
      local_footprint_->publish(polygon);global_footprint_->publish(polygon);
      V3 low=V3::Constant(std::numeric_limits<double>::infinity()),high=-low;
      for(const auto &b : model_->tracer)for(int x : {-1,1})for(int y : {-1,1})for(int z : {-1,1}) {
        const V3 p=b.pose*b.half.cwiseProduct(V3(x,y,z));low=low.cwiseMin(p);high=high.cwiseMax(p);
      }
      Json out=result_;out["clearance"]=cfg_.clearance;out["uncertainty"]=cfg_.uncertainty;out["frame"]=odom_;out["base_frame"]=base_;
      out["body_only"]=cfg_.body_only;
      out["tracer_bounds"]={point(low),point(high)};out["body_footprint"]=convex_footprint(model_->tracer);
      out["clearance_footprint"]=cfg_.clearance==0.?out["body_footprint"]:rectangle(model_->tracer,cfg_.clearance);
      out["planning_footprint"]=footprint;
      out["protected_topic"]=protected_;out["scene_parse_ms"]=s.scene.parse_ms;out["scene_tree_build_ms"]=s.scene.build_ms;
      {std::lock_guard<std::mutex> lock(mutex_);out["scene_dropped_pending"]=dropped_;}
      out["cloud"]=s.latest_health.value;
      const auto reference=s.latest_health.value.value("reference_stamp_ns",int64_t(0));
      if(reference>0)out["cloud"]["source_age"]=(now().nanoseconds()-reference)/NS;
      out["scene_reference_stamp_ns"]=s.scene.stamp;
      out["scene_source_age"]=s.scene.stamp?Json((now().nanoseconds()-s.scene.stamp)/NS):Json(nullptr);
      out["scene_receive_age"]=s.scene.scene?Json(age(s.scene.received)):Json(nullptr);
      out["cloud_health_receive_age"]=age(s.latest_health.received);
      out["stamp"]=now().seconds();std_msgs::msg::String m;m.data=out.dump();status_pub_->publish(m);
    } catch(const std::exception &e){RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Collision status: %s",e.what());}
  }
};
} // namespace
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<Guard>();rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
    executor.add_node(node);executor.spin();executor.remove_node(node);node.reset();
  } catch(const std::exception &e){std::cerr<<"collision_guard: "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
