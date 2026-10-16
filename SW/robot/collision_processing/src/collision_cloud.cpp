// ROS callbacks only collect input. One worker owns the rolling obstacle map.
// Source timestamps are preserved through deskew, filtering and publication.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include "ray_grid.hpp"
#include "robot_geometry.hpp"
#include "self_mask_bounds.hpp"
#include "radius_outlier.hpp"
#include "floor_plane.hpp"

namespace {
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
using Transform = Eigen::Isometry3d;
using Cloud = sensor_msgs::msg::PointCloud2;
using Steady = std::chrono::steady_clock;
constexpr double NS = 1e9;
double elapsed_ms(Steady::time_point a, Steady::time_point b = Steady::now()) {
  return std::chrono::duration<double, std::milli>(b-a).count();
}
int64_t stamp_ns(const builtin_interfaces::msg::Time &t) {
  return int64_t(t.sec)*1000000000LL+t.nanosec;
}
std::string quoted(const std::string &s) {
  std::ostringstream out; out << '"';
  for (const unsigned char c : s) {
    if (c=='"' || c=='\\') out << '\\' << c;
    else if (c < 32) out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << int(c) << std::dec;
    else out << c;
  }
  out << '"'; return out.str();
}
using collision_model::Box;
using collision_model::Geometry;
struct PoseSample { int64_t stamp; V3 pose; };
struct JointSample { int64_t stamp; std::vector<double> q; };
V3 pose_at(const std::deque<PoseSample> &poses, int64_t stamp, int64_t max_gap) {
  if (poses.size()<2 || stamp<poses.front().stamp || stamp>poses.back().stamp)
    throw std::runtime_error("missing_odometry_interval");
  auto right=std::upper_bound(poses.begin(),poses.end(),stamp,[](int64_t t,const PoseSample &p){return t<p.stamp;});
  if (right==poses.end()) --right;
  const auto left=std::prev(right);
  if (right->stamp-left->stamp>max_gap) throw std::runtime_error("odometry_gap");
  const double w=double(stamp-left->stamp)/double(right->stamp-left->stamp);
  return left->pose+w*(right->pose-left->pose);
}
V3 world_point(const V3 &point, const V3 &pose) {
  const double c=std::cos(pose.z()),s=std::sin(pose.z());
  return {c*point.x()-s*point.y()+pose.x(),s*point.x()+c*point.y()+pose.y(),point.z()};
}
using Key=collision_grid::Key;
using Hash=collision_grid::Hash;
struct Cell {int misses=0; uint64_t free_frame=0,hit_frame=0; int64_t last_seen=0; V3 source=V3::Zero();};
struct Pending {Cloud::ConstSharedPtr cloud; Steady::time_point received; uint64_t epoch,sequence;};
struct Metrics {
  int64_t start=0,end=0,reference=0;
  uint64_t sequence=0,epoch=0;
  size_t raw=0,obstacles=0,masked=0,near_sensor=0,isolated=0,arm_shadow=0;
  size_t ground=0,ground_cells_removed=0,free_cells_removed=0,clearing_rays=0,long_range_clearing_rays=0;
  collision_floor::Plane floor_plane;
  double queue_ms=0,parse_ms=0,deskew_ms=0,mask_ms=0,outlier_ms=0,floor_ms=0,map_ms=0,publish_ms=0,total_ms=0,odom_gap_ms=0;
  std::map<std::string,size_t> mask_hits;
};
class CollisionCloud : public rclcpp::Node {
 public:
  CollisionCloud():Node("collision_cloud") {
    auto text=[this](const char *n,const char *v){return declare_parameter<std::string>(n,v);};
    auto real=[this](const char *n,double v){return declare_parameter<double>(n,v);};
    base_=text("base_frame","base_link"); odom_=text("odom_frame","odom");
    format_=text("point_time_format","absolute_ns"); time_field_=text("point_time_field","timestamp");
    history_=real("odom_history_sec",3.); gap_=real("max_odom_gap_sec",.08);
    pending_timeout_=real("max_pending_sec",.25); timeout_=real("cloud_timeout",.4);
    joint_timeout_=real("joint_timeout",.5); scan_duration_=real("max_scan_duration",.25);
    require_arm_=declare_parameter<bool>("require_arm_feedback",true);
    floor_=real("floor_z",-.1425); floor_band_=real("floor_band",.05);
    floor_config_.nominal=floor_;floor_config_.band=floor_band_;
    floor_config_.radius=real("floor_fit_radius",3.);
    floor_config_.inlier_distance=real("floor_fit_distance",.012);
    floor_config_.max_tilt=real("floor_max_tilt_deg",5.)*std::acos(-1.)/180.;
    floor_config_.max_offset=real("floor_max_offset",.05);
    floor_config_.min_ratio=real("floor_min_inlier_ratio",.55);
    floor_interval_=real("floor_fit_interval",.25);floor_timeout_=real("floor_model_timeout",2.);
    free_observations_=declare_parameter<int>("free_observations_to_clear",3);
    height_=real("max_obstacle_height",2.1); range_=real("max_range",4.);
    voxel_=real("safety_voxel",.04); padding_=real("self_padding",.005);
    rack_padding_=real("rack_self_padding",.03);
    base_padding_=real("base_self_padding",.02);
    min_sensor_range_=real("min_sensor_range",.15);
    arm_shadow_radius_=real("arm_shadow_radius",.03);
    arm_shadow_depth_=real("arm_shadow_depth",.6);
    outlier_radius_=real("outlier_radius",.05);
    outlier_min_neighbors_=declare_parameter<int>("outlier_min_neighbors",1);
    radius_=real("memory_radius",4.); arm_speed_=real("arm_motion_limit",.08);
    minimum_points_=declare_parameter<int>("minimum_raw_points",100);
    maximum_cells_=declare_parameter<int>("maximum_cells",250000);
    const int odom_depth=declare_parameter<int>("odom_queue_depth",200);
    const int joint_depth=declare_parameter<int>("joint_queue_depth",200);
    if (!(voxel_>0 && voxel_<=.1 && padding_>=0 && padding_<=.01 &&
          std::isfinite(rack_padding_) && rack_padding_>=0 && rack_padding_<=.05 &&
          std::isfinite(base_padding_) && base_padding_>=0 && base_padding_<=.05 && timeout_>0 && gap_>0 &&
          history_>=scan_duration_+timeout_ && range_>0 && std::isfinite(min_sensor_range_) &&
          min_sensor_range_>=0 && min_sensor_range_<range_ && radius_>0 && maximum_cells_>0 &&
          std::isfinite(outlier_radius_) && outlier_radius_>0 && outlier_radius_<=.25 &&
          outlier_min_neighbors_>=0 && outlier_min_neighbors_<=64 &&
          odom_depth>1 && joint_depth>1 && minimum_points_>0 && pending_timeout_>0 && joint_timeout_>0 &&
          floor_band_>0 && floor_config_.radius>.65 && floor_config_.radius<=range_ &&
          floor_config_.inlier_distance>0 && floor_config_.inlier_distance<floor_band_ &&
          floor_config_.max_tilt>0 && floor_config_.max_tilt<=.174533 &&
          floor_config_.max_offset>0 && floor_config_.max_offset<=.1 &&
          floor_config_.min_ratio>.5 && floor_config_.min_ratio<=1. &&
          floor_interval_>0 && floor_timeout_>floor_interval_ && free_observations_>=2))
      throw std::runtime_error("invalid_collision_configuration");
    geometry_=std::make_unique<Geometry>(text("geometry_file",""));
    mask_rack_interior_=declare_parameter<bool>("mask_rack_interior",true);
    if(mask_rack_interior_ && geometry_->self_interiors.empty())
      throw std::runtime_error("missing_rack_interior_geometry");
    nav_pub_=create_publisher<Cloud>(text("filtered_topic","/mid360/points_filtered"),rclcpp::SensorDataQoS().keep_last(1));
    obstacle_pub_=create_publisher<Cloud>(text("obstacle_topic","/mid360/points_obstacles"),rclcpp::SensorDataQoS().keep_last(1));
    clearing_pub_=create_publisher<Cloud>(text("clearing_topic","/mid360/points_clearing"),rclcpp::SensorDataQoS().keep_last(1));
    // Guard input is an entire retained scene, several MB rather than a small
    // sensor sample. Recover transport loss without accumulating old scenes.
    safety_pub_=create_publisher<Cloud>(text("safety_topic","/collision/points"),
      rclcpp::QoS(1).reliable().lifespan(rclcpp::Duration::from_seconds(timeout_)));
    status_pub_=create_publisher<std_msgs::msg::String>(text("cloud_status_topic","/collision/cloud_status"),10);
    tf_=std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_=std::make_unique<tf2_ros::TransformListener>(*tf_);
    cloud_sub_=create_subscription<Cloud>(text("cloud_topic","/livox/lidar"),rclcpp::SensorDataQoS().keep_last(1),
      [this](Cloud::ConstSharedPtr msg){receive_cloud(std::move(msg));});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(text("odom_topic","/odom_ekf"),rclcpp::SensorDataQoS().keep_last(odom_depth),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){receive_odom(*msg);});
    joint_sub_=create_subscription<sensor_msgs::msg::JointState>(text("joint_topic","/robot/joint_states"),rclcpp::SensorDataQoS().keep_last(joint_depth),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg){receive_joints(*msg);});
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{std::lock_guard<std::mutex> lock(mutex_); status_locked();});
    worker_=std::thread([this]{work();});
  }
  ~CollisionCloud() override {
    {std::lock_guard<std::mutex> lock(mutex_); stopping_=true;}
    changed_.notify_all(); if (worker_.joinable()) worker_.join();
  }
 private:
  std::string base_,odom_,format_,time_field_;
  double history_,gap_,pending_timeout_,timeout_,joint_timeout_,scan_duration_,floor_,floor_band_,height_,range_,voxel_,padding_,radius_,arm_speed_;
  double min_sensor_range_,outlier_radius_,rack_padding_,base_padding_,arm_shadow_radius_,arm_shadow_depth_;
  collision_floor::Config floor_config_;
  collision_floor::Plane floor_plane_;uint64_t floor_epoch_=0;
  double floor_interval_,floor_timeout_;int free_observations_;
  int outlier_min_neighbors_;
  bool mask_rack_interior_;
  bool require_arm_,stopping_=false,valid_=false;
  int minimum_points_,maximum_cells_;
  std::mutex mutex_; std::condition_variable changed_; std::thread worker_;
  uint64_t epoch_=1,sequence_=0,dropped_=0,map_epoch_=0;
  const std::string session_=std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
  int64_t last_input_=0;
  std::optional<Pending> pending_;
  std::deque<PoseSample> poses_; std::deque<JointSample> joints_;
  Metrics metrics_; std::deque<double> durations_;
  std::string error_="waiting_for_mid360",previous_reason_,last_rejection_;
  uint64_t rejected_frames_=0;
  std::unordered_map<Key,Cell,Hash> cells_; // Worker thread only.
  std::unique_ptr<Geometry> geometry_;
  std::unique_ptr<tf2_ros::Buffer> tf_; std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<Cloud>::SharedPtr nav_pub_,obstacle_pub_,clearing_pub_,safety_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void reset_locked(const std::string &reason) {
    ++epoch_; pending_.reset(); valid_=false; error_=reason; last_input_=0;
    status_locked();
  }
  void receive_cloud(Cloud::ConstSharedPtr msg) {
    const auto stamp=stamp_ns(msg->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    if (stamp<=0 || msg->header.frame_id.empty()) {valid_=false;error_="invalid_cloud_header";return;}
    if (last_input_ && stamp<last_input_) reset_locked("cloud_clock_reset");
    if (stamp==last_input_) return;
    last_input_=stamp;
    if (pending_) ++dropped_;
    pending_=Pending{std::move(msg),Steady::now(),epoch_,++sequence_}; changed_.notify_one();
  }
  void receive_odom(const nav_msgs::msg::Odometry &msg) {
    if (msg.header.frame_id!=odom_ || msg.child_frame_id!=base_) return;
    const auto &p=msg.pose.pose.position; const auto &q=msg.pose.pose.orientation;
    const auto stamp=stamp_ns(msg.header.stamp);
    if (stamp<=0 || !std::isfinite(p.x+p.y+q.x+q.y+q.z+q.w) ||
        q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<1e-12) return;
    V3 pose(p.x,p.y,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)));
    std::lock_guard<std::mutex> lock(mutex_);
    if (!poses_.empty()) {
      const auto &last=poses_.back();
      const double heading=std::atan2(std::sin(pose.z()-last.pose.z()),std::cos(pose.z()-last.pose.z()));
      if (stamp<last.stamp || (pose.head<2>()-last.pose.head<2>()).norm()>.15 || std::abs(heading)>.15) {
        reset_locked("odometry_discontinuity"); poses_.clear(); joints_.clear();
      } else if (stamp==last.stamp) return;
      else pose.z()=last.pose.z()+heading;
    }
    poses_.push_back({stamp,pose});
    while (poses_.size()>2 && poses_[1].stamp<stamp-int64_t(history_*NS)) poses_.pop_front();
    changed_.notify_one();
  }
  void receive_joints(const sensor_msgs::msg::JointState &msg) {
    const auto stamp=stamp_ns(msg.header.stamp);
    if (stamp<=0 || msg.name.size()!=msg.position.size()) return;
    std::vector<double> values;
    for (const auto &name : geometry_->required) {
      const auto found=std::find(msg.name.begin(),msg.name.end(),name);
      if (found==msg.name.end()) return;
      const double value=msg.position[std::distance(msg.name.begin(),found)];
      if (!std::isfinite(value)) return;
      values.push_back(value);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!joints_.empty() && stamp<=joints_.back().stamp) return;
    joints_.push_back({stamp,std::move(values)});
    while (joints_.size()>2 && joints_[1].stamp<stamp-int64_t(history_*NS)) joints_.pop_front();
    changed_.notify_one();
  }
  template<typename T> static T packed(const uint8_t *data,bool swap) {
    std::array<uint8_t,sizeof(T)> bytes{}; std::copy_n(data,sizeof(T),bytes.data());
    if (swap) std::reverse(bytes.begin(),bytes.end());
    T out; std::memcpy(&out,bytes.data(),sizeof(T)); return out;
  }
  static double number(const uint8_t *data,const sensor_msgs::msg::PointField &field,bool swap) {
    if (field.datatype==sensor_msgs::msg::PointField::FLOAT32) return packed<float>(data+field.offset,swap);
    if (field.datatype==sensor_msgs::msg::PointField::FLOAT64) return packed<double>(data+field.offset,swap);
    if (field.datatype==sensor_msgs::msg::PointField::UINT32) return packed<uint32_t>(data+field.offset,swap);
    throw std::runtime_error("unsupported_point_field_type");
  }
  Key key(const V3 &point) const {return {int(std::floor(point.x()/voxel_)),int(std::floor(point.y()/voxel_)),int(std::floor(point.z()/voxel_))};}
  V3 center(const Key &k) const {return V3(k.x+.5,k.y+.5,k.z+.5)*voxel_;}
  Cloud make_cloud(const std::vector<V3> &points,const std::string &frame,int64_t reference,const std::vector<V3> *sources=nullptr,
      const std::vector<float> *ages=nullptr,const std::vector<float> *heights=nullptr) {
    Cloud out; out.header.frame_id=frame; out.header.stamp=rclcpp::Time(reference);
    out.height=1; out.width=points.size(); out.is_dense=true; out.is_bigendian=false;
    std::vector<std::string> names=sources?std::vector<std::string>{"x","y","z","source_x","source_y","source_z"}:std::vector<std::string>{"x","y","z"};
    if(ages)names.push_back("last_seen_age");
    if(heights)names.push_back("height_above_floor");
    for (size_t i=0;i<names.size();++i) {
      sensor_msgs::msg::PointField f;f.name=names[i];f.offset=4*i;f.count=1;f.datatype=sensor_msgs::msg::PointField::FLOAT32;out.fields.push_back(f);
    }
    out.point_step=4*names.size();out.row_step=out.width*out.point_step;out.data.resize(out.row_step);
    for (size_t i=0;i<points.size();++i) {
      std::array<float,8> values{};
      for (int j=0;j<3;++j) {values[j]=float(points[i][j]);if(sources)values[j+3]=float((*sources)[i][j]);}
      if(ages)values[6]=(*ages)[i];
      if(heights)values[7]=(*heights)[i];
      std::memcpy(out.data.data()+i*out.point_step,values.data(),out.point_step);
    }
    return out;
  }
  // Grid traversal marks only cells actually crossed by a new beam. Endpoints
  // and their vicinity are excluded; three separate clear observations needed.
  void free_ray(const V3 &origin,const V3 &hit,uint64_t sequence,const std::unordered_set<Key,Hash> &coarse,
      const collision_floor::Plane &floor,const V3 &reference_pose) {
    const V3 delta=hit-origin;
    const double length=delta.norm();double limit=length-2*voxel_;
    if (limit<=0) return;
    const V3 direction=delta/length;
    // A distant return proves that the near part of the beam is free too.
    // Clip traversal at the local XY range instead of dropping the entire ray.
    const Eigen::Vector2d local_origin=origin.head<2>()-reference_pose.head<2>();
    const Eigen::Vector2d xy_direction=direction.head<2>();
    const double a=xy_direction.squaredNorm(),b=local_origin.dot(xy_direction);
    if(a>1e-12) {
      const double discriminant=b*b+a*(range_*range_-local_origin.squaredNorm());
      if(discriminant<0.)return;
      limit=std::min(limit,(-b+std::sqrt(discriminant))/a);
    } else if(local_origin.squaredNorm()>range_*range_)return;
    if(direction.z()>1e-12) limit=std::min(limit,(height_-origin.z())/direction.z());
    const double floor_slope=direction.z()-floor.coefficients.x()*direction.x()-floor.coefficients.y()*direction.y();
    if(floor_slope< -1e-12)limit=std::min(limit,(floor_band_-floor.height(origin))/floor_slope);
    collision_grid::occupied_cells(origin,direction,limit,voxel_,coarse,[&](const Key &cell) {
      const auto found=cells_.find(cell);
      if(found!=cells_.end())found->second.free_frame=sequence;
    });
  }
  void process(const Pending &job) {
    const auto began=Steady::now(); Metrics m; m.sequence=job.sequence;m.epoch=job.epoch;m.queue_ms=elapsed_ms(job.received,began);
    const auto &msg=*job.cloud;
    std::array<sensor_msgs::msg::PointField,4> fields;
    const std::array<std::string,4> names{"x","y","z",time_field_};
    for (size_t i=0;i<names.size();++i) {
      auto f=std::find_if(msg.fields.begin(),msg.fields.end(),[&](const auto &item){return item.name==names[i];});
      if (f==msg.fields.end() || f->count!=1) throw std::runtime_error("missing_per_point_time");
      const size_t size=f->datatype==sensor_msgs::msg::PointField::FLOAT64?8:4;
      if (f->offset+size>msg.point_step) throw std::runtime_error("invalid_cloud_layout");
      fields[i]=*f;
    }
    if (!msg.point_step || msg.row_step<size_t(msg.width)*msg.point_step || msg.data.size()<size_t(msg.height)*msg.row_step)
      throw std::runtime_error("invalid_cloud_layout");
    const uint16_t endian=1; const bool host_big=*reinterpret_cast<const uint8_t *>(&endian)==0;
    const bool swap=host_big!=msg.is_bigendian;
    const int64_t header=stamp_ns(msg.header.stamp);
    std::vector<V3> raw; std::vector<int64_t> stamps;
    raw.reserve(size_t(msg.width)*msg.height);stamps.reserve(raw.capacity());
    for (size_t row=0;row<msg.height;++row) for(size_t col=0;col<msg.width;++col) {
      const auto *ptr=msg.data.data()+row*msg.row_step+col*msg.point_step;
      V3 p(number(ptr,fields[0],swap),number(ptr,fields[1],swap),number(ptr,fields[2],swap));
      const double value=number(ptr,fields[3],swap);
      if (!p.allFinite() || !std::isfinite(value)) continue;
      double offset=0;
      if (format_=="absolute_ns") offset=value-double(header);
      else if (format_=="relative_ns") offset=value;
      else if (format_=="relative_sec") offset=value*NS;
      else throw std::runtime_error("unsupported_point_time_format");
      if (offset< -1000 || offset>scan_duration_*NS) throw std::runtime_error("invalid_per_point_time");
      raw.push_back(p);stamps.push_back(header+int64_t(std::llround(std::max(0.,offset))));
    }
    m.raw=raw.size();
    if (raw.size()<size_t(minimum_points_)) throw std::runtime_error("insufficient_raw_points");
    const auto bounds=std::minmax_element(stamps.begin(),stamps.end());m.start=*bounds.first;m.end=*bounds.second;m.reference=m.start+(m.end-m.start)/2;
    m.parse_ms=elapsed_ms(began);
    auto fresh=[&]{const double age=(now().nanoseconds()-m.reference)/NS;return age>=-.05 && age<=timeout_;};
    if (!fresh()) throw std::runtime_error("stale_cloud_timestamp");
    std::deque<PoseSample> poses; std::deque<JointSample> joints;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      auto covered=[&]{return poses_.size()>1 && poses_.back().stamp>=m.end && (!require_arm_ || (joints_.size()>1 && joints_.back().stamp>=m.end));};
      while (!covered() && !stopping_ && epoch_==job.epoch && elapsed_ms(job.received)<pending_timeout_*1000 && fresh())
        changed_.wait_for(lock,std::chrono::milliseconds(5));
      if (stopping_ || epoch_!=job.epoch) return;
      if (!covered()) throw std::runtime_error(require_arm_ && (joints_.empty() || joints_.back().stamp<m.end)?"stale_arm_feedback":"missing_odometry_interval");
      poses=poses_;joints=joints_;
    }
    const auto max_gap=int64_t(gap_*NS);
    for (size_t i=1;i<poses.size();++i) if (poses[i].stamp>=m.start && poses[i-1].stamp<=m.end) {
      m.odom_gap_ms=std::max(m.odom_gap_ms,double(poses[i].stamp-poses[i-1].stamp)/1e6);
      if (poses[i].stamp-poses[i-1].stamp>max_gap) throw std::runtime_error("odometry_gap");
    }
    const V3 reference_pose=pose_at(poses,m.reference,max_gap);
    auto body_boxes=geometry_->fixed;
    if (require_arm_) {
      if (joints.front().stamp>m.start || joints.back().stamp<m.end) throw std::runtime_error("stale_arm_feedback");
      auto right=std::upper_bound(joints.begin(),joints.end(),m.reference,[](int64_t t,const JointSample &p){return t<p.stamp;});
      if (right==joints.end()) --right;
      const auto left=std::prev(right);
      if (right->stamp-left->stamp>int64_t(joint_timeout_*NS)) throw std::runtime_error("stale_arm_feedback");
      const double w=double(m.reference-left->stamp)/double(right->stamp-left->stamp);
      std::vector<double> q(left->q.size());
      std::vector<double> movement(q.size(),0.);
      for(size_t j=0;j<q.size();++j) q[j]=left->q[j]+w*(right->q[j]-left->q[j]);
      for(size_t i=1;i<joints.size();++i) if(joints[i].stamp>=m.start && joints[i-1].stamp<=m.end) {
        const double dt=(joints[i].stamp-joints[i-1].stamp)/NS;
        if(dt<=0 || dt>joint_timeout_) throw std::runtime_error("stale_arm_feedback");
        for(size_t j=0;j<q.size();++j) {
          if(std::abs(joints[i].q[j]-joints[i-1].q[j])/dt>arm_speed_) throw std::runtime_error("arm_is_moving");
          movement[j]=std::max({movement[j],std::abs(joints[i].q[j]-q[j]),std::abs(joints[i-1].q[j]-q[j])});
        }
      }
      double mask_motion=0.;for(size_t j=0;j<q.size();++j)mask_motion+=movement[j]*geometry_->joint_reach[j];
      if(mask_motion>padding_)throw std::runtime_error("arm_is_moving");
      body_boxes=geometry_->at(q,base_);
    }
    if(mask_rack_interior_)body_boxes.insert(body_boxes.end(),geometry_->self_interiors.begin(),geometry_->self_interiors.end());
    const auto tf=tf_->lookupTransform(base_,msg.header.frame_id,rclcpp::Time(m.reference));
    const auto &t=tf.transform.translation;const auto &r=tf.transform.rotation;
    Eigen::Quaterniond rotation(r.w,r.x,r.y,r.z);
    if(!rotation.coeffs().allFinite() || rotation.norm()<1e-9) throw std::runtime_error("invalid_sensor_transform");
    const M3 extrinsic=rotation.normalized().toRotationMatrix();const V3 offset(t.x,t.y,t.z);
    if(!offset.allFinite()) throw std::runtime_error("invalid_sensor_transform");
    auto section=Steady::now();
    std::vector<V3> body,world,origins;body.reserve(raw.size());world.reserve(raw.size());origins.reserve(raw.size());
    for(size_t i=0;i<raw.size();++i) {
      const V3 p=extrinsic*raw[i]+offset;const V3 pose=pose_at(poses,stamps[i],max_gap);
      body.push_back(p);world.push_back(world_point(p,pose));origins.push_back(world_point(offset,pose));
    }
    m.deskew_ms=elapsed_ms(section);section=Steady::now();
    std::vector<uint8_t> self(raw.size(),0);
    // Use the original sensor coordinates: base_link is a different origin.
    // Rejected returns do not mark obstacles or clear any free-space rays.
    for(size_t i=0;i<raw.size();++i)if(collision_mask::within_sensor_radius(raw[i],min_sensor_range_)) {
      self[i]=1;++m.masked;++m.near_sensor;
    }
    std::vector<collision_mask::Bounds> mask_bounds;mask_bounds.reserve(body_boxes.size());
    std::vector<double> mask_paddings;mask_paddings.reserve(body_boxes.size());
    collision_mask::Bounds overall;
    for(const auto &box : body_boxes) {
      double padding=padding_;
      if(box.name=="self_interior/rack_base_link")padding=std::max(padding,rack_padding_);
      if(box.name.rfind("base_link/",0)==0 || box.name.find("_wheel_link/")!=std::string::npos)
        padding=std::max(padding,base_padding_);
      mask_paddings.push_back(padding);
      mask_bounds.emplace_back(box.pose.linear(),box.pose.translation(),box.half,padding);
      overall.include(mask_bounds.back());
    }
    std::vector<size_t> candidates;candidates.reserve(body.size());
    for(size_t i=0;i<body.size();++i)if(overall.contains(body[i]))candidates.push_back(i);
    for(size_t b=0;b<body_boxes.size();++b) for(const auto i : candidates) {
      const auto &box=body_boxes[b];
      if(!self[i] && mask_bounds[b].contains(body[i]) && box.contains(body[i],mask_paddings[b])) {
        self[i]=1;++m.masked;++m.mask_hits[box.name];
      }
    }
    // A beam that grazes an arm edge returns a range between the arm and the
    // surface behind it. Those mixed returns lie on the ray just beyond the
    // arm and are swept into by the arm boxes when the base moves.
    // Only the retained safety map skips them; detection clouds keep every return.
    std::vector<uint8_t> shadow(raw.size(),0);
    if(require_arm_ && arm_shadow_radius_>0. && arm_shadow_depth_>0.) {
      std::vector<size_t> arm;
      for(size_t b=0;b<body_boxes.size();++b)if(body_boxes[b].name.rfind("piper_",0)==0)arm.push_back(b);
      for(size_t i=0;i<body.size();++i) {
        if(self[i])continue;
        const V3 ray=body[i]-offset;const double length=ray.norm();
        if(length<1e-6)continue;
        for(const auto b : arm) {
          const auto &box=body_boxes[b];const M3 inverse=box.pose.linear().transpose();
          const V3 origin=inverse*(offset-box.pose.translation()),direction=inverse*ray;
          const V3 half=box.half.array()+arm_shadow_radius_;
          double enter=0.,leave=1.;bool hit=true;
          for(int a=0;a<3 && hit;++a) {
            if(std::abs(direction[a])<1e-12) {hit=std::abs(origin[a])<=half[a];continue;}
            double t0=(-half[a]-origin[a])/direction[a],t1=(half[a]-origin[a])/direction[a];
            if(t0>t1)std::swap(t0,t1);
            enter=std::max(enter,t0);leave=std::min(leave,t1);hit=enter<=leave;
          }
          if(hit && (1.-leave)*length<=arm_shadow_depth_) {shadow[i]=1;++m.arm_shadow;break;}
        }
      }
    }
    m.mask_ms=elapsed_ms(section);section=Steady::now();
    std::vector<uint8_t> eligible(raw.size(),0);
    for(size_t i=0;i<world.size();++i) {
      // Keep valid distant endpoints for clearing. Only marking/display are
      // range-limited below; the same outlier/self filters apply to every ray.
      eligible[i]=!self[i];
    }
    const auto isolated=collision_filter::isolated_points(world,eligible,outlier_radius_,outlier_min_neighbors_);
    m.isolated=std::count(isolated.begin(),isolated.end(),uint8_t(1));
    m.outlier_ms=elapsed_ms(section);const auto floor_section=Steady::now();
    std::vector<uint8_t> floor_eligible=eligible;
    for(size_t i=0;i<isolated.size();++i)if(isolated[i])floor_eligible[i]=0;
    auto floor=floor_epoch_==job.epoch?floor_plane_:collision_floor::Plane{};
    if(!floor.stamp || (m.reference-floor.stamp)/NS>=floor_interval_) {
      const auto estimate=collision_floor::estimate(world,floor_eligible,reference_pose,m.reference,floor_config_);
      if(estimate)floor=*estimate;
    }
    if(!floor.stamp || (m.reference-floor.stamp)/NS>floor_timeout_)throw std::runtime_error("unreliable_floor_plane");
    m.floor_plane=floor;
    m.floor_ms=elapsed_ms(floor_section);section=Steady::now();
    if(map_epoch_!=job.epoch) {cells_.clear();map_epoch_=job.epoch;}
    auto next_cells=cells_; // Transaction: rejected/expired frames never modify the retained map.
    std::vector<V3> nav,obstacles,clearing;std::vector<size_t> rays;
    nav.reserve(raw.size());obstacles.reserve(raw.size());clearing.reserve(raw.size());rays.reserve(raw.size());
    const double c=std::cos(reference_pose.z()),s=std::sin(reference_pose.z());
    for(size_t i=0;i<world.size();++i) {
      if(!eligible[i] || isolated[i]) continue;
      const V3 delta=world[i]-V3(reference_pose.x(),reference_pose.y(),0);
      clearing.emplace_back(c*delta.x()+s*delta.y(),-s*delta.x()+c*delta.y(),world[i].z());rays.push_back(i);
      if(delta.head<2>().squaredNorm()>range_*range_) {++m.long_range_clearing_rays;continue;}
      nav.push_back(clearing.back());
      if(floor.height(world[i])>floor_band_ && world[i].z()<=height_) {
        obstacles.push_back(nav.back());
        if(shadow[i])continue;
        auto &cell=next_cells[key(world[i])];cell.hit_frame=job.sequence;cell.misses=0;
        if(stamps[i]>=cell.last_seen){cell.source=world[i];cell.last_seen=stamps[i];}
      } else if(floor.height(world[i])<=floor_band_) {
        ++m.ground;
      }
    }
    // Swap only worker-owned state temporarily for traversal; on any exception
    // the next frame starts a new map generation, never publishing partial work.
    cells_.swap(next_cells);
    try {
      if(cells_.size()>size_t(maximum_cells_)) throw std::runtime_error("obstacle_capacity_exceeded");
      std::unordered_set<Key,Hash> coarse;coarse.reserve(cells_.size()/4+1);
      for(const auto &[cell,value] : cells_) {static_cast<void>(value);coarse.insert(collision_grid::coarse_key(cell));}
      m.clearing_rays=rays.size();
      for(const auto i : rays) free_ray(origins[i],world[i],job.sequence,coarse,floor,reference_pose);
      for(auto it=cells_.begin();it!=cells_.end();) {
        const auto point=center(it->first);auto &cell=it->second;
        const bool outside=(point.head<2>()-reference_pose.head<2>()).squaredNorm()>radius_*radius_;
        // Unobserved / occluded is not contradictory evidence. Count distinct
        // free observations even when the non-repeating scan skips a frame.
        if(cell.hit_frame!=job.sequence && cell.free_frame==job.sequence)++cell.misses;
        const bool ground=floor.height(cell.source)<=floor_band_;
        if(ground)++m.ground_cells_removed;
        if(cell.misses>=free_observations_)++m.free_cells_removed;
        if(outside || ground || cell.misses>=free_observations_)it=cells_.erase(it);else ++it;
      }
      m.obstacles=cells_.size();m.map_ms=elapsed_ms(section);section=Steady::now();
      std::vector<V3> retained,sources;std::vector<float> ages,heights;
      retained.reserve(cells_.size());sources.reserve(cells_.size());ages.reserve(cells_.size());heights.reserve(cells_.size());
      for(const auto &[k,cell] : cells_) {
        retained.push_back(center(k));sources.push_back(cell.source);
        ages.push_back(float(std::max(0.,(m.reference-cell.last_seen)/NS)));heights.push_back(float(floor.height(cell.source)));
      }
      auto nav_msg=make_cloud(nav,base_,m.reference);auto obstacle_msg=make_cloud(obstacles,base_,m.reference);
      auto clearing_msg=make_cloud(clearing,base_,m.reference);
      auto safety_msg=make_cloud(retained,odom_,m.reference,&sources,&ages,&heights);
      m.publish_ms=elapsed_ms(section);m.total_ms=elapsed_ms(began);
      std::lock_guard<std::mutex> lock(mutex_);
      if(epoch_!=job.epoch || stopping_) {cells_.swap(next_cells);return;}
      if(!fresh()) throw std::runtime_error("processing_deadline_exceeded");
      metrics_=m;valid_=true;error_.clear();
      floor_plane_=floor;floor_epoch_=job.epoch;
      durations_.push_back(m.total_ms);if(durations_.size()>128)durations_.pop_front();
      nav_pub_->publish(std::move(nav_msg));obstacle_pub_->publish(std::move(obstacle_msg));
      clearing_pub_->publish(std::move(clearing_msg));safety_pub_->publish(std::move(safety_msg));status_locked();
    } catch(...) {cells_.swap(next_cells);throw;}
  }
  void status_locked() {
    const double age=metrics_.reference?(now().nanoseconds()-metrics_.reference)/NS:-1.;
    const bool ready=valid_ && metrics_.epoch==epoch_ && age>=-.05 && age<=timeout_;
    const std::string reason=ready?"live":(!error_.empty()?error_:"stale_mid360");
    std::vector<double> durations(durations_.begin(),durations_.end());std::sort(durations.begin(),durations.end());
    const double p95=durations.empty()?0.:durations[size_t(.95*(durations.size()-1))];
    std::ostringstream out;out<<std::setprecision(15)<<"{\"ready\":"<<(ready?"true":"false")<<",\"reason\":"<<quoted(reason)
      <<",\"source_age\":"<<age<<",\"floor_z\":"<<floor_<<",\"reference_stamp_ns\":"<<metrics_.reference
      <<",\"scan_start_ns\":"<<metrics_.start<<",\"scan_end_ns\":"<<metrics_.end<<",\"generation\":"<<epoch_
      <<",\"session_id\":"<<quoted(session_)
      <<",\"result_id\":"<<metrics_.sequence<<",\"raw_points\":"<<metrics_.raw<<",\"obstacles\":"<<metrics_.obstacles
      <<",\"self_masked_points\":"<<metrics_.masked<<",\"near_sensor_masked_points\":"<<metrics_.near_sensor<<",\"arm_shadow_masked_points\":"<<metrics_.arm_shadow
      <<",\"min_sensor_range\":"<<min_sensor_range_
      <<",\"mask_rack_interior\":"<<(mask_rack_interior_?"true":"false")<<",\"dropped_pending\":"<<dropped_
      <<",\"rejected_frames\":"<<rejected_frames_<<",\"last_rejection\":"<<quoted(last_rejection_)
      <<",\"rack_self_padding\":"<<rack_padding_
      <<",\"base_self_padding\":"<<base_padding_
      <<",\"ground_points\":"<<metrics_.ground<<",\"ground_cells_removed\":"<<metrics_.ground_cells_removed
      <<",\"free_cells_removed\":"<<metrics_.free_cells_removed<<",\"free_observations_to_clear\":"<<free_observations_
      <<",\"clearing_rays\":"<<metrics_.clearing_rays<<",\"long_range_clearing_rays\":"<<metrics_.long_range_clearing_rays
      <<",\"floor_band\":"<<floor_band_<<",\"floor_model_age\":"<<(metrics_.floor_plane.stamp?(now().nanoseconds()-metrics_.floor_plane.stamp)/NS:-1.)
      <<",\"floor_plane_frame\":"<<quoted(odom_)<<",\"floor_plane\":["<<metrics_.floor_plane.coefficients.x()<<','<<metrics_.floor_plane.coefficients.y()<<','<<metrics_.floor_plane.coefficients.z()<<']'
      <<",\"floor_fit_samples\":"<<metrics_.floor_plane.samples<<",\"floor_fit_inliers\":"<<metrics_.floor_plane.inliers
      <<",\"floor_residual_p95\":"<<metrics_.floor_plane.residual_p95
      <<",\"isolated_points_removed\":"<<metrics_.isolated<<",\"outlier_radius\":"<<outlier_radius_
      <<",\"outlier_min_neighbors\":"<<outlier_min_neighbors_
      <<",\"queue_wait_ms\":"<<metrics_.queue_ms<<",\"processing_ms\":"<<metrics_.total_ms<<",\"processing_p95_ms\":"<<p95
      <<",\"odom_interval_max_ms\":"<<metrics_.odom_gap_ms<<",\"input_end_age_ms\":"<<(metrics_.end?(now().nanoseconds()-metrics_.end)/1e6:-1.)
      <<",\"timings_ms\":{\"parse\":"<<metrics_.parse_ms<<",\"deskew\":"<<metrics_.deskew_ms<<",\"self_mask\":"<<metrics_.mask_ms
      <<",\"outlier_filter\":"<<metrics_.outlier_ms<<",\"floor_fit\":"<<metrics_.floor_ms<<",\"voxel_update\":"<<metrics_.map_ms<<",\"serialization\":"<<metrics_.publish_ms<<"},\"self_mask_hits\":{";
    bool first=true;for(const auto &[name,count] : metrics_.mask_hits){if(!first)out<<',';first=false;out<<quoted(name)<<':'<<count;}
    out<<"}}";std_msgs::msg::String msg;msg.data=out.str();status_pub_->publish(msg);
    if(reason!=previous_reason_) {RCLCPP_INFO(get_logger(),"MID-360 preprocessing: %s",reason.c_str());previous_reason_=reason;}
  }
  void work() {
    while(true) {
      Pending job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock,[this]{return stopping_ || pending_.has_value();});
        if(stopping_)return;
        job=std::move(*pending_);pending_.reset();
      }
      try {process(job);}
      catch(const tf2::TransformException &) {reject(job,"missing_sensor_tf");}
      catch(const std::exception &e) {reject(job,e.what());}
    }
  }
  void reject(const Pending &job,const std::string &reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(job.epoch!=epoch_ || stopping_)return;
    ++rejected_frames_;last_rejection_=reason;
    // Dropping one late frame does not invalidate an already accepted scene.
    // Its original timestamp still expires at timeout_; never renew it here.
    const bool late=reason=="stale_cloud_timestamp" || reason=="processing_deadline_exceeded";
    if(!late)valid_=false;
    error_=reason;status_locked();
  }
};
} // namespace
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {auto node=std::make_shared<CollisionCloud>();rclcpp::spin(node);node.reset();}
  catch(const std::exception &error) {std::cerr<<"collision_cloud: "<<error.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
