// Back away under the authoritative 3D swept-body guard. A front obstacle
// projected into a 2D starting footprint must not veto a retreat from it.
#include <nav2_behaviors/timed_behavior.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

namespace robocup_collision_processing {
class GuardedBackUp : public nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp> {
  using Status=nav2_behaviors::Status;
  using Clock=std::chrono::steady_clock;
  using Json=nlohmann::json;
  struct Input {
    Json guard=Json::object();bool actuator=false,odom=false;
    double x=0.,y=0.,yaw=0.,v=0.,w=0.,stamp=0.;
    Clock::time_point guard_at{},actuator_at{},odom_at{};
  };
  std::mutex mutex_;Input input_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr guard_sub_,actuator_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  Clock::time_point began_{},motion_at_{},progress_at_{},paused_at_{},stop_at_{};
  double distance_=0.,speed_=0.,allowance_=6.,x_=0.,y_=0.,yaw_=0.,best_=0.;
  bool moving_=false,stopping_=false;
  static double age(Clock::time_point value) {return std::chrono::duration<double>(Clock::now()-value).count();}
  static double wrap(double angle) {return std::atan2(std::sin(angle),std::cos(angle));}
  bool odom_fresh(const Input &s) const {
    const double elapsed=clock_->now().seconds()-s.stamp;
    return s.odom && age(s.odom_at)<=.4 && elapsed>=-.1 && elapsed<=.4;
  }
  bool ready(const Input &s) const {
    const double stamp=s.guard.value("stamp",0.);
    const double elapsed=clock_->now().seconds()-stamp;
    return s.guard.value("ready",false) && age(s.guard_at)<=.5 && elapsed>=-.1 && elapsed<=.5
      && s.actuator && age(s.actuator_at)<=.5 && odom_fresh(s);
  }
 public:
  void onConfigure() override {
    const auto node=node_.lock();if(!node)throw std::runtime_error("missing_behavior_node");
    guard_sub_=node->create_subscription<std_msgs::msg::String>("/collision/status",rclcpp::QoS(1),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        try {auto value=Json::parse(message->data);if(!value.is_object())return;
          std::lock_guard<std::mutex> lock(mutex_);input_.guard=std::move(value);input_.guard_at=Clock::now();
        } catch(const std::exception &) {}
      });
    actuator_sub_=node->create_subscription<std_msgs::msg::String>("/collision/actuator_status",rclcpp::QoS(1),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        try {const auto value=Json::parse(message->data);
          std::lock_guard<std::mutex> lock(mutex_);input_.actuator=value.value("ready",false);input_.actuator_at=Clock::now();
        } catch(const std::exception &) {}
      });
    odom_sub_=node->create_subscription<nav_msgs::msg::Odometry>("/odom_ekf",rclcpp::SensorDataQoS().keep_last(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        const auto &p=message->pose.pose.position;const auto &q=message->pose.pose.orientation;
        const double v=message->twist.twist.linear.x,w=message->twist.twist.angular.z;
        const double stamp=message->header.stamp.sec+message->header.stamp.nanosec/1e9;
        if(message->header.frame_id!="odom" || message->child_frame_id!=robot_base_frame_ ||
           !std::isfinite(p.x+p.y+q.x+q.y+q.z+q.w+v+w+stamp))return;
        std::lock_guard<std::mutex> lock(mutex_);
        input_.x=p.x;input_.y=p.y;input_.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        input_.v=v;input_.w=w;input_.stamp=stamp;input_.odom=true;input_.odom_at=Clock::now();
      });
  }
  Status onRun(const std::shared_ptr<const nav2_msgs::action::BackUp::Goal> command) override {
    distance_=std::abs(command->target.x);speed_=std::abs(command->speed);
    if(command->target.y!=0. || command->target.z!=0. || !std::isfinite(distance_+speed_) ||
       distance_<=0. || distance_>.100001 || speed_<=0. || speed_>.050001) {
      RCLCPP_ERROR(logger_,"Guarded backup requires distance <= 10 cm and speed <= 0.05 m/s");return Status::FAILED;
    }
    distance_=std::min(distance_,.10);speed_=std::min(speed_,.05);
    const double requested=rclcpp::Duration(command->time_allowance).seconds();
    allowance_=requested>0.?std::min(requested,6.):6.;
    began_=progress_at_=Clock::now();paused_at_=stop_at_=Clock::time_point{};
    moving_=stopping_=false;best_=0.;stopRobot();
    RCLCPP_INFO(logger_,"3D guarded backup: %.1f cm; checking rear swept trajectory, no 2D start-overlap veto",distance_*100.);
    return Status::SUCCEEDED;
  }
  Status onCycleUpdate() override {
    Input s;{std::lock_guard<std::mutex> lock(mutex_);s=input_;}
    if(age(began_)>12.) {stopRobot();RCLCPP_WARN(logger_,"Backup elapsed limit incl. input waits");return Status::FAILED;}
    if(!ready(s)) {
      stopRobot();stop_at_=Clock::time_point{};
      if(paused_at_==Clock::time_point{})paused_at_=Clock::now();
      if(age(paused_at_)>3.) {
        RCLCPP_WARN(logger_,"Backup input unavailable: %s",s.guard.value("reason",std::string("status_missing")).c_str());
        return Status::FAILED;
      }
      return Status::RUNNING;
    }
    if(paused_at_!=Clock::time_point{}) {
      if(moving_)motion_at_+=Clock::now()-paused_at_;
      paused_at_=Clock::time_point{};progress_at_=Clock::now();
    }
    if(!moving_) {
      stopRobot();
      if(std::abs(s.v)>.01 || std::abs(s.w)>.02) {stop_at_=Clock::time_point{};return Status::RUNNING;}
      if(stop_at_==Clock::time_point{})stop_at_=Clock::now();
      if(age(stop_at_)<.2)return Status::RUNNING;
      moving_=true;x_=s.x;y_=s.y;yaw_=s.yaw;motion_at_=progress_at_=Clock::now();stop_at_=Clock::time_point{};
    }
    const double dx=s.x-x_,dy=s.y-y_;
    const double progress=-(dx*std::cos(yaw_)+dy*std::sin(yaw_));
    if(std::abs(-dx*std::sin(yaw_)+dy*std::cos(yaw_))>.03 || std::abs(wrap(s.yaw-yaw_))>.12 || progress<-.02) {
      stopRobot();RCLCPP_WARN(logger_,"Backup odometry left its straight reverse trajectory");return Status::FAILED;
    }
    auto feedback=std::make_shared<nav2_msgs::action::BackUp::Feedback>();
    feedback->distance_traveled=static_cast<float>(std::max(0.,progress));action_server_->publish_feedback(feedback);
    if(progress>=distance_-.003 || stopping_) {
      stopping_=true;stopRobot();
      if(std::abs(s.v)>.01 || std::abs(s.w)>.02) {stop_at_=Clock::time_point{};return Status::RUNNING;}
      if(stop_at_==Clock::time_point{})stop_at_=Clock::now();
      return age(stop_at_)>=.2?Status::SUCCEEDED:Status::RUNNING;
    }
    const auto request=s.guard.value("command",Json::array());
    const bool rear_blocked=s.guard.value("state",std::string{})=="blocked" &&
      s.guard.value("reason",std::string{})=="predicted_clearance_violation" && request.size()==2 && request[0].get<double>()<-.005;
    if(rear_blocked) {
      stopRobot();RCLCPP_WARN(logger_,"3D reverse blocked: link=%s point_base=%s",s.guard.value("blocking_link",Json()).dump().c_str(),
        s.guard.value("blocking_source_point_base",Json()).dump().c_str());return Status::FAILED;
    }
    if(progress>best_+.002) {best_=progress;progress_at_=Clock::now();}
    if(age(progress_at_)>2.5 || age(motion_at_)>allowance_) {
      stopRobot();RCLCPP_WARN(logger_,"3D reverse no progress: %.1f cm, guard=%s",progress*100.,
        s.guard.value("reason",std::string("unknown")).c_str());return Status::FAILED;
    }
    geometry_msgs::msg::Twist command;command.linear.x=-std::min(speed_,std::max(.02,.8*(distance_-progress)));
    vel_pub_->publish(command);
    return Status::RUNNING;
  }
  void onCleanup() override {guard_sub_.reset();actuator_sub_.reset();odom_sub_.reset();}
};
} // namespace robocup_collision_processing
PLUGINLIB_EXPORT_CLASS(robocup_collision_processing::GuardedBackUp,nav2_core::Behavior)
