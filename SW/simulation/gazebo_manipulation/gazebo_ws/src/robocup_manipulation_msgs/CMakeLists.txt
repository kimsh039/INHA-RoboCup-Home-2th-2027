cmake_minimum_required(VERSION 3.8)
project(robocup_manipulation_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WristMask.msg" "msg/Target3D.msg" "msg/Observation.msg" "msg/GraspCandidate.msg"
  "msg/TaskContext.msg" "msg/TaskStatus.msg" "msg/ServoCommand.msg"
  "action/PickPlace.action" "action/RunStage.action" "action/SetGripper.action" "action/ServoSession.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs moveit_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
