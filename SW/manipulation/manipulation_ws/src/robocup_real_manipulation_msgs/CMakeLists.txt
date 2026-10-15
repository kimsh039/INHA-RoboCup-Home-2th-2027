cmake_minimum_required(VERSION 3.8)
project(robocup_real_manipulation_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  "msg/ApproachResult.msg"
  "msg/BaseLease.msg"
  "msg/ObservationRequest.msg"
  "msg/WristObservation.msg"
  "msg/TaskStatus.msg"
  "action/Manipulate.action" "action/SetWidth.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
