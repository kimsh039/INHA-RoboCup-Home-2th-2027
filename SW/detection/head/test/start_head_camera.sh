#!/usr/bin/env bash
# Test-only camera launch. Does not start the base, arm, lidar or full robot.
set -e
source /opt/ros/humble/setup.bash
TEST_HEAD_SERIAL="${1:-${HEAD_SERIAL:-244622071116}}"
if ros2 topic info /head/camera/color/image_raw 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
  echo 'Head RGB publisher already exists. Reuse it; no duplicate camera launched.'
  exit 0
fi
echo "Starting only Head D435 ${TEST_HEAD_SERIAL}; ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
exec ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=head camera_name:=camera serial_no:="_${TEST_HEAD_SERIAL}" \
  enable_color:=true enable_depth:=true align_depth.enable:=true enable_sync:=true \
  rgb_camera.color_profile:=640,480,30 depth_module.depth_profile:=640,480,30 \
  pointcloud.enable:=false publish_tf:=true
