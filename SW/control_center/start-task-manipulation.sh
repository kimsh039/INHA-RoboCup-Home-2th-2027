#!/usr/bin/env bash
# Started only by an explicit Task command. Never replaces another ROS owner.
set -euo pipefail
TASK_CENTER=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
TASK_REPO=$(cd -- "$TASK_CENTER/../.." && pwd)
TASK_CONFIG=${ROBOCUP_TASK_ROBOT_CONFIG:?Set the measured integrated robot profile}
TASK_VISION=${ROBOCUP_TASK_VISION_CONFIG:-$TASK_REPO/SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/wrist.yaml}
TASK_PYTHON=${ROBOCUP_TASK_VISION_PYTHON:-$HOME/venvs/sam21/bin/python}
TASK_OVERLAY=${ROBOCUP_MANIPULATION_OVERLAY:-$HOME/robot_setup/manipulation-runtime/current/install/local_setup.bash}
set +u
source /opt/ros/humble/setup.bash
source "$HOME/piper_ros/install/local_setup.bash"
source "$TASK_OVERLAY"
set -u
# An existing executor/driver must be selected explicitly, never duplicated.
/usr/bin/python3 - "$TASK_CONFIG" <<'PY'
import sys, time, yaml, rclpy
cfg=yaml.safe_load(open(sys.argv[1]))
rclpy.init()
node=rclpy.create_node('table_task_launch_preflight')
until=time.monotonic()+1.
while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
topics=(cfg['feedback_topic'],cfg['command_topic'],'/manipulation/readiness')
occupied=[topic for topic in topics if node.count_publishers(topic)]
node.destroy_node();rclpy.shutdown()
if occupied:raise SystemExit('기존 팔 실행과 중복 시작하지 않습니다: '+', '.join(occupied))
PY
exec ros2 launch robocup_real_manipulation integrated.launch.py \
  repo:="$TASK_REPO" config:="$TASK_CONFIG" vision_config:="$TASK_VISION" vision_python:="$TASK_PYTHON" \
  hardware:=true start_driver:=true start_wrist_frontend:=true start_control_center_adapter:=true \
  auto_start_on_approach:=false
