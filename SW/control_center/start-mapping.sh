#!/usr/bin/env bash
# Optional foreground SLAM process. Does not send velocity or navigation goals.
set -e
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
CONTROL_CENTER_SLAM_PARAMS="${ROBOCUP_SLAM_PARAMS:-$CONTROL_CENTER_DIR/slam-web.yaml}"
exec ros2 launch slam_toolbox online_async_launch.py \
  "slam_params_file:=$CONTROL_CENTER_SLAM_PARAMS" \
  "use_sim_time:=${ROBOCUP_USE_SIM_TIME:-false}" "$@"
