#!/usr/bin/env bash
# Optional foreground SLAM process. Does not send velocity or navigation goals.
set -e
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$CONTROL_CENTER_DIR/slam-scan.sh"   # sets SLAM_PARAMS; starts timed MID-360 processing when selected
exec ros2 launch "$CONTROL_CENTER_DIR/slam-runtime.launch.py" \
  "slam_params_file:=$SLAM_PARAMS" \
  "use_sim_time:=${ROBOCUP_USE_SIM_TIME:-false}" "$@"
