#!/usr/bin/env bash
# Optional foreground Nav2 process: Mid-360S self filter + Nav2 navigation servers.
# Needs SLAM (start-mapping.sh) for map -> odom. Sends velocity only for goals it receives.
set -e
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$CONTROL_CENTER_DIR/../.." && pwd)"
source /opt/ros/humble/setup.bash
NAV2_PARAMS="${ROBOCUP_NAV2_PARAMS:-$CONTROL_CENTER_DIR/nav2-web.yaml}"
USE_SIM_TIME="${ROBOCUP_USE_SIM_TIME:-false}"
python3 "$REPO_DIR/SW/simulation/ros2/cloud_self_filter.py" --ros-args \
  -p input:="${ROBOCUP_MID360_TOPIC:-/livox/lidar}" -p output:=/mid360/points_filtered \
  -p use_sim_time:="$USE_SIM_TIME" &
FILTER_PID=$!
trap 'kill -INT $FILTER_PID 2>/dev/null; wait $FILTER_PID 2>/dev/null' EXIT INT TERM
ros2 launch nav2_bringup navigation_launch.py \
  "params_file:=$NAV2_PARAMS" "use_sim_time:=$USE_SIM_TIME" "$@"
