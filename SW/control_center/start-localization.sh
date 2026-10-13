#!/usr/bin/env bash
# Optional foreground SLAM Toolbox localization on a saved pose graph (save-map.sh output).
# Usage: start-localization.sh /path/to/map_base  (without .posegraph). Starts at the map origin;
# publish /initialpose (control center "초기 위치 지정") when the robot is elsewhere.
set -e
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
MAP_BASE="${1:?saved map path without extension}"
[ -f "$MAP_BASE.posegraph" ] || { echo "missing $MAP_BASE.posegraph" >&2; exit 2; }
exec ros2 run slam_toolbox localization_slam_toolbox_node --ros-args \
  --params-file "${ROBOCUP_SLAM_PARAMS:-$CONTROL_CENTER_DIR/slam-web.yaml}" \
  -r __node:=slam_toolbox -p mode:=localization -p "map_file_name:=$MAP_BASE" \
  -p "map_start_pose:=[0.0, 0.0, 0.0]" -p "use_sim_time:=${ROBOCUP_USE_SIM_TIME:-false}"
