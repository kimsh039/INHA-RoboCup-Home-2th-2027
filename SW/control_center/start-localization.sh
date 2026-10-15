#!/usr/bin/env bash
# Optional foreground SLAM Toolbox localization on a saved pose graph (save-map.sh output).
# Usage: start-localization.sh /path/to/map_base (without extension).
# Searches the saved occupancy map globally, then SLAM Toolbox confirms/tracks the pose graph.
set -e
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
MAP_BASE="${1:?saved map path without extension}"
[ -f "$MAP_BASE.posegraph" ] || { echo "missing $MAP_BASE.posegraph" >&2; exit 2; }
[ -f "$MAP_BASE.yaml" ] || { echo "missing $MAP_BASE.yaml" >&2; exit 2; }
source "$CONTROL_CENTER_DIR/slam-scan.sh"   # mapping and localization share the same prepared scan
exec ros2 launch "$CONTROL_CENTER_DIR/slam-runtime.launch.py" \
  "slam_params_file:=$SLAM_PARAMS" "mode:=localization" "map_file_name:=$MAP_BASE" \
  "use_sim_time:=${ROBOCUP_USE_SIM_TIME:-false}"
