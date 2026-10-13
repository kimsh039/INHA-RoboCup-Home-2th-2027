#!/usr/bin/env bash
# Save the running SLAM Toolbox map: PGM/YAML for Nav2 map_server and a pose graph
# (.posegraph/.data) for SLAM Toolbox localization mode. Usage: save-map.sh [name]
set -e
source /opt/ros/humble/setup.bash
NAME="${1:-map_$(date +%Y%m%d_%H%M)}"
DIR="${ROBOCUP_MAP_DIR:-$HOME/maps}"
mkdir -p "$DIR"
ros2 run nav2_map_server map_saver_cli -f "$DIR/$NAME" --ros-args -p map_subscribe_transient_local:=true
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$DIR/$NAME'}"
ls -l "$DIR/$NAME".*
