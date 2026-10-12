#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/sparo/tracer_ws/install/setup.bash
source /home/sparo/ws_livox/install/setup.bash
source /home/sparo/piper_ros/install/setup.bash
ROBOT_RUNTIME_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 launch "$ROBOT_RUNTIME_DIR/physical.launch.py" "$@"
