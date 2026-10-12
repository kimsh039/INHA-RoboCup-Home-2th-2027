#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/sparo/tracer_ws/install/setup.bash
source /home/sparo/ws_livox/install/setup.bash
source /home/sparo/piper_ros/install/setup.bash
# PiPER SDK 0.6.2 and python-can 4.6.1 live outside a venv (see ~/robot_setup/VIRTUAL_ENVIRONMENTS.md)
export PYTHONPATH="/home/sparo/robot_setup/ros-python:/home/sparo/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
ROBOT_RUNTIME_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 launch "$ROBOT_RUNTIME_DIR/physical.launch.py" "$@"
