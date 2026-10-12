#!/usr/bin/env bash
set -e
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
for CONTROL_CENTER_OVERLAY in \
  "$HOME/tracer_ws/install/local_setup.bash" \
  "$HOME/ws_livox/install/local_setup.bash" \
  "$HOME/piper_ros/install/local_setup.bash" \
  "$HOME/detection_ws/install/local_setup.bash" \
  "$CONTROL_CENTER_DIR/../detection/head/head_detection_ws/install/local_setup.bash"; do
  if [[ -f "$CONTROL_CENTER_OVERLAY" ]]; then source "$CONTROL_CENTER_OVERLAY"; fi
done
export PYTHONPATH="$HOME/robot_setup/ros-python:$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$HOME/robot_setup/livox-sdk2/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd -- "$CONTROL_CENTER_DIR"
exec /usr/bin/python3 -u ros_bridge.py "$@"
