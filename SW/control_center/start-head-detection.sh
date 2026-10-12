#!/usr/bin/env bash
# Optional YOLO/CSRT ROS process; no approach or motion nodes are launched.
set -e
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/detection_ws/install/local_setup.bash" ]]; then
  source "$HOME/detection_ws/install/local_setup.bash"
fi
CONTROL_CENTER_HEAD_OVERLAY="$CONTROL_CENTER_DIR/../detection/head/head_detection_ws/install/local_setup.bash"
if [[ -f "$CONTROL_CENTER_HEAD_OVERLAY" ]]; then source "$CONTROL_CENTER_HEAD_OVERLAY"; fi
CONTROL_CENTER_YOLO_PYTHON="${ROBOCUP_YOLO_PYTHON:-$HOME/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/python}"
CONTROL_CENTER_MODEL="${ROBOCUP_MODEL:-$CONTROL_CENTER_DIR/../detection/head/models/head_yolo11n_20261013.pt}"
exec ros2 launch robocup_head_detection head_detection.launch.py \
  "config:=$CONTROL_CENTER_DIR/head-web.yaml" "model_path:=$CONTROL_CENTER_MODEL" \
  "device:=${ROBOCUP_DEVICE:-0}" "python_executable:=$CONTROL_CENTER_YOLO_PYTHON" \
  "use_sim_time:=${ROBOCUP_USE_SIM_TIME:-false}" "$@"
