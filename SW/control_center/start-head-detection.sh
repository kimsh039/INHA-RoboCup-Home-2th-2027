#!/usr/bin/env bash
# Full-frame Head objects and debug image using the detector in the current checkout.
set -e
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/detection_ws/install/local_setup.bash" ]]; then
  source "$HOME/detection_ws/install/local_setup.bash"
fi
CONTROL_CENTER_HEAD_OVERLAY="$CONTROL_CENTER_DIR/../detection/head/head_detection_ws/install/local_setup.bash"
if [[ -f "$CONTROL_CENTER_HEAD_OVERLAY" ]]; then source "$CONTROL_CENTER_HEAD_OVERLAY"; fi
CONTROL_CENTER_YOLO_PYTHON="${ROBOCUP_YOLO_PYTHON:-$HOME/venvs/head_detection/bin/python}"
if [[ ! -x "$CONTROL_CENTER_YOLO_PYTHON" ]]; then
  echo "YOLO Python 실행 파일 없음: $CONTROL_CENTER_YOLO_PYTHON" >&2
  exit 1
fi
CONTROL_CENTER_MODEL="${ROBOCUP_MODEL:-$HOME/INHA-RoboCup-Home-2th-2027/SW/detection/head/models/head_yolo11n_20261013.engine}"
if [[ -z "${ROBOCUP_MODEL:-}" && ! -f "$CONTROL_CENTER_MODEL" ]]; then
  CONTROL_CENTER_MODEL="$HOME/INHA-RoboCup-Home-2th-2027/SW/detection/head/models/head_yolo11n_20261013.pt"
fi
echo "Head full-frame detection model: $CONTROL_CENTER_MODEL"
exec "$CONTROL_CENTER_YOLO_PYTHON" -u "$CONTROL_CENTER_DIR/head-detection-node.py" \
  --ros-args --params-file "$CONTROL_CENTER_DIR/head-web.yaml" \
  -p "model_path:=$CONTROL_CENTER_MODEL" -p "device:='${ROBOCUP_DEVICE:-0}'" \
  -p "use_sim_time:=${ROBOCUP_USE_SIM_TIME:-false}" "$@"
