#!/usr/bin/env bash
set -e
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTROL_CENTER_LOG_ROOT="${XDG_STATE_HOME:-$HOME/.local/state}/inha-control-center/startup"
mkdir -p "$CONTROL_CENTER_LOG_ROOT"
CONTROL_CENTER_START_LOG="${ROBOCUP_CENTER_LOG:-$CONTROL_CENTER_LOG_ROOT/$(date +%Y%m%d-%H%M%S)-$$.log}"
exec > >(tee -a "$CONTROL_CENTER_START_LOG") 2>&1
printf '관제센터 시작 로그: %s\n' "$CONTROL_CENTER_START_LOG"
CONTROL_CENTER_STAGE='ROS 환경 준비'
trap 'CONTROL_CENTER_EXIT=$?; if (( CONTROL_CENTER_EXIT != 0 )); then printf "관제센터 시작 실패: %s (종료 코드 %s)\n" "$CONTROL_CENTER_STAGE" "$CONTROL_CENTER_EXIT"; fi' EXIT
source /opt/ros/humble/setup.bash
for CONTROL_CENTER_OVERLAY in \
  "$HOME/tracer_ws/install/local_setup.bash" \
  "$HOME/ws_livox/install/local_setup.bash" \
  "$HOME/piper_ros/install/local_setup.bash" \
  "$HOME/detection_ws/install/local_setup.bash" \
  "$CONTROL_CENTER_DIR/../detection/head/head_detection_ws/install/local_setup.bash"; do
  if [[ -f "$CONTROL_CENTER_OVERLAY" ]]; then
    CONTROL_CENTER_STAGE="ROS overlay: $CONTROL_CENTER_OVERLAY"
    source "$CONTROL_CENTER_OVERLAY"
  fi
done
# Custom ROS interfaces must be visible to the web action client as well.
CONTROL_CENTER_MANIP_OVERLAY="${ROBOCUP_MANIPULATION_OVERLAY:-$HOME/robot_setup/manipulation-runtime/current/install/local_setup.bash}"
if [[ -f "$CONTROL_CENTER_MANIP_OVERLAY" ]]; then
  source "$CONTROL_CENTER_MANIP_OVERLAY"
fi
export PYTHONPATH="$HOME/robot_setup/ros-python:$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$HOME/robot_setup/livox-sdk2/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd -- "$CONTROL_CENTER_DIR"
CONTROL_CENTER_STAGE='ROS bridge / HTTP 서버 시작'
exec /usr/bin/python3 -u ros_bridge.py "$@"
