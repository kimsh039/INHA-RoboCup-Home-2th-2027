#!/usr/bin/env bash
# Jetson: the ROS bridge, web server and owned driver processes run in tmux.
set -euo pipefail
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTROL_CENTER_SESSION=inha_control_center
case "${1:-}" in
  start)
    shift
    if tmux has-session -t "$CONTROL_CENTER_SESSION" 2>/dev/null; then
      echo '관제센터가 이미 실행 중입니다. 재시작: jetson-center.sh stop 후 start'; exit 0
    fi
    CONTROL_CENTER_ARGS=(--with-robot)
    for CONTROL_CENTER_ARG in "$@"; do
      if [[ "$CONTROL_CENTER_ARG" == --observe ]]; then
        CONTROL_CENTER_ARGS=()
        break
      fi
    done
    if [[ ${#CONTROL_CENTER_ARGS[@]} -gt 0 ]] && tmux has-session -t robot_physical 2>/dev/null; then
      echo '기존 통합 센서 실행을 먼저 종료하세요:' >&2
      echo "bash '$CONTROL_CENTER_DIR/../robot/jetson_sensors.sh' stop" >&2
      echo '그 다음 관제센터 start를 실행하면 카메라·LiDAR를 개별 제어합니다.' >&2
      exit 1
    fi
    for CONTROL_CENTER_ARG in "$@"; do
      [[ "$CONTROL_CENTER_ARG" == --observe ]] || CONTROL_CENTER_ARGS+=("$CONTROL_CENTER_ARG")
    done
    printf -v CONTROL_CENTER_COMMAND '%q ' bash "$CONTROL_CENTER_DIR/run-robot.sh" "${CONTROL_CENTER_ARGS[@]}"
    tmux new-session -d -s "$CONTROL_CENTER_SESSION" \
      -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
      -e "ROBOCUP_MODEL=${ROBOCUP_MODEL:-}" \
      -e "ROBOCUP_YOLO_PYTHON=${ROBOCUP_YOLO_PYTHON:-}" \
      -e "ROBOCUP_DEVICE=${ROBOCUP_DEVICE:-0}" \
      -e "ROBOCUP_SLAM_PARAMS=${ROBOCUP_SLAM_PARAMS:-}" \
      -e "ROBOCUP_USE_SIM_TIME=${ROBOCUP_USE_SIM_TIME:-false}" \
      "$CONTROL_CENTER_COMMAND"
    echo '젯슨 관제센터 실행 요청을 전달했습니다. 서버: http://127.0.0.1:8765 (기본 포트)'
    echo '출력 보기: tmux attach -t inha_control_center'
    echo '맥에서: ssh -N -L 127.0.0.1:8081:127.0.0.1:8765 sparo@192.168.50.70'
    ;;
  stop)
    if ! tmux has-session -t "$CONTROL_CENTER_SESSION" 2>/dev/null; then exit 0; fi
    tmux send-keys -t "$CONTROL_CENTER_SESSION" C-c
    echo '젯슨 관제센터와 여기서 시작한 프로세스의 종료를 요청했습니다.'
    ;;
  logs) exec tmux attach -t "$CONTROL_CENTER_SESSION" ;;
  *) echo '사용법: jetson-center.sh start [--mapping] [--detection] [--observe] [--port 8765] | stop | logs' >&2; exit 2 ;;
esac
