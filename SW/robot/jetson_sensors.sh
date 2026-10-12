#!/usr/bin/env bash
# Physical robot bringup in one tmux session; start does not run status checks.
set -e
ROBOT_RUNTIME_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  start)
    shift
    if tmux has-session -t robot_physical 2>/dev/null; then
      echo "robot_physical is already running; stop it before applying changed settings."
      exit 0
    fi
    printf -v ROBOT_START_COMMAND '%q ' bash "$ROBOT_RUNTIME_DIR/start_robot.sh" "$@"
    tmux new-session -d -s robot_physical -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" "$ROBOT_START_COMMAND"
    echo "Started robot_physical. Attach: tmux attach -t robot_physical"
    ;;
  stop)
    if ! tmux has-session -t robot_physical 2>/dev/null; then exit 0; fi
    tmux send-keys -t robot_physical C-c
    for ((attempt=0; attempt<40; attempt++)); do
      if ! tmux has-session -t robot_physical 2>/dev/null; then exit 0; fi
      sleep 0.5
    done
    echo "Shutdown is still in progress. Attach with: tmux attach -t robot_physical" >&2
    exit 1
    ;;
  status) tmux has-session -t robot_physical 2>/dev/null && echo running || echo stopped ;;
  *) echo "usage: $0 start [ROS launch arguments] | stop | status" >&2; exit 2 ;;
esac
