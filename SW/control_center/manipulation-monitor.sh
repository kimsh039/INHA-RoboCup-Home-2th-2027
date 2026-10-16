#!/usr/bin/env bash
# Real input adapter + read-only coordinator. No driver, camera, MoveIt, enable or goals.
set -euo pipefail
MANIP_SESSION=inha_manipulation_monitor
MANIP_CENTER=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
case "${1:-}" in
  start)
    if tmux has-session -t "$MANIP_SESSION" 2>/dev/null; then echo '매니퓰 모니터가 이미 실행 중입니다.'; exit 0; fi
    MANIP_OVERLAY="${ROBOCUP_MANIPULATION_OVERLAY:-$HOME/robot_setup/manipulation-runtime/current/install/local_setup.bash}"
    [[ -f "$MANIP_OVERLAY" ]] || { echo '매니퓰 runtime overlay가 없습니다.' >&2; exit 1; }
    printf -v MANIP_COMMAND '%q ' bash "$MANIP_CENTER/manipulation-monitor.sh" run
    tmux new-session -d -s "$MANIP_SESSION" "$MANIP_COMMAND"
    echo '읽기 전용 실행기와 실제 Detection 인계 어댑터 시작 요청. 준비/오류는 Manipulation 탭과 이벤트에서 확인하세요.'
    ;;
  run)
    set +u
    source /opt/ros/humble/setup.bash
    if [[ -f "$HOME/piper_ros/install/local_setup.bash" ]]; then source "$HOME/piper_ros/install/local_setup.bash"; fi
    source "${ROBOCUP_MANIPULATION_OVERLAY:-$HOME/robot_setup/manipulation-runtime/current/install/local_setup.bash}"
    set -u
    # Fail if a real coordinator already owns the interface; never replace it.
    python3 - <<'PY'
import rclpy,time
rclpy.init();n=rclpy.create_node('manipulation_monitor_preflight')
deadline=time.monotonic()+1.
while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
names=n.get_node_names()
n.destroy_node();rclpy.shutdown()
if 'real_manipulation' in names:raise SystemExit('기존 매니퓰 실행기가 있습니다. 모니터를 중복 실행하지 않습니다.')
PY
    exec ros2 launch robocup_real_manipulation integrated.launch.py hardware:=false auto_start_on_approach:=false start_control_center_adapter:=true
    ;;
  stop) tmux has-session -t "$MANIP_SESSION" 2>/dev/null && tmux send-keys -t "$MANIP_SESSION" C-c || true ;;
  status) tmux has-session -t "$MANIP_SESSION" 2>/dev/null && echo running || echo stopped ;;
  *) echo 'Usage: bash manipulation-monitor.sh start|stop|status' >&2; exit 2 ;;
esac
