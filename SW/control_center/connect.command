#!/usr/bin/env bash
# Mac: tunnel only. Start the web/ROS server on the Jetson first.
set -euo pipefail
ROBOCUP_SSH_TARGET="${ROBOCUP_SSH_TARGET:-sparo@192.168.50.70}"
ROBOCUP_LOCAL_PORT="${ROBOCUP_LOCAL_PORT:-8081}"
ROBOCUP_REMOTE_PORT="${ROBOCUP_REMOTE_PORT:-8765}"
if [[ ! "$ROBOCUP_SSH_TARGET" =~ ^[a-zA-Z0-9_.@:-]+$ ]] || [[ "$ROBOCUP_SSH_TARGET" == -* ]]; then
  echo 'ROBOCUP_SSH_TARGET에 SSH 주소 또는 SSH 별칭을 지정하세요.' >&2; exit 1
fi
for CONTROL_CENTER_PORT in "$ROBOCUP_LOCAL_PORT" "$ROBOCUP_REMOTE_PORT"; do
  if [[ ! "$CONTROL_CENTER_PORT" =~ ^[0-9]+$ ]] || (( 10#$CONTROL_CENTER_PORT < 1 || 10#$CONTROL_CENTER_PORT > 65535 )); then
    echo '포트는 1~65535 정수여야 합니다.' >&2; exit 1
  fi
done
CONTROL_CENTER_RUNTIME="$(mktemp -d "${TMPDIR:-/tmp}/inha-control-center.XXXXXX")"
CONTROL_CENTER_SOCKET="$CONTROL_CENTER_RUNTIME/ssh"
cleanup() {
  ssh -S "$CONTROL_CENTER_SOCKET" -O exit "$ROBOCUP_SSH_TARGET" >/dev/null 2>&1 || true
  rm -rf -- "$CONTROL_CENTER_RUNTIME"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
printf '젯슨 관제센터 터널: %s → http://localhost:%s\n' "$ROBOCUP_SSH_TARGET" "$ROBOCUP_LOCAL_PORT"
echo '젯슨에서 jetson-center.sh start를 먼저 실행하세요. SSH 암호는 터미널에만 입력합니다.'
ssh -M -S "$CONTROL_CENTER_SOCKET" -fN \
  -o ControlPersist=no -o ExitOnForwardFailure=yes -o ConnectTimeout=10 \
  -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  -L "127.0.0.1:$ROBOCUP_LOCAL_PORT:127.0.0.1:$ROBOCUP_REMOTE_PORT" "$ROBOCUP_SSH_TARGET"
open "http://localhost:$ROBOCUP_LOCAL_PORT"
echo '이 터미널을 유지하세요. Enter 또는 Ctrl+C로 맥의 터널만 종료합니다.'
read -r _ || true
