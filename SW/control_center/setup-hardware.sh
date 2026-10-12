#!/usr/bin/env bash
# Stop owned nodes and prepare both CAN links plus the Livox Ethernet profile.
set -euo pipefail
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $EUID -eq 0 ]]; then
  echo 'sudo 없이 bash setup-hardware.sh로 실행하세요. 필요한 단계에서 sudo를 요청합니다.' >&2
  exit 1
fi
/usr/bin/python3 "$CONTROL_CENTER_DIR/kill-center.py" --can-users
exec sudo /usr/bin/python3 "$CONTROL_CENTER_DIR/setup-hardware.py" --can --lidar "$@"
