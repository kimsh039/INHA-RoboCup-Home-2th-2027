#!/usr/bin/env bash
# Configure only Livox Ethernet; CAN adapter registration is independent.
set -euo pipefail
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec sudo /usr/bin/python3 "$CONTROL_CENTER_DIR/setup-hardware.py" --lidar "$@"
