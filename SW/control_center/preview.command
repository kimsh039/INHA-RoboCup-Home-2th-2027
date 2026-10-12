#!/usr/bin/env bash
set -euo pipefail
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$CONTROL_CENTER_DIR"
exec python3 server.py --demo --port "${ROBOCUP_PREVIEW_PORT:-8080}"
