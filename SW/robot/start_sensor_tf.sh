#!/usr/bin/env bash
# Keep this process running alongside the physical sensor drivers.
set -e
source /opt/ros/humble/setup.bash
ROBOT_REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec python3 "$ROBOT_REPO_DIR/SW/robot/sensor_tf.py" "$@"
