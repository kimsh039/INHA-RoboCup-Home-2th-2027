#!/usr/bin/env bash
# Install a frozen source snapshot and switch current only after colcon succeeds.
set -eo pipefail
MANIP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MANIP_REPO="$(cd -- "$MANIP_DIR/../.." && pwd)"
MANIP_RUNTIME="${ROBOCUP_MANIPULATION_RUNTIME:-$HOME/robot_setup/manipulation-runtime}"
mkdir -p "$MANIP_RUNTIME/releases"
exec 9>"$MANIP_RUNTIME/build.lock"
flock 9
source /opt/ros/humble/setup.bash
for MANIP_OVERLAY in "$HOME/piper_ros/install/local_setup.bash" \
  "$MANIP_REPO/SW/detection/head/head_detection_ws/install/local_setup.bash"; do
  if [[ -f "$MANIP_OVERLAY" ]]; then source "$MANIP_OVERLAY"; fi
done
export PYTHONPATH="$HOME/robot_setup/ros-python:$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
MANIP_RELEASE="$(mktemp -d "$MANIP_RUNTIME/releases/$(date +%Y%m%d-%H%M%S)-XXXXXX")"
/usr/bin/python3 - "$MANIP_DIR/manipulation_ws/src" "$MANIP_RELEASE/src" <<'PY'
from pathlib import Path
import shutil
import sys
shutil.copytree(sys.argv[1],sys.argv[2],ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.pyc'))
PY
colcon --log-base "$MANIP_RELEASE/log" build \
  --base-paths "$MANIP_RELEASE/src" --build-base "$MANIP_RELEASE/build" \
  --install-base "$MANIP_RELEASE/install" --executor sequential \
  --packages-select robocup_real_manipulation_msgs robocup_real_manipulation \
  --cmake-args -DBUILD_TESTING=OFF
ln -s "$MANIP_RELEASE" "$MANIP_RUNTIME/.current-$$"
mv -Tf "$MANIP_RUNTIME/.current-$$" "$MANIP_RUNTIME/current"
printf 'Manipulation runtime: %s\n' "$MANIP_RELEASE"
