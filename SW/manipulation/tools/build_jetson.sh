#!/usr/bin/env bash
# Build an isolated overlay; never modify the team's source workspace or start nodes.
set -euo pipefail
manip_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
repo_dir=$(cd -- "$manip_dir/../.." && pwd)
workspace=${1:?Usage: build_jetson.sh NEW_EMPTY_WORKSPACE [PIPER_INSTALL_SETUP]}
piper_setup=${2:-$HOME/piper_ros/install/setup.bash}
if [[ -e "$workspace" ]]; then
  echo "Refusing to overwrite existing workspace: $workspace" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$piper_setup" ]]; then
  echo 'Requires ROS 2 Humble and existing Piper install setup (argument 2).' >&2
  exit 2
fi
mkdir -p "$workspace/src"
# Copy source only, excluding stale x86/ARM build outputs and Python metadata.
tar -C "$manip_dir/manipulation_ws/src" --exclude=build --exclude=install --exclude=log --exclude=__pycache__ --exclude='*.egg-info' -cf - robocup_real_manipulation robocup_real_manipulation_msgs | tar -C "$workspace/src" -xf -
tar -C "$repo_dir/SW/detection/head/head_detection_ws/src" --exclude=build --exclude=install --exclude=log --exclude=__pycache__ --exclude='*.egg-info' -cf - robocup_head_detection robocup_detection_msgs | tar -C "$workspace/src" -xf -
set +u
source /opt/ros/humble/setup.bash
source "$piper_setup"
set -u
cd -- "$workspace"
/usr/bin/python3 -m colcon build --base-paths src --build-base build --install-base install --packages-select robocup_detection_msgs robocup_real_manipulation_msgs robocup_head_detection robocup_real_manipulation --cmake-args -DBUILD_TESTING=OFF
printf 'Build complete. Source %s/install/setup.bash before using this overlay.\n' "$workspace"
