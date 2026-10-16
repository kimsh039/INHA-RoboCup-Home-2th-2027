#!/usr/bin/env bash
# Tests only. All ROS traffic isolated from the robot domain.
set -euo pipefail
manip_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
overlay=${1:?Usage: verify_local.sh BUILT_OVERLAY PIPER_SETUP}
piper_setup=${2:?Use existing Piper setup.bash}
set +u
source /opt/ros/humble/setup.bash
source "$piper_setup"
source "$overlay/install/setup.bash"
set -u
export PYTHONPATH="$manip_dir/manipulation_ws/src/robocup_real_manipulation:${PYTHONPATH:-}"
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/robocup_manipulation_verification}
export ROS_DOMAIN_ID=187
/usr/bin/python3 -m pytest -q "$manip_dir/manipulation_ws/src/robocup_real_manipulation/test"
/usr/bin/python3 "$manip_dir/test_ros_contract.py"
