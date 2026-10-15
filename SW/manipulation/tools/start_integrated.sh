#!/usr/bin/env bash
# This is a REAL hardware launch. Run only after explicit physical-motion authorization.
set -euo pipefail
manip_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
repo_dir=${ROBOCUP_REPO:-$(cd -- "$manip_dir/../.." && pwd)}
overlay=${1:?Usage: start_integrated.sh OVERLAY CONFIG VISION_CONFIG [TUNING_FILE] [VISION_PYTHON]}
config=${2:?Set measured fixed-frame scene and robot config}
vision=${3:?Set existing Jetson YOLO/SAM paths and factory camera topics}
tuning=${4:-$manip_dir/manipulation_ws/src/robocup_real_manipulation/config/tuning.yaml}
vision_python=${5:?Select existing Jetson ROS-compatible YOLO+SAM Python 3.10 venv}
set +u
source /opt/ros/humble/setup.bash
source "$overlay/install/setup.bash"
set -u
"$vision_python" -c 'import rclpy, cv_bridge, torch, ultralytics, sam2'
/usr/bin/python3 "$manip_dir/tools/check_integrated.py" --config "$config" --vision "$vision" --repo "$repo_dir" --runtime
# This wrapper owns only this launch process group. Ctrl-C uses ROS action shutdown/hold.
exec ros2 launch robocup_real_manipulation integrated.launch.py repo:="$repo_dir" config:="$config" vision_config:="$vision" tuning_file:="$tuning" vision_python:="$vision_python" hardware:=true start_wrist_frontend:=true start_head_adapter:=false auto_start_on_approach:=false
