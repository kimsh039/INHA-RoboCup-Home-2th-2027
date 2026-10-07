#!/usr/bin/env bash
set -eo pipefail
gazebo_root="$(cd "$(dirname "$0")" && pwd)"
repository_root="${ROBOCUP_REPOSITORY:-$(dirname "$(dirname "$gazebo_root")")}"
source /opt/ros/humble/setup.bash
gazebo_overlay="${ROBOCUP_GAZEBO_CACHE:-/tmp/robocup-gazebo-$UID}/install"
if [ ! -f "$gazebo_overlay/setup.bash" ]; then
    bash "$gazebo_root/build.sh"
fi
source "$gazebo_overlay/setup.bash"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="$gazebo_root/runtime/ros_logs"
mkdir -p "$ROS_LOG_DIR"
# 동일 폴더의 중복 launch를 막고, 이전 Gazebo 서버와 discovery를 분리한다.
exec 9>"$gazebo_root/runtime/launch.lock"
if ! flock -n 9; then
    echo '이 폴더의 Gazebo가 이미 실행 중입니다. 기존 실행 터미널에서 Ctrl+C 후 다시 실행하세요.' >&2
    exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + $$ % 100))}"
printf '%s\n' "$ROS_DOMAIN_ID" > "$gazebo_root/runtime/ros_domain"
echo "ROS 통신 번호: $ROS_DOMAIN_ID (command.sh에서 자동 사용)"
gazebo_partition="robocup_manipulation_${UID}_$$"
printf '%s\n' "$gazebo_partition" > "$gazebo_root/runtime/gazebo_partition"
echo "Gazebo 통신 공간: $gazebo_partition (partition:= 인자를 주면 해당 값을 사용)"
if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
exec ros2 launch robocup_gazebo_manipulation bringup.launch.py repository:="$repository_root" generated:="$gazebo_root/runtime/generated" partition:="$gazebo_partition" task_config:="$gazebo_root/config/task.json" "$@"
