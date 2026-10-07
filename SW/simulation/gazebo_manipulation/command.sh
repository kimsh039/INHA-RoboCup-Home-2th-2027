#!/usr/bin/env bash
set -eo pipefail
gazebo_root="$(cd "$(dirname "$0")" && pwd)"
repository_root="${ROBOCUP_REPOSITORY:-$(dirname "$(dirname "$gazebo_root")")}"
source /opt/ros/humble/setup.bash
source "${ROBOCUP_GAZEBO_CACHE:-/tmp/robocup-gazebo-$UID}/install/setup.bash"
if [ -f "$gazebo_root/runtime/ros_domain" ]; then
    gazebo_domain="$(cat "$gazebo_root/runtime/ros_domain")"
else
    echo '먼저 터미널 1에서 bash run.sh를 실행하세요.' >&2
    exit 1
fi
export ROS_DOMAIN_ID="$gazebo_domain"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="$gazebo_root/runtime/ros_logs"
# Save the command terminal too: Python tracebacks are not in node log files.
mkdir -p "$gazebo_root/runtime/command_logs"
command_log="$gazebo_root/runtime/command_logs/command_$(date +%Y%m%d_%H%M%S)_$$.log"
exec > >(tee -a "$command_log") 2>&1
echo "명령 로그: $command_log"
mode="${1:-tutorial}"
if [ $# -gt 0 ]; then shift; fi
if [ "$mode" = tutorial ] || [ "$mode" = pick ]; then
    exec ros2 run robocup_gazebo_manipulation tutorial --slip-experiment --repository "$repository_root" --generated "$gazebo_root/runtime/generated" --output "$gazebo_root/reports/gazebo_tutorial.json" "$@" --ros-args -p use_sim_time:=true
elif [ "$mode" = task ]; then
    exec ros2 action send_goal /manipulation/pick_place robocup_manipulation_msgs/action/PickPlace '{task_id: gazebo_cube, object_class: cube, destination: {header: {frame_id: world}, pose: {position: {x: 0.5, y: -0.2, z: 0.7401}, orientation: {w: 1.0}}}, timeout_s: 1800.0, plan_only: false}' --feedback "$@"
elif [ "$mode" = probe ]; then
    exec ros2 run robocup_gazebo_manipulation probe --seconds 15 --output "$gazebo_root/reports/sensors.json" "$@" --ros-args -p use_sim_time:=true
else
    exec ros2 run robocup_gazebo_manipulation demo --repository "$repository_root" --mode "$mode" --output "$gazebo_root/reports/demo_$mode.json" "$@" --ros-args -p use_sim_time:=true
fi
