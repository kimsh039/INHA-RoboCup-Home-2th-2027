#!/usr/bin/env bash
set -eo pipefail
gazebo_root="$(cd "$(dirname "$0")" && pwd)"
repository_root="${ROBOCUP_REPOSITORY:-$(dirname "$(dirname "$gazebo_root")")}"
source /opt/ros/humble/setup.bash
cd "$gazebo_root/gazebo_ws"
gazebo_cache="${ROBOCUP_GAZEBO_CACHE:-/tmp/robocup-gazebo-$UID}"
colcon build --build-base "$gazebo_cache/build" --install-base "$gazebo_cache/install" --base-paths src --packages-up-to robocup_gazebo_manipulation
