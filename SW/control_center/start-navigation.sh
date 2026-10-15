#!/usr/bin/env bash
# Foreground Nav2; the physical core owns 3D perception and the velocity gate.
# Needs SLAM (start-mapping.sh) for map -> odom. Sends velocity only for goals it receives.
set -e
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$CONTROL_CENTER_DIR/../.." && pwd)"
source /opt/ros/humble/setup.bash
NAV2_PARAMS="${ROBOCUP_NAV2_PARAMS:-$CONTROL_CENTER_DIR/nav2-web.yaml}"
USE_SIM_TIME="${ROBOCUP_USE_SIM_TIME:-false}"
SAFETY_PARAMS="$REPO_DIR/SW/robot/config/collision_safety.yaml"
RUNTIME_PARAMS="$(mktemp --suffix=.yaml /tmp/robocup-navigation-XXXXXX)"
trap 'rm -f -- "$RUNTIME_PARAMS"' EXIT
/usr/bin/python3 "$CONTROL_CENTER_DIR/prepare-navigation-profile.py" \
  --input "$NAV2_PARAMS" --output "$RUNTIME_PARAMS" --safety "$SAFETY_PARAMS"
ros2 launch nav2_bringup navigation_launch.py \
  "params_file:=$RUNTIME_PARAMS" "use_sim_time:=$USE_SIM_TIME" "$@"
