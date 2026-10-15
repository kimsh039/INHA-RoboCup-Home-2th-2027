#!/usr/bin/env bash
# Foreground Nav2; the physical core owns 3D perception and the velocity gate.
# Needs SLAM (start-mapping.sh) for map -> odom. Sends velocity only for goals it receives.
set -e
CONTROL_CENTER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$CONTROL_CENTER_DIR/../.." && pwd)"
source /opt/ros/humble/setup.bash
COLLISION_RUNTIME="${ROBOCUP_COLLISION_RUNTIME:-$HOME/robot_setup/collision-runtime}"
COLLISION_PACKAGE="$COLLISION_RUNTIME/current/robocup_collision_processing"
if [[ ! -r "$COLLISION_RUNTIME/current/setup.bash" ||
      ! -s "$COLLISION_PACKAGE/lib/libcollision_scene_layer.so" ||
      ! -s "$COLLISION_PACKAGE/share/ament_index/resource_index/nav2_costmap_2d__pluginlib__plugin/robocup_collision_processing" ]]; then
  echo "Nav2 공유 장애물 플러그인이 필요합니다: bash $REPO_DIR/SW/robot/build-collision.sh" >&2
  exit 1
fi
source "$COLLISION_RUNTIME/current/setup.bash"
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO_DIR/SW/robot/config/collision_fastdds.xml"
fi
NAV2_PARAMS="${ROBOCUP_NAV2_PARAMS:-$CONTROL_CENTER_DIR/nav2-web.yaml}"
USE_SIM_TIME="${ROBOCUP_USE_SIM_TIME:-false}"
SAFETY_PARAMS="$REPO_DIR/SW/robot/config/collision_safety.yaml"
RUNTIME_PARAMS="$(mktemp --suffix=.yaml /tmp/robocup-navigation-XXXXXX)"
trap 'rm -f -- "$RUNTIME_PARAMS"' EXIT
/usr/bin/python3 "$CONTROL_CENTER_DIR/prepare-navigation-profile.py" \
  --input "$NAV2_PARAMS" --output "$RUNTIME_PARAMS" --safety "$SAFETY_PARAMS"
ros2 launch nav2_bringup navigation_launch.py \
  "params_file:=$RUNTIME_PARAMS" "use_sim_time:=$USE_SIM_TIME" "$@"
