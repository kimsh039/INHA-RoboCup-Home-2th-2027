#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
ROBOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOT_DIR/config/collision_fastdds.xml"
fi
COLLISION_RUNTIME="${ROBOCUP_COLLISION_RUNTIME:-$HOME/robot_setup/collision-runtime}"
COLLISION_BINARY="$COLLISION_RUNTIME/current/robocup_collision_processing/lib/robocup_collision_processing/collision_guard_node"
if [[ ! -x "$COLLISION_BINARY" || ! -s "$COLLISION_BINARY" ]]; then
  echo "C++ 주행 보호 실행파일이 없습니다: bash $ROBOT_DIR/build-collision.sh" >&2
  exit 1
fi
COLLISION_MODEL="$(mktemp /tmp/robocup-collision-guard-geometry-XXXXXX.yaml)"
trap 'rm -f -- "$COLLISION_MODEL"' EXIT
/usr/bin/python3 "$ROBOT_DIR/export-collision-geometry.py" --output "$COLLISION_MODEL"
"$COLLISION_BINARY" "$@" --ros-args -p "geometry_file:=$COLLISION_MODEL"
