#!/usr/bin/env bash
set -eo pipefail
ROBOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
COLLISION_RUNTIME="${ROBOCUP_COLLISION_RUNTIME:-$HOME/robot_setup/collision-runtime}"
mkdir -p "$COLLISION_RUNTIME"
# A shared CMake cache contains one install prefix. Serialize builds from
# different terminals before configuring it or replacing the current release.
exec 9>"$COLLISION_RUNTIME/build.lock"
flock 9
source /opt/ros/humble/setup.bash
COLLISION_RELEASE="$(date +%Y%m%d-%H%M%S)-$$"
COLLISION_INSTALL="$COLLISION_RUNTIME/releases/$COLLISION_RELEASE"
colcon --log-base "$COLLISION_RUNTIME/log" build \
  --base-paths "$ROBOT_DIR/collision_processing" \
  --build-base "$COLLISION_RUNTIME/build" --install-base "$COLLISION_INSTALL" \
  --packages-select robocup_collision_processing --parallel-workers 1 \
  --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    "-DCMAKE_INSTALL_PREFIX=$COLLISION_INSTALL/robocup_collision_processing"
for COLLISION_PROGRAM in collision_cloud_node collision_guard_node; do
  if [[ ! -x "$COLLISION_INSTALL/robocup_collision_processing/lib/robocup_collision_processing/$COLLISION_PROGRAM" ]]; then
    echo "C++ 실행파일 $COLLISION_PROGRAM 이 설치되지 않아 현재 버전을 유지합니다." >&2
    exit 1
  fi
done
ln -s "$COLLISION_INSTALL" "$COLLISION_RUNTIME/.current-$COLLISION_RELEASE"
mv -Tf "$COLLISION_RUNTIME/.current-$COLLISION_RELEASE" "$COLLISION_RUNTIME/current"
