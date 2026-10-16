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
if [[ "${ROBOCUP_COLLISION_CLEAN_BUILD:-0}" == 1 &&
      -f "$COLLISION_RUNTIME/build/robocup_collision_processing/CMakeCache.txt" ]]; then
  cmake --build "$COLLISION_RUNTIME/build/robocup_collision_processing" --target clean
fi
# An interrupted linker can leave a zero-byte executable newer than its
# objects. CMake otherwise treats that cached target as already built.
for COLLISION_CACHED in collision_cloud_node collision_guard_node libcollision_scene_layer.so \
    CMakeFiles/collision_cloud_node.dir/src/collision_cloud.cpp.o \
    CMakeFiles/collision_guard_node.dir/src/collision_guard.cpp.o \
    CMakeFiles/collision_scene_layer.dir/src/scene_layer.cpp.o; do
  COLLISION_CACHED_PATH="$COLLISION_RUNTIME/build/robocup_collision_processing/$COLLISION_CACHED"
  if [[ -e "$COLLISION_CACHED_PATH" && ! -s "$COLLISION_CACHED_PATH" ]]; then
    rm -f -- "$COLLISION_CACHED_PATH"
  fi
done
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
  if [[ ! -x "$COLLISION_INSTALL/robocup_collision_processing/lib/robocup_collision_processing/$COLLISION_PROGRAM" ||
        ! -s "$COLLISION_INSTALL/robocup_collision_processing/lib/robocup_collision_processing/$COLLISION_PROGRAM" ]]; then
    echo "C++ 실행파일 $COLLISION_PROGRAM 이 설치되지 않아 현재 버전을 유지합니다." >&2
    exit 1
  fi
done
if [[ ! -s "$COLLISION_INSTALL/robocup_collision_processing/lib/libcollision_scene_layer.so" ]]; then
  echo "Nav2 공유 장애물 플러그인이 설치되지 않아 현재 버전을 유지합니다." >&2
  exit 1
fi
ln -s "$COLLISION_INSTALL" "$COLLISION_RUNTIME/.current-$COLLISION_RELEASE"
mv -Tf "$COLLISION_RUNTIME/.current-$COLLISION_RELEASE" "$COLLISION_RUNTIME/current"
