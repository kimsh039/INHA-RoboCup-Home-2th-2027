#!/usr/bin/env bash
# ROS 2 Humble + Gazebo Harmonic 개발 환경 설치 (Ubuntu 22.04)
# 일반 사용자로 실행합니다. 필요한 단계에서 sudo 암호를 묻습니다.
#   bash SW/setup/install_ros2_gz.sh            # 개발 PC: ROS 2 + Gazebo Harmonic
#   bash SW/setup/install_ros2_gz.sh --no-sim   # Jetson 등 실기: ROS 2만
set -euo pipefail

WITH_SIM=1
[[ "${1:-}" == "--no-sim" ]] && WITH_SIM=0

. /etc/os-release
if [[ "$VERSION_CODENAME" != "jammy" ]]; then
  echo "Ubuntu 22.04 (jammy) 전용입니다: 현재 $PRETTY_NAME" >&2
  exit 1
fi
ARCH=$(dpkg --print-architecture)
# 미러 일시 연결 실패와 불안정한 IPv6 경로 대비
APT="sudo apt-get -o Acquire::Retries=5 -o Acquire::ForceIPv4=true"

echo "== Ubuntu 저장소 HTTPS 전환 =="
# 일부 네트워크(P2P 필터)는 URL에 'tracker'가 들어간 HTTP 요청을 끊어 tracker-miner-fs 등에서 apt가 멈춥니다.
if grep -qE '^deb http://([a-z]+\.)?(archive|security)\.ubuntu\.com' /etc/apt/sources.list; then
  sudo cp -n /etc/apt/sources.list /etc/apt/sources.list.bak
  sudo sed -i -E 's#^deb http://(([a-z]+\.)?(archive|security)\.ubuntu\.com)#deb https://\1#' /etc/apt/sources.list
fi

echo "== 기본 도구 =="
$APT update
$APT install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe

echo "== ROS 2 apt 저장소 =="
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ver=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  deb=$(mktemp --suffix=.deb)
  curl -fL -o "$deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i "$deb"
  rm -f "$deb"
fi

if (( WITH_SIM )); then
  echo "== Gazebo (OSRF) apt 저장소 =="
  key=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  [[ -f $key ]] || sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o "$key"
  echo "deb [arch=$ARCH signed-by=$key] http://packages.osrfoundation.org/gazebo/ubuntu-stable $VERSION_CODENAME main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
fi

$APT update
$APT upgrade -y

echo "== ROS 2 Humble =="
pkgs=(
  ros-humble-desktop ros-dev-tools python3-rosdep
  ros-humble-xacro
  ros-humble-robot-state-publisher
  ros-humble-joint-state-publisher-gui
  ros-humble-ros2-control ros-humble-ros2-controllers
  ros-humble-rmw-cyclonedds-cpp
)
if (( WITH_SIM )); then
  # Humble 기본 짝은 Fortress이므로 Harmonic용 ros_gz는 OSRF 저장소의 gzharmonic 패키지를 씁니다.
  # ros-humble-ros-gz(Fortress)와 함께 설치하면 충돌합니다.
  pkgs+=(gz-harmonic python3-gz-transport13 python3-gz-msgs10 ros-humble-ros-gzharmonic)
fi
$APT install -y "${pkgs[@]}"

echo "== rosdep =="
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update

echo "== ~/.bashrc =="
add_line() { grep -qxF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc; }
add_line 'source /opt/ros/humble/setup.bash'
add_line 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash'
(( WITH_SIM )) && add_line 'export GZ_VERSION=harmonic'

echo
echo "완료. 새 터미널을 열거나 'source ~/.bashrc' 후 확인하세요:"
echo "  ros2 doctor --report | head; gz sim --versions"
