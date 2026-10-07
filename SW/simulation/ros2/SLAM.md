# SLAM Quick Start (시뮬레이터)

Gazebo 방 world에서 키보드로 로봇을 움직이며 지도를 만듭니다. 명령은 저장소 루트에서 실행합니다.

## 설치 (최초 1회)

```bash
sudo apt-get install -y ros-humble-slam-toolbox ros-humble-nav2-map-server \
  ros-humble-nav2-bringup ros-humble-teleop-twist-keyboard
```

## 실행

터미널 4개를 열고, **각 터미널에서 먼저** 실행합니다.

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
```

```bash
# 터미널 1: 시뮬레이션
ros2 launch SW/simulation/ros2/sim.launch.py world:=room rviz:=false

# 터미널 2: SLAM
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$PWD/simulation/ros2/slam_params.yaml use_sim_time:=true

# 터미널 3: RViz
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true

# 터미널 4: 키보드 주행 (i 전진, , 후진, j/l 회전, k 정지)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.5
```

터미널 4를 선택한 상태에서 영문 입력으로 키를 누르며 방 안을 돌아다니면 RViz에 지도가 채워집니다.

지도는 로봇이 **0.25 m 이상 이동할 때마다** 새 스캔으로 갱신됩니다. Humble의 slam_toolbox는 이동 거리만 보므로 **제자리 회전만으로는 지도가 바뀌지 않습니다** (회전 기준은 Jazzy부터 지원). 처음에는 앞뒤로 0.5 m 정도 움직여 지도를 채운 뒤 Nav2 목표를 보내세요.

## 지도 저장

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/room --ros-args -p use_sim_time:=true
```

## 종료

각 터미널에서 Ctrl+C. Gazebo 창만 닫으면 시뮬레이션이 계속 실행됩니다.

## 안 될 때

- 키가 안 먹음: 한글 입력 상태인지, 모든 터미널에 `ROS_LOCALHOST_ONLY=1`을 했는지 확인
- 지도·라이다가 안 나옴: Gazebo 왼쪽 아래 ▶가 재생 상태인지 확인
