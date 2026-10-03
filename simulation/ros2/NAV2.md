# Nav2 Quick Start (시뮬레이터)

SLAM으로 지도를 만들면서 RViz에서 목표를 찍어 자율주행합니다. 명령은 저장소 루트에서 실행합니다.

## 설치 (최초 1회)

```bash
sudo apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
```

## 실행

터미널 4개를 열고, **각 터미널에서 먼저** 실행합니다.

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
```

```bash
# 터미널 1: 시뮬레이션
ros2 launch simulation/ros2/sim.launch.py world:=room rviz:=false

# 터미널 2: SLAM
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$PWD/simulation/ros2/slam_params.yaml use_sim_time:=true

# 터미널 3: Nav2 ("Managed nodes are active"가 나오면 준비 완료)
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$PWD/simulation/ros2/nav2_params.yaml use_sim_time:=true

# 터미널 4: RViz
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```

## 목표 보내기

RViz 상단 **Nav2 Goal** → 지도 위 목표 지점을 클릭한 채 끌어 방향을 정하고 놓습니다.

명령으로 보낼 때 (map 좌표, 시작 위치가 원점):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.8}, orientation: {w: 1.0}}}}"
```

## 종료

각 터미널에서 Ctrl+C. 지도를 남기려면 종료 전에 [SLAM](SLAM.md#지도-저장)의 저장 명령을 실행합니다.

## 안 될 때

- Nav2 Goal에 반응 없음: 터미널 3에 `Managed nodes are active`가 나왔는지 확인
- RViz의 `Localization: inactive`는 정상입니다 (SLAM이 위치 추정)
