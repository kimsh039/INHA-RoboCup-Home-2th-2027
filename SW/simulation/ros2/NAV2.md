# Nav2 Quick Start (시뮬레이터)

SLAM으로 지도를 만들면서 RViz에서 목표를 찍어 자율주행합니다. 명령은 저장소 루트에서 실행합니다.

## 장애물 입력과 센서 역할

**2D LiDAR는 기본 지도·위치 추정, Mid-360은 주변 3D 장애물 관측**을 담당합니다. 이 Nav2 안내는 기존 costmap 설정을 사용하는 단계입니다.

| 경로 | 역할 |
|---|---|
| `/scan` → SLAM Toolbox | 2D 지도·위치 추정 |
| `/scan` → local/global costmap | 스캔 평면의 장애물 |
| `/mid360/points` → `cloud_self_filter.py` → `/mid360/points_filtered` | 로봇 footprint 내부 점·고립 점 제거 |
| `/mid360/points_filtered` → local `VoxelLayer` / global `ObstacleLayer` | 설정한 높이 범위의 상판·돌출부 등 장애물 |

자기 점 필터는 footprint 밖의 팔을 제거하지 않습니다. 높이·거리 설정은 현재 Gazebo 바닥 기준이며 실기로 옮길 때 TF·바닥·차체 형상과 대조합니다. 현재 파라미터의 local voxel z 범위는 약 `-0.15 .. 1.35 m`, Mid-360 marking 높이는 `-0.03 .. 1.25 m`입니다.

Mid-360의 **작업면 추출 → 접근 자세 생성 → MoveIt 주변 충돌 장면**은 [센서 역할 문서](../../detection/SENSOR_ROLES.md)의 후속 단계입니다. 아래 Nav2 실행만으로 자동 작업면 접근이나 파지 노드가 시작되지는 않습니다.

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
ros2 launch SW/simulation/ros2/sim.launch.py world:=room rviz:=false

# 터미널 2: SLAM
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$PWD/SW/simulation/ros2/slam_params.yaml use_sim_time:=true

# 터미널 3: Nav2 ("Managed nodes are active"가 나오면 준비 완료)
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$PWD/SW/simulation/ros2/nav2_params.yaml use_sim_time:=true

# 터미널 4: RViz
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```

## 목표 보내기

RViz 상단 **Nav2 Goal** → 지도 위 목표 지점을 클릭한 채 끌어 방향을 정하고 놓습니다.

명령으로 보낼 때 (map 좌표, 시작 위치가 원점):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.8, y: 0.4}, orientation: {w: 1.0}}}}"
```

## 종료

각 터미널에서 Ctrl+C. 지도를 남기려면 종료 전에 [SLAM](SLAM.md#지도-저장)의 저장 명령을 실행합니다.

## 안 될 때

- Nav2 Goal에 반응 없음: 터미널 3에 `Managed nodes are active`가 나왔는지 확인
- RViz의 `Localization: inactive`는 정상입니다 (SLAM이 위치 추정)
