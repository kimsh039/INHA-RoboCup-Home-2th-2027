# ROS 2 시뮬레이션

저장소 루트에서 ROS 2 Humble 환경을 활성화하고 실행합니다.
최종 `robot_description/robocup.urdf`를 읽으며 손목 카메라는 항상 포함됩니다.
`gazebo/start_sim.sh`와 동시에 실행하지 마세요.

```bash
source /opt/ros/humble/setup.bash
ros2 launch simulation/ros2/sim.launch.py world:=room
# GUI/RViz 없이 서버만 실행
ros2 launch simulation/ros2/sim.launch.py world:=room gui:=false rviz:=false
```

기본 world는 `empty`, gui/rviz는 `true`입니다. partition은 `robocup_motion`, 기본 GZ_IP는 127.0.0.1입니다.
Gazebo에 NVIDIA가 사용 가능하면 PRIME offload를 자동 지정합니다.
TF는 DiffDrive의 odom→base_link와 robot_state_publisher의 base_link 이하 트리로 연결됩니다.
RViz Fixed Frame은 `odom`, 모든 노드는 simulation time을 사용합니다.

다른 터미널에서 ROS 환경을 활성화한 뒤:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /joint_states
ros2 topic hz /wrist_camera/color/image_raw
ros2 topic hz /wrist_camera/depth/image_raw
```

현재 브리지는 /clock, /cmd_vel, /odom, /tf, /joint_states, /scan, Mid-360S 점군 /mid360/points와 손목 color/depth 영상·CameraInfo·depth points입니다.
정확한 매핑은 [ros_bridge.yaml](ros_bridge.yaml)을 참고하세요.
헤드 영상은 Gazebo 토픽/팝업을 사용하며 현재 ROS 브리지에는 포함하지 않습니다.
런치는 Nav2용 Mid-360S 자기 점 필터(`/mid360/points_filtered`)도 실행합니다.
SLAM/Nav2 노드는 이 런치가 실행하지 않습니다. 실행 방법은 [SLAM.md](SLAM.md)와 [NAV2.md](NAV2.md)를 참고하세요.

## 센서 역할

| 센서 | 연결과 역할 |
|---|---|
| 2D LiDAR | `/scan` → SLAM Toolbox의 2D 지도·위치 추정 및 Nav2 장애물 입력 |
| Mid-360 | `/mid360/points` → 자기 점 필터 → `/mid360/points_filtered` → Nav2 local/global costmap의 3D 장애물 입력 |
| Head D435 | 목표 검출·추적. 헤드 영상 ROS 브리지는 아직 포함하지 않음 |
| Wrist D435 | 접근 후 물체 관측·정합 depth·파지 입력. 현재 Gazebo 영상은 핀홀 근사 |

실기 카메라 선택은 **Head/Wrist 모두 D435**입니다. CAD의 D435f 자산 이름은 형상 출처를 나타내며 실물 카메라 성능과 구분합니다. Mid-360의 작업면·경계 추출과 접근 자세 생성, MoveIt 주변 충돌 장면은 [후속 설계](../../detection/SENSOR_ROLES.md)이며 현재 launch가 실행하는 기능이 아닙니다.

## 종료와 확인 범위

Gazebo 서버와 창은 별도 프로세스입니다. 창을 닫아도 시뮬레이션은 계속되며 런치 터미널의 Ctrl+C로 종료합니다.

최종 URDF 정리 후 Gazebo 센서는 기존 개발 PC에서 재검증했습니다. 당시 개발 PC에는 ROS 2가 없어 ROS 런치는 구문 검사까지만 했습니다. 별도 Jetson에는 ROS Humble을 준비했지만 Gazebo·bridge를 설치하거나 이 시뮬레이션 launch를 실행하지 않았습니다.
