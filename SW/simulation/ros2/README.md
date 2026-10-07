# ROS 2 시뮬레이션

저장소 루트에서 ROS 2 Humble 환경을 활성화하고 실행합니다.
일반 `sim.launch.py`는 CAD 기준 원본 `robot_description/robocup.urdf`를 읽으며 손목 D435는 항상 포함됩니다. **현재 최종 보정 모델은 `robocup.calibrated.urdf`**이며, 2026-10-06 Base–2D/Mid360 보정을 반영·업로드했습니다. 적용은 [보정 URDF 적용하기](../robot_description/README.md#보정-urdf-적용하기), 날짜·커밋은 [모델 변경 이력](../robot_description/README.md#모델-변경보정업로드-이력)을 참고하세요.
`gazebo/start_sim.sh`와 동시에 실행하지 마세요.

```bash
source /opt/ros/humble/setup.bash
ros2 launch SW/simulation/ros2/sim.launch.py world:=room
# GUI/RViz 없이 서버만 실행
ros2 launch SW/simulation/ros2/sim.launch.py world:=room gui:=false rviz:=false
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

현재 브리지는 /clock, /cmd_vel, /odom, /tf, /joint_states, /scan, Mid-360S 점군 /mid360/points와 헤드·손목 color/depth 영상·CameraInfo, 손목 depth points입니다.
정확한 매핑은 [ros_bridge.yaml](ros_bridge.yaml)을 참고하세요.
헤드 ROS 영상은 `/head_camera/color/image_raw`, `/head_camera/depth/image_raw`로 받습니다.
[검출→SLAM/Nav2 이동 테스트](../../detection/head/SIM_NAV_TEST.md)에 테스트 표적과 실행 순서를 정리했습니다.
런치는 Nav2용 Mid-360S 자기 점 필터(`/mid360/points_filtered`)도 실행합니다.
SLAM/Nav2 노드는 이 런치가 실행하지 않습니다. 실행 방법은 [SLAM.md](SLAM.md)와 [NAV2.md](NAV2.md)를 참고하세요.

## 센서 역할

| 센서 | 연결과 역할 |
|---|---|
| 2D LiDAR | `/scan` → SLAM Toolbox의 2D 지도·위치 추정 및 Nav2 장애물 입력 |
| Mid-360 | `/mid360/points` → 자기 점 필터 → `/mid360/points_filtered` → Nav2 local/global costmap의 3D 장애물 입력 |
| Head D435 | 목표 검출·추적. `/head_camera/color/image_raw`, `/head_camera/depth/image_raw`와 각 CameraInfo를 ROS로 연결 |
| Wrist D435 | 접근 후 물체 관측·정합 depth·파지 입력. 현재 Gazebo 영상은 핀홀 근사 |

실기 카메라 선택은 **Head/Wrist 모두 D435**입니다. CAD의 D435f 자산 이름은 형상 출처를 나타내며 실물 카메라 성능과 구분합니다. Mid-360의 작업면·경계 추출과 접근 자세 생성, MoveIt 주변 충돌 장면은 [후속 설계](../../detection/SENSOR_ROLES.md)이며 현재 launch가 실행하는 기능이 아닙니다.

## 종료와 확인 범위

Gazebo 서버와 창은 별도 프로세스입니다. 창을 닫아도 시뮬레이션은 계속되며 런치 터미널의 Ctrl+C로 종료합니다.

최종 URDF 정리 후 Gazebo 센서는 기존 개발 PC에서 재검증했습니다. 당시 개발 PC에는 ROS 2가 없어 ROS 런치는 구문 검사까지만 했습니다. 별도 Jetson에는 ROS Humble을 준비했지만 Gazebo·bridge를 설치하거나 이 시뮬레이션 launch를 실행하지 않았습니다.

## 보정 TF 실행 — 측정·적용·정확도 평가

**2D/3D LiDAR 보정을 쓰려면 `robocup.calibrated.urdf`를 읽는 `calibration_runtime.launch.py`를 실행합니다.** [보정 URDF 적용하기](../robot_description/README.md#보정-urdf-적용하기)에 Ubuntu/Mac 전체 명령, RViz 표시, TF 수치 읽기, 기존 TF와 중복하지 않는 방법을 적었습니다. 일반 `sim.launch.py`와 Gazebo world 생성기는 기본으로 `robocup.urdf`를 읽으므로 이 파일 선택을 자동으로 대신하지 않습니다.

전체 진행 상태와 계산·평가 결과는 [Calibration 개요](../calibration/README.md)를 참고하세요. Mid360 측정·로봇 이동·독립 정확도 평가는 [Ubuntu 상세 실행 가이드](../calibration/BASE_MID360.md)에서 진행합니다.
