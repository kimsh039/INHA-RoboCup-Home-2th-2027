# 기존 디텍션·퓨전 결과를 받는 매니퓰레이션 연결

사용자 담당 범위는 매니퓰레이션이다. 물체 이름 입력, Head 검출, 센서 퓨전, 자율주행·클로즈 어프로치는 기존 외부 모듈이 담당한다. 이 패키지는 기존 결과를 받아 coarse 위치 위 손목 관측 → 실제 정밀 결과 수신 → 규칙 기반 pick/place → 팔 복귀를 수행한다. 새 디텍터·퓨전·상위 작업 실행기를 만들지 않는다.

## Head/퓨전 수신부

저장소에서 확인한 실제 출력:

| 외부 출력 | 타입 | 사용 |
|---|---|---|
| /detection/head/target | robocup_detection_msgs/HeadTarget | class_id/class_name, target_id, confidence, 영상 취득시각 |
| /detection/target_map_point | geometry_msgs/PointStamped | 퓨전된 coarse 3D, frame, 취득시각 |

기존 nav_goal_node.py는 world_point.header.stamp에 HeadTarget의 RGB 취득시각을 넣는다. 수신부 head_input_adapter는 이 두 stamp가 **정확히 일치**하는 메시지만 결합한다. 수신 순서는 무관하며 최근64개씩 보관한다. 서로 다른 이미지의 클래스와3D를 최근 수신값이라는 이유로 결합하지 않는다.

출력은 /manipulation/target의 Target다. target_id를 task_id/instance_id로 전달하고, 퓨전3D의 좌표계·시각을 유지한다. CENTER로 재추정하거나 TF를 다시 계산하지 않는다. 현재6클래스 이름/ID·confidence·valid/measured·좌표의 유한값·시간 신선도를 검사한다. 최신 검출 결과의 클래스가 이미 current6이므로 raw7 ID 재매핑은 하지 않는다.

이 수신부는 원래 target_id당 한 번 전달한다. 외부 모듈은 다른 작업에 기존ID를 재사용하지 않아야 한다. 외부 작업 ID가 target_id와 다른 설계라면 그 명시적인 관계를 제공하는 adapter를 사용한다. 다른 외부 모듈이 이미 Target을 발행한다면 이 수신부를 중복 실행하지 않는다.

먼저 Jetson에서 기존 Head 메시지 패키지와 manipulation을 빌드·source한다. 예:

```bash
source /opt/ros/humble/setup.bash
cd "$ROBOT_REPO/SW/detection/head/head_detection_ws"
colcon build --packages-select robocup_detection_msgs --executor sequential
source install/setup.bash
source "$HOME/piper_ros/install/setup.bash"
cd "$ROBOT_REPO/SW/manipulation/manipulation_ws"
colcon build --symlink-install --executor sequential
source install/setup.bash

ros2 run robocup_real_manipulation head_input_adapter --ros-args \
  -p config:="$HOME/robot_config/robot.measured.yaml" \
  -p head_topic:=/detection/head/target \
  -p point_topic:=/detection/target_map_point \
  -p source_max_age_s:=2.0
```

이 명령은 수신·변환 노드만 시작한다. 디텍션·Nav2·목표 선택은 기존 담당 모듈에서 실행한다. 같은 ROS_DOMAIN_ID와 실제 시각 기준을 사용하고 use_sim_time=false로 설정한다. 기존 nav_goal_node.py에는 Gazebo 전용 검사가 있어, 해당 노드를 실물 대응으로 수정한 것은 아니다. 수신부는 저장소의 출력 형식을 받는다.

## 接近完了と自動引き継ぎ

외부 navigation의 ApproachResult는 Target과 같은 task_id를 사용한다. /manipulation/approach_result에 실제 SUCCESS/FAILED/CANCELED, 실제 정지 base_stopped와 시각을 반환한다. /manipulation/base_lock_request 요청에 응답하여 전체 베이스 이동 명령을 차단한 뒤 /manipulation/base_lease heartbeat를 제공해야 한다.

기존 dock_node의 /detection/dock/status는 String, /detection/dock/target은 PointStamped이며 task_id가 포함된 실제 정지·이동 잠금 응답은 아니다. DOCKED 문자열만으로 base_stopped=true나 locked=true를 만들지 않는다. 실제 Nav/베이스 관리 측의 작업 ID와 이동 잠금 응답을 연결해야 한다.

접근 후 사용자의 추가 팔 실행 명령 없이 이어가는 구성:

```bash
ros2 launch robocup_real_manipulation bringup.launch.py \
  hardware:=true auto_start_on_approach:=true \
  config:="$HOME/robot_config/robot.measured.yaml" \
  tuning_file:="$ROBOT_REPO/SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/tuning.yaml" \
  repo:="$ROBOT_REPO" start_driver:=false
```

같은 task_id의 Target과 SUCCESS/base_stopped인 ApproachResult가 모두 도착하면 한 번만 내부 Manipulate goal을 보낸다. 실물 설정 검사·베이스 잠금·실제 피드백 확인은 기존 coordinator와 공통이다. 실패 후 자동 재시도하지 않는다. 이 옵션을 사용할 때 상위 client의 동일 goal을 중복 송신하지 않는다. 기존 상위 client가 goal을 관리하면 auto_start_on_approach=false를 사용한다.

## 손목 결과와 좌표 변환

/manipulation/wrist/request의 ObservationRequest에 외부 wrist 모듈이 /manipulation/wrist/result의 WristObservation으로 응답한다. 작업·관측 ID, 실제 재검출 class_id, 전체 후보 batch, 취득시각·frame·위치 품질·CENTER/SURFACE를 전달한다. 카메라 좌표로 받으면 coordinator가 취득시각의 TF로 base에 변환하고 CAD TCP/설정으로 파지 TCP 목표를 계산한다. 디텍션에서 TCP 목표까지 만들 필요는 없다.

기존 wrist_sam_node.py의 /detection/wrist/target은 PointStamped이고 Z는 점군의 윗면을 사용한다. 이를 검증된 CENTER나 새 클래스 분류·confidence로 취급하지 않는다. 요청 ID·클래스 재확인·후보 batch·위치 품질이 포함된 실제 외부 결과를 WristObservation에 연결한다. 손목 디텍션 알고리즘을 여기서 재구현하지 않는다.

## 저장된 실물 캘리브레이션과 퓨전

기존 SW/robot/sensor_tf.py, imu_base.py, physical.launch.py의 실물 결과를 재사용한다.

| 실물 결과 | 파일 | 기존 적용 경로 |
|---|---|---|
| koide3 Head–Mid360 | HW/calibration/head_mid360/calib.json | livox_frame→Head root→RGB optical |
| Base–Mid360 | HW/calibration/physical/base_mid360/base_mid360.json | base_link→livox_frame、IMU 좌표 변환 |
| G2–Mid360 | HW/calibration/physical/base_2dlidar/vs_mid360/calibration.json | livox_frame→laser_frame、거리 배율 |
| Tracer odometry/IMU | HW/calibration/physical/tracer_odom/odom_calibration.json、SW/robot/config/ekf.yaml | odom→base_link、기존 EKF |
| Head/Wrist 내부 파라미터 | HW/calibration/camera_intrinsics | calibrated_streams의 CameraInfo |
| Wrist hand–eye | HW/calibration/physical/wrist_handeye/calibration.json | piper_link6→Wrist RGB optical |

SW/robot/physical_application.json에 원본 경로와 hash가 있다. 저장된 결과를 미제공 값으로 취급하지 않는다. 현재 장착과 같은 조건인지, 실제 TF에 적용되는지는 연결 후 확인한다.

현재 센서 실행기와 manipulation의 robot_state_publisher에서 base/arm/sensor TF가 중복될 수 있다. 실제 센서 TF를 재사용하도록 생성 모델/launch를 통합하고 실제 wrist_color_optical_frame과 모델 wrist_camera_optical_frame의 원점 관계를 정리해야 한다. 수신부 추가만으로 TF 통합이나 Jetson 실물 검증이 완료된 것은 아니다.

노트북 단위 검사 41개에는 다른 이미지의 잘못된 결합, legacy 클래스 혼입, 오래된/비유한3D, 다른 작업 ID, 접근 실패, 미정지, 자동 인계 중복 방지 검사가 포함된다. Jetson의 실제 외부 topic·TF·동작 검증은 별도로 수행한다.
