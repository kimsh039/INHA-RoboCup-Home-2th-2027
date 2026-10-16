# Jetson에서 처음 연결할 때 실제로 할 일

현재 단계는 선 연결 → 실제 피드백 확인 → 측정값 입력 → 외부 계약/TF 통합 → 저속 작업이다. 아래 첫 연결 단계는 자동 픽앤플레이스를 시작하지 않는다. 모든 명령은 Jetson 터미널에서 실행한다. 노트북은 SSH 화면으로 써도 되지만 ROS 노드와 빌드는 Jetson에서 실행한다.

## 0. 준비와 선 연결

1. 예전 노트북 팔 캘리브레이션에서 사용한 Piper 전원·CAN 케이블·USB-CAN 어댑터를 사용한다. 전원을 끈 상태에서 Piper CAN ↔ USB-CAN ↔ Jetson USB를 연결하고 기존에 검증한 종단저항/배선을 유지한다. Jetson USB는 팔 전원을 공급하지 않는다. 연결 후 Piper 전원을 켠다.
2. 손목 D435IF를 Jetson USB3에 연결한다. 손목 카메라와 그리퍼 사이의 브래킷/자세가 기존 hand–eye 측정 때와 같아야 저장된 결과를 사용할 수 있다. 머리 D435와 혼동하지 않는다.
3. 팔·그리퍼의 동작 공간을 비우고 기존 실제 정지 방법을 준비한다. 처음 확인은 모터 자동 enable 없이 수행한다.
4. Piper용 CAN과 Tracer용 CAN을 구분한다. Jetson 기록의 can0/can1은 onboard CAN일 수 있으므로 이름만 보고 Piper라고 판단하지 않는다. 기존 Piper/CAN 독립 실행기와 robot_physical 세션의 팔 노드는 종료한 뒤 아래 단일 드라이버를 사용한다.

저장소가 Jetson의 ~/robocup에 있을 때 공통 준비:

```bash
export ROBOT_REPO="$HOME/robocup"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source "$HOME/piper_ros/install/setup.bash"
```

42는 선택한 도메인 예시다. 이미 운영하는 도메인이 있으면 그것을 쓴다. 위 환경은 **각 터미널에서** 적용한다. 모든 노드를 Jetson에서 실행하면 LOCALHOST_ONLY=1을 사용할 수 있다. ROS 노드가 다른 PC에서도 실행되면 양쪽 모두 0으로 바꾸고 같은 도메인/네트워크 및 시각 동기화를 확인한다. 실물 노드는 use_sim_time=false를 사용한다. SSH로 Jetson 명령을 실행하는 경우 노트북 ROS 시계는 개입하지 않는다.

## 1. Piper USB-CAN을 식별하고 올리기

USB-CAN을 꽂기 전과 후 다음 출력을 비교한다:

```bash
ip -br link show type can
lsusb
bash "$HOME/piper_ros/find_all_can_port.sh"
```

새로 나타난 CAN과 USB bus-info를 확인한다. Tracer CAN은 건드리지 않는다. CAN 장치가 나타나지 않으면 USB/드라이버 확인이 먼저다.

Piper CAN이 확인되면 실제 이름을 지정한다. 다음 이름은 예시이며 확인한 값으로 바꾼다:

```bash
export PIPER_CAN=can_piper
```

기존에 사용한 Piper bitrate가 1,000,000이고, find_all_can_port.sh가 출력한 Piper 어댑터 bus-info가 확인됐을 때 제조사 스크립트로 이름/bitrate를 설정한다:

```bash
bash "$HOME/piper_ros/can_activate.sh" "$PIPER_CAN" 1000000 실제_USB_bus_info
ip -details link show "$PIPER_CAN"
```

실제_USB_bus_info는 스크립트 출력의 값으로 치환한다. 다중 CAN 환경에서 인수를 생략하지 않는다. 이미 올바르게 활성화된 CAN이면 재설정하지 않는다. UP, bitrate와 CAN 오류 상태를 확인한다. 이 단계는 팔 이동 명령이 아니다.

## 2. 펌웨어 버전 읽기

팔 ROS 드라이버가 꺼진 상태에서, 기존 SDK 환경을 사용한다. Jetson 설치 기록의 SDK 경로가 현재 설치와 일치하면:

```bash
export PYTHONPATH="$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
python3 - <<'PY'
import os, time
from piper_sdk import C_PiperInterface_V2
arm = C_PiperInterface_V2(can_name=os.environ['PIPER_CAN'])
arm.ConnectPort()
try:
    arm.SearchPiperFirmwareVersion()
    time.sleep(1.0)
    version = arm.GetPiperFirmwareVersion()
    if not isinstance(version, str):
        raise RuntimeError('펌웨어 응답 없음: CAN/전원/SDK를 먼저 확인')
    print('firmware:', version)
finally:
    arm.DisconnectPort()
PY
```

이 코드는 SDK의 정보 조회 요청을 보내지만 enable/JointCtrl/GripperCtrl은 호출하지 않는다. 이번 작업에서는 노트북 SDK 소스의 API를 확인했으며 Jetson에서 실행한 결과는 아니다. import가 실패하면 기존 설치 경로를 먼저 확인한다. 버전 출력 문자열을 firmware에 기록하며 펌웨어 업데이트는 하지 않는다.

## 3. 팔 드라이버만 시작하고 피드백 확인

매니퓰레이션에 필요한 전용 그리퍼 timestamp 패치를 먼저 [DEPLOYMENT.md의 2절](DEPLOYMENT.md#2-piper-개별-피드백-timestamp-패치)에 따라 적용·재빌드한다. 이미 적용한 경우 반복하지 않는다.

터미널 A에서 계속 실행:

```bash
ros2 run piper piper_single_ctrl --ros-args \
  -p can_port:="$PIPER_CAN" -p auto_enable:=false \
  -p gripper_exist:=true -p gripper_val_mutiple:=1
```

터미널 B에서 공통 ROS 환경을 적용한 뒤:

```bash
ros2 topic echo /joint_states_feedback --once
ros2 topic echo /gripper_states_feedback --once
ros2 topic echo /arm_status --once
ros2 service type /enable_srv
```

관절 name/position, 그리퍼 position/effort, timestamp와 장비 상태가 보여야 한다. 원시 JointState의 position은 각 name에 대응한다. 배열의 마지막 값을 무조건 그리퍼라고 가정하지 않는다. gripper topic이 없으면 패치/실행 중인 드라이버를 확인한다. 원래 SW/robot 실행기의 /piper/joint_states_feedback은 읽기 노드 출력이며 이 제어 드라이버를 대신하지 않는다.

이 단계에서는 /enable_srv를 호출하지 않는다. 모터 enable과 교시/수동 제어는 이전에 검증했던 제조사 조작 절차로 진행한다. 켜진 모터를 손으로 강제로 움직이지 않는다.

## 4. 실측 파일을 만들고 편집하기

매니퓰레이션 코드를 Jetson에 복사한 뒤:

```bash
mkdir -p "$HOME/robot_config"
cp -n "$ROBOT_REPO/SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/robot.yaml" \
  "$HOME/robot_config/robot.measured.yaml"
nano "$HOME/robot_config/robot.measured.yaml"
```

이미 있는 실측 파일은 덮어쓰지 않는다. nano 저장은 Ctrl+O → Enter, 종료는 Ctrl+X다. null을 아래 측정 결과로 바꾼다. 길이는 m, 관절각은 rad다.

| 값 | 사람이 실제로 할 일 | 기록 |
|---|---|---|
| can_port / firmware | 위 CAN 식별과 SDK 정보 조회 | 실제 포트 이름/버전 문자열 |
| joint_signs / joint_offsets | 이전의 검증된 교시/제조사 저속 조작으로 관절을 하나씩 바꾸고 실제 피드백과 URDF 움직임을 대조. 적어도 두 자세로 방향/영점 확인 | q_model=sign*q_driver+offset |
| return_joints | 몸체·테이블과 부딪히지 않는 복귀 자세로 이동한 뒤 JointState 한 번 읽기 | 이름으로 정렬한6개 raw 각도를 위 식으로 바꾼 **모델 각도** |
| width_scale / width_bias_m | 기존 검증된 그리퍼 조작으로 최소2개, 권장3개 이상의 벌림 상태에서 양 손가락 접촉면 사이 폭을 자/캘리퍼로 측정하고 같은 시점의 raw position 기록 | physical_gap=scale*raw_position+bias. 두 점이면 scale=(w2-w1)/(r2-r1), bias=w1-scale*r1 |
| driver_width_max | 제조사 허용 범위 안에서 확인한 raw 최대 개방값 | 실제 폭100mm라고 raw값을0.100으로 가정하지 않음 |
| grasp_min_effort | 빈 닫힘과 시험 물체를 가볍게 잡은 정지 상태의 **feedback effort**를 각각 여러 번 읽어 비교 | 두 분포를 구분하는 임계값. 겹치면 threshold만으로 확인 불가 |
| table_z_m | URDF base_link 원점 기준으로 테이블 윗면 높이를 실측 | 바닥 높이와 base_link 높이는 같다고 가정하지 않음 |
| table_bounds_xy | 로봇이 실제 멈출 자리에서 사용할 테이블 사각 영역의 네 경계를 측정 | base_link 기준 [xmin,xmax,ymin,ymax], +X전방/+Y왼쪽 |
| obstacles | 작업 공간 주변 고정 물체의 박스 크기/중심을 측정 | 같은 기준의 {id, center:[x,y,z], size:[x,y,z]} 목록 |
| camera_min_depth_m | 실제 depth 해상도에서 유효 depth의 최소 거리를 확인 | 기존 실행기는640×480이며848×480 가정값을 그대로 쓰지 않음 |
| coarse_z_error_bound_m | 실제 head/융합 결과의 높이를 실측 위치와 여러 번 비교 | 관측 초기 거리25cm에 사용할 높이 오차 여유 |

테이블 측정은 로봇이 멈춘 위치에 종속된다. base_link 기준 정적 테이블 설정을 저장한 뒤 로봇을 다른 위치로 이동시키면 같은 숫자를 재사용할 수 없다. 라이브 접근의 최종 정차 위치가 달라지는 경우 작업면 추정으로 scene을 갱신하는 통합이 추가로 필요하다. 현재 고정 설정 데모에서는 동일 정차 위치를 사용한다.

최신 실물 실행기 SW/robot/piper_joint_bridge.py는 nominal signs=[1,1,1,1,1,1], offsets=[-1.6,0,0,0,0,0]을 사용한다. 이는 대조할 후보이며 새 실측이라고 표현하지 않는다. 그대로 입력하기 전에 실제 자세와 모델을 확인한다. raw 복귀각을 그대로 return_joints에 넣으면 q1 기준이 다를 수 있다.

handeye_transform/CAD base/TCP와 최신6클래스 이름은 이미 반영돼 있다. 재입력할 필요는 없지만 실제 손목 장착이 바뀌었으면 hand–eye를 재측정한다. 처음에는 청사과 한 개만 enabled=true로 두고 바나나는 계속 false로 둔다. 확인되지 않은 commissioning flag를 모두 true로 바꾸어 검사를 우회하지 않는다.

## 5. 카메라·외부 모듈 연결: 현재 남은 코드 작업

현재 저장소의 SW/robot/start_robot.sh는 Jetson /home/sparo 환경에서 센서·EKF·카메라·팔 읽기 노드·팔 robot_state_publisher를 시작한다. 이것과 manipulation bringup을 그대로 동시에 실행하면 팔 CAN 노드와 TF가 중복된다. 새 실물 실행기는 실제 wrist_color_optical_frame을 사용하고 매니퓰레이션 모델은 wrist_camera_optical_frame을 요구한다. 프레임 이름만 YAML에서 바꾸는 것은 현재 생성기의 제한 때문에 해결책이 아니다.

통합 시 결정해야 할 소유권:

- Piper 제어 드라이버 하나만 CAN을 담당하고 그 실제 피드백을 재사용한다. 기존 piper_read_slave_joint를 동시에 띄우지 않는다.
- 팔/base/TCP TF는 선택한 통합 robot_state_publisher 하나가 담당한다. 기존 physical_arm_state_publisher와 manipulation publisher를 동시에 띄우지 않는다.
- RealSense는 카메라 내부 optical/depth TF를 담당한다. 외부 hand–eye mount와 모델 optical frame의 연결은 실제 광학 원점에 맞춰 단일 경로로 통합한다. 센서 mount의 시뮬레이션 값이 실물 sensor_tf.py 결과와 겹치지 않도록 모델을 조정한다.
- EKF/Nav는 odom→base_link와 map→odom을 담당한다.

따라서 기존 실물 센서 launch와 매니퓰레이션 launch의 통합 수정은 아직 필요한 작업이다. 손목 카메라 영상이 있다는 것과 외부 WristObservation 메시지가 있다는 것은 다르다.

Head의 현재 /detection/head/target은2D HeadTarget이다. 매니퓰레이션의 /manipulation/target은3D Target이다. topic remap만으로 변환되지 않는다. 실제 head/융합 adapter는3D·frame·취득시각·task_id를 채워야 한다. navigation adapter는 실제 접근 결과와 모든 이동 명령의 잠금/정지 heartbeat를 연결해야 한다. wrist adapter는 ObservationRequest에 응답해 새 영상의 같은6클래스ID·중심3D/표면구분·후보batch를 반환해야 한다. HeadTarget+동일 취득시각 퓨전3D 수신부는 [EXTERNAL_INPUT.md](EXTERNAL_INPUT.md)에 추가했다. 실제 navigation의 task_id/이동 잠금 응답과 wrist 요청·정밀 결과 연결은 여전히 확인해야 한다. [INTERFACES.md](INTERFACES.md)에 구현할 계약이 있다.

closed_approach.launch.py는 기본 use_sim_time=true와 Gazebo 손목 이동 경로를 포함하므로 실물 통합 실행기로 그대로 쓰지 않는다. Head만 실행할 경우도 최신 실제 영상 토픽(/head/camera/color/image_raw), 가중치, Jetson inference Python 환경을 맞춰야 한다. 이것만 실행해도 자동 픽앤플레이스 입력 계약은 완성되지 않는다.

## 6. 빌드와 읽기 전용 점검

[DEPLOYMENT.md](DEPLOYMENT.md)의 의존성 설치 후 Jetson에서 native build한다:

```bash
cd "$ROBOT_REPO/SW/manipulation/manipulation_ws"
colcon build --symlink-install --executor sequential
source install/setup.bash
cd "$ROBOT_REPO"
python3 SW/manipulation/tools/check_live_demo.py \
  --config "$HOME/robot_config/robot.measured.yaml" \
  --output /tmp/manipulation_config_check.json
```

missing_values는 측정해서 채울 목록이다. unacknowledged_flags는 현장에서 확인한 후 표시할 목록이다. 이 도구는 팔 명령을 보내지 않는다.

## 7. 통합 완료 후 기동과 실제 한 개 시험

위 외부 계약/TF 통합과 설정 확인이 끝난 다음에만, 각 터미널에서 동일 환경과 manipulation install/setup.bash를 source하고:

```bash
ros2 launch robocup_real_manipulation bringup.launch.py \
  hardware:=true config:="$HOME/robot_config/robot.measured.yaml" \
  tuning_file:="$ROBOT_REPO/SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/tuning.yaml" \
  repo:="$ROBOT_REPO" start_driver:=false
```

start_driver=false는 터미널 A의 제어 드라이버를 재사용한다는 뜻이다. 읽기 노드만 켜져 있는 상태를 뜻하지 않는다. 해당 bringup 자체는 자동 enable하거나 자동 goal을 보내지 않는다. 현장 확인 후 제조사 enable 절차를 수행하고 실제 목표 선택/접근 완료를 받는다. 상위 모듈이 goal을 보내면 CLI를 중복 실행하지 않는다.

상위 client가 아직 없고 **실제 계약 입력이 모두 연결돼 있을 때** goal만 CLI로 시작:

```bash
ros2 topic echo /manipulation/target --once
ros2 topic echo /manipulation/approach_result --once
ros2 topic echo /manipulation/status
# 별도 터미널: 위 실제 메시지의 동일 task_id로 치환
ros2 action send_goal /manipulation/run \
  robocup_real_manipulation_msgs/action/Manipulate \
  '{task_id: "실제-작업-ID"}' --feedback
```

처음 작업은 같은 정차 위치/정적 테이블/물체 한 개에서 저속으로 수행한다. coarse 위 관측 이동 → 실제 손목 재검출 → 파지 → place → 복귀를 확인하고, 작업을 종료한 뒤 tuning.yaml의 관측 높이·grip_height_m·open_m·close_m를 조절해 재시작한다. 현재 문서 명령은 Jetson 실물 실행 검증을 대신하지 않는다.
