# Jetson 빌드·설치·실행 및 실물 실험

최종 실행은 Jetson ARM64다. 노트북의 x86_64 build/install 디렉터리를 복사하지 않는다. 코드·msg·config·launch·문서와 HW mesh를 포함한 저장소를 Jetson으로 옮겨 native build한다. x86 detection Docker나 CPU torch wheel은 설치하지 않는다. 본 패키지는 NumPy/YAML/ROS 외 GPU 라이브러리를 요구하지 않으며 디텍터의 기존 JetPack GPU 환경을 유지한다.

## 1. 환경 재확인과 의존성

Jetson 터미널에서:

```bash
uname -m
dpkg --print-architecture
cat /etc/os-release
cat /etc/nv_tegra_release
python3 --version
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
ros2 doctor --report
```

기록과 다른 환경이면 기록 값을 설치된 사실처럼 사용하지 않는다. Humble/Jammy/Python 3.10을 기준으로 작성했다. 현재 Jetson의 RealSense/librealsense와 MoveIt 설치는 직접 확인해야 한다.

기존 piper source/SDK를 재사용한다. 기록의 `$HOME/robot_setup/piper-python`에 piper-sdk 0.6.2와 python-can 4.6.1이 있는지 확인한다. 전역 pip/NumPy/CUDA/JetPack 업그레이드는 필요 없다.

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
python3 -c 'import piper_sdk, can; print(piper_sdk.__file__, can.__version__)'
git -C ~/piper_ros rev-parse HEAD
cd ~/piper_ros
colcon build --packages-select piper_msgs piper piper_description --executor sequential
source install/setup.bash
```

RealSense D435 계열이 실제 손목 카메라로 확인되면 ARM64용 ROS wrapper/librealsense의 JetPack 지원과 장비 serial·USB3 연결을 확인한다. 이미 정상 동작하는 설치를 우선 재사용한다. MID360 SDK2/driver2는 기존 외부 모듈 의존성이며 본 패키지에서 새로 실행하거나 IP/CAN 설정을 변경하지 않는다.

추가 ROS/apt 의존성은 package.xml에 선언했다. 저장소 루트의 `SW/manipulation/manipulation_ws`에서:

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
source "$ROBOT_REPO/SW/detection/head/head_detection_ws/install/setup.bash"
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "piper piper_msgs robocup_detection_msgs"
colcon build --symlink-install --executor sequential --event-handlers desktop_notification- terminal_title- status-
source install/setup.bash
```

`piper_msgs`는 ROS apt에 있다고 가정하지 않고 위 제조사 underlay에서 제공한다. 필요한 주요 패키지는 `moveit_msgs`, `moveit_ros_move_group`, `moveit_kinematics`, `moveit_planners_ompl`, `moveit_simple_controller_manager`, `control_msgs`, `tf2_ros`, `cv_bridge`, `python3-numpy`, `python3-yaml`다. 본 패키지는 Piper SDK를 직접 호출하지 않고 기존 ROS 드라이버에 연결한다.

기존 Head 출력 수신부의 `robocup_detection_msgs`도 저장소 Head workspace에서 제공한다. 아직 빌드되지 않았다면 [EXTERNAL_INPUT.md](EXTERNAL_INPUT.md)의 메시지 패키지 빌드 후 source한다. Head 모델/추론 환경을 새로 설치하는 작업은 아니다.

## 2. Piper 개별 피드백 timestamp 패치

확인한 고정 소스의 arm JointState timestamp는 joint/highspeed 최신 시각의 max이고, gripper 데이터의 개별 timestamp를 전달하지 않는다. 최신 gripper feedback의 근거를 마련하려고 작은 패치를 제공한다. 원본 소스는 이번 작업에서 변경하지 않았다.

저장소 루트에서 review용 patch를 생성/검사:

```bash
python3 SW/manipulation/tools/prepare_piper_feedback_patch.py ~/piper_ros /tmp/piper_feedback.patch
cd ~/piper_ros
git apply --check /tmp/piper_feedback.patch
git diff
```

patch 내용을 검토한 뒤 적용하고 드라이버를 다시 build한다:

```bash
git apply /tmp/piper_feedback.patch
colcon build --packages-select piper --executor sequential
source install/setup.bash
```

추가되는 `/gripper_states_feedback`은 name=`gripper`, position=SDK angle/1e6, effort=SDK effort/1e3, stamp=SDK gripper timestamp다. 기존 arm feedback stamp는 SDK joint timestamp를 사용한다. 센서 시각이 ROS wall clock과 호환되는지 실제 장비에서 확인한다. SDK가 영점 시각/미갱신 값을 주면 bridge는 거부한다. SDK 객체의 timestamp와 데이터의 원자적 일관성도 실기에서 확인한다.

고정 SHA가 다르면 자동 patch는 거부한다. 새 소스의 API와 피드백 의미를 비교해야 한다. 한 번 적용한 patch를 중복 적용하지 않는다.

## 3. 구동 없는 확인

```bash
ros2 launch robocup_real_manipulation bringup.launch.py
ros2 interface show robocup_real_manipulation_msgs/msg/Target
ros2 interface show robocup_real_manipulation_msgs/action/Manipulate
ros2 topic echo /manipulation/status
```

기본 read-only launch는 goal을 거부하고 실제 드라이버를 실행하지 않는다. 코드 문법·단위·상태 검사는 아래처럼 실행한다:

```bash
python3 -m unittest discover -s src/robocup_real_manipulation/test -v
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 python3 ../test_ros_contract.py
```

test sender/mock은 **실제 로봇과 다른 ROS_DOMAIN_ID**에서만 실행한다. 조합별 수동 검사:

```bash
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 ros2 launch robocup_real_manipulation contract_test.launch.py
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 ros2 run robocup_real_manipulation test_sender --ros-args -p approach_first:=true
```

`scenario:=cancel`, `scenario:=stale_observation`, `scenario:=wrist_timeout`, `base_stopped:=false`, `approach_outcome:=FAILED`도 지원한다. 복귀 실패는 mock launch의 `fail_stage:=RETURN_ARM`으로 검사한다. 실패 후 mock journal에도 잠금이 남으므로 자동 계약 검사는 각 case 별 임시 journal을 사용한다. 수동 재실행은 mock journal만 별도 파일로 지정한다.

## 4. 실물 구성 파일과 연결 확인

`config/robot.yaml`을 로봇 관리 디렉터리의 `robot.measured.yaml`로 복사한다. [필수 입력 목록](REQUIRED_VALUES.md)을 채운다. 최초에는 한 종류만 enabled로 둔다. actual model names와 물체 상태(세워진 컵·캔, 테이블 위 대상)를 확인한다.

2026-10-12 기본 robot.yaml에는 feature/wrist-handeye-calibration의 실물 PARK hand–eye와 CAD base/TCP가 이미 들어 있다. 이전에 복사한 robot.measured.yaml을 사용하는 경우 원본과 대조해 handeye_transform·base_arm_transform·link6_tcp_transform 및 출처 항목을 반영한다. 기존 실측 테이블/그리퍼/관절 설정을 새 기본 파일로 덮어쓰지 않는다. 손목 보정값의 xyz는m, quaternion은xyzw이며 camera optical→link6 방향이다. 구동 허용 flag는 보정값 반영만으로 자동 변경하지 않는다.

CAN을 임의 `can0`로 올리거나 자동 enable하지 않는다. 알려진 연결명·bitrate·firmware에 대해 제조사 절차로 연결하고 다음을 확인한다:

```bash
ip -details link show
ros2 topic list -t
ros2 service list -t
ros2 action list -t
ros2 topic info /joint_states_feedback -v
ros2 topic hz /joint_states_feedback
ros2 topic hz /gripper_states_feedback
ros2 topic echo /arm_status
```

관절 순서와 sign/offset은 알려진 안전 자세에서 확인하고 실제 개방 폭을 자로 측정한다. upstream `gripper_val_mutiple`은 반드시 1로 유지하며 width scale은 bridge 한 곳에서만 적용한다. upstream driver의 velocity[6]은 global speed percent이며 gripper 속도가 아니다. `driver_speed_percent=5` 초기값의 실제 속도·braking은 측정한다.

제조사 `start_single_piper.launch.py`는 `auto_enable=true`와 command→`/joint_states` remap 기본값이 있으므로 이 패키지와 함께 사용하지 않는다. 그런 remap은 피드백이 명령으로 재입력되는 구조를 만들 수 있다. 본 launch는 remap 없이 별도 command topic으로 구성한다.

드라이버만 읽기 상태로 시작할 때에도 장비 연결·프로세스 소유권을 먼저 확인한다. 이미 driver가 있으면 중복 시작하지 않는다:

```bash
ros2 run piper piper_single_ctrl --ros-args -p can_port:=실제_CAN_이름 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=1
```

이 명령은 모터를 enable하지 않지만 CAN 연결은 연다. 실제 움직임 전 enable 서비스 이름·타입과 장비 정지 상태를 확인하고 현장 운영자가 제조사 절차에 따라 enable한다. 패키지는 자동 enable/disable을 하지 않는다.

## 5. 실물 launch

실측 config 형식 검사를 통과하고 현장 실험 준비가 된 경우:

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
source /absolute/path/to/repo/SW/manipulation/manipulation_ws/install/setup.bash
ros2 launch robocup_real_manipulation bringup.launch.py \
  hardware:=true \
  config:=/absolute/path/robot.measured.yaml \
  repo:=/absolute/path/to/repo \
  start_driver:=false
```

이미 드라이버가 있으면 `start_driver=false`. 본 launch로 드라이버도 시작하려면 `start_driver=true`를 명시한다. 여전히 auto_enable=false다. 카메라·head/MID360·Tracer/Nav2·손목 detector는 외부의 검증된 실행 방법으로 시작한다. 카메라 TF publisher 중복을 피하도록 기존 wrapper의 `publish_tf`와 robot_state_publisher 소유권을 정한다.

실물 작업 goal은 외부 모듈이 실제 Target/ApproachResult와 베이스 잠금 heartbeat를 제공해야 진행된다. notebook과 Jetson의 실제 도메인/네트워크/clock sync를 확인하고 test_sender를 실물 도메인에서 사용하지 않는다.

```bash
ros2 action send_goal /manipulation/run robocup_real_manipulation_msgs/action/Manipulate \
  '{task_id: "고유한-외부-작업-ID"}' --feedback
```

취소는 상위 ROS action client의 `goal_handle.cancel_goal_async()`를 사용한다. 명령 전송 완료와 action 성공은 구분되며, 모듈은 실제 관절·폭·정지 피드백까지 기다린다.

## 6. 단계별 실물 실험

1. **센서/TF**: serial·intrinsics·정합 depth·hand–eye·TCP·기준 frame·관절 한계·폭 측정. 모터 미구동으로 timestamp/TF/모델 차이를 확인한다.
2. **외부 접근 완료**: 접근 실패·base_stopped=false·잠금 heartbeat 누락에서 팔 명령이 없어야 한다. Nav 전체 command source가 lock에 의해 차단되는지 확인한다.
3. **손목 재관측**: coarse 좌표와 실측 카메라/TCP 관계로 계산한 관측 위치를 먼저 표시하고 수동 저속 검증한 뒤 요청 연동. 같은 클래스의 두 물체, stale frame, mask background, TF 누락을 넣고 거부를 확인한다.
4. **파지 목표 확인**: `/manipulation/status`의 precise_center/grasp_tcp와 RViz scene을 실측 위치와 비교한다. 외부 기록에 실제 위치 오차를 적는다. coarse↔precise 거리는 실제 오차의 정답이 아니다.
5. **저속 파지**: 대상 한 개·clear table·현장 정지 가능 조건. 손가락 개방/수직 하강/닫힘 폭/effort 판정을 확인한다. 빈 닫힘을 성공으로 판정하면 중단하고 threshold를 조정한다.
6. **lift/place**: 접촉 유지·object box attachment·오른쪽 계획·왼쪽 fallback·지지 영역 확인. 실패 시 HELD/UNKNOWN 상태로 유지되는지 확인한다.
7. **후퇴/복귀**: release 후 수직 후퇴와 실제 return 관절 도달·정지 확인. place 성공+return 실패가 전체 FAILED로 보고되는지 확인한다.
8. **실패/취소**: 단계별 cancel, CAN/센서 heartbeat 상실, base lock 상실, joint feedback 지연. 실제 정지 거리·object 유지·stop_confirmed 판정과 수동 복구를 측정한다.

로그는 `log_path`의 JSONL에 task_id, stamp, stage, reason, holding, place/return/stop, center/TCP, association distance를 남긴다. 실물 검증 기록에는 추가로 실제 위치 오차(mm), 폭 실측, 파지/placement 성공 관찰, 최종 팔 상태를 적는다. 실행 결과를 시뮬레이션/mock 성공과 섞지 않는다.
