# 1차 실물 시험: 접근 완료 → 실제 손목 관측 → 집기 → 옆에 놓기

같은 robocup_real_manipulation 패키지를 사용한다. phase1.launch.py는 기존 bringup/MoveIt/Piper bridge/coordinator에 실제 손목 YOLO+SAM2 ROS 수신부를 연결한다. 별도의 완성용·시험용 ROS 패키지를 만들지 않는다. 시험 시작 조건과 입력 송신 프로세스만 분리한다.

첫 관측 위치는 **기존 디텍션·퓨전의 실제3D**에서 계산한다. 사용자가XYZ를 입력하거나 손목 결과를 합성하지 않는다. 첫번째 coarse로 관측 이동 → 새 손목 RGB의 YOLO 분류 → SAM2 마스크 → 실제 aligned depth로3D → 취득시각 TF/hand–eye 적용 → 클래스별 규칙/TCP → 집기 → 오른쪽10cm place(계획 불가시 왼쪽) → 후퇴/복귀 순서다. 손목의 서로 다른 실제 프레임3개가 안정적으로 일치해야 파지한다.

## 1. 노트북에서 SSH 접속과 코드 전달

예전에 쓴SSH 접속 명령을 사용한다. 저장소 위치가 헷갈리면 Jetson SSH 창에서:

```bash
find "$HOME" -type d -name robocup_head_detection 2>/dev/null
```

예를 들어 /home/sparo/robocup/SW/detection/head/head_detection_ws/src/robocup_head_detection가 나오면 저장소는 /home/sparo/robocup이다. 아래에서는 ~/robocup으로 가정한다. 다른 위치이면 ROBOT_REPO만 바꾼다.

현재 SW/manipulation은 노트북 로컬 작업이며 아직 push하지 않았다. Jetson git pull만으로 새 패키지 수정이 전달되지 않는다. 노트북의 별도 터미널에서 실제SSH 주소로 바꿔 복사한다:

```bash
rsync -av --exclude build --exclude install --exclude log --exclude __pycache__ --exclude '*.egg-info' \
  '/home/doyeonlee/문서/ChatGPT/RoboCup@Home2027/INHA-RoboCup-Home-2th-2027/SW/manipulation/' \
  실제사용자명@실제Jetson_IP:~/robocup/SW/manipulation/
```

## 2. 앞으로의 명령은 Jetson SSH 창에서 실행

각 SSH 터미널에 같은 환경을 적용한다. 실제 외부 센서/디텍션이 다른도메인을 사용하면0을 그 값으로 바꾼다.

```bash
export ROBOT_REPO="$HOME/robocup"
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source "$HOME/piper_ros/install/setup.bash"
export PYTHONPATH="$HOME/robot_setup/ros-python:$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
```

모든 노드를Jetson에서 실행하고 노트북은SSH 화면으로만 사용한다. use_sim_time=false다. 노트북에서 별도GPU 추론을 수행하는 구조는 이번 절차에 포함하지 않는다.

## 3. 첫번째로 실제 coarse 입력 존재를 확인

기존 센서·Head 디텍션·퓨전은 기존 실행기로 시작한다. 먼저 다음 출력이 있어야 한다:

```bash
ros2 topic info /detection/head/target -v
ros2 topic echo /detection/head/target --once
ros2 topic info /detection/target_map_point -v
ros2 topic echo /detection/target_map_point --once
```

HeadTarget은 선택한물체의class_id/class_name/target_id/영상취득시각을 포함한다. PointStamped는 **그 검출에 대응하는 실제3D**이며 같은 영상취득stamp를 보존해야 한다. 단위m와frame_id가 필요하다. 현재 수신부가 기본으로 받는 두 토픽이다. 실제퓨전 출력이 /detection/target_position 등 다른 이름이면 아래launch의point_topic을 바꾼다.

ROS 토픽이 없는데 bbox의x/y를3D로 넣거나 지연된좌표에 현재stamp를 붙이지 않는다. 기존 nav_goal_node.py는 시뮬레이션 전용검사가 있는 코드이므로 이 문서가 실물3D 퓨전을 새로 구현했다고 해석하지 않는다. 실제 외부3D 출력이 없으면 첫관측 좌표가 없으므로 이 단계에서 연동해야 한다. native Target 타입을 이미 제공하면 start_head_adapter=false로 실행하고 phase1_pick의 --input-topic에 그 토픽을 지정한다.

**좌표를 넣는 방법은 구독이다.** phase1_pick green_apple은 그클래스의 실제 coarse Target을 받아 유일한시험task_id로 coordinator에 전달한다. 원본좌표·frame·취득stamp는 유지하며 관측 이동에만 쓴다. 새 손목좌표가 확인되기 전에는 파지하지 않는다.

## 4. 실물 구성값과 시험 환경 준비

Piper 전원/CAN과손목D435IF USB3를 기존검증한 배선으로 연결한다. 클로즈어프로치가 완료된 작업위치에 로봇을 정차시킨다. 이번 시험에서는 **베이스 구동만 실제로 격리**한다. Jetson·카메라·Piper전원은 유지한다. 시험flag는 격리를 수행했다는 운영자확인이지 프로그램이 베이스를잠그는기능이 아니다. 모바일시험에서는 이client를쓰지말고 실제 Nav lease를사용한다.

```bash
mkdir -p "$HOME/robot_config"
cp -n "$ROBOT_REPO/SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/robot.yaml" \
  "$HOME/robot_config/robot.measured.yaml"
nano "$HOME/robot_config/robot.measured.yaml"
```

이미 있는실측파일은 덮어쓰지않는다. 아래실측값을 채우고확인한다. 길이m/각도rad다.

| 항목 | 실제할일 |
|---|---|
| can_port/firmware | 연결한Piper CAN/실제버전. 최신센서실행기 기본 can_arm은 참고이며 실제연결확인 |
| joint_signs/joint_offsets | 기존교시/저속조작으로 실제피드백과URDF방향·영점 대조. 기존 실물bridge는signs=[1,1,1,1,1,1],offsets=[-1.6,0,0,0,0,0]을사용하지만 확인없이실측값이라고처리하지않음 |
| return_joints | 충돌없는복귀자세의 모델기준6각도. raw피드백을sign/offset으로변환 |
| table_z_m/table_bounds_xy/obstacles | 정차한현재위치의base_link기준테이블 윗면/사용사각형/주변장애물. 정차위치를바꾸면scene도갱신 |
| width_scale/width_bias_m/driver_width_max | 실제손가락사이폭과raw피드백을여러개개방상태에서측정 |
| grasp_min_effort | 빈닫힘/실제접촉의정지feedback effort 비교 |
| camera_min_depth_m/coarse_z_error_bound_m | 현재depth profile의유효최소거리/실제coarse Z오차여유 |

기존hand–eye/CAD base/TCP/최신클래스높이는 이미있다. 현재장착과대조한다. external_sensor_tf: true가필수다. 각verification flag는현장확인후에만true로한다. 처음에는green_apple만enabled=true,나머지는false로두어도된다. [전체필수값](REQUIRED_VALUES.md), [측정명령](FIRST_CONNECTION.md).

관측초기거리는phase1.tuning.yaml의윗면→렌즈30cm다. 기존SAM실험의depth최소필터25cm와높이오차/여유를고려한초기값이며확정값이아니다. 해당profile에서더가까운depth가유효하면수정한다. open/close/grip_height/pregrasp/lift/place거리도tuning 파일로조정한다.

## 5. 드라이버 timestamp 패치와 빌드

[DEPLOYMENT.md의Piper패치](DEPLOYMENT.md#2-piper-개별-피드백-timestamp-패치)를검토·적용하고piper를재빌드한다. /gripper_states_feedback가 있어야 한다. 패치가이미적용되었으면중복적용하지않는다.

Head workspace를이미빌드했다면그install을source한다. 처음이면메시지와기존YOLO adapter 패키지를빌드한다:

```bash
cd "$ROBOT_REPO/SW/detection/head/head_detection_ws"
colcon build --packages-select robocup_detection_msgs robocup_head_detection --executor sequential
source install/setup.bash
cd "$ROBOT_REPO/SW/manipulation/manipulation_ws"
rosdep install --from-paths src --ignore-src --rosdistro humble -y \
  --skip-keys 'piper piper_msgs robocup_detection_msgs robocup_head_detection'
colcon build --symlink-install --executor sequential
source install/setup.bash
```

Jetson ARM64에서 native build한다. 노트북 build/install을 복사하지 않는다. GPU torch/CUDA/NumPy를 일괄 업그레이드하지 않는다. SAM2/Ultralytics는 기존 GPU Python 환경을 사용한다.

## 6. 추론 Python과 가중치 확인

```bash
cp -n "$ROBOT_REPO/SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/wrist.yaml" \
  "$HOME/robot_config/wrist.yaml"
find "$HOME" -type f -name 'sam2.1_hiera*.pt' 2>/dev/null
nano "$HOME/robot_config/wrist.yaml"
```

sam_checkpoint에 실제 존재하는 파일의 절대 경로를 넣는다. 기본은 기존 SAM 실험의 small과 configs/sam2.1/sam2.1_hiera_s.yaml이다. Jetson에 tiny만 있으면 tiny checkpoint와 configs/sam2.1/sam2.1_hiera_t.yaml을 함께 지정한다. 현재 Git에는 SAM 가중치가 포함되어 있지 않다. YOLO 기본은 SW/detection/head/models/head_yolo11n_20261013.pt다. 기존 TensorRT .engine을 사용하면 Jetson에서 생성된 실제 파일을 지정한다.

기존 SAM GPU 환경을 사용하는 예:

```bash
export VISION_PYTHON="$HOME/venvs/sam21/bin/python"
"$VISION_PYTHON" "$ROBOT_REPO/SW/manipulation/tools/check_wrist_environment.py" \
  --repo "$ROBOT_REPO" --vision-config "$HOME/robot_config/wrist.yaml" --load
```

errors가 비어 있고 model_load가 OK인 Python을 사용한다. 같은 환경에서 torch(CUDA)/ultralytics/sam2/cv_bridge/rclpy가 import되어야 한다. ROS workspace를 source한 뒤 확인한다. sam21에 ultralytics가 없는 등 import 실패는 이 도구에 표시된다. 기존 동작하는 추론 환경에 맞춰 필요한 의존성을 제공한다. 이번 노트북에는 torch/sam2가 없어 실제 추론은 검증하지 못했다.

## 7. TF와 카메라 실행기를 하나로 정리

기존 SW/robot/start_robot.sh를 사용하는 경우 그 터미널에서 Ctrl+C로 종료하고 arm:=false로 다시 시작한다. 기존 Piper 읽기 노드와 팔 publisher가 시작되지 않는다. 센서·실물 sensor_tf·카메라는 기존 실행기가 담당하고, 팔/TCP/내부 손목 기준 프레임은 매니퓰레이션이 담당한다.

```bash
bash "$ROBOT_REPO/SW/robot/start_robot.sh" arm:=false
```

베이스 구동 격리는 유지한다. 이 스크립트는 Jetson /home/sparo의 기존 underlay를 사용하므로 사용자·설치 위치가 다르면 실제 실행 환경에 맞춰야 한다. 코이데/Base–Mid360/G2/손목 hand–eye의 기존 실물 결과를 사용한다.

기존 Piper 제어 드라이버도 종료해 하나만 실행한다. 아래 launch의 start_driver=true가 설정한 CAN에서 제어 드라이버를 시작하며 auto_enable=false다.

## 8. 설정 검사 후 기동

별도 SSH 터미널에 공통 ROS 환경, Head install, manipulation install을 source한 뒤:

```bash
python3 "$ROBOT_REPO/SW/manipulation/tools/check_live_demo.py" \
  --config "$HOME/robot_config/robot.measured.yaml" \
  --tuning-file "$ROBOT_REPO/SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/phase1.tuning.yaml" \
  --output /tmp/phase1_config_check.json

ros2 launch robocup_real_manipulation phase1.launch.py \
  repo:="$ROBOT_REPO" config:="$HOME/robot_config/robot.measured.yaml" \
  vision_config:="$HOME/robot_config/wrist.yaml" vision_python:="$VISION_PYTHON" \
  start_driver:=true start_camera:=false start_mount_tf:=false
```

hardware_config_ready=false이면 미정 값과 확인 flag를 해결한 다음 기동한다. 기존 Piper **제어** 드라이버를 재사용하면 start_driver=false다. 실물 Head/퓨전3D는 기존 담당 모듈의 실행 방법으로 계속 제공한다.

기존 실행기가 카메라와 sensor_tf를 시작하지 않는 단독 구성에서는 start_camera=true start_mount_tf=true로 설정한다. 두 실행기가 같은 카메라·TF를 동시에 시작하지 않는다. 실제 coarse source의 map/odom TF와 취득시각도 필요하므로 센서 실행기를 중단할 때 그 경로가 끊기지 않는지 확인한다.

## 9. 실제 입력 확인 후 물체 한 개로 실행

별도 SSH 터미널에 같은 환경을 적용한 뒤:

```bash
ros2 topic echo /joint_states_feedback --once
ros2 topic echo /gripper_states_feedback --once
ros2 topic echo /manipulation/coarse_target --once
ros2 topic echo /manipulation/wrist/status --once
ros2 run tf2_ros tf2_echo piper_link6 wrist_color_optical_frame
```

wrist/status가 READY이고 coarse_target이 선택한 물체의 신선한 실제3D이며, feedback/TF가 실물과 일치하는지 확인한다. 모터 활성화는 마지막에 한다. 서비스 타입과 현장 정지 조건을 확인한 뒤:

```bash
ros2 service call /enable_srv piper_msgs/srv/Enable '{enable_request: true}'
```

청사과 한 개로 실행하는 예:

```bash
ros2 run robocup_real_manipulation phase1_pick green_apple \
  --config "$HOME/robot_config/robot.measured.yaml" \
  --already-docked --base-drive-isolated --input-durable
```

실제 coarse 수신 → 관측 이동 → 새로운 손목 관측3개 → 규칙 기반 파지 → lift → 옆에 놓기 → 복귀가 자동으로 진행된다. 두 flag는 실제 조건을 수행했다는 운영자 확인이며 Nav2가 접근을 실행했다는 증거가 아니다. 이 client는 손목 결과·팔 피드백·TF·enable 명령을 합성하거나 전송하지 않는다.

별도 터미널에서 ros2 topic echo /manipulation/status를 확인한다. Ctrl+C는 취소를 요청한다. 실제 정지·물체 유지가 미확인 상태라면 현장 정지와 수동 복구를 수행한다. 실패 후 journal을 지워 반복하지 않는다. 성공은 place_success와 return_success가 모두 true인 결과로 확인한다.

## 10. 다음 trial

작업 종료 후 phase1.tuning.yaml의 관측 거리·속도·클래스별 폭/파지 높이를 바꾸고 launch를 재시작한다. 실물 튜닝 사본을 사용하면 tuning_file 인자로 그 경로를 지정한다. 미세 조정 때문에 코드를 수정하지 않는다. 최종 성공률과 Jetson GPU 추론 시간은 실제 로봇에서 기록한다.
