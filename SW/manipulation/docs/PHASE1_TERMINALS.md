# Jetson 터미널별 실행 순서 — SSH 접속 이후

기존 robocup_real_manipulation 패키지 하나를 수정했다. 접근 완료 상태 시험은 phase1.launch/phase1_pick, 전체 외부 연동은 bringup/실제 Nav lease를 사용한다. 별도 ROS 패키지 두 개를 만들지 않는다.

이번 수정 코드가 Jetson의 SW/manipulation에 전달된 상태에서 시작한다. 아직 노트북 로컬 코드이므로 Git pull만으로 전달되지는 않는다. 설치/실측 상세는 [PHASE1_RUN.md](PHASE1_RUN.md)에 있다. 아래는 저장소가 ~/robocup인 예이며 실제 위치가 다르면 ROBOT_REPO를 바꾼다.

## 터미널 0 — 최초 준비와 빌드

```bash
export ROBOT_REPO="$HOME/robocup"
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source "$HOME/piper_ros/install/setup.bash"
export PYTHONPATH="$HOME/robot_setup/ros-python:$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"

cd "$ROBOT_REPO/SW/detection/head/head_detection_ws"
colcon build --packages-select robocup_detection_msgs robocup_head_detection --executor sequential
source install/setup.bash

cd "$ROBOT_REPO/SW/manipulation/manipulation_ws"
rosdep install --from-paths src --ignore-src --rosdistro humble -y \
  --skip-keys 'piper piper_msgs robocup_detection_msgs robocup_head_detection'
colcon build --symlink-install --executor sequential
source install/setup.bash

mkdir -p "$HOME/robot_config"
cp -n src/robocup_real_manipulation/config/robot.yaml "$HOME/robot_config/robot.measured.yaml"
cp -n src/robocup_real_manipulation/config/wrist.yaml "$HOME/robot_config/wrist.yaml"
cp -n src/robocup_real_manipulation/config/phase1.tuning.yaml "$HOME/robot_config/phase1.tuning.yaml"
```

Piper의 전용 그리퍼 피드백 timestamp 패치가 적용돼 있어야 한다. [드라이버 패치 절차](DEPLOYMENT.md#2-piper-개별-피드백-timestamp-패치). 기존 실측 파일은 복사 명령으로 덮어쓰지 않는다.

이어서 설정을 편집한다:

```bash
nano "$HOME/robot_config/robot.measured.yaml"
nano "$HOME/robot_config/wrist.yaml"
find "$HOME" -type f -name 'sam2.1_hiera*.pt' 2>/dev/null
```

robot.measured의 null을 실제 CAN/펌웨어, 관절 방향·영점·복귀각, 테이블 높이/영역/장애물, 그리퍼 폭·접촉 대응, depth 유효거리/coarse 오차로 채운다. 저장된 hand–eye/CAD TCP는 이미 있다. 확인한 flag만 true로 바꾼다. external_sensor_tf: true가 필요하다. 처음에는 청사과만 enabled로 둬도 된다.

wrist.yaml의 sam_checkpoint는 찾은 파일의 절대 경로다. small은 sam2.1_hiera_s.yaml, tiny는 sam2.1_hiera_t.yaml과 짝을 맞춘다. Git에는 SAM checkpoint가 없다.

각 터미널에서 재사용할 환경 파일을 최초 한 번 만든다. 다른 domain을 쓰는 기존 센서/디텍션이 있으면 0을 해당 값으로 바꾼다. VISION_PYTHON은 같은 환경에서 SAM2·YOLO·CUDA·ROS를 import할 수 있는 실제 경로를 사용한다.

```bash
cat > "$HOME/robot_config/phase1_env.sh" <<'EOF'
export ROBOT_REPO="$HOME/robocup"
export ROS_DOMAIN_ID=0
export VISION_PYTHON="$HOME/venvs/sam21/bin/python"
source /opt/ros/humble/setup.bash
source "$HOME/piper_ros/install/setup.bash"
source "$ROBOT_REPO/SW/detection/head/head_detection_ws/install/setup.bash"
source "$ROBOT_REPO/SW/manipulation/manipulation_ws/install/setup.bash"
export PYTHONPATH="$HOME/robot_setup/ros-python:$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
EOF
source "$HOME/robot_config/phase1_env.sh"

"$VISION_PYTHON" "$ROBOT_REPO/SW/manipulation/tools/check_wrist_environment.py" \
  --repo "$ROBOT_REPO" --vision-config "$HOME/robot_config/wrist.yaml" --load

python3 "$ROBOT_REPO/SW/manipulation/tools/check_live_demo.py" \
  --config "$HOME/robot_config/robot.measured.yaml" \
  --tuning-file "$HOME/robot_config/phase1.tuning.yaml" \
  --output /tmp/phase1_config_check.json
```

추론 환경의 errors가 비어 있고 model_load가 OK, 구성의 hardware_config_ready가 true인 뒤 다음 단계로 간다. 값이나 확인 항목이 비어 있으면 그 출력 목록을 해결한다. 시스템 CUDA/torch/NumPy를 일괄 교체하지 않는다.

## 터미널 1 — 기존 센서·카메라·실물 TF

로봇을 접근 완료 위치에 정차시키고 베이스 구동을 실제 격리한다. Jetson/센서/Piper 전원은 유지한다. 기존 센서 실행기를 그 터미널에서 Ctrl+C로 종료한 뒤:

```bash
source "$HOME/robot_config/phase1_env.sh"
bash "$ROBOT_REPO/SW/robot/start_robot.sh" arm:=false
```

이 창은 유지한다. 기존 팔 읽기 노드와 팔 TF publisher를 끄고, 기존 실물 센서 TF·두 카메라는 유지하는 구성이다. 기존 Piper 제어 드라이버도 있으면 담당 터미널에서 종료해 터미널 2의 제어 드라이버 하나만 사용한다. 이미 같은 소유권으로 운용 중이면 중복 시작하지 않는다.

## 터미널 1B — 기존 디텍션/퓨전

기존 모듈이 실행 중이면 그대로 사용한다. 이 터미널은 확인용이다:

```bash
source "$HOME/robot_config/phase1_env.sh"
ros2 topic echo /detection/head/target --once
ros2 topic info /detection/target_map_point -v
```

HeadTarget의 실제 클래스/검출 박스가 나와야 한다. 선택한 검출에 대응하는 실제3D PointStamped가 있으면 터미널 2에서 point_source=fusion을 사용한다. 다른3D 토픽이면 point_topic을 해당 이름으로 바꾼다.

3D 토픽 없이 현재 Head의 실제 2D 검출만 제공하는 구성이라면 point_source=aligned_depth를 사용할 수 있다. 기존 검출 박스와 Head 정합 depth/CameraInfo/실물 TF에서 coarse XYZ를 만든다. 새 디텍터를 만드는 것이 아니며 XY는 표면상의 대략점, Z는 실측 테이블+클래스 중심높이다. 이 coarse로 바로 파지하지 않고 반드시 손목에서 재측정한다. 두 경로 모두 터미널로 XYZ를 임의 입력하지 않는다.

## 터미널 2 — 실제 매니퓰레이션 실행

실제 퓨전3D가 제공되는 기본 구성:

```bash
source "$HOME/robot_config/phase1_env.sh"
ros2 launch robocup_real_manipulation phase1.launch.py \
  repo:="$ROBOT_REPO" config:="$HOME/robot_config/robot.measured.yaml" \
  vision_config:="$HOME/robot_config/wrist.yaml" vision_python:="$VISION_PYTHON" \
  tuning_file:="$HOME/robot_config/phase1.tuning.yaml" \
  start_driver:=true start_camera:=false start_mount_tf:=false \
  point_source:=fusion point_topic:=/detection/target_map_point
```

2D 검출+실제 Head depth로 coarse를 받으면 마지막 줄만 point_source:=aligned_depth로 바꾼다. 기존 제어 드라이버를 재사용하면 start_driver=false다. 터미널 1에서 카메라/실물 sensor_tf를 담당하므로 start_camera=false와 start_mount_tf=false를 유지한다. 이 창을 유지한다. 자동 모터 enable은 하지 않는다.

## 터미널 3 — 최종 확인 후 시작 명령

```bash
source "$HOME/robot_config/phase1_env.sh"
ros2 topic echo /joint_states_feedback --once
ros2 topic echo /gripper_states_feedback --once
ros2 topic echo /manipulation/coarse_target --once
ros2 topic echo /manipulation/wrist/status --once
ros2 run tf2_ros tf2_echo piper_link6 wrist_color_optical_frame
```

마지막 tf2_echo는 확인 후 Ctrl+C로 종료한다. coarse는 선택 물체의 신선한3D, 손목 상태는 READY여야 한다. 실제 공간/모델/정지 조건이 확인되면 모터 활성화를 마지막에 수행한다:

```bash
ros2 service call /enable_srv piper_msgs/srv/Enable '{enable_request: true}'

ros2 run robocup_real_manipulation phase1_pick green_apple \
  --config "$HOME/robot_config/robot.measured.yaml" \
  --already-docked --base-drive-isolated --input-durable
```

청사과만 한 개 놓고 시작한다. 이후 peach/tomato/fanta_can/cup으로 바꿀 수 있으나 해당 설정과 실제 인식이 검증되어 있어야 한다. 바나나는 [PCA·몸통 파지점과 활성화 방법](BANANA.md)을 확인한 뒤 banana로 실행한다. 알고리즘과 초기 치수는 반영됐으며 실제 수직 두께/그리퍼/scene 확인 전에는 기본 비활성화다.

이 명령이 실제 coarse를 선택하고 작업 ID를 만들며, 접근 완료/격리 조건을 운영자가 확인한 것으로 기록한다. 손목 결과·관절 피드백·TF를 합성하지 않는다. 물체 선택 뒤 **coarse 기반 탑다운 이동→실제 새 손목 관측→정밀 TCP→집기→옆10cm에 놓기→후퇴/복귀**가 이어진다.

## 터미널 4 — 진행 상태

```bash
source "$HOME/robot_config/phase1_env.sh"
ros2 topic echo /manipulation/status
```

성공은 place_success와 return_success가 모두 true인지 확인한다. 취소는 시험 명령의 터미널 3에서 Ctrl+C다. 취소 요청과 실제 정지는 다르므로 holding/stop 결과를 확인한다. 실패 후 journal을 삭제해 다시 시도하지 않는다. 다음 trial은 작업 종료 후 phase1.tuning.yaml에서 관측 높이/open/close/grip_height/속도를 수정하고 터미널 2를 재시작한다.

노트북 검사·설치와 실제 Jetson/로봇 검증은 별개다. 현재 실제 GPU 모델·카메라·Piper를 움직여 성공을 확인한 상태는 아니다.
