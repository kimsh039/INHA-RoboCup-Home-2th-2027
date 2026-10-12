# INHA @HOME 웹 관제센터

**관제센터는 젯슨에서 실행합니다. 맥은 SSH 터널을 통해 `http://localhost:8081`에 접속합니다.** ROS, 센서 드라이버, SLAM, YOLO 추론, HTTP 서버는 모두 젯슨에서 동작하고 맥 브라우저는 화면을 표시합니다. 맥에 ROS/RViz, Python 서버, Node.js를 설치할 필요가 없습니다.

SLAM 지도·로봇 위치·2D 스캔·MID-360 점군·Head/Wrist 영상·검출 결과를 표시하며, 웹에서 Head D435, Wrist D435, YDLIDAR G2, MID-360 드라이버를 각각 켜고 끌 수 있습니다.

```text
Jetson / Ubuntu (sparo@192.168.50.70)
  센서 드라이버 ── ROS 2 ── SLAM / Head 검출
                     │
               관제 ROS bridge + HTTP server
                     127.0.0.1:8765
                         │ SSH 포트 포워딩
Mac                      │
  브라우저 http://localhost:8081
```

## 바로 실행하기

### 1. 젯슨: 저장소 업데이트

맥에서 SSH로 접속하거나 젯슨 터미널에서 실행합니다. 아래는 기존 저장소 위치가 `~/robocup`인 경우입니다. 다른 경로에 clone했다면 해당 경로에서 실행하세요.

```bash
ssh sparo@192.168.50.70
cd ~/robocup
git pull --ff-only origin main
```

처음 받는 젯슨에서는 대신 다음 명령을 사용합니다.

```bash
git clone https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027.git ~/robocup
cd ~/robocup
```

관제센터에 필요한 ROS/Python 패키지가 없다면 **젯슨에서 한 번** 설치합니다. 이 스크립트는 apt를 사용하므로 sudo 인증을 요청할 수 있습니다.

```bash
bash SW/control_center/setup-jetson.sh
```

기존 프로젝트의 센서 드라이버·CAN 설정·보정 파일·YOLO 환경은 그대로 사용합니다. 최초 하드웨어 설치가 안 된 젯슨에서는 [설치 안내](../setup/jetson/README_SETUP.md)와 [실물 로봇 안내](../robot/README.md)의 준비가 먼저 필요합니다. 관제센터 자체에는 npm 설치나 웹 빌드가 없습니다.

### 2. 젯슨: 관제센터 시작

기존 `robot_physical` 센서 tmux 세션을 사용하고 있었다면 **처음 전환할 때** 종료합니다. 관제센터가 각 센서 프로세스를 소유해야 웹의 개별 끄기 버튼을 사용할 수 있습니다. 다른 터미널에서 따로 실행한 동일 센서/SLAM/검출도 먼저 종료해 중복 실행을 피하세요.

```bash
bash SW/robot/jetson_sensors.sh stop
bash SW/control_center/jetson-center.sh start --mapping --detection
```

이 명령은 젯슨의 `inha_control_center` tmux 세션에 다음 프로세스를 실행하도록 요청합니다.

- 실물 TF·보정 스트림·Tracer 오도메트리·PiPER 관절 피드백
- 개별 Head/Wrist D435, G2, MID-360 드라이버
- `--mapping`: SLAM Toolbox (`/map`)
- `--detection`: Head YOLO/CSRT (`/detection/head/target`, `/detection/head/debug_image`)
- ROS 구독 bridge와 HTTP 서버 (`127.0.0.1:8765`)

차체와 팔의 상태 노드는 실행하지만, 이 관제센터는 주행 속도·접근·파지 명령을 발행하지 않습니다. 지도는 LiDAR 스캔과 오도메트리 입력이 들어올 때 생성됩니다.

이미 SLAM이나 검출을 다른 실행기로 켜 두었다면 해당 옵션을 생략합니다. 예를 들어 센서와 관제만 시작하려면:

```bash
bash SW/control_center/jetson-center.sh start
```

tmux 실행은 요청을 전달한 것입니다. 패키지 누락이나 장치 연결 실패가 있으면 출력/프로세스 로그에 기록됩니다. 출력은 다음 명령으로 볼 수 있습니다.

```bash
bash SW/control_center/jetson-center.sh logs
```

`Ctrl+B`, `D`로 tmux 화면에서 나오면 젯슨의 관제센터는 계속 실행합니다. tmux 안에서 `Ctrl+C`를 누르면 관제센터와 여기서 시작한 프로세스를 종료합니다.

### 3. 맥: 터널 연결 후 localhost 열기

**맥의 별도 터미널에서** 아래 명령을 실행하고 터미널을 유지합니다.

```bash
ssh -N \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:8081:127.0.0.1:8765 \
  sparo@192.168.50.70
```

맥 브라우저에서 **[http://localhost:8081](http://localhost:8081)**을 엽니다. 맥의 `localhost:8081`은 SSH 터널을 통해 젯슨의 `127.0.0.1:8765`에 연결됩니다. SSH 비밀번호는 터미널 프롬프트에만 입력합니다.

맥에도 이 저장소가 있다면 선택적으로 다음 실행기를 사용할 수 있습니다. 이 스크립트는 터널을 만들고 브라우저를 열며, 코드를 전송하거나 젯슨 서버를 시작하지 않습니다.

```bash
bash SW/control_center/connect.command
```

맥 SSH 터널을 Ctrl+C로 닫거나 브라우저를 닫아도 젯슨의 tmux 관제센터는 계속 실행합니다. 다시 터널을 연결하면 다시 접속할 수 있습니다.

## 센서 켜기 / 끄기

웹 상단의 **카메라 · LiDAR**에서 센서별 **켜기 / 끄기**를 누릅니다.

| 버튼 | 젯슨에서 관리하는 드라이버 | 기본 입력 |
|---|---|---|
| Head D435 | `realsense2_camera` / serial `244622071116` | `/head/camera/color/image_raw` |
| Wrist D435 | `realsense2_camera` / serial `243222070076` | `/wrist/wrist/color/image_raw` |
| YDLIDAR G2 | `ydlidar_ros2_driver` / G2.yaml | `/scan_raw` → 보정 `/scan` |
| Livox MID-360 | `livox_ros_driver2` / MID360s_config.json | `/livox/lidar` |

**끄기는 ROS 드라이버의 실행을 중지하는 기능이며 장치 전원 차단은 아닙니다.** 전원 차단에는 별도 전원 제어 장치가 필요합니다. 카메라를 끄면 영상/인지 입력이 멈추고 G2를 끄면 SLAM의 새 스캔 입력이 멈춥니다. 중지된 입력의 마지막 화면은 수신 지연으로 표시합니다.

실행 중인 프로세스 상태와 데이터 수신 상태를 따로 표시합니다. `드라이버 실행 중`이면서 `데이터 수신 대기`이면 프로세스는 있지만 센서 데이터가 들어오지 않는 상태입니다. 시작/중지 요청 처리 중에는 해당 버튼을 잠급니다. 끄기는 관제센터가 실행한 프로세스 그룹에 SIGINT를 보내고, 종료가 지연되면 SIGTERM/SIGKILL로 정리합니다. 다른 ROS 프로세스에 일괄 `pkill`을 사용하지 않습니다.

`외부 드라이버 실행 중`이면 기존 실행기가 관리하는 센서입니다. 중복 시작과 임의 종료를 막기 위해 버튼을 비활성화합니다. 위의 기존 실행 전환 절차를 사용하면 관제센터에서 개별 제어할 수 있습니다.

기존 `SW/robot/physical.launch.py`의 일반 실행은 센서가 모두 켜지는 기본값을 유지합니다. 관제센터는 다음 인자를 전달해 TF/odom/보정 노드와 센서 드라이버를 분리합니다.

```text
head_camera:=false wrist_camera:=false g2_lidar:=false mid360_lidar:=false
```

## 종료, 재시작, 업데이트

젯슨에서:

```bash
cd ~/robocup
bash SW/control_center/jetson-center.sh stop
# 기존 세션의 종료가 끝난 뒤 업데이트/재시작합니다.
git pull --ff-only origin main
bash SW/control_center/jetson-center.sh start --mapping --detection
```

`stop`은 종료 요청을 전달하며 종료 상태를 폴링하지 않습니다. 아직 같은 tmux 세션이 있으면 `start`는 중복 실행하지 않습니다. 웹 접속은 다시 SSH 터널을 열면 됩니다.

`config.json`은 Git에서 제외되어 장치 설정을 직접 관리할 수 있습니다. 소스 업데이트에는 `git pull --ff-only`를 사용하고, 로컬 변경 때문에 pull이 거절되면 변경을 보존한 채 처리하세요.

## 설정과 실행 옵션

젯슨에서 필요할 때:

```bash
cp SW/control_center/config.example.json SW/control_center/config.json
```

기존 `config.json`이 있다면 덮어쓰지 말고 필요한 키만 수정합니다. 설정이 없으면 `config.example.json`을 읽습니다. 주요 설정은 다음과 같습니다.

- `sensors`: 카메라 serial/FPS, G2 YAML, MID-360 설정 경로. `~`는 젯슨 사용자 홈으로 해석합니다.
- `channels`: 지도/영상/검출/점군의 자동 탐색 토픽 후보. UI의 **토픽 선택**은 현재 서버 세션에만 적용합니다.
- `fixed_frame` / `base_frame`: 기본 `map` / `base_link`.
- `image_hz` / `image_width` / `jpeg_quality`: 기본 6 FPS / 최대 너비 960 / 75.
- `cloud_hz` / `max_cloud_points`: 기본 2 Hz / 5,000개 샘플 점.

영상 전송 FPS와 모델 추론 FPS는 서로 다릅니다. 전체 포인트 처리와 추론은 젯슨에서 수행합니다.

같은 ROS_DOMAIN_ID를 사용합니다. 다른 domain은 젯슨에서 다음과 같이 시작하세요.

```bash
ROS_DOMAIN_ID=0 bash SW/control_center/jetson-center.sh start --mapping --detection
```

서버 포트를 바꿀 때는 양쪽 포워딩 값을 맞춥니다.

```bash
# Jetson
bash SW/control_center/jetson-center.sh start --port 8877
# Mac (선택 실행기)
ROBOCUP_REMOTE_PORT=8877 ROBOCUP_LOCAL_PORT=8082 bash SW/control_center/connect.command
```

`ROBOCUP_SSH_TARGET`은 다른 SSH 주소나 `~/.ssh/config` 별칭으로 바꿀 수 있습니다. HTTP와 터널은 기본 loopback으로 연결하며 인터넷 공개 서버를 만들지 않습니다. 제어 API에는 서버 세션 토큰과 같은 출처 요청 검사를 사용합니다.

직접 포그라운드로 실행할 수도 있습니다.

```bash
# Jetson
bash SW/control_center/run-robot.sh --with-robot --mapping --detection
```

기존 센서 실행을 유지하고 데이터만 받는 경우:

```bash
bash SW/control_center/jetson-center.sh start --observe
```

이 모드는 물리 bringup/센서를 자동 시작하지 않습니다. 외부 센서는 개별 종료할 수 없습니다. 시뮬레이션이나 bag의 `/clock`을 사용할 때는 `start --observe --sim-time`으로 실행합니다. 시뮬레이션에서 SLAM/검출도 이 실행기로 켜려면 `ROBOCUP_USE_SIM_TIME=true`를 적용하고 Head YAML 입력을 시뮬레이션 토픽으로 맞추세요.

## SLAM과 검출 준비

`--mapping`은 이 폴더의 `slam-web.yaml`을 사용합니다. 기존 `SW/simulation/ros2/slam_params.yaml`에서 가져온 설정에 실물 시간 `use_sim_time=false`를 적용했습니다. `/scan`, `odom → base_link` TF가 필요하며 물리 환경의 G2 필터링·SLAM 임계값은 환경에 맞춰 조정하세요. 다른 YAML은 `ROBOCUP_SLAM_PARAMS=/절대/경로.yaml`로 선택합니다.

`--detection`에는 기존 `robocup_head_detection`과 `robocup_detection_msgs`의 ROS 빌드, GPU YOLO 가상환경, 모델이 필요합니다. 패키지 설치가 아직 안 됐다면 기존 [Head 구현 안내](../detection/head/README.md)에 따라 설치하거나 젯슨에서 다음과 같이 해당 워크스페이스를 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/robocup/SW/detection/head/head_detection_ws
colcon build --symlink-install
```

관제 실행기는 `run-robot.sh`에서 ROS와 아래 overlay를 source합니다.

```text
~/tracer_ws/install/local_setup.bash
~/ws_livox/install/local_setup.bash
~/piper_ros/install/local_setup.bash
~/detection_ws/install/local_setup.bash
SW/detection/head/head_detection_ws/install/local_setup.bash
```

기본 모델은 `SW/detection/head/models/head_yolo11n_20261007.pt`, 기본 GPU Python은 `~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/python`입니다. 다른 환경은 시작 전에 지정합니다.

```bash
ROBOCUP_MODEL=/절대/경로/model.pt \
ROBOCUP_YOLO_PYTHON=/절대/경로/venv/bin/python \
ROBOCUP_DEVICE=0 \
bash SW/control_center/jetson-center.sh start --mapping --detection
```

Head의 보정 RGB를 `head-web.yaml`로 받아 검출 영상과 HeadTarget를 표시합니다. HeadTarget는 선택된 목표 **1개**의 상태입니다. 전체 검출 목록은 `yolo_msgs/DetectionArray` 또는 `vision_msgs/Detection2DArray` 토픽을 UI에서 선택합니다. 유효하지 않은 목표는 유효 검출로 계산하지 않으며 추적 프레임의 신뢰도는 마지막 YOLO 측정값으로 표시합니다.

Wrist SAM 실행은 기존 파이프라인을 사용합니다. `/detection/wrist/debug_image`, `/detection/wrist/status`, `/detection/wrist/object_cloud`를 발행하면 관제센터에 분할 영상·상태·물체 점군이 연결됩니다. Wrist RGB는 해당 분할 노드 없이도 볼 수 있습니다.

## 화면과 데이터 범위

| 화면 | 입력 | 표시 |
|---|---|---|
| 2D SLAM | `/map` OccupancyGrid | 원점·yaw·해상도, 확대/이동, 지도 PNG 다운로드 |
| 로봇 위치 | TF `map ← base_link`, odom fallback | 로봇 위치·방향·궤적·좌표계 |
| 2D LiDAR | `/scan`, `/scan_raw` | 메시지 시각의 TF로 변환한 스캔 |
| 경로 | `/plan` 등 Path | 마지막 수신 계획 |
| Head/Wrist 영상 | RGB / debug Image, CompressedImage | 원본/검출/분할 영상 전환 |
| 검출 | HeadTarget / DetectionArray / Detection2DArray | 클래스·신뢰도·유효성·추적 상태 |
| 3D 점군 | `/livox/lidar`, MID-360/Wrist PointCloud2 | 회전·확대, 환경/물체 점군 전환 |
| 운영 상태 | odom, `/rosout`, Linux 정보 | 속도·이벤트·CPU·RAM·온도 |

TF가 없는 점군은 원래 frame으로 표시합니다. 지도와 다른 frame의 odom 위치를 지도 위에 잘못 겹치지 않습니다. 3D 화면은 현재 PointCloud2를 샘플링한 것이며 누적 3D SLAM을 자체 계산하지 않습니다. 지도는 최대 2,048셀 변으로 줄여 표시하고 원본 크기를 따로 표시합니다.

연결이 끊기거나 수신이 오래되면 마지막 데이터임을 표시합니다. 지도·계획·물체 점군의 저장된 결과는 지속 발행 데이터와 구분합니다. PNG는 보기용 지도이며 Nav2의 YAML/PGM 저장을 대체하지 않습니다. 상태 JSON에는 토픽·frame·수신 나이 등이 포함됩니다.

RViz와 같은 종류의 관측 데이터를 웹에서 볼 수 있으며 RViz의 전체 플러그인이나 URDF 메시·MoveIt 조작 화면은 포함하지 않습니다. [Hexapod-Robot](https://github.com/twwawy/Hexapod-Robot)은 관제 화면 아이디어의 참고 저장소입니다. 이 관제센터의 구현은 INHA ROS 토픽과 센서 구성에 맞춘 별도 코드입니다.

## 문제 해결과 로그

- **localhost 접속 실패:** 젯슨 관제센터를 먼저 실행하고 맥의 SSH 터널 터미널을 유지하세요. 포트가 겹치면 양쪽 포트를 함께 바꿉니다.
- **SSH 인증 거절:** 일반 SSH 키/암호 인증을 터미널에서 설정합니다. 암호를 코드, 환경변수, Git에 넣지 않습니다.
- **센서 외부 실행 표시:** 기존 통합 센서 실행을 종료하고 관제센터 기본 `start`로 전환합니다.
- **프로세스는 있지만 영상/점군 없음:** serial, USB 장치 접근 권한, G2 장치 경로, MID-360 IP 설정과 로그를 확인합니다.
- **영상은 있고 지도 없음:** SLAM 실행 옵션, `/scan`, TF, ROS_DOMAIN_ID를 맞춥니다.
- **검출 없음:** Head package overlay, YOLO Python, 모델 경로, Head 입력 토픽을 맞춥니다.
- **센서를 껐는데 지도/영상이 남아 있음:** 마지막 수신값을 표시하는 것이며 화면의 수신 지연 상태를 함께 봅니다.

프로세스 로그는 젯슨의 `${XDG_STATE_HOME:-~/.local/state}/inha-control-center/<실행시각-PID>/`에 `head_camera.log`, `wrist_camera.log`, `g2_lidar.log`, `mid360_lidar.log`, `robot_core.log`, `mapping.log`, `head_detection.log`로 저장됩니다. 센서 카드의 설명에 마우스를 올리면 해당 로그 경로가 표시됩니다.

## 파일 구성

```text
SW/control_center/
├── jetson-center.sh           # 젯슨 tmux 실행/종료
├── run-robot.sh               # 젯슨 ROS 환경 + bridge
├── setup-jetson.sh            # 관제/SLAM apt 의존성 설치
├── connect.command           # 맥 SSH 터널 + 브라우저
├── ros_bridge.py             # ROS 구독/TF/영상/점군 변환
├── sensor_manager.py         # 개별 센서 프로세스 소유/제어
├── server.py                 # 표준 라이브러리 HTTP/SSE/MJPEG/API
├── config.example.json       # 토픽/센서 설정
├── start-mapping.sh · slam-web.yaml
├── start-head-detection.sh · head-web.yaml
└── static/                   # 빌드 없는 HTML/CSS/JS
```

선택적인 개발용 샘플은 `python3 server.py --demo --port 8080`으로 실행합니다. 실제 데이터가 아닌 SAMPLE MODE이며 센서 제어는 비활성화합니다. 실사용 흐름은 젯슨 서버와 맥의 SSH 터널입니다.

2026-10-12 작성. 사용자 요청에 따라 작업 후 테스트·빌드·린트·화면 검사·실기 검증은 수행하지 않았습니다. 원격 배포와 실제 센서 동작을 확인했다고 주장하지 않습니다.
