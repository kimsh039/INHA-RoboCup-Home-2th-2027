# INHA @HOME 웹 관제센터

2026-10-13: 기본 Head 화면은 최신 6클래스 모델의 **전체 객체 디텍션**이다.
ROI·단일 목표 추적을 기본 화면에서 사용하지 않는다. 대상 Jetson에 최신 FP16
TensorRT 엔진이 있으면 이를 우선 사용하고, 없으면 최신 `.pt`를 선택한다.
cup/banana 신뢰도는 각각 0.50, 나머지는 0.25다. 디텍션은 기본 활성화되며
`--no-detection`으로 끌 수 있다. [배포 설정](../detection/head/TENSORRT_JETSON.md).

**관제센터는 젯슨에서 실행합니다. 맥은 SSH 터널을 통해 `http://localhost:8081`에 접속합니다.** ROS, 센서 드라이버, SLAM, YOLO 추론, HTTP 서버는 모두 젯슨에서 동작하고 맥 브라우저는 화면을 표시합니다. 맥에 ROS/RViz, Python 서버, Node.js를 설치할 필요가 없습니다.

SLAM 지도·로봇 위치·2D 스캔·MID-360 점군·Head/Wrist 영상·검출 결과를 표시하며, 웹에서 Head D435, Wrist D435, YDLIDAR G2, MID-360 드라이버를 각각 켜고 끌 수 있습니다. 화면 상단의 **서버 종료**는 관제센터와 여기서 시작한 센서·SLAM·검출 프로세스를 함께 종료합니다. 센서 그룹에는 SIGINT, SIGTERM, SIGKILL 순서로 종료를 요청하며 SSH 터널은 그대로 유지됩니다. 다시 사용하려면 젯슨에서 `start`를 실행합니다.

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

### 3D 점군 표시 밀도

기본 MID-360 표시를 5 Hz, 입력 프레임당 최대 20,000점, 2초 누적(Decay Time), 점 크기 2 px로 설정했습니다. 누적 점군은 4 cm voxel로 중복을 줄이고 최대 80,000점으로 제한합니다. PointCloud2는 NumPy로 변환해 점 수 증가에 따른 Python 반복 비용을 줄입니다. 실제 화면 점 수는 센서 입력과 voxel 중복 제거에 따라 달라집니다.

`config.json`의 `cloud_hz`, `max_cloud_points`, `cloud_decay_sec`, `cloud_voxel_size`, `max_accumulated_cloud_points`, `cloud_point_size`로 조절하며 변경 후 관제센터를 재시작합니다. map 또는 odom TF를 해당 센서 timestamp에 조회한 뒤 누적하고, 변환이 없으면 원본 좌표의 현재 프레임만 표시하며 누적 중지 이유를 화면에 표시합니다. Wrist 물체 점군은 현재 관측만 표시합니다.

### 디텍션 업데이트와 MID-360 수신 복구

`--detection`은 현재 checkout의 `robocup_head_detection.backends.YoloDetector`와 클래스 변환을 직접 불러옵니다. 이전 단일 목표 YOLO/CSRT 실행 대신 매 프레임 전체 객체를 검출하고 `/detection/head/objects` (`std_msgs/String` JSON), `/detection/head/debug_image`를 발행합니다. 화면에는 green_apple, peach, tomato, fanta_can, cup, banana의 모든 박스와 Camera/YOLO FPS, 추론 시간을 표시합니다. 기존 7출력 체크포인트는 새 소스의 클래스 변환을 사용하며 가중치를 새로 학습한 것은 아닙니다. 이 관제용 실행은 커스텀 메시지 빌드 없이 기존 GPU venv를 사용하고, pull 후 관제센터 재시작으로 소스 변경을 반영합니다. `/detection/head/target` 추적기가 필요한 별도 접근 시스템은 해당 실행기를 따로 사용합니다.

MID-360 로그에 `host IP 없음: 192.168.1.5`가 나오면 Livox 전용 Ethernet 프로필을 활성화해야 합니다. Wi-Fi의 `192.168.50.70`은 MID-360 설정의 수신 주소를 대신하지 않습니다. 아래 명령은 CAN 등록과 별도로 Ethernet만 설정하므로 팔 USB CAN이 없어도 사용할 수 있습니다. sudo 인증은 젯슨 터미널에서 입력합니다.

```bash
bash ~/robocup/SW/control_center/jetson-center.sh kill
bash ~/robocup/SW/control_center/setup-lidar.sh &&
ROBOCUP_YOLO_PYTHON="$HOME/venvs/head_detection/bin/python" \
bash ~/robocup/SW/control_center/jetson-center.sh start --mapping --detection
```

회전은 보정된 `/imu/base`가 신선하게 들어올 때 MID-360 gyro를 사용합니다. IMU 시작 전·중지·수신 지연에는 `/odom`의 바퀴 각속도만 `/odom/rotation_fallback`으로 전달해 EKF 회전 입력을 이어갑니다. 대체 입력에는 유한 공분산을 넣고, gyro 수신이 돌아오면 발행을 중지합니다. 바퀴 회전 입력은 IMU 대체이며 같은 정확도를 보장하지 않습니다. CAN·오도메트리 패널에 바퀴/융합 yaw·각속도와 현재 회전 소스를 표시합니다.

Humble SLAM Toolbox의 translation gate가 제자리 회전 스캔을 버리지 않도록 `minimum_travel_distance: 0.0`을 사용합니다. 스캔 처리는 최소 0.5초 간격, 지도 출력은 1초 간격이며 정지 중에도 스캔 처리량이 증가할 수 있습니다. 다른 `ROBOCUP_SLAM_PARAMS` 파일을 지정하면 그 파일의 설정이 우선합니다.

### 현재 젯슨의 최초 설정과 재부팅 후 실행

Tracer와 팔의 전원을 켜고 각각의 USB CAN 어댑터를 젯슨에 연결합니다. USB CAN은 총 두 개여야 하며 내장 `can0`/`can1`은 이 개수에 포함하지 않습니다. MID-360 Ethernet과 사용할 카메라·G2도 연결합니다.

이번 복구에서는 젯슨 터미널에서 한 번 아래를 실행합니다. 기존 관제센터·통합 센서·`tracer` tmux 세션과 그 하위 프로세스를 먼저 정리하고, 두 USB CAN의 시리얼 고정·bitrate와 Livox 전용 Ethernet 설정을 저장합니다. sudo 비밀번호는 터미널에서 입력합니다.

```bash
bash ~/robocup/SW/control_center/setup-hardware.sh
```

USB CAN 한 개만 감지됐다는 오류가 나오면 서버 시작으로 넘어가지 않습니다. 팔 USB가 Linux CAN 인터페이스로 등록돼야 두 장치의 역할을 고정할 수 있습니다. 종료할 세션이 없다는 메시지는 정상이며 설정을 계속합니다.

설정이 성공한 뒤, 그리고 다음 재부팅부터는 젯슨에서 다음 순서로 실행합니다. `kill-can-users`는 알려진 세 세션만 정리하며 다른 tmux 세션을 일괄 종료하지 않습니다.

```bash
bash ~/robocup/SW/control_center/jetson-center.sh kill-can-users &&
ROBOCUP_YOLO_PYTHON="$HOME/venvs/head_detection/bin/python" \
bash ~/robocup/SW/control_center/jetson-center.sh start --mapping --detection
```

젯슨에서 서버 준비 완료 메시지가 나오면 맥의 별도 터미널에서 터널을 연결하고, 브라우저에서 `http://localhost:8081`을 엽니다. 맥의 SSH 터널 터미널은 계속 열어 둡니다.

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8081:127.0.0.1:8765 sparo@192.168.50.70
```

재부팅 후에는 위 서버 실행과 맥의 터널 연결을 반복합니다. 어댑터를 교체하거나 하드웨어 설정이 바뀐 경우에는 최초 설정을 다시 실행합니다.

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
- `--detection`: 현재 Head 6종 전체 객체 YOLO (`/detection/head/objects`, `/detection/head/debug_image`)
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

## CAN 자동 연결과 오도메트리 상태

기본 `start`는 `config.json`에 지정한 두 CAN 인터페이스를 살펴보고, 내려간 CAN 인터페이스의 연결과 BUS-OFF 복구를 자동 요청합니다. Tracer와 팔은 각각 USB CAN을 사용하며, Tracer는 `can_tracer`(500 kbps), PiPER는 `can_arm`(1 Mbps)로 구분합니다. 내장 `can0`/`can1`을 USB 어댑터 대신 배정하지 않습니다. `can.tracer.usb_serial`, `can.piper.usb_serial`로 각 어댑터를 확인하며, 다른 어댑터가 해당 이름을 사용 중이면 피드백을 시작하지 않습니다. 최초 설정은 두 USB CAN을 연결하고 관제센터와 기존 센서 실행을 종료한 뒤 실행합니다. `setup-jetson.sh`에도 이 준비가 포함됩니다.

```bash
bash SW/control_center/setup-can.sh
```

이 스크립트는 두 USB CAN의 고유 시리얼로 udev 이름 고정 규칙과 CAN 전용 systemd-networkd 설정을 저장하고, 현재 인터페이스도 이름과 bitrate를 적용합니다. 모든 `gs_usb` 장치에 같은 `can_arm` 이름을 주던 기존 `/etc/udev/rules.d/80-can-usb.rules`는 장치별 규칙으로 교체합니다. 현재 Tracer 어댑터 시리얼은 `003500294656530B20353433`이며, 팔 시리얼이 아직 비어 있을 때는 정확히 두 USB CAN 중 Tracer와 다른 어댑터를 팔로 등록합니다. 세 개 이상, 시리얼 중복/누락, 등록된 장치 미발견은 오류로 표시합니다. USB CAN이 한 개만 보이면 설정을 중단하며 내장 CAN을 대신 사용하지 않습니다. 이전 규칙·네트워크 설정·등록 파일은 `/var/backups/inha-control-center/`에 보관합니다.

다른 어댑터를 쓰는 설치에서는 `setup-can.sh --tracer-serial SERIAL --piper-serial SERIAL`로 각 역할을 지정합니다. 현재 Linux에 등록된 CAN의 USB 경로와 시리얼은 `/usr/bin/python3 SW/control_center/can_devices.py`로 출력할 수 있습니다. 시리얼로 고정하므로 USB 연결 순서나 포트를 바꿔도 등록된 역할을 유지합니다.

MID-360의 `bind failed`까지 함께 복구하려면 다음을 실행합니다. 관제센터의 소유 프로세스를 먼저 종료하고 현재 CAN 설정과 Livox JSON의 host IP를 적용합니다. 기본 Livox 전용 Ethernet 프로필은 `livox`, 인터페이스는 `eno1`입니다. `sensors.mid360_lidar`의 `connection`, `interface`, `prefix`로 바꿀 수 있습니다. Wi-Fi 연결을 수정하지 않으며 Livox Ethernet은 기본 경로로 사용하지 않습니다.

```bash
bash SW/control_center/setup-hardware.sh
ROBOCUP_YOLO_PYTHON="$HOME/venvs/head_detection/bin/python" \
bash SW/control_center/jetson-center.sh start --mapping --detection
```

센서 TF, 보정 `/scan`, 카메라 보정, IMU relay, EKF는 `sensor_core`로 CAN 상태와 관계없이 시작합니다. 차체와 팔 피드백은 각 CAN의 준비 상태에 따라 독립적으로 시작합니다. 지도 생성에는 실제 차체 odom과 G2 스캔이 필요합니다. MID-360 시작 시 host IP와 UDP 포트를 점검하며 주소 누락·포트 점유·SDK 초기화 실패를 센서 오류와 로그에 표시합니다.

웹의 **CAN 연결 · 오도메트리** 패널에 다음을 각각 표시합니다.

- CAN 인터페이스/bitrate/bus 상태, RX 누적 프레임 수와 실제 최근 수신 여부
- 바퀴 `/odom`과 융합 `/odom_ekf`의 Hz, 속도, 최근 수신 및 원본 timestamp 지연
- `odom → base_link` TF의 수신 지연

최대 3초 이내 CAN 프레임 증가와 바퀴/융합 odom의 유효한 새 timestamp, 최신 TF가 모두 있을 때 **정상 수신**을 표시합니다. 정지 상태의 속도 0도 정상입니다. CAN 인터페이스 UP만으로 정상 수신으로 표시하지 않으며, NaN/Inf, 잘못된 좌표계, 멈춘 timestamp, BUS-OFF 등을 구분합니다. 이 표시는 데이터 수신 상태이며 실제 주행 거리 정확도나 CAN 발신 장치의 신원을 측정한 결과는 아닙니다.

차체와 팔 피드백은 각 CAN의 준비 상태에 따라 실행합니다. 한쪽만 연결돼도 해당 피드백을 시작합니다. 연결이 끊기면 관제센터가 시작한 피드백 그룹만 중지하고, 다시 연결되거나 장치 번호가 바뀌면 새 그룹을 시작합니다. 센서 TF와 보정은 독립적으로 유지합니다. 주행 속도·팔 이동 명령을 보내는 동작 확인은 하지 않습니다.

`--observe`에서는 CAN과 odom을 관측만 하고 CAN 설정이나 피드백 프로세스를 바꾸지 않습니다. `config.json`의 `can.auto_connect=false`는 CAN 링크 변경 요청만 비활성화하며, 기본 `start`의 피드백 관리 기능은 유지합니다. 사용 중인 인터페이스의 bitrate가 다르면 임의로 내려 재설정하지 않고 오류를 표시합니다. 권한 부족/장치 누락/실행 실패는 패널과 이벤트에 표시되고 재시도합니다.

코드를 갱신한 뒤 관제센터를 stop → start로 재시작하면 적용됩니다. 이번 변경 후 테스트·빌드·브라우저/실기 검증은 실행하지 않았습니다.

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

`stop`은 종료 요청을 전달하며 종료 상태를 폴링하지 않습니다. 아직 같은 tmux 세션이 있으면 `start`는 중복 실행하지 않습니다. 바로 강제 종료하고 다시 시작하려면 아래를 사용하세요.

```bash
bash SW/control_center/jetson-center.sh kill
ROBOCUP_YOLO_PYTHON="$HOME/venvs/head_detection/bin/python" \
bash SW/control_center/jetson-center.sh start --mapping --detection
```

`kill`은 관제센터 tmux 세션의 프로세스 트리를 중지해 자동 재실행을 막은 후 SIGKILL로 정리하고 세션을 제거합니다. 별도 프로세스 그룹으로 띄운 센서·SLAM·검출도 함께 대상에 포함됩니다. 다른 터미널에서 실행한 ROS 프로세스에는 신호를 보내지 않습니다. 강제 종료이므로 해당 프로세스의 정상 종료 처리와 기록 저장은 생략됩니다. 웹 접속은 다시 SSH 터널을 열면 됩니다.

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

## 관측 · SLAM · NAV 모드

화면 위쪽 탭(또는 왼쪽 레일)에서 모드를 고릅니다. 지도·카메라·검출 화면은 모든 모드에서 같고, 위쪽 제어 패널만 바뀝니다.

| 모드 | 제어 | 동작 |
|---|---|---|
| **관측** | 카메라·LiDAR 켜기/끄기, CAN·오도메트리 | 기존 화면 |
| **SLAM** | SLAM 시작 · 중지 · **초기화** · 저장 · 저장된 지도 | `start-mapping.sh`(SLAM Toolbox)를 관제센터가 소유해 실행합니다. **초기화**는 SLAM 프로세스를 다시 시작해 현재 지도를 지우고 지금 위치를 원점으로 새로 만듭니다. **저장**은 `save-map.sh`로 `~/maps/<이름>.{pgm,yaml}`(Nav2 map_server)과 `.posegraph/.data`(SLAM Toolbox 위치추정)를 남깁니다. |
| **NAV** | 위치추정 시작 · 초기 위치 지정 · Nav2 시작/중지 · 목표 지정 · 이동 취소 | 저장 지도 위치추정은 `start-localization.sh`(SLAM Toolbox localization)입니다. `start-navigation.sh`가 Mid-360S 자기 점 필터(`/livox/lidar` → `/mid360/points_filtered`)와 Nav2(`nav2-web.yaml`)를 올립니다. **목표 지정**/**초기 위치 지정**은 2D 지도에서 클릭한 뒤 드래그해 방향을 정합니다(Esc 취소). |

- API: `POST /api/mapping {action: start|stop|reset|localize|save, map?, name?}`, `POST /api/navigation {action: start|stop|goal|cancel|initial_pose, x?, y?, yaw?}` (map 좌표, `X-Control-Token` 필요).
- Nav2가 실행 중이면 SLAM 중지·초기화·위치추정 전환을 막습니다(목표 좌표계가 바뀌지 않게). 먼저 Nav2를 중지합니다.
- Nav2 설치(최초 1회): `sudo apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup`
- 실물 Nav2 설정 `nav2-web.yaml`: 최대 0.25 m/s · 0.6 rad/s, 가속 0.5 m/s²([Tracer odom 보정](../../HW/calibration/physical/tracer_odom/README.md) 권장), 목표 허용 0.15 m · 0.2 rad(정밀 접근은 dock 노드), Mid-360S 장애물 높이 바닥 위 0.11–1.39 m. Tracer가 리모컨(RC) 모드면 속도 명령이 적용되지 않습니다.
- 3D 점군과 2D 스캔은 메시지 시각의 `map ← 센서` TF로 변환합니다. SLAM의 `map → odom`은 스캔 주기로만 갱신되어 정확한 시각 조회가 가끔 실패하므로, 그때는 최신 `map → odom`과 메시지 시각의 `odom → 센서`를 합성해 `map` 좌표를 유지합니다.

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

기본 모델은 `SW/detection/head/models/head_yolo11n_20261013.pt` (native 6클래스, cup=0.70 / 나머지=0.25), 기본 GPU Python은 `~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/python`입니다. 다른 환경은 시작 전에 지정합니다.

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

`start`는 최대 30초 동안 ROS bridge의 서버 준비 완료 메시지를 기다립니다. 기동 중 종료되면 오류와 시작 로그 경로를 터미널에 출력합니다. 전체 시작 출력은 `~/.local/state/inha-control-center/startup/`에 저장합니다 (`XDG_STATE_HOME`이 있으면 그 경로 사용). 최근 시작 오류는 다음 명령으로 볼 수 있습니다.

```bash
bash SW/control_center/jetson-center.sh errors
```

SSH 터널에서 `open failed: connect failed: Connection refused`가 나오면 터널의 원격 주소/포트에서 HTTP 서버가 연결을 받지 못한 것입니다. 젯슨의 시작 로그로 기동 오류를 파악하세요. 직접 기동 오류를 보려면 기존 관제센터를 `kill`로 종료한 뒤 젯슨 터미널에서 `bash SW/control_center/run-robot.sh --with-robot --mapping --detection`을 실행하고 그 터미널을 유지합니다.

- **localhost 접속 실패:** 젯슨 관제센터를 먼저 실행하고 맥의 SSH 터널 터미널을 유지하세요. 포트가 겹치면 양쪽 포트를 함께 바꿉니다.
- **SSH 인증 거절:** 일반 SSH 키/암호 인증을 터미널에서 설정합니다. 암호를 코드, 환경변수, Git에 넣지 않습니다.
- **센서 외부 실행 표시:** 기존 통합 센서 실행을 종료하고 관제센터 기본 `start`로 전환합니다.
- **프로세스는 있지만 영상/점군 없음:** serial, USB 장치 접근 권한, G2 장치 경로, MID-360 IP 설정과 로그를 확인합니다.
- **영상은 있고 지도 없음:** SLAM 실행 옵션, `/scan`, TF, ROS_DOMAIN_ID를 맞춥니다.
- **검출 없음:** Head package overlay, YOLO Python, 모델 경로, Head 입력 토픽을 맞춥니다.
- **센서를 껐는데 지도/영상이 남아 있음:** 마지막 수신값을 표시하는 것이며 화면의 수신 지연 상태를 함께 봅니다.

프로세스 로그는 젯슨의 `${XDG_STATE_HOME:-~/.local/state}/inha-control-center/<실행시각-PID>/`에 `head_camera.log`, `wrist_camera.log`, `g2_lidar.log`, `mid360_lidar.log`, `sensor_core.log`, `robot_core.log`, `mapping.log`, `head_detection.log`로 저장됩니다. 센서 카드의 설명에 마우스를 올리면 해당 로그 경로가 표시됩니다.

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
├── start-mapping.sh · slam-web.yaml · start-localization.sh · save-map.sh
├── start-navigation.sh · nav2-web.yaml
├── start-head-detection.sh · head-web.yaml
└── static/                   # 빌드 없는 HTML/CSS/JS
```

선택적인 개발용 샘플은 `python3 server.py --demo --port 8080`으로 실행합니다. 실제 데이터가 아닌 SAMPLE MODE이며 센서 제어는 비활성화합니다. 실사용 흐름은 젯슨 서버와 맥의 SSH 터널입니다.

2026-10-12 작성. 사용자 요청에 따라 작업 후 테스트·빌드·린트·화면 검사·실기 검증은 수행하지 않았습니다. 원격 배포와 실제 센서 동작을 확인했다고 주장하지 않습니다.
