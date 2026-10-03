# RoboCup 로봇 시뮬레이션

원본 Tracer·Piper·센서 랙 모델과 메시는 `../HW/URDF/`에 보관하고,
이 폴더에는 결합 모델, Gazebo 설정과 제어 도구를 관리합니다.
저장소 전체를 받아야 원본 메시 상대 경로가 유효합니다.

```text
simulation/
├── robot_description/           # 통합 URDF 및 조립 스크립트
│   └── sensor_rack_piper/       # 랙 + 팔 중간 조립 모델
├── gazebo/                      # 센서 설정, world 생성 및 실행
│   └── build/                   # 로컬 생성 world/URDF (Git 제외)
├── ros2/                        # ROS 2 런치, 브리지 설정, RViz 설정
├── tools/                       # 웹·터미널 제어, 카메라, 센서 검증
└── docs/                        # 조립 기준 및 센서 사양
```

## Gazebo 실행

Gazebo Harmonic과 Python `gz.transport13`, `gz.msgs10`을 사용합니다.
아래 명령은 저장소 루트에서 실행합니다.

```bash
cd simulation
./gazebo/start_sim.sh
```

이 스크립트는 URDF/world를 재생성하고 Gazebo와 웹 제어 화면
`http://127.0.0.1:8081`을 실행합니다. partition은 `robocup_motion`이며 통신은 기본적으로 이 PC의 `127.0.0.1`로 제한합니다.
NVIDIA 드라이버가 활성화되어 있으면 실행 스크립트가 PRIME offload를 자동 설정합니다. 명시적으로 지정하려면 다음처럼 실행합니다.

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only ./gazebo/start_sim.sh
```

## 터미널 제어와 카메라

다른 터미널에서 `simulation/` 폴더로 이동한 뒤 실행합니다.

```bash
python3 tools/robotctl.py --partition robocup_motion teleop
python3 tools/robotctl.py --partition robocup_motion joint 1 0.5
python3 tools/robotctl.py --partition robocup_motion grip 0.03
python3 tools/robotctl.py --partition robocup_motion home
python3 tools/camera_view.py --partition robocup_motion
```

키보드 주행: W/S 전후진, A/D 회전, Space/X 정지, Q 종료.
키 입력이 0.4초 없으면 정지합니다. `home`은 팔·그리퍼 영점 복귀와 주행 정지이며 차체 위치를 초기화하지 않습니다.
카메라는 GTK3(PyGObject), Pillow, NumPy가 필요하며 RGB/뎁스 팝업을 각각 띄웁니다.

현재 PC의 `robotctl`, `robot-camera` 명령도 새 `tools/` 위치를 참조합니다.
인자를 생략하면 현재 NVIDIA 미리보기 partition `robocup_projectsh_sensor_view_20261003`에 연결합니다
(`GZ_PARTITION` 환경변수가 있으면 그 값 우선).

## ROS 2 시뮬레이션

`ros2/sim.launch.py` 하나로 URDF/SDF 재생성, Gazebo 실행, ROS 2 브리지, `robot_state_publisher`, RViz를 함께 띄웁니다.
`GZ_PARTITION=robocup_motion`, `GZ_IP=127.0.0.1`, `use_sim_time`은 런치 파일 안에서 설정합니다. 같은 partition을 쓰므로 `gazebo/start_sim.sh`와 동시에 실행하지 마세요. 아래 명령은 저장소 루트에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 launch simulation/ros2/sim.launch.py world:=room
# 다른 터미널에서 주행
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `world` | `empty` | `empty`: 바닥만, `room`: 아래 테스트 방 |
| `gui` | `true` | `false`면 Gazebo 서버만 실행 |
| `rviz` | `true` | `false`면 RViz 생략 |

| ROS 2 토픽 | Gazebo 토픽 | 방향 |
|---|---|---|
| `/clock` | `/world/robocup_motion/clock` | GZ → ROS |
| `/cmd_vel` | `/robocup/cmd_vel` | ROS → GZ |
| `/odom` | `/robocup/odometry` | GZ → ROS |
| `/tf` | `/model/robocup/tf` (DiffDrive `odom` → `base_link`) | GZ → ROS |
| `/joint_states` | `/robocup/joint_states` | GZ → ROS |
| `/scan` | `/robocup/g2/scan` (`laser_frame`, 10 Hz, 500 rays, 0.12~12 m) | GZ → ROS |

TF는 `odom` → `base_link`(DiffDrive)와 `base_link` 이하 URDF 트리(`robot_state_publisher`)로 이어집니다. RViz Fixed Frame은 `odom`입니다.
주행 시험 결과 `/cmd_vel` 0.3 m/s를 3초 보냈을 때 odom이 0.90 m 전진했습니다.

### 테스트 방 (`world:=room`)

좌표는 world 기준이며 로봇은 원점에서 +X를 보고 시작합니다. 모든 물체는 static box입니다.

| 대상 | 중심 (x, y, z) m | 크기 (X × Y × Z) m | 비고 |
|---|---|---|---|
| 방 내부 | (0, 0) | 6.0 × 4.0 | 벽 안쪽 면 기준 x ±3.0, y ±2.0 |
| 북쪽 벽 | (0, 2.05, 0.5) | 6.2 × 0.1 × 1.0 | 두께 0.1, 높이 1.0 |
| 남쪽 벽 | (0, −2.05, 0.5) | 6.2 × 0.1 × 1.0 | |
| 동쪽 벽 | (3.05, 0, 0.5) | 0.1 × 4.0 × 1.0 | |
| 서쪽 벽 | (−3.05, 0, 0.5) | 0.1 × 4.0 × 1.0 | |
| 칸막이 | (−1.5, 1.25, 0.5) | 0.1 × 1.5 × 1.0 | 북쪽 벽에서 y=0.5까지, 방의 대칭을 깨서 스캔 매칭 모호성 방지 |
| 테이블 상판 | (1.8, −1.0, 0.73) | 1.2 × 0.8 × 0.04 | 윗면 높이 0.75 |
| 테이블 다리 ×4 | (1.8 ± 0.525, −1.0 ± 0.325, 0.355) | 0.05 × 0.05 × 0.71 | 가장자리에서 0.05 안쪽 |

G2 스캔 평면은 바닥에서 약 0.49 m(`base_link` 0.1425 m + 0.343 m)입니다. 따라서 2D 스캔에는 테이블 상판이 아니라 **다리 4개만** 점으로 잡힙니다. 상판 아래 공간을 Nav2가 빈 곳으로 판단할 수 있으므로, 3D 센서(Mid-360S, D435f)를 costmap에 넣기 전까지는 테이블 주변 주행에 주의해야 합니다.
빈 world에서도 G2 ray 약 13%가 랙 기둥에 0.20~0.28 m로 닿습니다([SENSORS.md](docs/SENSORS.md#확인-결과-2026-10-03)). SLAM/Nav2의 최소 거리는 약 0.3 m로 두세요.

## 모델 재생성과 센서 검사

```bash
python3 robot_description/build.py
python3 gazebo/make_sim.py --test-wall
GZ_IP=127.0.0.1 GZ_PARTITION=robocup_sensor_test gz sim -s -r gazebo/build/motion.world.sdf
# 별도 터미널, simulation/에서 실행
GZ_PARTITION=robocup_sensor_test python3 tools/check_sensors.py --wall
```

통합 URDF는 상대 메시 경로를 사용해 Git에 저장합니다. 실행용 world와 절대 경로 URDF,
Python 캐시, 로그는 Git에서 제외합니다.

- [조립 위치·좌표 및 제어 상세](docs/ASSEMBLY.md)
- [센서 사양·토픽·모델 제한·검증](docs/SENSORS.md)
- [랙 + Piper 중간 모델](robot_description/sensor_rack_piper/README.md)
