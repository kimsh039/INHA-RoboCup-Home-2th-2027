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
