# RoboCup 시뮬레이션

> **Gazebo·MuJoCo 개발 PC용 안내입니다.** 현재 Jetson의 실기 setup과 카메라 구성은 [프로젝트 README](../README.md)와 [Jetson 운영 문서](../setup/jetson/README.md)를 참고하세요. Jetson에는 Gazebo 및 robot-* 단축 명령을 설치하지 않았습니다.

**현재 최종 보정 모델은 [robot_description/robocup.calibrated.urdf](robot_description/robocup.calibrated.urdf)입니다.** 2026-10-05 Base–2D LiDAR, 2026-10-06 Base–Mid360 보정 결과를 최신 랙·PiPER 구조에 적용해 2026-10-06 Git에 업로드했습니다. [보정·변경·업로드 이력](robot_description/README.md#모델-변경보정업로드-이력)에 날짜·커밋·평가 보고서를 기록합니다. [robot_description/robocup.urdf](robot_description/robocup.urdf)는 CAD 기준 원본이며 Gazebo 생성기의 기본 입력으로 유지합니다.
Tracer, 프로파일 랙, Piper·그리퍼, G2, Mid-360S, 헤드 D435f와 손목 D435f·마운트를 포함합니다.
원본 메시·CAD·물성 자료는 `../HW/URDF/`에 유지하며 예전 독립·중간 URDF와 조립 생성기는 삭제했습니다.
URDF의 상대 메시 경로가 유효하도록 저장소 전체를 사용하세요.

팔의 기본 자세는 원본 Piper의 1번 관절을 1.6 rad(약 91.7°) 돌린 모습입니다.
이 자세가 최종 URDF의 관절 영점이므로 처음부터 모든 팔 관절값이 0인 상태로 생성되며,
`home`과 웹·조이스틱 HOME도 이 자세로 돌아옵니다. 좌표와 가동 범위는 [모델 안내](robot_description/README.md)를 참고하세요.

| 폴더 | 내용·사용법 |
|---|---|
| [robot_description](robot_description/README.md) | 최종 모델과 좌표·물성 |
| [mujoco](mujoco/README.md) | 최신 nominal URDF의 손목 관측·물리 파지·배치·시작 자세 복귀 검증 |
| [gazebo](gazebo/README.md) | GPU 실행, 빈 월드·책상/벽 월드, 센서 실시간 수신 |
| [tools](tools/README.md) | 조이스틱·터미널 제어·카메라 팝업 |
| [ros2](ros2/README.md) | ROS 2 브리지·RViz·토픽 |
| [docs/SENSORS.md](docs/SENSORS.md) | 센서 사양·한계·검증 |
| [docs/ASSEMBLY.md](docs/ASSEMBLY.md) | 조립 위치·좌표 |

## Gazebo 빠른 실행

저장소 루트에서 실행합니다. 실행 중인 기존 Gazebo는 먼저 종료합니다.

```bash
./SW/simulation/gazebo/start_sim.sh --world room
```

다른 터미널에서 컨트롤러와 카메라를 각각 실행합니다.

```bash
robot-joystick --partition robocup_motion
robot-camera --partition robocup_motion --camera head
robot-camera --partition robocup_motion --camera wrist
```

위 별칭은 기존 Gazebo 개발 PC에서 사용한 단축 명령입니다. 다른 PC에서는 [tools 안내](tools/README.md)의 Python 명령을 사용합니다.
손목 카메라는 항상 포함되므로 이전 `--with-wrist-camera`, `wrist_camera:=true` 옵션은 사용하지 않습니다.

## 캘리브레이션 — 측정·적용·정확도 평가

**2D/3D LiDAR 보정을 쓰려면 `robocup.calibrated.urdf`를 읽는 `calibration_runtime.launch.py`를 실행합니다.** [보정 URDF 적용하기](robot_description/README.md#보정-urdf-적용하기)에 Ubuntu/Mac 전체 명령, RViz 표시, TF 수치 읽기, 기존 TF와 중복하지 않는 방법을 적었습니다. 일반 `sim.launch.py`와 Gazebo world 생성기는 기본으로 `robocup.urdf`를 읽으므로 이 파일 선택을 자동으로 대신하지 않습니다.

전체 센서 보정 순서·현재 계산값·정확도 결과·저장 위치는 [Calibration 전체 과정과 결과](calibration/README.md)에 정리했습니다. Mid360 측정·로봇 이동·독립 정확도 평가는 [Ubuntu 상세 실행 가이드](calibration/BASE_MID360.md)에서 진행합니다.
