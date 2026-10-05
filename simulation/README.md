# RoboCup 시뮬레이션

> **Gazebo·MuJoCo 개발 PC용 안내입니다.** 현재 Jetson의 실기 setup과 카메라 구성은 [프로젝트 README](../README.md)와 [Jetson 운영 문서](../setup/jetson/README.md)를 참고하세요. Jetson에는 Gazebo 및 robot-* 단축 명령을 설치하지 않았습니다.

최종 URDF는 [robot_description/robocup.urdf](robot_description/robocup.urdf) 하나입니다.
Tracer, 프로파일 랙, Piper·그리퍼, G2, Mid-360S, 헤드 D435f와 손목 D435f·마운트를 포함합니다.
원본 메시·CAD·물성 자료는 `../HW/URDF/`에 유지하며 예전 독립·중간 URDF와 조립 생성기는 삭제했습니다.
URDF의 상대 메시 경로가 유효하도록 저장소 전체를 사용하세요.

팔의 기본 자세는 원본 Piper의 1번 관절을 1.6 rad(약 91.7°) 돌린 모습입니다.
이 자세가 최종 URDF의 관절 영점이므로 처음부터 모든 팔 관절값이 0인 상태로 생성되며,
`home`과 웹·조이스틱 HOME도 이 자세로 돌아옵니다. 좌표와 가동 범위는 [모델 안내](robot_description/README.md)를 참고하세요.

| 폴더 | 내용·사용법 |
|---|---|
| [robot_description](robot_description/README.md) | 최종 모델과 좌표·물성 |
| [mujoco](mujoco/README.md) | 손목 탑다운 관측·GraspNet·RViz·PiPER 물리 픽앤플레이스 튜토리얼 |
| [gazebo](gazebo/README.md) | GPU 실행, 빈 월드·책상/벽 월드, 센서 실시간 수신 |
| [tools](tools/README.md) | 조이스틱·터미널 제어·카메라 팝업 |
| [ros2](ros2/README.md) | ROS 2 브리지·RViz·토픽 |
| [docs/SENSORS.md](docs/SENSORS.md) | 센서 사양·한계·검증 |
| [docs/ASSEMBLY.md](docs/ASSEMBLY.md) | 조립 위치·좌표 |

## Gazebo 빠른 실행

저장소 루트에서 실행합니다. 실행 중인 기존 Gazebo는 먼저 종료합니다.

```bash
./simulation/gazebo/start_sim.sh --world room
```

다른 터미널에서 컨트롤러와 카메라를 각각 실행합니다.

```bash
robot-joystick --partition robocup_motion
robot-camera --partition robocup_motion --camera head
robot-camera --partition robocup_motion --camera wrist
```

위 별칭은 기존 Gazebo 개발 PC에서 사용한 단축 명령입니다. 다른 PC에서는 [tools 안내](tools/README.md)의 Python 명령을 사용합니다.
손목 카메라는 항상 포함되므로 이전 `--with-wrist-camera`, `wrist_camera:=true` 옵션은 사용하지 않습니다.
