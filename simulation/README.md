# RoboCup 시뮬레이션

최종 URDF는 [robot_description/robocup.urdf](robot_description/robocup.urdf) 하나입니다.
Tracer, 프로파일 랙, Piper·그리퍼, G2, Mid-360S, 헤드 D435f와 손목 D435f·마운트를 포함합니다.
원본 메시·CAD·물성 자료는 `../HW/URDF/`에 유지하며 예전 독립·중간 URDF와 조립 생성기는 삭제했습니다.
URDF의 상대 메시 경로가 유효하도록 저장소 전체를 사용하세요.

| 폴더 | 내용·사용법 |
|---|---|
| [robot_description](robot_description/README.md) | 최종 모델과 좌표·물성 |
| [gazebo](gazebo/README.md) | GPU 실행, 빈 월드·책상/벽 월드, 센서 실시간 수신 |
| [tools](tools/README.md) | 조이스틱·터미널 제어·카메라 팝업 |
| [ros2](ros2/README.md) | ROS 2 브리지·RViz·토픽 |
| [docs/SENSORS.md](docs/SENSORS.md) | 센서 사양·한계·검증 |
| [docs/ASSEMBLY.md](docs/ASSEMBLY.md) | 조립 위치·좌표 |

## 빠른 실행

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

위 별칭은 현재 PC에 설치돼 있습니다. 다른 PC에서는 [tools 안내](tools/README.md)의 Python 명령을 사용합니다.
손목 카메라는 항상 포함되므로 이전 `--with-wrist-camera`, `wrist_camera:=true` 옵션은 사용하지 않습니다.
