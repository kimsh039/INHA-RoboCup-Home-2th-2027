# Tracer 객체 추종 테스트

이 폴더에만 추종 코드와 테스트를 추가했다. 기존 디텍션 모듈·미리보기 서버를 import해 사용하지만 기존 파일과 로봇 bringup·launch·파라미터는 변경하지 않는다. 실행은 아래 Python 파일을 직접 호출한다. Jetson에서 실물 주행과 FPS는 아직 측정하지 않았다.

기본 대상은 **banana**다. 클래스 이름은 green_apple, peach, tomato, fanta_can, cup, banana 중 선택한다. Head RGB에서 같은 객체가 3프레임 연속 확인되면 bbox 중심을 따라 회전하고, aligned depth의 bbox 안쪽 거리로 약 **1.0 m** 간격을 유지한다. 큰 좌우 오차는 제자리 회전부터 한다. 기본은 최대 **0.08 m/s**, **0.25 rad/s**, 전진·회전만 사용한다. 후진은 `--allow-reverse`로 추가한다(최대 0.04 m/s).

검출 누락·낮은 confidence·유효 depth 부족·색상/depth 시각 차이 0.1초 초과·해상도 불일치·거리 0.45~3.0 m 범위 밖이면 정지한다. 별도 20 Hz 명령 타이머가 원본 RGB 시각부터 **0.3초**가 지난 추종 명령을 0으로 만든다. bbox/depth는 손·배경이 섞일 수 있는 추정이며, 움직이는 객체의 실제 거리·동일성은 실기에서 확인해야 한다. 장애물 회피는 포함하지 않은 근거리 저속 테스트다.

## 확인 모드

이미 Head 카메라·aligned depth·Tracer 드라이버가 실행 중인 Jetson SSH 터미널에서:

```bash
cd ~/robocup
git pull --ff-only
source /opt/ros/humble/setup.bash
source ~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/activate
python SW/detection/head/test/tracer_follow.py --target banana
```

기본값은 **DRY RUN**이다. `/object_follow/candidate_cmd_vel`에 계산 결과만 발행하며 `/cmd_vel` publisher를 만들지 않는다. 화면에는 대상 박스·거리·상태·계산 속도, 터미널에는 Camera FPS·Detection FPS·v·w가 표시된다. 센서나 모터 드라이버를 새로 실행하지 않는다.

기본 입력은 `/head/camera/color/image_raw`와 `/head/camera/aligned_depth_to_color/image_raw`다. factory RGB와 그 RGB에 aligned된 depth의 같은 픽셀 좌표를 사용한다. 인트린식으로 3D 좌표를 투영하지 않으며 회전은 화면 중심 오차 기준이다. ROS_DOMAIN_ID는 기존 센서 노드와 같아야 한다.

거리 값은 카메라 광학축 z 기준이며 범퍼에서 객체까지의 간격이 아니다. Camera FPS는 구독 콜백에 도착한 프레임 수, Detection FPS는 실제 YOLO 호출을 완료한 프레임 수다. depth가 누락되면 화면은 갱신돼도 Detection FPS는 0일 수 있다.

Windows 화면을 보려면 별도 PowerShell에서 SSH 포트 전달을 연결하고 브라우저로 **http://127.0.0.1:8787/**를 연다.

```powershell
ssh -N -L 8787:127.0.0.1:8787 sparo@192.168.50.70
```

이전 미리보기 프로그램이 8787을 사용 중이면 `--port 8788`로 바꾸고 SSH 전달 양쪽 포트와 브라우저 주소도 8788로 맞춘다.

## 실제 바퀴 명령

대상 박스·depth·회전 방향이 맞는 것을 확인한 뒤 빈 공간에서 기존 teleop·Nav2 등 다른 `/cmd_vel` 제어기를 종료하고 실행한다.

```bash
python SW/detection/head/test/tracer_follow.py --target banana --move
```

이 명령은 **실제로 로봇을 움직인다.** `--move` 모드에 다른 `/cmd_vel` publisher가 있으면 시작을 거부하고, 실행 중 발견하면 이 테스트를 정지한다. 다른 publisher의 명령까지 차단하는 mux는 아니다. 후진 테스트를 추가하려면 `--allow-reverse`를 붙인다. 임의 객체 크기를 거리 기준으로 쓰지 않는다.

정지와 종료는 **Ctrl+C**다. Ctrl+C·SIGTERM·SSH hangup 처리에서 이 실행기가 제어 중이었다면 0 속도를 반복 발행한다. 카메라/YOLO가 지연되면 독립 타이머가 0 속도를 발행한다. 프로세스 강제 종료·통신 단절까지 보장하는 드라이버 watchdog은 실기에서 확인하지 않았으므로 실물 비상정지 수단을 준비한다.

프로그램을 유지하면서 추종만 정지하려면 다른 ROS 터미널에서 다음을 실행한다. 정지는 latch되며 재개하려면 프로그램을 다시 실행한다.

```bash
ros2 topic pub --once /object_follow/stop std_msgs/msg/Empty '{}'
```

대상과 간격 변경 예:

```bash
python SW/detection/head/test/tracer_follow.py --target tomato --distance 1.2
```

## 코드·검증

- `tracer_follow.py`: 기존 모델의 6클래스 변환, ROS RGB-D 입력, 실시간 화면·FPS, 명령 발행과 입력 watchdog.
- `follow_control.py`: 대상 연속 대응, depth 판독, 회전 방향·거리 제어, 정지 판단. ROS 없이 검증한다.
- `test_follow_control.py`: 방향·속도 한계·연속 확인·대상 누락·다른 객체·depth 오류·watchdog 등을 확인한다.

```bash
python -m unittest discover -s SW/detection/head/test -p 'test_*.py' -v
```

2026-10-12 로컬 CPU 제어 테스트 **13개 통과**. CLI 도움말과 Python 문법 확인 완료. Jetson ROS 카메라·주행 통합 검증은 아직 수행하지 않았다. 종료 신호 옵션은 [ROS 2 Humble rclpy API](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/__init__.py)에 맞췄다.
