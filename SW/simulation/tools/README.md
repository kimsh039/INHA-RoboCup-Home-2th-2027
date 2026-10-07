# 컨트롤러·카메라 도구

Gazebo를 실행한 상태에서 사용합니다. 아래 Python 명령은 저장소 루트 기준입니다.
Python Gazebo Transport(`gz.transport13`, `gz.msgs10`)가 필요합니다.
이 문서의 `현재 PC`와 robot-* 단축 명령은 기존 Gazebo 개발 PC 기준입니다. Jetson에는 해당 단축 명령을 설치하지 않았으므로 의존성을 갖춘 PC에서 `python3 SW/simulation/tools/...` 명령을 사용합니다.
팝업은 GTK3(PyGObject), 카메라는 추가로 Pillow·NumPy가 필요합니다.
모든 도구의 기본 partition은 환경변수 `GZ_PARTITION` 또는 `robocup_motion`입니다.
토픽 이름, 관절 한계, 주행·관절·그리퍼 명령은 공통 모듈 [robocup_gz.py](robocup_gz.py)에 있으며 `control.py`, `robotctl.py`, `joystick.py`가 함께 사용합니다.
1번 관절은 원본 Piper의 1.6 rad 자세를 영점으로 사용하며 범위는 `[-4.2179938, 1.0179938] rad`입니다.
HOME과 관절 원점 버튼은 모든 팔 관절을 0으로 보내 이 기본 자세로 복귀합니다.

## 조이스틱 팝업

```bash
python3 SW/simulation/tools/joystick.py --partition robocup_motion
# 현재 PC의 단축 명령
robot-joystick --partition robocup_motion
```

파란 손잡이를 누르고 위/아래로 드래그하면 전진/후진, 왼쪽/오른쪽은 회전합니다.
마우스 버튼 해제·창 전환·창 종료·STOP·Space/Escape는 정지 명령을 보냅니다.
최대 0.2 m/s, 0.5 rad/s이며 주행 명령은 20 Hz로 보냅니다.
팔 관절 1~6의 목표 각도(rad)와 그리퍼 한쪽 이동량(m)을 입력한 뒤 Apply를 누릅니다.
입력값은 현재 측정값이 아닌 목표값입니다. HOME은 팔·그리퍼 영점 복귀와 주행 정지입니다.

## 터미널 주행·팔 제어

```bash
python3 SW/simulation/tools/robotctl.py --partition robocup_motion teleop
# 현재 PC에서는 아래 별칭 사용 가능
robotctl --partition robocup_motion teleop
```

W/S 전후진, A/D 제자리 회전, Space/X 정지, Q 종료.
키 입력이 0.4초 없으면 자동 정지합니다. 키를 반복하거나 길게 눌러 주행합니다.

```bash
robotctl --partition robocup_motion drive 0.1 0 --seconds 2
robotctl --partition robocup_motion drive -0.1 0 --seconds 2
robotctl --partition robocup_motion drive 0 0.3 --seconds 1
robotctl --partition robocup_motion stop
robotctl --partition robocup_motion joint 1 0.5
robotctl --partition robocup_motion joint 2 0.5
robotctl --partition robocup_motion joint 3 -0.5
robotctl --partition robocup_motion joint 4 0.3
robotctl --partition robocup_motion joint 5 0.3
robotctl --partition robocup_motion joint 6 0.3
robotctl --partition robocup_motion grip 0.03
robotctl --partition robocup_motion grip 0
robotctl --partition robocup_motion home
```

`drive`는 선속도(m/s), 각속도(rad/s), 지속 시간(초)이며 완료/Ctrl+C 시 정지합니다.
`joint`는 URDF 관절 한계를 검사합니다. `grip` 범위는 0~0.05 m(한쪽 손가락)입니다.
`home`은 Tracer의 world 위치까지 초기화하지 않습니다.
다른 PC에서는 `robotctl` 대신 `python3 SW/simulation/tools/robotctl.py`를 사용합니다.

## 카메라 팝업

각각 별도 터미널에서 실행합니다. 카메라당 RGB와 뎁스 창 두 개가 열립니다.

```bash
python3 SW/simulation/tools/camera_view.py --partition robocup_motion --camera head
python3 SW/simulation/tools/camera_view.py --partition robocup_motion --camera wrist
# 현재 PC의 단축 명령
robot-camera --partition robocup_motion --camera head
robot-camera --partition robocup_motion --camera wrist
```

RGB 1920×1080, 뎁스 1280×720. 뎁스는 0~3 m를 빨강(가까움)~파랑(멀어짐)으로 표시합니다.
검정은 유효 반환이 없는 픽셀이며 창 아래에 마지막 프레임 경과 시간을 표시합니다.
팝업 표시 갱신은 최대 10 Hz, 센서 설정은 30 Hz입니다.

## 웹 제어

`start_sim.sh`로 실행하면 `http://127.0.0.1:8081`에서 제어할 수 있습니다.
직접 Gazebo를 실행한 경우 별도 터미널에서:

```bash
python3 SW/simulation/tools/control.py
```

[센서값 실시간 수신·검사](../gazebo/README.md#실시간-센서-수신)
