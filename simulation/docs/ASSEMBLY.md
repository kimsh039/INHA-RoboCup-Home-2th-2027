# Tracer + 센서 랙 + Piper

아래 명령은 저장소의 `simulation/` 폴더에서 실행합니다. 통합 URDF는 `robot_description/`에 있습니다.

최종 모델은 [robocup.urdf](../robot_description/robocup.urdf) 하나이며 손목 카메라와 Gazebo 센서를 포함합니다.

Tracer의 `base_link`를 전체 루트로 사용합니다. 랙 루트는 `rack_base_link`로 변경하고 센서 및 Piper 프레임 이름은 유지했습니다.
`tracer_to_rack` fixed joint로 랙 바닥 중심을 Tracer 기준 `(0, 0, 0.01611)` m에 연결합니다.
랙과 Tracer의 +X 전방, +Z 상방을 맞췄습니다. 기존 Piper 마운트 연결은 유지합니다.

배치 근거: Tracer 차체 DAE의 geometry11/geometry57은 길이 약 458.5 mm, 폭 40 mm인 상단 프로파일입니다.
URDF visual 회전 RPY=(1.57,0,0)을 적용하면 두 프로파일의 Y 중심은 약 ±115 mm, X 중심은 0입니다.
두 프로파일의 최대 Z는 각각 0.016106 m / 0.015923 m입니다.
랙 하단 프레임 바닥 Z=0을 높은 쪽 윗면에 맞춰 관통을 피했습니다.
랙의 X=±180 mm에 있는 Y 방향 하단 프로파일이 Tracer의 두 레일을 가로지르는 배치입니다.
이는 메시 기반 중앙 정렬이며 실제 체결 구멍/볼트 위치를 검증한 배치는 아닙니다.
원본 Tracer의 약식 회전 1.57 rad을 유지해 두 레일에 약 0.18 mm 높이 차이가 있습니다.

위치를 변경하려면 최종 URDF의 fixed joint origin을 수정하고 Gazebo를 재실행합니다.

모든 메시를 기존 폴더의 상대 경로로 참조합니다. 독립·중간 URDF는 삭제했으며 메시와 CAD 자료는 유지합니다.
URDF는 차륜과 팔 관절을 유지하며, 동작용 제어 플러그인은 생성된 Gazebo world에 추가합니다.
Gazebo 동작 world는 `gazebo/make_sim.py`로 생성합니다.

## Gazebo 움직임 확인

Gazebo Harmonic 설치 후 `./gazebo/start_sim.sh`를 실행하고 `http://127.0.0.1:8081`을 엽니다.
팔 6개 관절 슬라이더와 그리퍼 버튼, 1초 단위 전후진/회전/정지를 지원합니다.
별도 `robocup_motion` Gazebo Transport partition만 제어하며 실제 하드웨어에는 연결하지 않습니다.
동작 world에서는 static을 해제하고 바닥, 중력, 차륜 DiffDrive와 팔 JointPositionController를 추가합니다.
팔은 `use_velocity_commands`의 이상적인 위치 제어로 속도를 제한합니다. 실제 모터 토크 성능 검증용이 아닙니다.
차륜 좌측의 원본 joint 회전으로 뒤집힌 축은 생성된 SDF에서만 보정합니다.
Tracer 총 질량은 제조사 명목값 30 kg으로 보정했으며 통합 모델 총 질량은 51.07908 kg입니다. 부품별 질량 분배·관성은 메시 기반 근사이며 [Tracer 물성 설명](../../HW/URDF/tracer/README.md)에 가정과 출처를 기록했습니다. 물성, 접촉 메시와 캐스터 구성은 실측 검증되지 않아 주행 동역학 결과를 실제 성능으로 해석하면 안 됩니다.
`make_sim.py`는 현재 checkout의 절대 메시 경로로 임시 SDF/URDF를 재생성합니다.

## 저장 기준 자세

저장한 URDF의 모든 가동 관절 기준값은 0입니다. Piper 팔 6축과 그리퍼, Tracer 바퀴의 영점 자세를 사용합니다.
URDF는 로봇 링크 간 변환을 정의하며 Gazebo에서 이동한 world 위치는 기록하지 않습니다.
동작 world 생성 시 차체 위치는 X=0, Y=0, yaw=0, Z=0.145 m로 시작합니다. Z는 바퀴가 바닥 위에 놓이도록 준 높이입니다.

## 센서 기능

Gazebo용 URDF에 Mid-360S, YDLIDAR G2와 D435f 시뮬레이션 센서를 추가했습니다.
사양 출처, 모델별 적용값, 데이터 토픽, 실제 장치와의 차이 및 검증 방법은 [SENSORS.md](SENSORS.md)에 정리했습니다.
센서 설정은 최종 URDF에서 수정한 뒤 Gazebo world를 다시 생성합니다.

## 터미널 제어

Gazebo가 실행 중이면 웹 서버 없이 `robotctl.py`로 직접 제어할 수 있습니다.
Python의 `gz.transport13`, `gz.msgs10` 모듈이 필요합니다.

```bash
python3 tools/robotctl.py teleop
python3 tools/robotctl.py drive 0.1 0 --seconds 2
python3 tools/robotctl.py drive 0 0.3 --seconds 1
python3 tools/robotctl.py stop
python3 tools/robotctl.py joint 1 0.5
python3 tools/robotctl.py joint 2 0.5
python3 tools/robotctl.py joint 3 -0.5
python3 tools/robotctl.py grip 0.03
python3 tools/robotctl.py grip 0
python3 tools/robotctl.py home
```

`teleop`: W/S 전후진, A/D 제자리 회전, X/Space 정지, Q 종료.
키 입력이 0.4초 없으면 정지합니다. 키를 반복하거나 길게 눌러 주행합니다.
`drive`: 선속도(m/s), 각속도(rad/s), 지속 시간(초). 완료하거나 Ctrl+C로 중단하면 정지합니다.
`joint`: 관절 번호 1~6, 목표 각도(rad). URDF 관절 한계를 검사합니다.
`grip`: 손가락 한쪽 이동량 0~0.05 m; 0은 닫기, 0.05는 총 0.1 m 벌리기입니다.
`home`: 팔과 그리퍼 목표를 0으로 설정하고 주행을 정지합니다. 차체의 world 위치는 초기화하지 않습니다.
선속도는 ±0.2 m/s, 각속도는 ±0.5 rad/s로 제한합니다.

기본 partition은 `GZ_PARTITION` 환경변수가 있으면 그 값, 없으면 현재 NVIDIA 미리보기의
`robocup_motion`입니다.
`./gazebo/start_sim.sh`로 시작한 Gazebo에는 다음처럼 지정합니다.

```bash
python3 tools/robotctl.py --partition robocup_motion teleop
```

현재 PC에는 `~/.local/bin/robotctl` 실행기를 설치해 어느 폴더에서든 `robotctl teleop`로 사용할 수 있습니다.
이 실행기는 이 checkout의 스크립트를 참조합니다. 다른 PC에서는 위의 Python 명령을 사용하세요.

## 카메라 팝업

Gazebo 실행 중 `python3 tools/camera_view.py`를 실행하면 RGB와 뎁스 창이 각각 열립니다.
현재 PC에서는 어느 폴더에서든 `robot-camera`로 실행합니다.
`./gazebo/start_sim.sh`를 사용하는 경우 `robot-camera --partition robocup_motion`을 실행합니다.
GTK3(PyGObject), Pillow, NumPy와 Gazebo Python Transport가 필요합니다.
뎁스는 0~3 m를 빨강(가까움)~파랑(멀어짐)으로 표시하며 반환이 없는 픽셀은 검정입니다.
새 프레임의 수신 경과 시간을 창 아래에 표시합니다. Gazebo가 일시정지되면 화면도 멈춥니다.
창 표시 갱신은 최대 10 Hz이며 원본 센서의 생성 주기는 변경하지 않습니다.
