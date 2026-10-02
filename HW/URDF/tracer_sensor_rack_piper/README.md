# Tracer + 센서 랙 + Piper

`tracer_sensor_rack_piper.urdf`: 원본 Piper 그리퍼 mimic 관계 포함.
`tracer_sensor_rack_piper_gazebo.urdf`: 기존 Gazebo용 Piper 변형 사용.

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

재생성 및 위치 조정:
```bash
python3 build.py --xyz 0 0 0.01611 --yaw 0
```

모든 메시를 기존 폴더의 상대 경로로 참조합니다. 세 원본 모델은 그대로 보관합니다.
차륜과 팔 관절은 유지하지만 주행 제어 플러그인은 포함하지 않습니다.
Gazebo 미리보기 world는 배치 확인용으로 전체 모델을 static으로 표시합니다.

## Gazebo 움직임 확인

Gazebo Harmonic 설치 후 `./start_sim.sh`를 실행하고 `http://127.0.0.1:8081`을 엽니다.
팔 6개 관절 슬라이더와 그리퍼 버튼, 1초 단위 전후진/회전/정지를 지원합니다.
별도 `robocup_motion` Gazebo Transport partition만 제어하며 실제 하드웨어에는 연결하지 않습니다.
동작 world에서는 static을 해제하고 바닥, 중력, 차륜 DiffDrive와 팔 JointPositionController를 추가합니다.
팔은 `use_velocity_commands`의 이상적인 위치 제어로 속도를 제한합니다. 실제 모터 토크 성능 검증용이 아닙니다.
차륜 좌측의 원본 joint 회전으로 뒤집힌 축은 생성된 SDF에서만 보정합니다.
원본 질량/관성, 접촉 메시와 캐스터 구성은 실측 검증되지 않아 주행 동역학 결과를 실제 성능으로 해석하면 안 됩니다.
`make_sim.py`는 현재 checkout의 절대 메시 경로로 임시 SDF/URDF를 재생성합니다.

## 저장 기준 자세

저장한 URDF의 모든 가동 관절 기준값은 0입니다. Piper 팔 6축과 그리퍼, Tracer 바퀴의 영점 자세를 사용합니다.
URDF는 로봇 링크 간 변환을 정의하며 Gazebo에서 이동한 world 위치는 기록하지 않습니다.
동작 world 생성 시 차체 위치는 X=0, Y=0, yaw=0, Z=0.145 m로 시작합니다. Z는 바퀴가 바닥 위에 놓이도록 준 높이입니다.
