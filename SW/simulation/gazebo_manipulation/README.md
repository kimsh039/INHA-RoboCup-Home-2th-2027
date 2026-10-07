# Gazebo Manipulation · 베이스 밀림을 포함한 픽앤플레이스

> **Ubuntu 22.04 / ROS 2 Humble / Gazebo Harmonic 개발 PC용 튜토리얼입니다.** 40cm close approach를 마친 기준 장면을 가정하고, 손목 관측 → 기록 GraspNet 후보 → 실제 접촉 파지 → 20cm lift → 옆으로 10cm 배치 → 초기 자세 복귀를 실행합니다. 실물 구동·새 GPU 추론·실제 Nav2 접근의 통합 검증은 아닙니다.

> [!IMPORTANT]
> **사용자 실행 기록: 2026-10-07.** 결과는 **PHYSICAL_PICK_PLACE_PASS_WITH_RELAXED_HOLD / DONE**입니다. 물체 유지 허용값을 위치25mm·회전60도로 완화한 밀림 실험입니다. 원래 MuJoCo 유지 기준(10mm·0.2rad)은 회전 검사에서 통과하지 못했습니다. 정밀 파지 또는 실물 배포 성공으로 해석하지 않습니다. Git 게시를 위한 경로 이식 후에는 사용자 요청에 따라 실행을 반복하지 않았습니다.

## 먼저 확인할 문서

| 내용 | 문서 |
|---|---|
| 설치·터미널별 실행·GUI·옵션 | [상세 튜토리얼](docs/TUTORIAL.md) |
| 실제 성공 조건·수치·한계 | [검증 보고서](docs/VALIDATION.md) |
| 성공 결과 원본 | [성공 JSON](reports/gazebo_tutorial.json) |
| stage별 base/TCP/object·접촉 기록 | [trace JSONL](reports/gazebo_tutorial.trace.jsonl) |
| 모델·코드·기록 checksum | [provenance](docs/provenance.json) |
| 재사용한 MuJoCo 흐름·후보 | [MuJoCo 튜토리얼](../mujoco/README.md) |

## 이번 실행의 결과

| 항목 | 기록 |
|---|---|
| 전체 작업 | 양손가락 접촉 → 물체 상승 → 오른쪽10cm 배치 → 초기 관절각 복귀 |
| 선택 후보 | 741 · 기존 MuJoCo의 object-frame GraspNet 후보 |
| lift 높이 | 최소19.316cm · 목표20cm±2cm |
| lift 중 최대 물체 상대 변화 | 위치6.847mm / 회전0.50839rad(약29.1°) |
| 물체 유지 허용값 | 위치25mm / 회전60° · 실험값 |
| 배치 위치 오차 | 약8.679mm · 기준20mm |
| 초기 관절각 복귀 오차 | 최대0.003581rad · 기준0.02rad |
| 원래 유지 기준 | **미통과** · original_hold_criteria_passed=false |

베이스를 월드에 고정하거나 물체를 gripper에 weld하지 않습니다. 물체는 Gazebo 접촉·마찰로 움직입니다. MoveIt attached object는 충돌 계획용입니다. 명목 모델은 [robocup.urdf](../robot_description/robocup.urdf), CAD/메시는 [HW](../../../HW/README.md)를 재사용하며 calibrated URDF 검증과 구분합니다.

## 빠른 실행

기존 Gazebo/ROS 관절 실행을 먼저 종료하고 저장소 루트에서 실행합니다.

터미널 1:

```bash
bash SW/simulation/gazebo_manipulation/build.sh
bash SW/simulation/gazebo_manipulation/run.sh gui:=true
```

터미널 2, 같은 저장소 루트:

```bash
bash SW/simulation/gazebo_manipulation/command.sh tutorial
```

GUI 없이 실행하려면 `run.sh`에서 `gui:=true`를 생략합니다. 종료는 터미널1의 Ctrl+C입니다. 재실험 시 world를 재시작합니다. 기본 실행은 손목 RGB-D만 켜고 헤드·두 라이다·점군 필터·RViz·상위 task 서버는 끕니다. 센서 장착 링크·TF·질량·접촉 물리는 유지합니다.

## 파일 구조

```text
SW/simulation/gazebo_manipulation/
├── README.md
├── build.sh / run.sh / command.sh
├── gazebo_ws/src/
│   ├── robocup_gazebo_manipulation/  # 생성기·MoveIt backend·simulation controller
│   └── robocup_manipulation_msgs/    # 독립 빌드에 필요한 ROS interfaces
├── config/                         # Nav2 연결 예시, 기본 navigation=null
├── docs/                           # 재현·검증·출처
└── reports/                        # 성공 JSON·수치 trace
```

빌드/install은 `/tmp/robocup-gazebo-$UID`, 생성 world·명령 로그는 로컬 `runtime/`에 저장합니다. 원본 메시·URDF·MuJoCo 코드·후보는 변경하지 않습니다.
