# 최종 로봇 URDF

**현재 최종 보정 모델: [robocup.calibrated.urdf](robocup.calibrated.urdf) — 2026-10-06 반영·업로드.** Base–2D LiDAR와 Base–Mid360 보정값을 최신 랙·PiPER 구조에 적용했습니다. [robocup.urdf](robocup.urdf)는 CAD 기준 원본이며, 현재 Gazebo world 생성기와 일반 `sim.launch.py`가 읽는 입력입니다. 아래 [보정 URDF 적용하기](#보정-urdf-적용하기)에서 최종 모델을 선택하는 실행 명령을 확인합니다. 예전 모델은 Git 이력에 보관합니다.
94개 링크, 93개 조인트이며 단일 루트는 `base_link`입니다.
Head D435–Mid360과 Link6–Wrist D435의 2026-10-06 결과는 [캘리브레이션 기록](../calibration/README.md)에 별도로 업로드했으며, 두 카메라 보정값은 이 최종 모델에 아직 반영하지 않았습니다.
두 모델 모두 일반 URDF 구조에 Gazebo 센서 확장을 포함합니다. 현재 Gazebo 측정 장면은 CAD 원본에서 만들고, ROS에 보정값을 적용할 때는 최종 보정 모델을 선택합니다.
그리퍼 두 관절은 별도로 정의하며 제어 도구가 대칭 목표값을 보냅니다.

- Tracer → 랙: fixed, xyz `(0, 0, 0.01611)` m.
- 랙 → Piper 베이스: fixed, 랙 기준 XYZ `(-0.0195, 0, 0.790)` m, RPY `(0, 0, 0)`.
- `piper_gripper_base` → 손목 마운트: CAD 정합 변환으로 fixed 연결.
- 2D/3D 라이다, 헤드 RGB/depth, 손목 RGB/depth 센서 포함.
- 명목 총질량 약 50.96143 kg. Tracer 30 kg, 랙 15.64637 kg, 손목 카메라 75 g, 손목 마운트 약 15.07 g.
- 모든 가동 관절의 URDF 기준 자세는 0입니다. Gazebo 시작 위치는 X/Y/yaw=0, Z=0.145 m입니다.

팔의 영점 자세는 원본 Piper의 1번 관절을 1.6 rad(약 91.7°) 돌린 모습입니다.
`piper_joint1`의 `origin rpy="0 0 1.6"`으로 이 자세를 모델에 반영했으며 시작 후 회전할 필요가 없습니다.
각도는 `q1 = 원본 Piper q1 - 1.6`이며 한계는 `[-4.2179938, 1.0179938] rad`로 옮겨 기존 물리적 가동 범위를 유지합니다.
Gazebo·ROS의 관절값과 수동 제어 명령은 이 새 영점을 기준으로 하며 `home`은 사진 속 기본 자세로 복귀합니다.

메시는 `../../HW/URDF/` 아래를 상대 경로로 참조합니다. 메시 폴더는 삭제하지 마세요.
CAD 형상·장착 좌표·관성·Gazebo 센서 설정 변경은 원본 `robocup.urdf`에 반영하고 world를 다시 생성합니다. 이후 보정 결과 JSON을 새 원본에 누적 적용해 `robocup.calibrated.urdf`를 갱신하고, 변경 날짜·센서·근거를 아래 이력에 기록합니다. 장착 위치가 달라졌다면 이전 보정값의 유효성도 별도로 평가합니다.
실물 체결 좌표·물성·광학 보정은 별도 검증이 필요합니다.

2026-10-06: Fusion `final_assembly`의 변경된 프로파일·브래킷 형상과 `Assembly`의 Piper 베이스 장착 위치를 반영했습니다. 기존 매니퓰레이터 판형 마운트는 제거됐습니다. 센서 마운트·센서 링크·Gazebo 센서 확장, Piper 내부 관절·영점·가동 범위, 손목 카메라 연결 및 제어 스크립트는 유지했습니다. 새 알루미늄 랙의 질량·무게중심·관성·경계 박스 충돌 형상을 재계산했으며, [변경 근거](../../../HW/URDF/sensor_rack_description/rack_revision_20261006.json)를 보관합니다. 아래 Gazebo 실행 검증은 이번 형상 변경 전의 기록이며 새 모델의 실구동을 뜻하지 않습니다.

[조립 상세](../docs/ASSEMBLY.md) · [손목 장착 계산](../../../HW/URDF/WRIST_CAMERA_INTEGRATION.md)

최종 확정 후 별도 Gazebo 서버에서 G2 10 Hz/500 rays, Mid-360S 10 Hz/20,000 rays·points,
헤드·손목 RGB/depth 약 30 Hz 메시지와 프레임을 확인했고 정면 검사 벽 거리 검사를 통과했습니다.
ROS 런치는 당시 Gazebo 개발 PC에 ROS가 없어 구문 확인까지만 했습니다. 별도 Jetson 실기 setup은 [Jetson 문서](../../setup/jetson/README.md)에 기록했으며 이 시뮬레이션 실행 검증과 구분합니다.

## 모델 변경·보정·업로드 이력

날짜는 한국 시간 기준입니다. **측정·계산 날짜와 Git에 반영한 날짜를 구분**합니다. 아래 결과는 Gazebo에서 얻은 보정값이며 실물 센서 보정 완료를 뜻하지 않습니다.

| 측정·작업 날짜 | 무엇을 변경했나 | 최종 모델 반영·업로드 | 결과·근거 |
|---|---|---|---|
| 2026-10-05 | Base–2D LiDAR의 `x`, `y`, `yaw` 추정. `base_link ← laser_frame`: x=0.000123610m, y=0.000520516m, yaw=8.844783°. z=0.34271m와 roll/pitch는 고정 입력 | 2026-10-06, [`b871b5b`](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/commit/b871b5b)에서 `robocup.calibrated.urdf`에 적용·업로드 | [보정 JSON](../calibration/records/20261005_base_2dlidar/calibration/calibration.json), [별도 자세 001/002 평가](../calibration/records/20261005_base_2dlidar/validation/validation_report.md) |
| 2026-10-06 | Base–Mid360 위치·방향 추정. `base_link ← livox_frame`: xyz≈(-0.179962, 0.000156, 1.198867)m, roll≈180° | 2026-10-06, [`b871b5b`](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/commit/b871b5b)에서 2D 보정과 함께 누적 적용·업로드 | [보정 JSON](../calibration/records/20261006_base_mid360/results/auto_room_20261006_031114/base_mid360.json), [측정 자료](../calibration/records/20261006_base_mid360/README.md) |
| 2026-10-06 | 프로파일 랙 형상·물성과 PiPER 베이스 장착 위치 변경. 새 CAD 원본에 기존 2D·Mid360 보정값을 다시 적용 | CAD 변경 [`1509ede`](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/commit/1509ede) → 보정 누적 [`b871b5b`](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/commit/b871b5b) | [랙 변경 기록](../../../HW/URDF/sensor_rack_description/rack_revision_20261006.json). CAD 갱신과 센서 재측정은 별개 작업 |
| 2026-10-06 | Mid360 보정값을 고정한 채 별도 자세 A/B 자료를 검사. 평면 오차의 임시 기준 10mm/1° 통과 | 2026-10-06, 기존 보정값을 유지하고 A/B 원자료·평가 보고서를 추가 업로드 | [독립 정확도 평가 기록](../calibration/records/20261006_base_mid360/validation_20261006_201607/README.md) |
| 2026-10-06 | Ubuntu/Mac 적용 명령, RViz 표시, TF 확인 방법을 README에 추가 | [`41ed5fc`](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/commit/41ed5fc) | 아래 [보정 URDF 적용하기](#보정-urdf-적용하기) |
| 2026-10-06 | Head D435–Mid360을 학습 25개로 계산하고 별도 10개 평면으로 평가. 최대 4.247mm/0.814°, GT 위치 오차 11.720mm | 2026-10-06, JSON·사진·점군·자세·평가 업로드. Base–Head 합성 및 최종 URDF 반영은 미실시 | [Head 보정 기록](../calibration/records/20261006_head_mid360/README.md) |
| 2026-10-06 | Link6–Wrist D435 hand-eye를 학습 25개로 계산하고 별도 10개 자세로 평가. 최대 0.339mm/0.294°, GT 위치 오차 0.598mm | 2026-10-06, JSON·사진·관절·FK·평가 업로드. 최종 URDF 반영은 미실시 | [Wrist 보정 기록](../calibration/records/20261006_wrist_d435/README.md) |

### 현재 보정 결과를 어떻게 해석하나

- **2D LiDAR:** 별도 자세 001/002에서 선택한 외벽 점의 RMS는 6.737/6.846mm, P95는 14.736/14.621mm입니다. 당시 임시 기준을 통과했습니다. z/roll/pitch를 새로 보정한 결과는 아닙니다.
- **Mid360:** A/B의 최대 평면 위치 오차는 3.594/3.167mm, 최대 각도 오차는 0.2100/0.1892°입니다. Gazebo 장착 기준값 대비 위치 오차는 0.291mm, 회전 오차는 0.00404°입니다. 이는 평면과 변환에 대한 평가이며 개별 점의 거리 오차와 구분합니다.
- **원본 상태 기록:** Mid360 계산 JSON의 `computed_validation_pending`은 계산 당시 상태로 보존합니다. 후속 A/B 판정은 [별도 보고서](../calibration/records/20261006_base_mid360/validation_20261006_201607/results/audit_20261006_202216/accuracy_report.md)에 기록했습니다.
- **Head D435–Mid360:** [결과와 원자료](../calibration/records/20261006_head_mid360/README.md)를 등록했습니다. 별도 평면 일관성은 통과했지만 GT 위치 오차 11.720mm가 남았습니다. Base–Head 합성과 URDF 반영은 아직 하지 않았습니다.
- **Wrist D435–Link6:** [결과와 원자료](../calibration/records/20261006_wrist_d435/README.md)를 등록했습니다. 별도 태그 위치/방향 오차는 최대 0.339mm/0.294°, GT 장착 오차는 0.598mm/0.195°입니다. URDF 반영은 아직 하지 않았습니다.
- **남은 항목:** Head 위치 오차 개선, 카메라 보정값의 URDF 반영, 팔 베이스·TCP·전체 교차 검증 등. 최종 파일 이름은 전체 센서·실물 보정 완료를 의미하지 않습니다.

## 보정 URDF 적용하기

**2D LiDAR와 Mid360 보정이 들어 있는 파일은 [robocup.calibrated.urdf](robocup.calibrated.urdf)입니다.** 최신 랙/PiPER 구조에 기존 보정 결과를 누적한 모델입니다. 원본을 덮어쓰거나 URDF 내용을 손으로 복사하지 않고, ROS 런치에 이 파일의 경로를 지정합니다.

| 모델 | 쓰는 곳 |
|---|---|
| `robocup.urdf` | 명목 CAD 모델. 현재 Gazebo world 생성기와 일반 `sim.launch.py`가 기본으로 읽음 |
| `robocup.calibrated.urdf` | 보정된 `base_link → laser_frame`(2D)와 `base_link → livox_frame`(3D)를 ROS TF로 발행할 때 읽음 |

[Base–2D 결과](../calibration/records/20261005_base_2dlidar/calibration/calibration.json)와 [Base–Mid360 결과](../calibration/records/20261006_base_mid360/results/auto_room_20261006_031114/base_mid360.json)는 별도로 보관합니다. URDF에 이미 반영돼 있으므로 아래 실행 전에 solver를 다시 돌릴 필요는 없습니다. 이 파일은 시뮬레이션 측정 결과이며 실물 보정값과 구분합니다. Head/Wrist 카메라는 모두 D435입니다.

### 1. 준비 — 저장소 전체와 ROS 환경

메시가 `HW/URDF/`에 있으므로 URDF 한 파일만 내려받지 않고 저장소 전체를 사용합니다. 아래 명령은 **저장소 루트에서 실행**합니다. 이미 clone한 위치가 다르면 먼저 그 위치로 `cd`합니다.

Gazebo Server/GUI를 별도 터미널에서 실행 중이라면 그대로 둡니다. 같은 ROS 통신 그룹에서 이전 `robot_state_publisher`를 실행한 창은 Ctrl+C로 종료하고 보정 런치를 시작합니다. 같은 센서 TF를 기존 URDF와 새 URDF가 동시에 발행하면 안 됩니다. `sim.launch.py` 전체를 종료하면 그 런치가 실행한 Gazebo도 종료되므로, GUI를 유지할 실험은 [Gazebo Server/GUI 분리 실행](../calibration/BASE_MID360.md#2-gazebo-방과-gui-실행)을 사용합니다.

### 2-A. Ubuntu에서 적용 — ROS 2 Jazzy

ROS 2 Jazzy, `robot_state_publisher`, RViz와 Gazebo가 준비된 Ubuntu 24.04 기준입니다. 별도 빌드 없이 저장소에 있는 런치 파일을 직접 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
export REPO="$PWD"
export RUNTIME_FILE="$REPO/simulation/robot_description/robocup.calibrated.urdf"
export ROS_DOMAIN_ID=73
export GZ_IP=127.0.0.1
export GZ_PARTITION=robocup_motion
ros2 launch "$REPO/simulation/ros2/calibration_runtime.launch.py" \
  urdf:="$RUNTIME_FILE" joints:=true rviz:=true
```

ROS 2 Humble 환경에서는 첫 줄을 `source /opt/ros/humble/setup.bash`로 바꾸고 해당 ROS 배포판의 패키지를 사용합니다. Jazzy/Humble 환경을 같은 터미널에 섞지 않습니다.

### 2-B. Mac에서 적용 — 설치된 ros_jazzy conda 환경

새 터미널에서 **먼저 자신의 저장소 루트로 이동한 뒤** 아래 블록 전체를 입력합니다. 현재 사용하는 Miniforge `ros_jazzy` 환경 기준입니다.

```bash
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate ros_jazzy
export REPO="$PWD"
export RUNTIME_FILE="$REPO/simulation/robot_description/robocup.calibrated.urdf"
export ROS_DOMAIN_ID=73
export GZ_IP=127.0.0.1
export GZ_PARTITION=robocup_motion
ros2 launch "$REPO/simulation/ros2/calibration_runtime.launch.py" \
  urdf:="$RUNTIME_FILE" joints:=true rviz:=true
```

| 인수/설정 | 무엇을 적용하나? |
|---|---|
| `urdf:=...robocup.calibrated.urdf` | 이 파일을 `robot_state_publisher`가 읽어 보정 TF 발행 |
| `joints:=true` | 실행 중 Gazebo의 실제 관절값으로 팔·Wrist TF도 표시 |
| `rviz:=true` | 보정 TF를 볼 RViz 창 실행 |
| `ROS_DOMAIN_ID=73` | 이 런치와 TF를 읽는 다른 ROS 터미널의 통신 그룹 |
| `GZ_PARTITION=robocup_motion` | 관절 표시가 연결할 Gazebo Server/GUI의 partition |

Gazebo 없이 **Base–2D/Mid360의 고정 TF만** 보려면 마지막 명령의 `joints:=true`를 `joints:=false`로 바꿉니다. 이때 실제 관절에 따른 팔/Wrist TF는 발행하지 않습니다. 이 런치는 표시용 wall-clock 시간을 사용하며, 일반 `sim.launch.py`의 simulation-time 브리지·Nav2 전체 실행과는 별도입니다.

### 3. 적용한 값을 어디서 보나?

RViz의 Fixed Frame은 `base_link`입니다. Displays → Calibrated TF에서 `Filter (whitelist)`를 다음처럼 바꾸면 베이스와 두 LiDAR만 볼 수 있습니다.

```text
^(base_link|laser_frame|livox_frame)$
```

Show Axes/Names는 켜고 Show Arrows는 끕니다. 팔·Wrist까지 보고 싶으면 필터를 기존 값으로 되돌립니다. 런치가 켜진 상태에서 다른 ROS 터미널에 **같은 ROS 환경과 ROS_DOMAIN_ID=73**을 설정하면 실제 발행된 수치를 읽을 수 있습니다.

Ubuntu의 TF 읽기 터미널:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=73
ros2 run tf2_ros tf2_echo base_link laser_frame
```

Mac의 TF 읽기 터미널:

```bash
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate ros_jazzy
export ROS_DOMAIN_ID=73
ros2 run tf2_ros tf2_echo base_link laser_frame
```

위 명령을 Ctrl+C로 종료한 뒤 같은 터미널에서 Mid360도 읽습니다.

```bash
ros2 run tf2_ros tf2_echo base_link livox_frame
```

Mid360 결과의 translation은 약 `(-0.179962, 0.000156, 1.198867)`m이며 roll은 약 180°입니다. `tf2_echo`의 Euler 표시에 +180° 대신 -180° 부근이 나올 수 있습니다. 같은 센서 장착 방향입니다. URDF 센서 joint의 로컬 `origin xyz`와 전체 `base_link → 센서` 위치를 혼동하지 않습니다.

### 4. 이 실행이 바꾸는 범위

- **ROS TF:** 위 런치가 보정 URDF를 읽는 동안 2D/3D LiDAR의 보정 좌표 관계를 사용합니다. 종료하면 이 발행도 끝납니다.
- **Gazebo:** 실행 중인 로봇/SDF를 자동 변경하지 않습니다. 현재 `make_sim.py`와 일반 `sim.launch.py`의 기본 모델은 명목 `robocup.urdf`입니다. 아래 보정 런치를 실행했다고 Gazebo 센서 장착값도 바뀐 것으로 해석하지 않습니다.
- **센서 데이터·주행:** 이 런치는 점군/영상 bridge, SLAM, Nav2를 시작하지 않습니다. 기존 TF 발행기와 중복시키거나 다른 시간 기준의 전체 런치에 그대로 추가하지 않습니다.

**적용 확인과 정확도 평가는 별개입니다.** TF 수치를 읽은 다음 새 base 자세에서 점군·실제 pose를 저장하고 같은 보정값으로 평가하는 절차는 [캘리브레이션 가이드 8~12번](../calibration/BASE_MID360.md#8-검증-a-준비--보정값-고정-새-저장-폴더-만들기)에 있습니다. 로봇 이동·회전 명령, 보고서 경로, 통과/실패 숫자와 기록 방법까지 포함합니다. 보정 JSON의 `computed_validation_pending`은 별도 평가 보고서가 생겨도 자동으로 바뀌지 않습니다.

### 5. 이후 새 보정값을 얻었을 때

기존 보정을 보존하려면 **직전 보정 모델을 입력**, 새 결과 JSON을 적용해 **새 이름의 runtime URDF를 출력**합니다. 아래는 레포의 계산 환경을 준비한 뒤 저장소 루트에서 실행하는 예입니다. JSON 입력 경로는 실제 새 계산 결과로 바꿉니다.

```bash
export REPO="$PWD"
export NEW_RESULT="$REPO/simulation/calibration/data/이번_세션/results/이번_계산/base_mid360.json"
export RUNTIME_FILE="$REPO/simulation/calibration/data/runtime_mid360_$(date +%Y%m%d_%H%M%S).urdf"
"$REPO/simulation/calibration/.venv/bin/python" \
  "$REPO/simulation/calibration/calibration_workflow.py" patch-urdf \
  --urdf "$REPO/simulation/robot_description/robocup.calibrated.urdf" \
  --result "$NEW_RESULT" \
  --mount-child livox_frame \
  --output "$RUNTIME_FILE"
```

출력 URDF 옆의 `.application.json`에 입력 모델·보정 결과의 SHA-256과 수정한 joint가 기록됩니다. 출력 파일은 새 이름으로 만들며 기존 파일을 덮어쓰지 않습니다. ROS 런치를 실행한 창에서 Ctrl+C 후, 같은 환경에서 위 `RUNTIME_FILE` 경로를 `urdf:=...`에 지정해 다시 실행합니다.

이 README 수정에서는 런치 실행·TF 조회·빌드·테스트를 하지 않았습니다.
