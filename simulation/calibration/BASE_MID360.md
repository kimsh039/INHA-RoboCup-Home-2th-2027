# Calibration 01 — Base와 Mid360: 측정부터 적용·정확도 평가까지

**기준: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic. Head와 Wrist 카메라는 모두 D435.**

[전체 보정 순서·현재 결과·자료 보관 위치](README.md)를 먼저 읽고 이 문서에서 Mid360 측정·계산·적용·평가 명령을 실행합니다.

이 과정은 `base_link ← livox_frame`의 위치·방향을 구한다. Mid360 점군을 로봇 기준으로 바꿀 때 사용할 변환이다. 아래 1~7에서 준비·측정·계산·ROS 적용을 하고, **8~11에서 다른 자세로 측정하고 정확도를 판정**한다. RViz에 축이 보인 것만으로 정확도 평가가 끝나지 않는다.

| 내가 현재 어디까지 했나? | 이어서 할 단계 |
|---|---|
| 처음 시작 | 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8~11 |
| 보정 JSON과 적용 URDF가 이미 있음 | 7에서 적용 파일 선택 → 8~11 |
| 새 Git URDF로 Gazebo를 다시 생성함 | 새 세션에 새 world를 보관하고 새 관측 수집. 이전 측정 world를 덮어쓰지 않음 |

**현재 저장된 보정·후속 평가:** [2026-10-06 Mid360 보정 JSON](records/20261006_base_mid360/results/auto_room_20261006_031114/base_mid360.json)의 위치는 약 `(-0.179962, 0.000156, 1.198867)m`, roll 약 180°다. 같은 날짜에 `robocup.calibrated.urdf`에 반영·업로드했고, 기존 고정값으로 별도 자세 A/B를 검사한 결과는 임시 10mm/1° 기준을 통과했다. [평가 보고서·A/B 원자료](records/20261006_base_mid360/validation_20261006_201607/README.md)와 [모델 업로드 이력](../robot_description/README.md#모델-변경보정업로드-이력)을 참고한다. 계산 JSON의 **computed_validation_pending**은 계산 당시 기록으로 유지하며 후속 판정을 별도 보고서에 보관한다. 이번 문서 수정에서는 추가 측정·검증을 실행하지 않았다.

## 1. 파일과 터미널 준비

로봇 레포를 처음 받는 경우에만:

```bash
mkdir -p "$HOME/Documents"
cd "$HOME/Documents"
git clone https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027.git
```

이미 받은 레포는 다시 clone하지 않는다. 아래 `REPO`를 자신의 레포 위치로 바꾼다. 이후 경로는 자동으로 만들어진다.

Control 터미널에서:

```bash
export REPO="$HOME/Documents/INHA-RoboCup-Home-2th-2027"
export PROJECT="$REPO/simulation/calibration"
cd "$PROJECT"
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
export GZ_IP=127.0.0.1
export GZ_PARTITION=robocup_motion
set -o pipefail
set -o noclobber
```

| 터미널 | 역할 | 수집 중 상태 |
|---|---|---|
| Server | Gazebo 물리·센서 실행 | 계속 켜 둠 |
| GUI | Gazebo 화면 | 계속 켜 둠 |
| Control | 로봇 이동·저장·계산·평가 | 각 명령이 끝난 뒤 다음 명령 입력 |
| ROS | 적용 URDF로 TF·RViz 실행 | 필요하면 계속 켜 둠 |

각 창의 환경변수는 독립적이다. 새 Control 창에서는 위 export 설정을 다시 입력한다. 아래 명령은 GUI를 끄고 재시작하며 반복하는 방식이 아니다.

## 2. Gazebo 방과 GUI 실행

**현재 같은 방이 실행 중이면 생성·재시작을 생략**하고 3번으로 간다. 최신 URDF로 새 방을 만들 때만 Server에서 다음을 실행한다. 기존 서버는 그 서버를 실행한 터미널에서 Ctrl+C로 종료한다.

```bash
export REPO="$HOME/Documents/INHA-RoboCup-Home-2th-2027"
cd "$REPO"
if [ -f simulation/gazebo/build/motion.world.sdf ]; then
  cp simulation/gazebo/build/motion.world.sdf "simulation/gazebo/build/motion.before_$(date +%Y%m%d_%H%M%S).world.sdf"
fi
python3 simulation/gazebo/make_sim.py --world room
export GZ_IP=127.0.0.1
export GZ_PARTITION=robocup_motion
gz sim -s -r simulation/gazebo/build/motion.world.sdf
```

이 생성 명령은 최신 명목 `robocup.urdf`로 측정 장면을 만든다. ROS에 적용할 보정 URDF와 Gazebo의 측정 센서 설정을 구분한다.

GUI 터미널:

```bash
export GZ_IP=127.0.0.1
export GZ_PARTITION=robocup_motion
gz sim -g
```

GUI에서 바닥·로봇·네 외벽이 보이는 방을 사용한다. 벽과 바닥을 이동하지 않는다. 현재 `room`의 외벽 안쪽은 x/y=±3m이고 바닥은 world z=0이다. 계산기는 **실제 저장한 방 SDF의 벽 치수**를 사용한다. `--floor-verified`는 사용자가 바닥 z=0 조건을 확인했다는 선언이며 자동 검사 옵션이 아니다.

## 3. 새 실험의 저장 경로와 기록 파일 생성

Control에서 실행한다. 날짜가 붙은 새 세션을 만들므로 이전 자료를 덮어쓰지 않는다.

```bash
export RUN_ID="$(date +%Y%m%d_%H%M%S)"
export SESSION_DIR="$PROJECT/data/base_mid360_$RUN_ID"
export SAMPLE_DIR="$SESSION_DIR/train/001"
export CALIB_DIR="$SESSION_DIR/results/auto_room_$RUN_ID"
export RUNTIME_FILE="$SESSION_DIR/results/runtime_mid360_$RUN_ID.urdf"
mkdir -p "$SESSION_DIR/config" "$SESSION_DIR/results"
mkdir -p "$SESSION_DIR/train"
mkdir "$SAMPLE_DIR"
cp "$REPO/simulation/gazebo/build/motion.world.sdf" "$SESSION_DIR/config/world.sdf"
git -C "$REPO" rev-parse HEAD > "$SESSION_DIR/config/repo_commit.txt"
printf '프로젝트: Base–Mid360\n세션: %s\n환경: Ubuntu\n보정 결과: %s/base_mid360.json\n적용 URDF: %s\n정확도 평가: 미실행\n\n## 내가 기록할 관찰\n' \
  "$SESSION_DIR" "$CALIB_DIR" "$RUNTIME_FILE" > "$SESSION_DIR/session.md"
```

`session.md`의 경로·상태와 파일 틀은 자동 생성된다. 직접 적을 부분은 흔들림·가림·실험 선택 이유 같은 **관찰 메모**다. 수집 pose, 점군, 평면 CSV, 보정 JSON은 아래 명령이 만든다. **CALIB_DIR은 미리 mkdir하지 않는다.** 계산기가 새 결과 폴더를 직접 만든다.

## 4. 정지한 로봇의 pose → 점군 → pose 저장

```bash
gz topic -t /robocup/cmd_vel -m gz.msgs.Twist \
  -p 'linear { x: 0 } angular { z: 0 }'
```

GUI에서 움직임이 멈춘 뒤 다음을 실행한다. **Gazebo는 ▶ 실행 상태**로 둬야 센서 메시지가 나온다. 로봇 주행 정지와 시뮬레이션 일시정지는 다르다.

```bash
gz topic -e -n 1 --json-output -t /world/robocup_motion/pose/info \
  | jq -s -e 'select(length == 1) | .[0]' > "$SAMPLE_DIR/base_pose_before.json" &&
gz topic -e -n 1 --json-output -t /robocup/mid360s/scan/points \
  | jq -s -e 'select(length == 1) | .[0] | select(.data != null)' \
  | gzip > "$SAMPLE_DIR/points_raw.json.gz" &&
gz topic -e -n 1 --json-output -t /world/robocup_motion/pose/info \
  | jq -s -e 'select(length == 1) | .[0]' > "$SAMPLE_DIR/base_pose_after.json" &&
gzip -cd "$SAMPLE_DIR/points_raw.json.gz" \
  | jq 'del(.data)' > "$SAMPLE_DIR/points_header.json"
```

한 번의 메시지를 저장하고 명령이 끝난다. 계속 녹화하거나 rosbag에 전체 센서를 저장하지 않는다. 전후 pose를 보관하지만 현재 Mid360 도구가 정지·타임스탬프 일치를 자동 판정하지는 않는다. 측정 중 움직였다면 새 번호에서 다시 수집한다.

## 5. 점 좌표와 이번 자세의 방 기준표 생성

```bash
.venv/bin/python calibration_workflow.py decode-cloud \
  --input "$SAMPLE_DIR/points_raw.json.gz" \
  --output "$SAMPLE_DIR/xyz.npz" &&
.venv/bin/python calibration_workflow.py room-targets \
  --world "$SESSION_DIR/config/world.sdf" \
  --pose "$SAMPLE_DIR/base_pose_before.json" \
  --floor-verified --all-walls \
  --output "$SESSION_DIR/config/target_planes_train.csv"
```

`xyz.npz`는 **livox 좌표의 원시 XYZ**다. `xyz.html`은 XY/XZ/YZ 세 방향 보기다. 기준표는 실제 base pose와 독립적인 바닥·벽으로 만든 **base 좌표의 평면**이며 센서 장착 정답을 계산 입력으로 넣지 않는다.

```bash
xdg-open "$SAMPLE_DIR/xyz.html"
xdg-open "$SESSION_DIR/config/target_planes_train.csv"
```

## 6. 학습 점군으로 보정값 계산

```bash
.venv/bin/python calibration_workflow.py auto-room \
  --input "$SAMPLE_DIR/xyz.npz" \
  --targets "$SESSION_DIR/config/target_planes_train.csv" \
  --initial-rpy-deg 180 0 0 \
  --output-dir "$CALIB_DIR"
```

현재 장면의 Mid360은 roll 약 180°로 뒤집혀 있다. 초기 방향을 0°로 놓으면 바닥을 잘못 대응할 수 있다. 이 옵션은 대략적인 탐색 방향이며, 정확한 이동·회전은 학습 점군으로 구한다.

```bash
xdg-open "$CALIB_DIR/base_mid360.json"
xdg-open "$CALIB_DIR/automatic_extraction.json"
```

| 파일/항목 | 뜻 |
|---|---|
| `base_mid360.json` | Base←Mid의 위치·방향. 다음 적용·평가의 입력 |
| `translation_xyz_m` | base에서 센서 원점까지의 위치, m |
| `quaternion_xyzw` / `matrix4x4` | 센서 방향 / 전체 변환 |
| `computed_validation_pending` | 계산됨. 독립 정확도 평가 완료 상태는 아님 |
| `pairs.csv`·평면 JSON | 계산에 사용한 바닥/벽 대응 |
| `automatic_extraction.json` | 점 개수·평면 지지점·추출 기록 |

## 7. 최신 로봇 구조에 보정값을 넣고 RViz 열기

처음 Mid360만 보정하면 `robocup.urdf`를 사용한다. **이전 센서 보정도 누적하려면 직전 보정 URDF**를 입력한다. 현재 Git의 `robocup.calibrated.urdf`에는 보관된 Base–2D와 Mid360 결과가 최신 랙/PiPER 구조 위에 적용돼 있다. 이전 랙의 전체 URDF를 새 명목 파일에 덮어쓰지 않는다.

```bash
.venv/bin/python calibration_workflow.py patch-urdf \
  --urdf "$REPO/simulation/robot_description/robocup.calibrated.urdf" \
  --result "$CALIB_DIR/base_mid360.json" \
  --mount-child livox_frame \
  --output "$RUNTIME_FILE"
```

생성 파일은 `RUNTIME_FILE`과 그 옆의 `.application.json`이다. 후자는 어떤 모델·결과로 적용했는지 남긴 기록이다. 센서 joint의 **로컬 xyz**는 전체 Base←Mid 위치와 다를 수 있다.

새 ROS 터미널에서 아래를 입력한다. 새로 계산한 모델은 `RUNTIME_FILE` 경로를 자신의 결과 파일로 바꾼다. 아래 기본값은 Git에 보관한 최신 구조의 후보 모델이다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=73
export GZ_IP=127.0.0.1
export GZ_PARTITION=robocup_motion
export REPO="$HOME/Documents/INHA-RoboCup-Home-2th-2027"
export RUNTIME_FILE="$REPO/simulation/robot_description/robocup.calibrated.urdf"
cd "$REPO"
ros2 launch simulation/ros2/calibration_runtime.launch.py \
  urdf:="$RUNTIME_FILE" joints:=true rviz:=true
```

기존 `robot_state_publisher`를 켜둔 터미널은 Ctrl+C로 종료한 후 새 모델을 실행한다. 같은 base/sensor TF를 중복 발행하지 않는다. Gazebo Server/GUI는 계속 켜둔다.

RViz에서 TF가 겹쳐 보이면:

1. Fixed Frame을 `base_link`로 둔다.
2. Displays → Calibrated TF → `Filter (whitelist)`를 `^(base_link|livox_frame)$`로 바꾼다.
3. Show Axes/Names는 켜고 Show Arrows는 끈다. Marker Scale은 0.15 정도로 둔다.
4. 팔·손목을 볼 때는 필터를 기존 값으로 되돌린다.

이 런치는 TF와 실제 관절 표시용이다. 점군 영상 bridge나 Nav2 전체 실행은 포함하지 않는다. **이 다음이 정확도 평가다. 여기서 튜토리얼을 끝내지 않는다.**

## 8. 검증 A 준비 — 보정값 고정, 새 저장 폴더 만들기

Server/GUI는 그대로 유지한다. 아래는 **Git에 보관된 기존 보정값**을 평가하는 독립 Control 설정이다. 방은 현재 실행 중인 방을 새 세션에 복사한다. 새로 계산한 값을 평가하려면 `MID_RESULT`만 해당 `base_mid360.json`으로 바꾼다.

```bash
export REPO="$HOME/Documents/INHA-RoboCup-Home-2th-2027"
export PROJECT="$REPO/simulation/calibration"
cd "$PROJECT"
export GZ_IP=127.0.0.1
export GZ_PARTITION=robocup_motion
set -o pipefail
set -o noclobber
export MID_RESULT="$PROJECT/records/20261006_base_mid360/results/auto_room_20261006_031114/base_mid360.json"
export EVAL_RUN="$(date +%Y%m%d_%H%M%S)"
export SESSION_DIR="$PROJECT/data/mid360_evaluation_$EVAL_RUN"
mkdir -p "$SESSION_DIR/config" "$SESSION_DIR/validation" "$SESSION_DIR/results"
cp "$REPO/simulation/gazebo/build/motion.world.sdf" "$SESSION_DIR/config/world.sdf"
git -C "$REPO" rev-parse HEAD > "$SESSION_DIR/config/repo_commit.txt"
printf 'Base–Mid360 정확도 평가\n보정값: %s\n실행 world: %s/config/world.sdf\n상태: 새 관측 대기\n\n## 내가 기록할 관찰\n' \
  "$MID_RESULT" "$SESSION_DIR" > "$SESSION_DIR/session.md"
export VALIDATION_ID=A
export SAMPLE_DIR="$SESSION_DIR/validation/$VALIDATION_ID"
export EVAL_DIR="$SESSION_DIR/results/validation_$VALIDATION_ID"
mkdir "$SAMPLE_DIR"
```

이전 측정 당시의 `config/world.sdf`를 새 방으로 덮어쓰지 않는다. Gazebo가 다른 world를 실행 중이면 위 cp의 입력을 **실제 실행한 world**로 지정한다. 저장할 방과 실행한 방이 같아야 한다.

## 9. GUI를 켜둔 상태에서 로봇 이동·회전

### 검증 A — x=1.0m, y=0.5m, yaw=30°

Control에서 실행한다. 먼저 현재 높이를 읽어서 바닥 위 로봇을 불필요하게 위아래로 이동시키지 않는다.

```bash
gz topic -t /robocup/cmd_vel -m gz.msgs.Twist \
  -p 'linear { x: 0 } angular { z: 0 }' &&
gz topic -e -n 1 --json-output -t /world/robocup_motion/pose/info \
  | jq -s -e 'select(length == 1) | .[0]' > "$SAMPLE_DIR/move_from_pose.json"
export BASE_Z="$(jq -er '.pose[] | select(.name == "robocup") | .position.z' "$SAMPLE_DIR/move_from_pose.json")"
gz service -s /world/robocup_motion/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
  --req "name: \"robocup\" position { x: 1.0 y: 0.5 z: ${BASE_Z:?현재 높이 읽기부터 실행} } orientation { x: 0 y: 0 z: 0.2588190451 w: 0.9659258263 }" \
  | tee "$SAMPLE_DIR/set_pose_response.txt"
```

`data:true` 응답 뒤 GUI에서 로봇이 새 위치로 옮겨지고 흔들림이 멈출 때까지 기다린다. 물체와 충돌하는 위치에서는 측정하지 않는다. **요청한 자세를 실제 측정 pose로 대신 적지 않는다.** 다음 단계가 실제 pose를 다시 저장한다. 이동 명령이 실패했으면 다음 측정을 진행하지 않는다.

### 검증 B — x=-0.5m, y=1.0m, yaw=-45°

**A의 10번·11번까지 끝낸 뒤** 같은 Control에서 다음을 실행한다.

```bash
export VALIDATION_ID=B
export SAMPLE_DIR="$SESSION_DIR/validation/$VALIDATION_ID"
export EVAL_DIR="$SESSION_DIR/results/validation_$VALIDATION_ID"
mkdir "$SAMPLE_DIR"
gz topic -t /robocup/cmd_vel -m gz.msgs.Twist \
  -p 'linear { x: 0 } angular { z: 0 }' &&
gz topic -e -n 1 --json-output -t /world/robocup_motion/pose/info \
  | jq -s -e 'select(length == 1) | .[0]' > "$SAMPLE_DIR/move_from_pose.json"
export BASE_Z="$(jq -er '.pose[] | select(.name == "robocup") | .position.z' "$SAMPLE_DIR/move_from_pose.json")"
gz service -s /world/robocup_motion/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
  --req "name: \"robocup\" position { x: -0.5 y: 1.0 z: ${BASE_Z:?현재 높이 읽기부터 실행} } orientation { x: 0 y: 0 z: -0.3826834324 w: 0.9238795325 }" \
  | tee "$SAMPLE_DIR/set_pose_response.txt"
```

다시 GUI에서 안정된 모습을 보고 **10번·11번을 그대로 반복**한다. `MID_RESULT`는 A에서 사용한 파일을 유지한다. 새 extrinsic을 B에 맞춰 재계산하지 않는다.

## 10. 새 실제 pose·점군 저장 → 기존 보정값으로 오차 계산

A와 B 각각에서 실행한다. GUI는 ▶ 실행 상태로 두고 로봇은 정지시킨다.

```bash
gz topic -e -n 1 --json-output -t /world/robocup_motion/pose/info \
  | jq -s -e 'select(length == 1) | .[0]' > "$SAMPLE_DIR/base_pose_before.json" &&
gz topic -e -n 1 --json-output -t /robocup/mid360s/scan/points \
  | jq -s -e 'select(length == 1) | .[0] | select(.data != null)' \
  | gzip > "$SAMPLE_DIR/points_raw.json.gz" &&
gz topic -e -n 1 --json-output -t /world/robocup_motion/pose/info \
  | jq -s -e 'select(length == 1) | .[0]' > "$SAMPLE_DIR/base_pose_after.json" &&
.venv/bin/python calibration_workflow.py decode-cloud \
  --input "$SAMPLE_DIR/points_raw.json.gz" \
  --output "$SAMPLE_DIR/xyz.npz" &&
.venv/bin/python calibration_workflow.py room-targets \
  --world "$SESSION_DIR/config/world.sdf" \
  --pose "$SAMPLE_DIR/base_pose_before.json" \
  --floor-verified --all-walls \
  --output "$SAMPLE_DIR/target_planes.csv" &&
.venv/bin/python calibration_workflow.py auto-room \
  --input "$SAMPLE_DIR/xyz.npz" \
  --targets "$SAMPLE_DIR/target_planes.csv" \
  --reference "$MID_RESULT" \
  --output-dir "$EVAL_DIR" &&
.venv/bin/python calibration_workflow.py validate-planes \
  --result "$MID_RESULT" \
  --validation "$EVAL_DIR/pairs.csv" \
  --max-angle-deg 1 --max-offset-mm 10 \
  --output "$EVAL_DIR/validation_report.json"
```

- **새 pose → 새 기준표:** 벽은 그대로지만 로봇이 움직였으므로 base에서 본 벽·바닥 위치가 달라진다. train의 기준표를 재사용하지 않는다.
- **`--reference`:** A/B에서 기존 보정값을 고정한다. 평면은 새 점군에서 추출하지만 extrinsic은 다시 추정하지 않는다. 새 `base_mid360.json`은 생성하지 않는다.
- 평면이 부족하거나 대응을 못 찾으면 평가 불가다. 보정값·시야·방·pose를 구분해서 확인하고 새 관측을 얻는다.
- 기준을 넘으면 마지막 명령은 ERROR로 끝나지만 **실패 보고서는 먼저 저장**된다. 보고서는 다음 블록으로 따로 연다.

## 11. 어디서 보고, 무엇이면 잘 된 것인가?

```bash
xdg-open "$EVAL_DIR/validation_report.json"
xdg-open "$EVAL_DIR/automatic_extraction.json"
xdg-open "$EVAL_DIR/pairs.csv"
xdg-open "$SAMPLE_DIR/xyz.html"
xdg-open "$SESSION_DIR"
```

| 볼 값 | 읽는 방법 | 이번 임시 판정 |
|---|---|---|
| `normal_errors_deg` | 보정 후 벽·바닥 법선 방향 차이, ° | 모든 평면 ≤1° |
| `offset_errors_mm` | 보정 후 벽·바닥 위치 차이, mm | 모든 평면의 절댓값 ≤10mm |
| `status` | passed/failed_provisional_limits | 위 기준의 통과/실패 |
| `refitted` / `extrinsic_refitted` | 기존 보정값으로 평가했는가 | false |
| `observed_planes[].inlier_count` | 각 면을 뒷받침하는 점 수 | 모든 필요한 면이 관측되어야 함 |
| `automatically_assigned_points` / `unassigned_points` | 평가에 사용·제외한 점 수 | 제외 수와 시야를 함께 해석 |

오차 배열의 면 이름은 `pairs.csv`의 행 순서로 찾는다. 1°/10mm는 **실습의 임시 기준**이며 제조사 보증이나 실제 집기 성공 기준이 아니다. 위치 약 5mm·각도 0.2~0.5° 같은 별도 목표와도 구분한다.

현재 자동 대응은 기준 평면에서 기본 50mm 이내인 점을 골라 평가한다. 따라서 이 보고서는 **선택된 방 평면의 일치도**다. 전체 점군 RMS나 Gazebo 장착 정답과의 직접 위치 오차를 뜻하지 않는다. 점을 많이 버렸거나 일부 면만 보이는 상황에서 작은 오차만 보고 성공으로 쓰지 않는다. 추출 평면의 `rms_m`은 평면의 평평한 정도이지 extrinsic 정확도 자체가 아니다.

**A와 B에서 같은 보정값으로 모든 필요한 면이 관측되고 각각 임시 기준을 만족하는지** 판단한다. 통과하면 이번 방/관측 조건에서의 검증 근거를 얻은 것이다. 원본 보정 JSON의 `computed_validation_pending`은 자동으로 바뀌지 않으며, 별도 `validation_report.json`이 평가 기록이다.

## 12. 자동 기록, 실패했을 때, 다음 단계

A/B 평가 명령이 끝난 뒤 경로와 판정 요약을 일지에 추가한다. 실패 보고서가 저장됐으면 실패 상태도 기록된다. 보고서가 없으면 미생성으로 남는다.

```bash
printf '\n## 검증 %s\n보정값: %s\n원자료: %s\n보고서: %s/validation_report.json\n' \
  "$VALIDATION_ID" "$MID_RESULT" "$SAMPLE_DIR" "$EVAL_DIR" >> "$SESSION_DIR/session.md"
if [ -f "$EVAL_DIR/validation_report.json" ]; then
  jq -r '"판정: " + .status, "각도 오차(deg): " + (.normal_errors_deg | tostring), "위치 오차(mm): " + (.offset_errors_mm | tostring)' \
    "$EVAL_DIR/validation_report.json" >> "$SESSION_DIR/session.md"
else
  printf '판정: 보고서 미생성 — 수집/대응 오류를 먼저 해결\n' >> "$SESSION_DIR/session.md"
fi
xdg-open "$SESSION_DIR/session.md"
```

직접 적을 부분은 **GUI에서 본 가림·흔들림, 실패 때 실제 오류 문구, 실험 판단**이다. 실험 파일은 Ubuntu PC의 `SESSION_DIR`에 남는다. Notion/옵시디언에는 자동 업로드되지 않는다. 거기에는 이 경로와 보고서 요약을 기록한다.

| 막힌 상황 | 다음 행동 |
|---|---|
| `File exists: '.'` / `/base_mid360.json` 없음 | 새 창에서 경로 export가 빠짐. 1번 또는 독립 설정인 8번부터 다시 설정 |
| `File exists`로 출력 거부 | 원본 삭제 대신 새 세션/VALIDATION_ID 사용. EVAL_DIR/CALIB_DIR은 미리 만들지 않음 |
| 이동 응답 실패 / 로봇이 안 움직임 | Server와 Control partition 일치, Gazebo 실행 상태, robocup 모델 선택 확인 |
| 수집 명령이 끝나지 않음 | ▶ 재생·센서 토픽·partition 확인. Control에서 Ctrl+C로 중단 후 새 번호로 수집 |
| 평면 부족 / 대응 실패 | 바닥·네 벽 시야와 로봇 가림, 방과 pose, roll 180° 조건 확인 |
| 각도/거리 기준 초과 | 같은 보정값으로 실패를 기록. frame 방향·방 기준·움직임부터 확인하고 필요한 새 train 수집 |
| 새 Git URDF와 옛 보정 URDF의 팔 위치가 다름 | 최신 nominal에서 결과 JSON만 다시 적용. 옛 전체 URDF로 최신 구조를 덮어쓰지 않음 |

**자료 보존:** train과 validation은 분리한다. 새 URDF로 재시작한 실험은 새 world/commit/pose와 함께 기록한다. JSON 결과를 새로 만들어도 A/B 오차를 다시 맞춰 통과시키지 않는다. 평가가 만족스러우면 그 보정 JSON과 적용 URDF, A/B 보고서 경로를 다음 Head D435–Mid360 과정에 넘긴다.

이 문서와 Git 후보 모델을 작성한 작업에서는 새 수집·Gazebo/ROS 실행·정확도 평가·테스트를 하지 않았다.
