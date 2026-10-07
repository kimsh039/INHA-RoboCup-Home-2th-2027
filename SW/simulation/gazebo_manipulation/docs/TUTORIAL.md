# 실행 가이드

## 환경

Ubuntu22.04 / Python3.10 / ROS2 Humble / Gazebo Harmonic(gz-sim8), ROS↔Gazebo bridge가 있는 PC에서 사용합니다. 기존 ROS Python 환경을 사용하며 학습 venv를 겹쳐 활성화하지 않습니다. 설치가 필요하면 기존 [개발 PC setup](../../../setup/README.md)을 읽고 Gazebo 설치와 ROS distribution 조합을 확인합니다.

ROS 의존성은 패키지 manifest에 선언되어 있습니다. rosdep이 준비된 환경에서 저장소 루트 기준:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths SW/simulation/gazebo_manipulation/gazebo_ws/src --ignore-src -r -y
bash SW/simulation/gazebo_manipulation/build.sh
```

사용자가 성공 실행한 원래 PC에는 이 환경이 이미 설치되어 있습니다. 위 설치·relocated build를 게시 과정에서 다시 실행하지 않았습니다.

## 기준 장면과 실행

공용 nominal URDF로 TRACER/PiPER를 만들고 테이블(0.68,0,0.695)m, 40mm cube(0.5,-0.1,0.7401)m, 접근 완료 base 위치를 MuJoCo와 맞춥니다. 주행을 실제 실행하지 않습니다. wrist 관측 자세는 MuJoCo의 live_observation 기록을 사용합니다.

터미널1, 저장소 루트:

```bash
bash SW/simulation/gazebo_manipulation/run.sh gui:=true
```

MoveIt 준비 후 터미널2, 저장소 루트:

```bash
bash SW/simulation/gazebo_manipulation/command.sh tutorial
```

아래 stage를 거칩니다.

```text
WAIT_FOR_GAZEBO → MOVE_TO_INITIAL → OPEN_GRIPPER
→ MOVE_TO_WRIST_OBSERVATION → ACQUIRE_WRIST_RGBD
→ SELECT_GRASP → APPROACH_GRASP → CLOSE_GRIPPER → VERIFY_GRASP
→ LIFT_OBJECT → PLAN_PLACE_LEFT/RIGHT → MOVE_TO_PLACE → LOWER_OBJECT
→ OPEN_GRIPPER → RETREAT → RETURN_TO_INITIAL → DONE
```

기본은 GUI off입니다. `room:=true`는 내비게이션용 방 형상을 추가합니다. `mid360:=true g2:=true head_camera:=true`로 해당 센서를 켤 수 있습니다. 실제 성공 경량 경로는 wrist RGB/depth320×240@5Hz입니다. `task_server:=true`는 향후 Nav2→PickPlace 연계용이며 현재 성공 실행에서 사용하지 않았습니다.

ROS_DOMAIN_ID와 Gazebo partition을 실행별로 선택합니다. command.sh는 현재 runtime/ros_domain을 읽습니다. 다른 ROS 모듈도 같은 domain/use_sim_time를 사용해야 합니다. 기존 server에 붙는 start_sim:=false는 기존 server의 모델·센서·TCP·contact 설정이 이 generator와 맞아야 하며 검증된 재현 명령은 아닙니다.

## 밀림과 허용값

기본 `slip_experiment:=true`와 tutorial의 `--slip-experiment`가 베이스 밀림을 관찰합니다. base pose/state/속도를 그대로 기록하며 고정하지 않습니다. 통신·clock·관절 한계·cancel 보호는 유지합니다. TCP 정밀 도달 기준(3mm/0.04rad)을 넘으면 오차를 기록하고 최대20mm/0.1rad 범위에서 실험을 계속합니다. 물체 유지 기준은 위치25mm/회전60°입니다. 양손가락 접촉·lift 높이·지지·배치 검사는 계속 적용합니다. 실험이 정밀 기준을 충족했다고 위장하지 않습니다.

원래 정밀 기준으로 실행하려면:

```bash
# 터미널1
bash SW/simulation/gazebo_manipulation/run.sh gui:=true slip_experiment:=false
# 터미널2
bash SW/simulation/gazebo_manipulation/command.sh tutorial --no-slip-experiment
```

이는 기록된 성공 조건이 아니며 strict 성공을 주장하지 않습니다.

## 결과와 진단

`reports/gazebo_tutorial.json`은 최신 실행으로 덮어써집니다. 게시된 기록을 보존하려면 별도 복사합니다. stage별 수치는 `.trace.jsonl`, stderr/traceback은 `runtime/command_logs/`, ROS launch 로그는 `runtime/ros_logs/`에 남습니다.

- ARM_PROGRESS: 현재 궤적 시간 진행률이며 전체 완료율이 아닙니다.
- real_time_factor: sim time / wall time. 작으면 physics/rendering 처리 자체가 느립니다.
- CARTESIAN_TIMING: MoveIt TOTG가 생성한 trajectory의 simulation 길이입니다. 2mm 샘플마다 정지 시간을 합산하던 Gazebo backend를 수정했습니다.
- TCP_DRIFT_OBSERVED: 정밀 기준 초과를 기록하고 bounded 실험으로 진행합니다.
- GRASP/LIFT_VERIFICATION_FAILED: 실제 접촉/물체 유지 검사 실패. 성공으로 간주하지 않습니다.

물체 감지는 known-cube ground truth ROI입니다. 실제 wrist depth 점군은 저장하지만 기록 GraspNet 후보를 재사용하며 이 capture로 새 GPU 추론이나 visual servo를 실행하지 않습니다.
