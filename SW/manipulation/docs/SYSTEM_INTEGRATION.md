# Jetson 실물 시스템 통합 패키지 — 2026-10-15

실행 코드는 기존 `robocup_real_manipulation` 및 `robocup_real_manipulation_msgs` 패키지에 있다. 새 ROS 메시지/패키지를 만들지 않았다. 실제 팔·그리퍼·MoveIt·RGB-D 입력을 사용하는 경로이며, `integrated.launch.py`는 mock/bench/phase1 client를 실행하지 않는다. 네비게이션·센서 퓨전을 대신 구현하지 않는다.

**현재 납품은 로컬 소스와 업로드용 압축 파일이다. 현재 실환경은 필요한 외부 입력이 없어 전체 작업을 실행할 수 없다. 기본 `execution_enabled: false`와 미정 수치를 유지한다.** 하드웨어 구동은 하지 않았다.

## 1. 확인한 실제 상태와 연결 경계

**추가 확인:** 아래 그래프 기록은 최초 조사 시점의 상태다. 이후 Jetson의 최신 관제센터에서 YOLO+MID-360 좌표와 접근 상태를 발행하는 별도 JSON 계약을 확인하여, 웹 변경 없이 받는 `control_center_input`을 추가했다. 현재 연결 방법과 남은 실제 베이스 잠금 조건은 [CONTROL_CENTER_CONNECTION.md](CONTROL_CENTER_CONNECTION.md)를 따른다. 아래의 HeadTarget 미발행 기록을 현재 3D 좌표 전체가 없다는 뜻으로 해석하지 않는다.

- GitHub origin HEAD와 로컬 tracked HEAD: `a75350164913198315b3c674fd14e2e2a0be2b36` (2026-10-15 조회). GitHub 변경을 받기 위해 기존 checkout을 reset/pull하지 않았다.
- Jetson checkout: `ee6f9c1bb3440b6edbe4367851746ffac6b28fb8`; 관제·충돌·SLAM·드라이버 관련 다른 사람의 다수 수정/새 파일이 존재했다. 전부 보존했다.
- 실제 관제 프로세스는 `head-detection-node.py` + `head_yolo11n_20261013.engine`. `/detection/head/objects`의 `std_msgs/String` JSON은 여섯 클래스의 **픽셀 bbox**다. 실제 받은 메시지에서 `classes=[green_apple,peach,tomato,fanta_can,cup,banana]`, 물체별 class_id/label/confidence/x/y/width/height를 확인했다. 이를 3D나 작업 선택 결과로 변환하지 않는다.
- 저장소 `robocup_head_detection/nav_goal_node.py`가 Mid360 점들을 각 취득시각 TF로 map에 누적하고, Head bbox에 투영하여 `nav_geometry.lidar_target()`에서 3D를 만든다. 3회 확인 후 `/detection/target_map_point`에 `PointStamped(frame=map,stamp=Head취득시각)`를 발행한다. 이 노드는 `use_sim_time=true`를 요구하는 Gazebo 경로다. 이번에 실물 퓨전으로 재작성/실행하지 않았다.
- 현재 Jetson ROS 그래프에는 위 `HeadTarget` 및 퓨전 3D publisher, `/manipulation/target`, `approach_result`, `base_lease`가 없다. `dock_node`의 `DOCKED` 문자열과 `/cmd_vel=0`은 이 계약을 대체하지 못한다.
- 기존 `head_input_adapter`는 **동일 정수 sec/nanosec의 HeadTarget + PointStamped**만 결합한다. 취득시각 TF로 `odom`에 즉시 보존하며 원래 stamp를 유지한다. 중복 발행과 cache 크기를 제한한다. JSON bbox만 있는 현재 실물 그래프에서는 사용할 수 없다. 기본 실행은 adapter 비활성, 외부의 기존 `Target` 직접 입력이다.
- 이 호환 adapter는 Head의 target_id를 task_id/instance_id로 쓴다. 한 track당 한 번만 가능하며 상위 작업 UUID와 instance ID를 별도로 관리하는 시스템은 `Target`을 직접 보내야 한다. 같은 물체의 새 작업에도 새 task_id가 필요하다.

```text
상위 대상 선택 + 기존 실제 3D 퓨전
  ├─ Target(task_id, class_id, instance_id, 고정좌표 3D, 취득시각) ─────┐
  └─ 실제 네비게이션/접근 ─ ApproachResult(동일 task_id) ────────────┤
상위 Manipulate(task_id) ───────────────────────────────────────────┤
                                        WAIT_FOR_APPROACH          │
접근 성공 후 base_lock_request(true) → 외부 이동 명령 소유자         │
                           ← BaseLease(동일 ID, 실제 locked/stopped) │
                                        관측팔 이동 → 손목 요청     │
기존 RealSense RGB + factory 정합 depth + factory RGB CameraInfo    │
     → 요청 중에만 YOLO 재검출 + SAM2 → 실제 마스크/표면점/취득 TF     │
     → 클래스별 탑다운 규칙 → MoveIt 장면/계획 → Piper bridge        │
     → 접촉폭/힘 확인 → lift → 오른쪽/왼쪽 place → retreat/return     │
base_lock_request(false) → 외부 실제 해제 응답(BaseLease false)       │
     → TaskStatus/Manipulate SUCCESS → 다음 작업/다음 이동           ┘
```

`auto_start_on_approach=true`는 Target와 실제 성공 ApproachResult가 있을 때 자기 action client로 시작하는 선택 기능이다. 기본은 false. 외부 action client와 동시에 사용하지 않는다. 현재 누락된 접근·정지 입력을 만들어내지 않는다.

## 2. 실제 인터페이스 계약

아래 사용자 정의 타입의 패키지는 모두 `robocup_real_manipulation_msgs`다. 단위는 m/rad/s, stamp는 ROS clock의 **원본 취득시각**이다. 현재 하드웨어는 `use_sim_time=false`를 쓴다.

| 이름 | 타입 / 주요 필드 | QoS / 소유자 |
|---|---|---|
| `/manipulation/target` | `Target`: header, task_id, class_id, instance_id, coarse_position | reliable/volatile depth10 수신. 외부 선택/퓨전 소유 |
| `/manipulation/approach_result` | `ApproachResult`: header, task_id, outcome SUCCESS/FAILED/CANCELED, base_stopped, reason | reliable/volatile depth10, 실제 접근 소유자 |
| `/manipulation/run` | `Manipulate` action: goal.task_id; feedback/result.status=`TaskStatus` | 상위 action client. 표준 ROS cancel |
| `/manipulation/base_lock_request` | `BaseLease`: header, task_id, locked. 요청의 stopped는 증거가 아님 | reliable/transient_local depth1, coordinator 10Hz |
| `/manipulation/base_lease` | `BaseLease`: header, 같은 task_id, locked, stopped | reliable/volatile depth10 수신, **최종 이동 명령 경로의 외부 소유자** |
| `/manipulation/wrist/request` | `ObservationRequest`: header, task_id, observation_id, class_id, instance_id, coarse_position(PointStamped) | reliable/volatile depth10 |
| `/manipulation/wrist/result` | `WristObservation`: header, 두 ID, class_id, 전체 batch count/index, confidence, position_kind=SURFACE, position, std, has_position, target_mask, aligned_depth, camera_info, axis_valid/yaw | reliable/volatile depth10. 취득 프레임/시각 유지 |
| `/manipulation/status`, `/manipulation/result` | `TaskStatus`: stage/outcome/reason, holding EMPTY/HELD/UNKNOWN, place_success, return_success, stop_confirmed, class/center/TCP | reliable/transient_local depth1 |
| `/manipulation/wrist/status` | `std_msgs/String` JSON: state/valid/reason/task/observation/class/frame/capture_stamp | reliable/volatile depth10. 실패 이유도 확인 |
| `/manipulation/wrist/debug_image`, `/manipulation/wrist/mask` | `sensor_msgs/Image`, bgr8/mono8 | reliable/volatile depth1, 원본 영상 header. debug는 구독자가 있을 때 생성 |
| `/manipulation/timing` | `std_msgs/String` JSON | reliable/volatile depth1, 1Hz, 채널당 최신 값만 |
| `/wrist/wrist/color/image_raw` | `sensor_msgs/Image` | sensor_data(best_effort/volatile depth5) 수신 |
| `/wrist/wrist/aligned_depth_to_color/image_raw` | `sensor_msgs/Image`, 16UC1 mm 또는 32FC1 m | 동일 factory RGB grid |
| `/wrist/wrist/color/camera_info` | `sensor_msgs/CameraInfo` | factory RGB K/D, 같은 프레임 및 25ms 이내 |
| `/joint_states_feedback` | `sensor_msgs/JointState`: driver_joint_names, raw rad | 기본 Piper driver → bridge. 실제 이름은 measured config로 변경 |
| `/gripper_states_feedback` | `JointState`: name=['gripper'], position 1개, effort 1개 | 원본 driver stamp, width_scale/bias 적용 |
| `/arm_status` | `piper_msgs/PiperStatusMsg` | bridge, err_code/6관절 각도·통신 상태 확인 |
| `/joint_ctrl_single` | `JointState`: 6 arm + gripper position | bridge 단독 publisher. velocity[6]=전역 속도%, effort[6]=그리퍼 명령힘 |
| `/joint_states` | `JointState`: URDF 6관절+두 손가락, 원본 arm stamp | bridge → MoveIt. 다른 publisher와 동시 사용 금지 |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | MoveIt → bridge, 실제 피드백 기반 완료 |
| `/manipulation/set_width` | `SetWidth` action, width_m/timeout_s → measured_width_m/effort/reason | coordinator → bridge, 팔/그리퍼 동시 요청 배제 |
| `/manipulation/stop` | `std_srvs/srv/Trigger` | 마지막 닫힘 목표 유지, 실제 관절 정지 확인 |
| `/move_action`, `/execute_trajectory` | MoveIt actions | 기존 Humble MoveIt |
| `/compute_cartesian_path`, `/apply_planning_scene`, `/get_planning_scene`, `/check_state_validity` | MoveIt services | 실제 scene/ACM/상태검사 |

외부 BaseLease provider는 Nav2·dock·teleop·관제·collision-approved 경로 등 **모든** 최종 구동 입력을 gate해야 한다. 자체 odometry/구동 피드백으로 정지를 확인하고, 잠금 요청 이후 시각의 동일 task_id 응답과 설정 `base_age_s` 이내 heartbeat를 보내야 한다. provider가 죽거나 heartbeat가 오래되면 팔 실행은 중단된다. 이 provider는 현재 실물 네비게이션에 없으며 이번 패키지가 가짜로 대신 발행하지 않는다.

접근이 끝나기 전에 이동 잠금을 요청하지 않는다. Action 예약과 물리 잠금 요청은 분리했다. 성공 후 잠금 해제 요청에도 동일 task_id의 **새 `locked=false, stopped=true` 응답**을 기다린다. 해제 미응답은 `BASE_UNLOCK_UNCONFIRMED` 실패이며 SUCCESS로 보고하지 않는다. 상위는 최종 결과 전 새 주행을 시작하지 않는다.

필요 TF: `map→odom→base_link`, 실제 URDF 팔 사슬 `base_link→piper_link6`, calibrated `piper_link6→wrist_color_optical_frame`, 내부 TCP 정의. 이동 전 Target는 `coarse_fixed_frame`(기본 odom)에 고정한다. 도착 후 lease 취득시각의 base_link로 변환한다. `base_link`에 고정한 정차 좌표는 통합 action에서 거절한다. TF가 취득시각에 없으면 진행하지 않는다.

외부 RSP/sensor_tf/RealSense가 이미 public TF를 발행하면 `integrated.launch.py`의 `start_robot_state_publisher=false`를 유지한다. 이 옵션은 bringup의 과거 기본값과 다르다. 외부 TF가 전혀 없을 때만 별도 소유권 확인 후 true로 설정한다. 관측 계산 전 실제 `piper_link6←camera` TF와 설정 hand-eye가 2mm/0.02rad 이내 일치하는지 검사한다. 새 측정 정확도를 보증하는 오차 값이 아니라 서로 다른 보정본 사용을 방지하는 비교 한계다.

## 3. 파지 규칙·관측·충돌

| class ID / 대상 | 크기 정보 mm | 수직 높이 / 파지 높이 mm | 개방 / 닫힘 mm | 현재 준비 상태 |
|---|---|---|---|---|
| 0 green_apple 청사과 | 80×80 | 68 / 34 | 100 / 79 | 정차 실물 성공 근거 있음. 전체 주행 통합 미검증 |
| 1 peach 복숭아 | 80×70 | 68 / 34 | 100 / 79 | 규칙 구현, 접촉·충돌 실물 commissioning 미확정 |
| 2 tomato 토마토 | 83×78 | 68 / 34 | 100 / 82 | 규칙 구현, 연질 물체 접촉힘·경로 미확정 |
| 3 fanta_can 환타 | 지름64.6 | 124 / 62 | 80 / 64 | 몸통 양옆 파지 구현, 접촉·공간 미확정 |
| 4 cup 컵 | **몸통**지름91 | 78 / 39 | 100 / 90 | 손잡이 포함 충돌 범위·접촉 미확정 |
| 5 banana 바나나 | 길이160, 폭30 | **30 가정 / 15 가정** | **40 / 26 trial** | 전체 PCA/몸통점 구현, 실측 활성화 필요 |

- 크기/사용자 높이와 휴리스틱 폭·접촉 높이를 구분한다. 모든 숫자를 검증 완료값으로 표시하지 않는다. 설정은 m로 저장한다. `robot.yaml`은 기하/보정/준비 상태, `tuning.yaml`은 관측거리·접촉높이·개폐 폭·속도 등 trial overlay다. `class_execution.<name>.ready`와 미확정 이유를 측정 config에서 관리한다. 물리 성공 기록 유무만으로 모든 규칙을 청사과 전용으로 축소하지 않았다. 다른 클래스의 실제 구동 허용은 필요한 기하·접촉 조건 검토 후 설정한다.
- 과일의 XY는 실제 손목 mask/depth에서 얻는다. 표면점 Z에 대해 `table_z + height`와의 일치 검사를 한 뒤 center=`table_z + center_height`, TCP grasp=`table_z + grip_height`를 각각 한 번만 계산한다. 관측 표면점을 중심으로 오인하거나 높이를 두 번 빼지 않는다. 복숭아/토마토 방향 PCA 요구는 추가하지 않았다.
- 환타/컵은 세워진 몸통 양옆을 잡는다. 컵 손잡이 파지는 도입하지 않았다. cup.radius_m의 기존45.5mm는 몸통 반경에 불과하다. 전체 mask가 이를 초과하면 `MASK_EXCEEDS_COLLISION_EXTENT`; handle 포함 collision radius를 실제 확인 전 ready=false다.
- 바나나는 전체 mask PCA를 gravity XY에서 계산한다. 중심 투영 구간의 두꺼운 **실제 몸통 내부점**을 선택해 그 점의 depth로 XY를 정하고 축을 평행이동한다. 무게중심은 grasp point가 아니다. CAD 손가락은 TCP ±Y로 닫히며, topdown Rx(pi)에서 yaw=`PCA + π/2 + closing_axis_yaw_offset`; offset=-π/2이면 TCP X가 장축, TCP Y가 폭 방향이다. 이 축 관계는 CAD 초기값이다. 110mm 초기 collision radius 안에 선택 몸통점 기준 전체 실루엣이 들어오는지 검사하며 3개 프레임의 위치 및 modulo-π 축 안정성을 요구한다.
- 관측 광축을 coarse target으로 향하게 하고 hand-eye/TCP를 반영한다. 현재 CameraInfo의 양쪽 시야, 클래스 collision radius, coarse XY/Z 오차 경계, 최소 depth거리 및 이미지 여백으로 **전체 물체**가 들어오는 최소 광학 거리를 계산한다. 250mm 표면 clearance는 초기 최소값이며 강제 고정거리가 아니다. 거리 상한/팔 작업 범위를 넘거나 MoveIt이 거절하면 실패한다. 관측 재시도 최대3회, XY 이동 반경 기본80mm, offset 목록 최대4개, 각 요청 시간·프레임 수 제한을 적용한다. 같은 위치의 성공 반복을 전제하지 않는다.
- `scene_frame`과 `table_bounds_xy/table_z_m/obstacles`는 odom(또는 설정 map)에 측정되어야 한다. 현재 base_link 상대 정차 설정을 복사하지 않는다. 도착 시 scene 전체를 변환하고 회전 테이블은 실제 convex polygon으로 support 검사한다. 상판 두께28mm는 사용자 확인값이다. 세계 배치는 임의로 정하지 않는다.
- 손목의 실제 전체 mask 범위와 depth로 collision extent를 확인한다. target mask 밖에서 테이블보다 높은 관측 depth를 bounded voxel box로 scene에 추가한다(기본 stride6px/20mm/최대600개). 기존 외부 장애물과 로봇 collision geometry도 유지한다. 관측하지 못한 뒤쪽/가려진 공간까지 안전하다고 보증하는 구현은 아니다. 주변 환경의 미관측 장애물은 외부 scene의 책임이다.
- 놓기는 물체의 관측 위치 기준 **수평 base_link -Y 100mm(오른쪽) 우선, +Y 100mm(왼쪽) 차선**이다. support polygon·반경·여유, static/measured objects·로봇, hover와 descent 계획을 검사한다. 오른쪽 계획이 안 되면 왼쪽을 검사한다. 실제 실행 실패를 무시하고 반대편으로 재실행하지 않는다.
- 접촉 ACM은 현재 전체 matrix/SRDF/default를 읽은 후 manip_target 쌍만 바꾼다. lift/place 후 table 접촉 허용도 다시 철회한다. attach와 world REMOVE를 중복하지 않는다. detach 후 생길 수 있는 target alias는 scene에서 존재를 확인한 뒤 별도 제거한다. 놓은 객체는 고정 좌표에 보존해 다음 작업의 도착 좌표로 다시 변환한다. history는32개 제한으로 무한 scene 누적을 차단하며 한계 시 scene 갱신이 필요하다.

## 4. 성공 기록 반영 범위

실제 기록: `7ece08d387f547cd942d794076f82bb0`, 2026-10-15 00:33:20.046874 KST, COMPLETE/SUCCESS, place/return=true, EMPTY/stop=true. 초기 XYZ는 임시 입력이므로 실제 이동 통합 성공 근거가 아니다.

- 닫힘 목표79mm에서 실제80.605562578mm, effort=-0.358을 기록했다. 접촉을 목표폭 오차1mm 이내로 강제하던 판정을 수정했다. 실제 폭/힘/열림 및 빈 닫힘 조건을 함께 검사한다. 힘은 기록된 squeeze 부호를 고려해 magnitude를 사용한다.
- grasp 때 얻은 실제 폭을 별도 저장하고 이후 변화량+힘으로 hold를 확인한다. release의 개방 결과로 최초 grasp 폭을 덮어쓰지 않는다.
- bridge는 첫 닫힘 명령을 보내기 전 목표를 저장한다. 팔 이동·hold·실패 정지 중에도 그 목표를 유지한다. 실측80.6mm를 새 목표로 보내 쥐는 힘을 푸는 오류를 방지한다.
- contact stall은 **닫기 중에만**, 서로 다른 원본 timestamp의 최소5개 피드백과 settle 구간을 만족해야 인정한다. 같은 샘플 반복 polling이나 여는 중 장애물에 걸림을 완료로 인정하지 않는다. history는 deque256 및 시간창이다.
- 성공 이후 `.before_near`, `.before_search`, `.before_2deg` 및 `.before_*fix` 차이를 검토했다. 성공 config hash는 `2d6245769bc5b9a4488c6d5a765120b3d797aa453bdc5f90eb45ceb674f9474a`. 원본 소스 전체의 성공시점 hash는 없어 같은 버전이라고 주장하지 않는다.
- 성공 config의 base_link 고정, nominal_trial, 1.5초 feedback allowance, 비인접 link1↔link5 8mm prefix contact 허용을 통합 기본값에 복사하지 않았다. 이 collision 예외가 들어 있는 config는 통합 action에서 명시적으로 거절한다. 엄격한 충돌검사 때문에 같은 처진 시작 자세의 계획은 실패할 수 있다. 실제 시작 자세 검증이 필요하다.
- 성공 폴더의 class-agnostic `sam2_auto` 코드는 요청 클래스 이름을 붙이는 경로다. 이번 요구의 독립 클래스 재검출을 보증하지 못하므로 가져오지 않았다. 기존 YOLO 재검출+SAM2 또는 검증된 같은 프로젝트 yolo_seg 경로를 쓴다. 새 손목 클래스 성능/latency는 실환경 미검증이다.

## 5. 설치·빌드·기동·작업·종료 (사용자가 실행)

로컬 release는 소스만 포함한다. 모델 weights, 개발 PC venv, 노트북 빌드 결과, 다운로드한 ARM64 install을 넣지 않는다. 아래 명령은 안내이며 이번 작업에서 수행할 업로드 명령이 아니다.

로컬에서 새 배포 경로로 전송:

```bash
cd '/home/doyeonlee/문서/ChatGPT/RoboCup@Home2027'
sha256sum -c artifacts/manipulation_integration_20261015.tar.gz.sha256
scp artifacts/manipulation_integration_20261015.tar.gz jetson:/tmp/
ssh jetson 'mkdir -p "$HOME/robocup_work/manipulation_integration_20261015"; tar --keep-old-files -xzf /tmp/manipulation_integration_20261015.tar.gz -C "$HOME/robocup_work/manipulation_integration_20261015"'
```

추출 대상이 이미 있으면 tar가 중단한다. 기존 `manip_fix`, 공유 Git checkout 및 남의 파일 위에 풀지 않는다. 공유 repo의 robot_description/meshes는 읽기만 한다.

Jetson 터미널에서:

```bash
BUNDLE="$HOME/robocup_work/manipulation_integration_20261015"
MANIP="$BUNDLE/SW/manipulation"
# 없는 ROS 의존성만 설치. 기존 Jetson torch/CUDA/Piper/카메라 venv는 재설치하지 않음.
bash "$MANIP/tools/install_dependencies_jetson.sh"
bash "$MANIP/tools/build_jetson.sh" "$BUNDLE/overlay" "$HOME/piper_ros/install/setup.bash"
source /opt/ros/humble/setup.bash
source "$HOME/piper_ros/install/setup.bash"
source "$BUNDLE/overlay/install/setup.bash"
export ROBOCUP_REPO="$HOME/INHA-RoboCup-Home-2th-2027"
```

`build_jetson.sh`는 **존재하지 않는 overlay 경로만** 받으며 원본을 건드리지 않는 source 사본에서 네 패키지를 빌드한다. 이미 존재하는 overlay를 다시 빌드하려면 그 overlay/src에서 일반 colcon 명령을 사용한다. ROS 시스템 노드/bridge는 /usr/bin/python3 3.10, 손목 모델은 ROS 모듈이 보이는 기존 Jetson venv를 `vision_python`으로 선택한다. 읽기 점검에서 `$HOME/venvs/sam21/bin/python`에 rclpy/cv_bridge/torch/ultralytics/sam2가 존재함을 확인했다. 가상환경 경로는 해당 Jetson 운영자가 선택하며 개발 PC 절대경로에 의존하지 않는다.

설정 사본 만들기(최초 한 번, 기존 사본 덮어쓰지 않기):

```bash
mkdir -p "$BUNDLE/config"
cp -n "$MANIP/manipulation_ws/src/robocup_real_manipulation/config/robot.yaml" "$BUNDLE/config/robot.measured.yaml"
cp -n "$MANIP/manipulation_ws/src/robocup_real_manipulation/config/wrist.yaml" "$BUNDLE/config/wrist.runtime.yaml"
```

measured 파일에는 기존 실물 hand-eye/CAD 정의와 피드백 이름·sign/offset·width 변환·정지/복귀 자세·고정좌표 scene·실제 commissioning 상태를 확인해 반영한다. 미정 수치를 자동으로 채우거나 verified=true로 바꾸는 스크립트는 없다. 기존 nominal_trial은 여전히 청사과/5% 이하 제한을 유지한다. 다중물체는 해당 클래스 조건과 measured width 정책을 검토해야 한다.

모델 경로는 wrist.runtime.yaml의 실제 파일로 지정한다. 현재 확인한 Jetson 경로 예:

```yaml
# 같은 프로젝트의 canonical 6-class output인지 frontend가 names를 검사한다.
yolo_model: /home/sparo/INHA-RoboCup-Home-2th-2027/SW/detection/head/models/head_yolo11n_20261013.pt
mask_source: sam2
sam_checkpoint: /home/sparo/detection_data/models/sam2.1_hiera_tiny.pt
sam_config: configs/sam2.1/sam2.1_hiera_t.yaml
```

이는 해당 Jetson의 예시이며 절대경로를 source에 강제하지 않는다. 상대 model path는 `ROBOCUP_REPO` 기준이다. `.engine`/`.pt`의 raw names와 canonical class_mapping을 검사하며 COCO weights를 같은 의미로 사용하지 않는다. 자동 model download는 없다. SAM config는 설치된 SAM2와 일치해야 한다.

외부 기동 조건:

1. 기존 카메라/센서/TF 소유자가 제공하는 factory RGB-D/Info와 동기 ROS clock.
2. 실제 대상 선택/퓨전이 Target를 발행하고 실제 접근 소유자가 ApproachResult를 발행.
3. 최종 구동 경로에서 실제 정지·전소스 잠금·해제 handshake를 구현한 BaseLease provider.
4. 기존 Piper 단독 command owner 구조: raw arm/gripper/status publisher + `/joint_ctrl_single` subscriber. 통합 launch는 driver나 모터 enable을 실행하지 않는다. 현재 passive `/piper/joint_states_feedback`만으로는 gripper/status가 없어 충분하지 않다. 다운로드된 수정 driver의 stamp·CAN/펌웨어·gripper 단위 검증 후 해당 driver 소유자와 연결한다. 기존 관제 프로세스를 임의 종료해 대체하지 않는다.
5. 이미 외부 robot_state_publisher가 팔 TF를 제공하면 중복 RSP를 실행하지 않는다. MoveIt은 이 overlay의 모델과 bridge `/joint_states`를 사용한다.

읽기 점검(구동 없음):

```bash
/usr/bin/python3 "$MANIP/tools/read_only_probe.py" --seconds 8 > "$BUNDLE/read_only_probe.json"
/usr/bin/python3 "$MANIP/tools/check_integrated.py" --config "$BUNDLE/config/robot.measured.yaml" --vision "$BUNDLE/config/wrist.runtime.yaml" --repo "$ROBOCUP_REPO" --runtime
```

이 점검은 publish/action/service 호출을 하지 않는다. missing type, publisher 수, 기존 command owner/손목 worker 중복, model 파일, fixed scene 설정을 검사한다. 실제 정지·TF 정확도·물리 경로를 인증하는 도구는 아니다. 현재 환경에서는 누락된 외부 계약 때문에 실패하는 것이 맞다.

**별도 로봇 구동 허가 및 위 조건 충족 후에만**:

```bash
bash "$MANIP/tools/start_integrated.sh" "$BUNDLE/overlay" "$BUNDLE/config/robot.measured.yaml" "$BUNDLE/config/wrist.runtime.yaml" "$MANIP/manipulation_ws/src/robocup_real_manipulation/config/tuning.yaml" "$HOME/venvs/sam21/bin/python"
# 다른 Jetson 터미널, 같은 overlay source 후. 외부가 만든 고유 작업 ID만 전달.
ros2 run robocup_real_manipulation task_client --task-id 'EXTERNAL_UNIQUE_TASK_ID'
```

이 client는 XYZ/접근/잠금/손목 결과를 발행하지 않는다. 외부 Target/ApproachResult와 결합하며 최종 JSON 결과를 출력하고 종료한다. `auto_start_on_approach`를 쓸 경우 수동 task_client를 함께 쓰지 않는다. 필요하면 launch를 직접 호출해 명시적으로 옵션을 설정한다.

취소/종료: task_client 터미널 Ctrl-C → 표준 action cancel → 정지 결과 확인 → launch 터미널 Ctrl-C 순서다. 실패/취소 시 EMPTY라 해도 무조건 복귀/개방하지 않는다. held/unknown/stop-unconfirmed면 base 잠금과 journal을 유지한다. journal JSON과 같은 이름의 SQLite ledger를 함께 보존한다. 이전 journal에 ledger 기록이 있는데 DB가 사라지면 `TERMINAL_LEDGER_MISSING`으로 시작을 거부한다. journal/SQLite 파일을 지워 강제로 다음 작업을 시작하지 않는다. 관제·카메라·네비게이션·다른 사람 프로세스에 pkill을 사용하지 않는다. 성공 때는 팔 복귀·정지와 실제 unlock ACK까지 확인한 뒤 다음 고유 ID를 받을 수 있다. 물리 복구 후 실패 잠금 해제 절차는 운영자의 별도 조치이며 이 패키지가 자동으로 물체를 놓거나 잠금을 강제로 지우지 않는다.

## 6. 연산·동기화·기록 한계

- 카메라/driver/Nav를 중복 기동하지 않는다. 손목 모델은 프로세스당 한 번 load, 요청 기간에만 한 worker에서 추론한다. OS file lock으로 같은 손목/bridge/journal 프로세스 중복을 막고 command publisher 수도 검사한다.
- 각 카메라 stream history6, 추론 queue0, 요청당 최대12프레임, 최대8후보. 새 완전한 tuple 우선. callback 수신 순서로 다른 프레임을 결합하지 않는다. 취득시각 이후 새 프레임, 25ms sync, 최대 observation age, acquisition TF, 클래스·mask·depth 품질, 전체 batch 및 안정 관측을 요구한다. 작업/request ID가 바뀌면 진행 중 추론 결과를 폐기한다.
- 요청 TTL이 끝나면 상위 상태 메시지가 안 와도 추론을 종료한다. 이전 request replay128개를 제한 캐시에 보존하며 종료 시 worker 종료를 요청한다. SAM image embedding은 추론 후 reset한다. GPU 호출 자체의 드라이버 hang은 Python thread로 강제 중단하지 못하므로 그 상황은 운영 프로세스 복구 대상이다.
- timing은 원본 촬영 stamp→수신, 수신→추론 시작, 추론 시작/완료, capture→result, coordinator 결과 수신, 취득 TF, plan/execute 시작/끝을 구분한다. arm/gripper 원본 stamp와 수신 age도 별도 채널이다. 원본 stamp를 현재시각으로 덮지 않는다. 최신 telemetry만 보존한다.
- 팔/그리퍼 stale/future 검사와 관절 속도·정지 검사는 유지한다. feedback_age 기본0.3초를 과거 성공 trial의1.5초로 일괄 늘리지 않았다. 실제 경로별 지연 표본을 먼저 확인한다.
- EventJoin 각64개 및 source-age 정리, observation batch/미완성 frame 수 제한, gripper deque256. Task ID tombstone은 SQLite에 영속 저장하되 메모리에 전체 과거 ID를 로드하지 않는다. JSON 기존 completed를 SQLite로 이관하고 recovery lock은 유지한다. 정확한 영속 replay 방지를 위해 SQLite ID 행은 삭제하지 않는다. 이벤트 로그는8MiB×현재+4 backup, config snapshot20개로 제한한다. 성공 당시 다운로드 artifact는 변경하지 않는다.

## 7. 검증과 남은 연결

검증 수치와 파일은 [INTEGRATION_VALIDATION.md](INTEGRATION_VALIDATION.md)에 있다. 소프트웨어 검사, 실제 원시 데이터/로그 분석, 중간 Jetson ARM64 빌드, 미실행한 물리 경로를 구분한다.

네비게이션 완성본을 받을 때 남은 작업:

1. 실제 선택/퓨전 Target의 task_id·instance_id·출력class·취득시각·고정frame을 연결. 현재 관제 JSON bbox만으로는 불가. 기존 fusion producer를 활성화/전달받아야 한다.
2. 실제 접근 terminal 이벤트를 같은 작업 ID의 ApproachResult에 연결. Nav2 standoff 도착을 close approach 성공으로 바꾸지 않는다.
3. 실제 collision/actuator 최종 경로에서 모든 입력 잠금, odometry 기반 정지 및 lease heartbeat/해제 ACK 구현. DOCKED 문자열 adapter로 대체하지 않는다.
4. static table/주변 장애물을 선택 고정frame에 측정하고, 외부 팔 TF/hand-eye와 MoveIt 모델의 일치 및 raw gripper/status 소유자를 확인.
5. 클래스별 미확정 접촉/기하/작업공간 설정 후, 별도 허가 하에 단계별 관측→계획→집기→놓기→unlock→다음 주행을 실환경 검증. 현재 네비게이션 장애물 문제 해결과 이 검증은 수행하지 않았다.
