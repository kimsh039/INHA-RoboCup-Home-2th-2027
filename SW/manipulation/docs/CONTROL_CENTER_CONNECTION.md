# 기존 웹을 변경하지 않는 매니퓰레이션 연결

2026-10-15 Jetson에서 실제 실행 중인 `SW/control_center/detection_approach.py`를 읽어 확인한 계약이다. 이 파일은 당시 공유 Git main에 없는 미커밋 코드였으며 이 작업에서 수정·복사·배포하지 않는다. 웹의 다음 버전이 계약을 변경하면 재검증한다.

## 연결 방식

현재 사용자 흐름은 클로즈 어프로치→실제 정지·이동 잠금→픽앤플레이스·팔 복귀로 종료된다. `base_completion_policy: hold`가 기본이며 성공 후에도 동일 task ID로 잠금을 요청하고 journal에 유지한다. 마지막 결과는 `SUCCESS / BASE_LOCK_RETAINED`다. 다음 주행이나 새 팔 작업은 자동 시작하지 않는다. 해제 ACK는 이 모드의 완료 조건이 아니다. 이후 주행이 있는 별도 구성에서만 `release`를 명시하고 실제 해제 응답을 확인한다. 최초 잠금과 동작 중 fresh locked/stopped lease는 두 모드 모두 필요하다.

`/detection/closed_approach/status` (`std_msgs/String` JSON)
→ `control_center_input`
→ `/manipulation/target` (`Target`)
및 `/manipulation/approach_result` (`ApproachResult`).

JSON에는 `phase`, `active`, `label`, canonical `classes`, `target`이 함께 있다. target의 `point_odom`, `capture_stamp`, `lidar_stamp`, `lidar_oldest_stamp`, `lidar_count`, `point_method`, `source_topic`을 검증한다. 현재 알려진 LiDAR bbox 군집 방식만 받으며 새 추론·퓨전은 하지 않는다. `target.frame=map`일 때도 `point_odom`은 odom이고 `point_map`과 구별한다. 원본 취득시각을 현재시각으로 바꾸지 않는다.

외부 `HeadTarget`/퓨전 adapter와 동시에 켜지 않는다. 기존 웹은 task UUID/영구 instance ID를 제공하지 않으므로 새 접근 전환마다 어댑터가 UUID를 생성하고 해당 접근의 Target/ApproachResult에 동일 ID를 쓴다. instance_id는 그 접근 안에서만 유효한 내부 ID이며 영구적인 detector track ID가 아니다.

## 동작 조건과 제한

관측된 최신 관제 phase `waiting_target`, `navigating_target`, `rerouting_table`, `planning_table`도 수용한다. 비활성 상태를 먼저 본 새 접근에서 `waiting_target`의 target=null은 실측 좌표 대기로 유지한다. 좌표가 실제 도착한 이후에만 UUID를 생성하며, 시작 시 이미 진행 중인 접근이나 과거 성공을 재생하지 않는다. 알 수 없는 phase는 계속 거절한다.

추가 최신 phase `target_settling`, `point_aligning`, `point_approaching`, `recovering`, `observing_table`에서도 같은 작업 ID와 원본 좌표를 유지한다. `at_target`은 현재 웹 정의상 '3D 목표 도착/몸체 한계 접근, 파지 위치 별도·미확정'이므로 성공 인계하지 않고 `GRASP_STAGING_NOT_CONFIRMED`로 대기한다. 이후 새 active 접근은 새 ID로 받을 수 있다. 오직 관제 `phase=succeeded, active=false`와 측정 정지가 완료됐을 때만 인계한다. 내부 Nav2의 succeeded·복구 완료·차단·취소를 팔 시작으로 해석하지 않는다.

최신 복구 후 선정렬 단계 `realigning`도 같은 작업을 유지한다. 최종 정렬이 `begin_final_alignment`를 통해 지연된 `succeeded`/`at_target`을 내보내는 구조도 확인했다. 두 도착 의미는 위와 같이 계속 구별한다.

관제 소스 변경 후 이름/단계 누락을 수동 검사할 수 있다. 이 검사는 파일의 클래스 순서·발행 토픽 문자열·정적 접근 phase만 확인하며 서버/로봇을 실행하거나 바꾸지 않는다. 실제 센서 freshness·메시지 전달·파지 준비 검증을 대신하지 않는다.

```bash
python3 SW/manipulation/tools/check_control_center_source.py \
  --source /실제/최신/checkout/SW/control_center/detection_approach.py
```

- 실행 후 `ready/searching/idle/error/canceled` 등 비활성 상태를 본 다음 새 active 접근을 관측해야 한다. 시작 시 이미 succeeded이거나 접근 중인 경우 과거 작업을 복원·자동 시작하지 않는다.
- 새 접근 시작 시점의 실측 odom 좌표·원본 촬영시각을 보존한다. 완료 시 클래스가 바뀌거나 위치가 25cm 이상 달라지면 인계를 거절한다. 바뀐/오래된/비정상 메시지는 작업을 무효화한다.
- `phase=succeeded`, `active=false`를 본 후에도 실제 `/odom_ekf`를 확인한다. odom/base_link 프레임, 신선한 증가 시각, 선속도≤0.02m/s·각속도≤0.04rad/s가 완료 후 최소0.6초 유지돼야 인계한다.
- 접근 스트림은 1초 안에 갱신돼야 하고 원본 목표는 최대120초다. 노드 중복은 프로세스 소유 잠금으로 거절한다. 출력 구독자가 연결되기 전에는 단발 인계를 소비하지 않는다.
- JSON 구독 큐는1, odom 큐는 sensor_data 크기이고 단일 작업만 메모리에 보존한다. JSON은64KiB 제한, 상태는1Hz. 모델·이미지·점군 처리 및 HTTP polling은 없다.
- **이 어댑터는 BaseLease를 발행하거나 /cmd_vel을 제어하지 않는다.** 접근 성공+오도메트리 정지는 이동 명령 잠금과 다르다. 실제 최종 이동 명령 소유자가 `/manipulation/base_lock_request`를 받아 모든 명령원을 차단하고 같은 UUID의 `/manipulation/base_lease`를 응답해야 팔 실행이 가능하다. 현재 웹에는 이 응답이 없으므로 이 부분은 미연결이다. 검사 없이 true를 보내는 임시 publisher를 쓰지 않는다.
- `auto_start_on_approach`는 기존 coordinator의 선택 기능이다. 기본 false를 유지하며 수동 task_client와 중복 사용하지 않는다. 어댑터를 켰다는 이유로 실제 팔 작업이 시작되지는 않는다.

## 빌드 후 입력 어댑터만 실행

### 팔 관절 이름 계약

원시 드라이버는 `joint1..joint6`, CAD/관제 접근 계획/매니퓰레이션 모델은 `piper_joint1..piper_joint6`를 쓴다. robot.yaml의 `driver_joint_names`와 `joint_names`가 각각 이 이름에 대응한다. raw 이름을 그대로 모델 이름으로만 바꾸면 안 된다. 기존 로봇 bridge는 q1에 CAD offset -1.6rad를 적용하고, 매니퓰 bridge는 별도 검증된 measured `joint_signs/joint_offsets`로 변환한다. 모델 출력에 이 변환을 이중 적용하지 않는다.

관제센터의 화면용 joints 채널은 raw 피드백을 선택할 수 있으므로 접근 계획 입력과 분리한다. 관제 `detection.model_joint_topic=/robot/joint_states`는 기존 물리 로봇 bridge의 보정된 모델 출력을 사용한다. 향후 매니퓰 PiperBridge가 팔/TF 단일 소유자가 되면 모델 출력 `/joint_states`를 관제 입력으로 지정한다. `/piper/joint_states_feedback` 및 `/joint_states_feedback`는 raw 입력이며 둘의 topic remap만으로 모델 보정이 끝나지 않는다. 기존 물리 robot bridge와 새 제어 bridge를 중복 기동하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
source /실제/매니퓰레이션/overlay/install/setup.bash
ros2 launch robocup_real_manipulation control_center_input.launch.py
```

이 launch는 팔 bridge·MoveIt·모터·카메라·네비게이션을 시작하지 않는다. 실제 상태를 구독하고 매니퓰레이션 입력만 발행한다. 상태는 `/manipulation/control_center/status`에서 볼 수 있다. 출력 타입을 확인하려면 `ros2 topic echo /manipulation/target` 및 `ros2 topic echo /manipulation/approach_result`를 새 접근 전에 구독한다.

기존 통합 launch와 함께 쓸 때는 `start_control_center_adapter:=true start_head_adapter:=false`를 지정한다. 실제 실행은 기존 measured config, 고정좌표계 scene, 실제 base lease 및 손목 카메라 조건을 충족해야 한다. 이 문서는 팔 구동 승인이나 실물 통합 성공을 뜻하지 않는다.

## 검증

### 과거 성공 설정 재사용

청사과 성공 task `7ece08d387f547cd942d794076f82bb0`의 config SHA `2d6245769bc5b9a4488c6d5a765120b3d797aa453bdc5f90eb45ceb674f9474a`와 일치하는 원본 snapshot을 확인했다. `references/success_driver_contract_20261015.json`에 관련 설정을 보존했다. 아래 도구는 **비어 있는 드라이버 설정만** 새 파일에 채운다. 기존 값이 다른 경우 덮어쓰지 않고 거절하며, commissioning·실행 플래그·장면·그리퍼 trial 보정은 가져오지 않는다.

```bash
python3 SW/manipulation/tools/restore_success_driver_settings.py \
  --config SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/robot.yaml \
  --output /원하는/새/robot.driver-reference.yaml
```

과거 성공 드라이버는 `/joint_states_feedback`, `/gripper_states_feedback`, `/arm_status`를 발행했다. 현재 관제의 read-only `/piper/joint_states_feedback`와 이름만 remap해서 대체하면 그리퍼 effort/status가 생기는 것이 아니다. 실제 제어 드라이버를 중복 없이 연결해 정지 상태에서 원본 시각·필드·타입을 확인하면 된다. 팔 이동은 이 확인의 전제 조건이 아니다. 과거 그리퍼 폭 설정은 nominal_trial/width_mapping_verified=false였으므로 자동 실측 완료 처리하지 않는다.

관제 손목 상태 채널은 원래 `/detection/wrist/status`만 선택했지만 우리 frontend는 `/manipulation/wrist/status` (`std_msgs/String`)를 발행한다. 새 예제 설정은 매니퓰 상태를 우선하고 기존 검출 상태를 fallback으로 유지한다. 기존 실제 config.json은 통째로 교체하지 않고 아래 도구로 해당 채널 후보만 연결한다. 원본 설정 backup을 만들고 다른 설정은 보존하며 전체 설정/비밀값은 출력하지 않는다. 적용 후 관제센터를 운영자가 재시작해야 선택이 반영된다. 모터·팔·카메라·네비게이션을 시작하는 도구가 아니다.

```bash
python3 SW/manipulation/tools/link_control_center_wrist_status.py \
  --config /실제/최신/checkout/SW/control_center/config.json --check
python3 SW/manipulation/tools/link_control_center_wrist_status.py \
  --config /실제/최신/checkout/SW/control_center/config.json
```

단순 상태 표시 이름의 연결이며, 손목 이미지 입력은 실제 factory RGB+factory aligned depth+factory CameraInfo 조합을 유지한다. 화면용 calibrated RGB 토픽으로 무조건 치환하지 않는다.

순수 로직 테스트는 `test_control_center_input.py`에서 클래스·시각·좌표, 시작 시 과거 성공 거절, 목표 변경·취소·스트림 공백·시계 역행, 정지 대기와 중복 방지를 검사한다. `test_control_center_ros.py`는 로봇과 격리된 ROS 도메인187에서 합성 웹 상태·합성 odom을 보내 실제 어댑터의 메시지 인계를 검사한다. 합성 검증과 실물 접근/파지 성공을 구분한다.
