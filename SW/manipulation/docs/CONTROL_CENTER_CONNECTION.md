# 기존 웹을 변경하지 않는 매니퓰레이션 연결

2026-10-15 Jetson에서 실제 실행 중인 `SW/control_center/detection_approach.py`를 읽어 확인한 계약이다. 이 파일은 당시 공유 Git main에 없는 미커밋 코드였으며 이 작업에서 수정·복사·배포하지 않는다. 웹의 다음 버전이 계약을 변경하면 재검증한다.

## 연결 방식

`/detection/closed_approach/status` (`std_msgs/String` JSON)
→ `control_center_input`
→ `/manipulation/target` (`Target`)
및 `/manipulation/approach_result` (`ApproachResult`).

JSON에는 `phase`, `active`, `label`, canonical `classes`, `target`이 함께 있다. target의 `point_odom`, `capture_stamp`, `lidar_stamp`, `lidar_oldest_stamp`, `lidar_count`, `point_method`, `source_topic`을 검증한다. 현재 알려진 LiDAR bbox 군집 방식만 받으며 새 추론·퓨전은 하지 않는다. `target.frame=map`일 때도 `point_odom`은 odom이고 `point_map`과 구별한다. 원본 취득시각을 현재시각으로 바꾸지 않는다.

외부 `HeadTarget`/퓨전 adapter와 동시에 켜지 않는다. 기존 웹은 task UUID/영구 instance ID를 제공하지 않으므로 새 접근 전환마다 어댑터가 UUID를 생성하고 해당 접근의 Target/ApproachResult에 동일 ID를 쓴다. instance_id는 그 접근 안에서만 유효한 내부 ID이며 영구적인 detector track ID가 아니다.

## 동작 조건과 제한

관측된 최신 관제 phase `waiting_target`, `navigating_target`, `rerouting_table`, `planning_table`도 수용한다. 비활성 상태를 먼저 본 새 접근에서 `waiting_target`의 target=null은 실측 좌표 대기로 유지한다. 좌표가 실제 도착한 이후에만 UUID를 생성하며, 시작 시 이미 진행 중인 접근이나 과거 성공을 재생하지 않는다. 알 수 없는 phase는 계속 거절한다.

추가 최신 phase `target_settling`, `point_aligning`, `point_approaching`, `recovering`, `observing_table`에서도 같은 작업 ID와 원본 좌표를 유지한다. `at_target`은 현재 웹 정의상 '3D 목표 도착/몸체 한계 접근, 파지 위치 별도·미확정'이므로 성공 인계하지 않고 `GRASP_STAGING_NOT_CONFIRMED`로 대기한다. 이후 새 active 접근은 새 ID로 받을 수 있다. 오직 관제 `phase=succeeded, active=false`와 측정 정지가 완료됐을 때만 인계한다. 내부 Nav2의 succeeded·복구 완료·차단·취소를 팔 시작으로 해석하지 않는다.

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

순수 로직 테스트는 `test_control_center_input.py`에서 클래스·시각·좌표, 시작 시 과거 성공 거절, 목표 변경·취소·스트림 공백·시계 역행, 정지 대기와 중복 방지를 검사한다. `test_control_center_ros.py`는 로봇과 격리된 ROS 도메인187에서 합성 웹 상태·합성 odom을 보내 실제 어댑터의 메시지 인계를 검사한다. 합성 검증과 실물 접근/파지 성공을 구분한다.
