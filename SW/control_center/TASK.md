# 전체 Task: 책상 위 접시에 놓고 홈 복귀

관제센터 마지막 **전체 Task** 탭에서 물체를 선택하고 실행한다.

`센서 준비 → SLAM/저장 지도 위치추정 → NAV lifecycle 준비 → 팔·손목 준비 → Head 탐지 → closed approach → 베이스 잠금 → 집기 → Wrist SAM 접시 관측 → 접시에 놓기 → 팔 수납 → 잠금 인계 → 직선 후진 → 실측 정지 → 제자리 회전·Head 정렬 → 실측 정지 → 홈 복귀·정지`

사용자 지정대로 **접시는 책상 위**에 있다. 물체를 로봇에 싣거나 손으로 들고 홈까지 운반하는 작업이 아니다. 기존 Manipulation 탭의 수동 픽앤플레이스는 유지하며, 전체 Task만 `operation=table_plate`를 요청한다.

## 홈 설정

홈으로 사용할 시작 위치에서 정지한 뒤 SLAM 탭의 `SLAM 시작` 또는 전체 Task 탭의 `SLAM 시작·시작 위치를 홈으로 등록`을 누른다. 지도 완성·저장 없이 등록을 예약하며, 첫 유효 `map → odom` TF가 나오면 보관한 시작 odom 위치·방향을 map 좌표로 변환해 저장한다. 등록 완료까지 정지한다. 기다리는 동안 시작 위치가 달라지거나 90초 내 좌표를 받지 못하면 이유를 표시하고 등록을 중단한다. SLAM이 이미 실행 중일 때는 `현재 위치를 홈으로 등록`으로 다시 지정할 수 있다.

- 저장 지도 위치추정 중 등록한 홈: 해당 지도의 SLAM Toolbox 위치추정을 자동 시작하고, 위치추정 완료 후 NAV를 시작한다.
- 새 지도 작성 중 등록한 홈: 해당 실행 세션에서만 유효하다. SLAM/관제 재시작 뒤 이전 임시 지도 좌표를 재사용하지 않는다. 다음 실행까지 유지하려면 저장 지도에서 위치추정한 뒤 홈을 등록한다.
- 홈 없이 전체 Task를 시작하면 SLAM 시작·홈 등록부터 진행한다. 새 SLAM 세션에는 새 시작 홈을 등록하며, 저장 지도 홈과 현재 지도가 다르거나 기존 이동/잠금이 유지되는 경우에는 진행하지 않는다.

## 실물 설정과 실행 환경

새 소스는 실행 중인 ROS 프로세스에 자동 반영되지 않는다. `Manipulate`, `ObservationRequest`, `WristObservation` 인터페이스가 확장됐으므로 **관제·watchdog·팔·손목을 동일 버전으로 설치한 뒤 재시작**해야 한다. 이전 overlay와 혼용하지 않는다. 2026-10-16 로컬 적용에서는 실행용 패키지를 빌드·설치하고 관제센터 및 읽기 전용 팔 모니터를 재시작했다. 팔의 실물 실행 모드는 활성화하지 않았다.

메시지와 팔·손목 runtime을 설치하는 기존 진입점은 저장소 루트에서 다음과 같다. 이 명령은 빌드 후 `current` runtime 링크를 전환한다. 실행 중인 작업을 종료하고 버전을 맞춰 배포할 때 사용한다.

```bash
bash SW/manipulation/build-runtime.sh
```

관제센터와 최종 Tracer watchdog은 해당 checkout의 새 소스로 함께 재시작해야 한다. runtime 빌드만으로 이미 떠 있는 관제·watchdog 프로세스가 갱신되지는 않는다.

팔 실행기가 없을 때 전체 Task가 `start-task-manipulation.sh`로 시작한다. 기존 읽기 전용 실행기나 다른 드라이버를 강제 종료·교체하지 않는다. 중복 실행은 거절한다. 실물 실행 프로파일은 관제 프로세스 환경에서 지정한다.

```bash
export ROBOCUP_TASK_ROBOT_CONFIG=/absolute/path/to/robot.measured.yaml
export ROBOCUP_TASK_VISION_CONFIG=/absolute/path/to/wrist.yaml
export ROBOCUP_TASK_VISION_PYTHON=/home/sparo/venvs/sam21/bin/python
```

`SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/robot.integrated-trial.yaml`은 로컬 성공 작업본에서 관절·드라이버 값을 이식한 준비용 프로파일이다. 전역 execution/commissioned 플래그는 켜지 않았다. 기존 실물 검증 필드 외에 다음 값이 필요하다.

- `plate_observation_joints`: **물체를 들고** 책상 위 접시를 위에서 관측할 수 있는 실측 6축 자세. 임의 자세로 채우지 않았다.
- `rest_joints`, `transit_waypoints`: 주행용 팔 수납 자세와 경유 자세. 성공 작업본 값을 후보로 가져왔으며 MoveIt가 실행 궤적을 검사한다.
- `plate_placement_verified`: 새 접시 관측·배치 동작을 확인한 프로파일에서만 설정한다. 기본 false.
- `base_completion_policy: hold`: 팔 완료 후 잠금을 유지한다. 별도의 동일 작업 이탈 요청과 실제 gate ACK로만 해제한다.
- 물체별 `class_execution`과 기존 폭·접촉 보정 조건을 유지한다. 청사과 성공 기록이 다른 5종의 성공을 의미하지 않는다.

손목 기본 경로는 이미 있는 SAM 2.1 tiny checkpoint와 `sam2_auto`다. SAM은 클래스 분류기가 아니다. 물체 후보는 Head의 요청 클래스, coarse 위치, 깊이, 크기로 연결한다. 접시는 자동 마스크의 원형성·크기·실측 높이·빈 평면 영역으로 후보를 정하고, 여러 후보면 중단한다. 휘어진/무늬 있는/투명 접시 등은 이 조건에서 관측에 실패할 수 있다.

## 제어와 중단

- `task_control.py`: 한 번의 요청, 단계별 준비 확인·시간 제한·동일 작업 인계·상태 저장. 재시작 후 자동 재실행하지 않는다.
- `dock_departure.py`: 웹 영상과 분리된 제어 프로세스에서 실제 도킹 문맥, 팔 수납 피드백, 점군, TF, odom, 홈 경로를 확인한다. URDF 차체·랙·팔의 회전 반경을 확보할 후진 거리를 계산한다. 후방 관측이나 회전 공간이 부족하면 중단한다.
- `dock_motion_gate.py`: 최종 Tracer 출력 직전에 도킹 잠금과 단계 순서를 유지한다. 후진 중 각속도, 회전 중 선속도, 정렬 전 일반 주행을 허용하지 않는다. 명령/팔 피드백 만료 시 0을 출력한다.
- `plate_task.py`: 접시 위 빈 공간이 물체보다 넓고 여러 프레임에서 안정적인 경우에만 배치한다. 팔 수납·정지·실제 잠금 해제 ACK를 확인한다.
- 취소는 대기 중 팔 요청과 늦게 수락된 Action도 취소한다. 전체 Task heartbeat가 사라지면 해당 팔 작업도 정지 경로로 들어간다.
- 개별 NAV/Detection/Manipulation 명령은 전체 Task 실행 중 우회하지 못한다. 개별 취소 버튼은 전체 Task 취소로 연결한다.
- 후진·회전 도중 실패/취소/재시작한 도킹 문맥은 잠금을 남긴다. **자동 재시도/잠금 강제 삭제 기능은 제공하지 않는다.** 현장 복구 절차가 필요하다.

## 현재 한계

접시 관측 자세와 실물 프로파일의 commissioning이 남아 있다. 다른 장비에도 적용할 때는 동일 버전 runtime을 설치해야 한다. 전체 Task가 준비되지 않은 단계를 건너뛰어 성공 처리하지 않는다. 로컬 실행용 패키지 설치와 관제 반영 외에 자동 테스트·전체 Task ROS 통합 시험·실물 주행·파지는 수행하지 않았다.

홈 복귀는 일반 Nav2의 경로 재계획을 사용한다. 이탈 전 선택한 경로의 첫 진행 방향에 정렬하지만, 인계 후 장애물 때문에 Nav2가 경로를 바꿀 수 있다. 전체 이동 경로를 사전에 고정하거나 알려지지 않은 공간의 무충돌을 보증하지 않는다.
