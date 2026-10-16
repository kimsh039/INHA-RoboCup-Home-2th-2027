# 매니퓰레이션 통합

기존 상단 스타일의 Manipulation 탭을 추가한다. 공통 `이벤트` 목록을 그대로 사용한다. 별도 매니퓰 오류 창이나 고주파 HTTP polling은 만들지 않는다. 이벤트는 서버 메모리의 기존 최대60개 목록이며 이 기능은 Nav/SLAM 로그 파일을 복사하거나 저장하지 않는다.

## 실제 연결

- `/manipulation/control_center/status`: Detection 접근 인계 어댑터 상태 및 작업 ID.
- `/manipulation/target` + `/manipulation/approach_result`: 동일 ID의 실제 관측 좌표와 접근 완료·실측 정지 결과. 시작 전 완료된 접근은 재생하지 않는다.
- `/manipulation/run`: `robocup_real_manipulation_msgs/action/Manipulate`. 웹은 임의 ID나 좌표를 합성하지 않는다.
- `/manipulation/status`, `/manipulation/result`: TaskStatus의 stage/outcome/reason/holding/정지·배치·복귀 결과.
- `/manipulation/readiness`: 실행기 검증 결과 및 피드백 연결 상태. hardware=false는 READ_ONLY_MODE이며 실제 실행 버튼을 활성화하지 않는다.
- `/manipulation/wrist/status`: 카메라·재검출 readiness 및 오류. 추론은 기존 wrist frontend가 요청에 따라 수행한다.
- `/manipulation/base_gate/status`, `/manipulation/base_lease`: 최종 Tracer 속도 출력 watchdog의 실제 잠금·오도메트리 정지 증거. 가짜 ACK나 웹의 zero cmd로 대체하지 않는다.

TaskStatus의 단계 변경·성공·실패·취소, readiness 오류, 어댑터/손목 오류, 상태 수신 끊김/복구를 공통 이벤트로 보낸다. 시각, 심각도, 모듈, 원인 코드, 작업 ID가 표시된다. 같은 연속 오류/heartbeat는 중복 표시하지 않고, 복구 후 재발하면 새 이벤트를 만든다. DDS 보존된 오래된 결과는 새 실행으로 해석하지 않는다.

`다음 접근 후 자동 실행`은 실행기·손목·베이스 경로가 준비된 경우에만 예약한다. 예약 이후 새 Detection 접근을 관측하고 실제 인계가 확인되면 같은 ID로 Action을 한 번 보낸다. 실패/취소/5분 시간 초과 시 예약이 해제된다. `인계된 물체 픽앤플레이스`는 새 인계 후30초 안에 명시적으로 실행하는 경로다. 팔 조작 또는 베이스 잠금 유지 중에는 웹에서 새 주행·센서 정지·서버 종료를 거부한다. 작업 성공 후에도 잠금 유지가 기본값이다. 이 웹에서 접수한 Action만 취소한다.

## 반영 및 모니터 연결

별도 runtime은 `~/robot_setup/manipulation-runtime/current/install/local_setup.bash`이다. 관제센터 재시작 시 run-robot.sh가 이 overlay를 읽는다. 기존 driver/YOLO 설치는 교체하지 않는다.

Jetson에서:

```bash
cd ~/INHA-RoboCup-Home-2th-2027/SW/control_center
bash jetson-center.sh stop
```

관제센터 tmux 세션이 종료된 뒤:

```bash
bash jetson-center.sh start --detection
bash manipulation-monitor.sh start
```

브라우저를 새로고침한다. 기존 SLAM/Nav는 종료되므로 UI에서 사용하던 지도를 다시 선택하고 Nav2를 시작한다. 주행·팔 작업 중에는 재시작하지 않는다. `manipulation-monitor.sh`는 실제 입력 어댑터와 읽기 전용 coordinator만 실행한다. 팔/그리퍼/카메라/MoveIt을 시작하거나 enable하지 않는다. 기본 미검증 설정과 연결되지 않은 실제 피드백은 탭/이벤트에 명시적으로 표시된다. 이 상태를 실물 실행 준비 완료로 표현하지 않는다.

실물 실행기를 별도로 시작할 때 먼저 `bash manipulation-monitor.sh stop`으로 읽기 전용 coordinator를 종료한다. 검증된 실제 robot config, Piper 제어 피드백, MoveIt, 손목 frontend를 갖춘 기존 integrated launch를 사용한다. 하드웨어 설정의 commissioning flag나 실측값은 이 배포에서 임의로 활성화/채우지 않는다. 기존 physical reader와 새 제어 driver의 중복 CAN/TF 소유는 확인해야 한다. 과거 성공 설정은 이름·변환의 근거이며 현재 피드백/장면 검증을 대신하지 않는다.

## 롤백

배포 도구는 수정 대상 파일만 해시 검사 후 백업한다. 동시 변경이 있으면 쓰기 전에 중단한다. 백업은 `.manipulation-backup-날짜/`이며 manifest, 원본 소스 및 rollback.py를 포함한다. 설정/센서 데이터/네비·SLAM 로그는 백업하지 않는다.

```bash
python3 ~/INHA-RoboCup-Home-2th-2027/SW/control_center/.manipulation-backup-20261015-232329-304399/rollback.py
```

이 명령은 원본 소스와 이전 runtime 포인터만 복원한다. 실제 잠금/journal을 지우거나 팔을 정지/이동시키지 않는다. 배포 이후 다른 사람이 파일을 변경했으면 덮어쓰지 않고 중단한다. 소스 복원 후에는 팔 작업이 없는 상태에서 관제센터를 재시작한다. 신규 베이스 잠금이 실제 걸렸다면 잠금을 해제한 뒤 예전 watchdog으로 재시작해야 한다.

## 검증 범위

기존 매니퓰 계약 테스트, HTTP 인증·명령 검증, 공통 이벤트 중복 억제·끊김 복구, 과거 인계 재생 방지, 실행 준비 차단 조건을 검사한다. 로봇과 분리한 ROS187에서 TaskStatus/wrist→웹 이벤트 전달과 최종 watchdog의 잠금/실측 정지/명시적 해제를 확인한다. 이 검증은 실물 픽앤플레이스 성공이 아니다.

## Phase 0 팔 브리지 복구와 접촉 파지

`/manipulation/reset` (`std_srvs/srv/Trigger`)은 작업이 없고, 실제 팔·그리퍼 피드백과 Piper 상태가 신선하고 오류가 없으며, 정지 상태가 `joint_stable_s` 동안 유지된 경우에만 브리지 오류 래치를 해제한다. 복구 중 새 오류나 정지 요청이 들어오면 해제하지 않는다. 이 서비스는 모터 enable이나 이동 명령을 발행하지 않으며 다음 동작에는 다시 유효한 베이스 잠금 lease가 필요하다.

대기 상태의 정상 `/manipulation/stop`은 확인된 정지 뒤 새 래치를 남기지 않는다. 진행 중 동작의 정지·취소·오류는 래치로 동작을 중단하고 명시적 reset을 기다린다.

그리퍼는 목표 폭 도달 외에 닫힘 방향의 접촉을 성공으로 인정한다. 접촉에는 실제 닫힘 이동 `gripper_contact_min_travel_m`(기본 2mm), `grasp_min_effort` 이상 힘, `gripper_stable_speed_m_s`(기본 2mm/s) 이하 정체와 `gripper_settle_s` 동안의 새 피드백 3개 이상이 필요하다. 열림 동작의 장애물 정체와 같은 피드백 반복은 접촉 성공이 아니다. 상위 `VERIFY_GRASP`와 물체 유지 감시도 목표 폭과의 1mm 일치 대신 접촉 범위와 힘을 사용하며, 유지 감시는 실제 파지 폭에서 3mm 이상 벗어나면 실패한다. 실제 effort/폭 보정값과 기존 commissioning 조건은 그대로 필요하다.

수정은 저장소 소스에 반영했다. 실행 중 노드·설치된 overlay·하드웨어 설정은 자동으로 교체하지 않는다.


## Phase 2 실측 장면·연속 궤적·파지 방향

저장소 기본 설정은 `table_source: dock_surface`다. 실물 Detection은 엄격한 도킹 오차와 실제 정지를 확인한 뒤 `/detection/dock/surface` (`geometry_msgs/PolygonStamped`, reliable/transient-local)에 odom 좌표의 네 꼭짓점을 발행한다. header 시각은 도킹 완료 시각이다. 접근 도중 사각 구간에서는 Phase 1에서 유지한 마지막 실측 경계를 사용한다. 새 접근·취소·오류 단계는 빈 polygon으로 이전 인계를 무효화한다.

Coordinator는 같은 작업의 target/approach와 정지된 base lease를 받은 뒤 테이블을 고정한다. 표면 시각은 approach 결과와 2초 이내, target 촬영 이후이며 기존 event 유효기간 안이어야 한다. 잘못된 frame, 비볼록/퇴화 경계, 1.5cm 초과 높이차와 지지면 밖 목표는 거부한다. 표면은 approach 시각의 `gravity_frame`으로 변환하고 작업 중 고정한다. `PIN_DOCK_SURFACE` 로그에 작업 ID·원본 인계 시각·실제 경계를 남긴다. 별도 task ID가 없는 polygon 계약이므로 어댑터는 **완료 시각에 맞는 ApproachResult**를 전달해야 하며, 오래된 접근 결과를 새 시각으로 재발행하면 안 된다.

MoveIt `manip_table`은 회전된 네 꼭짓점을 그대로 사용하는 닫힌 삼각형 메쉬다. 상판에서 `table_thickness_m`(기본 12cm) 아래까지 포함한다. 물체 높이와 배치 지지 검사도 이 실측 상판·polygon을 사용한다. 다리·주변 물체는 기존 `obstacles`/충돌 모델 설정이 필요하다. `table_source: static`은 기존 실측 고정 fixture 설정용이며 동적 표면이 없다고 자동 전환하지 않는다. 장면을 비롯한 commissioning flag는 자동 활성화하지 않는다.

`trajectory.py`의 동일 보간기를 계획 검사와 Piper 실행에서 공유한다. 속도만 있으면 cubic Hermite, 속도·가속도가 있으면 quintic을 사용하며 누락된 속도는 단조 접선으로 계산한다. 내부 경유점에서 일률적으로 정지시키지 않는다. 전체 시간을 같은 비율로 늘려 경로와 제공된 미분값을 보존하고, 다항식 극값에서 관절 위치·속도·가속도 제한을 검사한다. 실행 전 실제 스플라인을 최대 0.01rad 관절 간격으로 MoveIt 충돌 검사한다. 이는 이산 충돌 검사이며 연속 충돌 검출 보장은 아니다. 브리지는 monotonic 시계 기준 100Hz를 목표로 명령을 발행한다. 실제 CAN 응답률·주기 지터는 실물 측정이 필요하다.

청사과·복숭아·토마토·캔·컵은 `yaw_sample_degrees: [0,30,60,90,120,150]`를 사용한다. 그리퍼를 연 다음 후보별 pregrasp와 전체 Cartesian descent를 계획하고 첫 가능한 방향을 사용한다. 실제 TCP 편심을 반영해 grip center는 고정한다. 실패한 후보로 팔을 움직이지 않으며 실행 실패를 다른 방향으로 자동 재시도하지 않는다. 바나나는 실측 주축 방향을 유지한다. 클래스별 yaw 목록은 tuning overlay로 조정할 수 있다.

## Phase 3 손목 인지와 계측

사용자 결정에 따라 **범용 SAM 2.1 small + 기존 YOLO 탐지**를 유지한다. 커스텀 YOLO-seg 전환은 적용하지 않는다. `config/wrist.yaml`의 YOLO는 기존 로컬 `.engine`을 사용하고, SAM은 지정한 로컬 공식 checkpoint/config를 사용한다. SAM 설치·checkpoint 준비는 별도이며 암묵적으로 다운로드하거나 탐지 모델을 분할 모델로 간주하지 않는다. 요청한 클래스 후보가 없으면 SAM image encoder도 실행하지 않는다.

`/manipulation/wrist/status`는 기존 설치 runtime의 JSON 상태 계약을 유지한다. 새 `/manipulation/wrist/profile` JSON에는 YOLO wall time, 마스크+depth 처리 시간, 두 시간의 합, 촬영 후 age, 후보 수, 최근 100회 p50/p95와 50ms 목표 충족 여부를 발행한다. 후보 0개인 프레임도 후보 수와 함께 기록한다. 50ms 달성이나 GPU 메모리 절감 수치를 실측한 것은 아니다. 손목 aligned depth는 기존 factory CameraInfo와 짝지으며 실측 RGB K로 교체하지 않는다.

초기 Phase 2·3 작업은 소스만 수정했으며, 이후 사용자 요청으로 기존 설치 runtime의 인계·readiness 기능을 통합하고 `build-runtime.sh` 배포 경로를 추가했다. 실제 설치·실행 반영은 [변경 정리](../CHANGES_20261016.md)를 따른다.

통합 runtime의 영구 작업 ID·단일 소유자·잠금 완료 정책과 손목 관측/주변 장애물 처리도 저장소에 포함한다. 기존 설치에만 있던 코드를 새 빌드에서 잃지 않도록 integrated launch와 입력 어댑터를 함께 설치한다.

## 2026-10-16 원격 인계 개선 통합

Target/ApproachResult는 adapter와 웹에서 reliable/transient-local depth 1로 연결한다. 완료 snapshot이 진행 상태보다 먼저 도착해도 웹 시작 이후의 새 approach 시작 시각으로 연결하며, 웹 시작 전 작업은 재실행하지 않는다. 자동 실행 예약도 예약 이후 시작한 작업만 사용한다. 인계 좌표·완료·실측 정지 표시는 팔 실행 readiness와 구분하며, 현재 정지 상태와 원본 시각이 유효해야 실행 후보가 된다. 접근 상태 유실 3초 초과 및 완료 후 실제 베이스 이동은 인계를 무효화한다.

관제센터 소스 배포 도구는 현재 runtime 포인터를 바꾸지 않는다. 패키지 변경 시 `bash SW/manipulation/build-runtime.sh`로 같은 소스의 새 release를 먼저 설치한다.
