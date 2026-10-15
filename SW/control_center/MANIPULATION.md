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

`다음 접근 후 자동 실행`은 실행기·손목·베이스 경로가 준비된 경우에만 예약한다. 예약 이후 새 Detection 접근을 관측하고 실제 인계가 확인되면 같은 ID로 Action을 한 번 보낸다. 실패/취소/5분 시간 초과 시 예약이 해제된다. `인계된 물체 픽앤플레이스`는 새 인계 후10초 안에 명시적으로 실행하는 경로다. 팔 조작 또는 베이스 잠금 유지 중에는 웹에서 새 주행·센서 정지·서버 종료를 거부한다. 작업 성공 후에도 잠금 유지가 기본값이다. 이 웹에서 접수한 Action만 취소한다.

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
