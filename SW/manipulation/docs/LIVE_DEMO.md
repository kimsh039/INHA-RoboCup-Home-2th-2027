# 실제 디텍션을 받는 라이브 데모 — 2026-10-12

라이브 입력은 실제 head/손목 디텍션과 실제 접근 완료·베이스 잠금이다. bench_pick_place.py, test_sender, mock_coordinator는 데모에서 실행하지 않는다. 자동 픽앤플레이스 알고리즘은 그대로 두고 외부 모듈이 기존 ROS 계약에 연결한다.

최신6클래스 출력 매핑을 기본 설정에 반영했다. 장비/입력/TF 연결은 [DEMO_CHECK.md](DEMO_CHECK.md)의 읽기 전용 도구로 먼저 확인한다.

## 물체 설정

| 이름 | 설정 키 | 전체 높이 | 중심/몸통 파지 초기 높이 | open/close |
|---|---|---:|---:|---:|
| 청사과 |green_apple|68mm|34mm|100/79mm|
| 복숭아 |peach|68mm|34mm|100/79mm|
| 토마토 |tomato|68mm|34mm|100/82mm|
| 환타 |fanta_can|124mm|62mm|80/64mm|
| 컵 |cup|78mm|39mm|100/90mm|
| 바나나 |banana|중앙 몸통 높이 확인 필요|별도 몸통점·장축 방향 필요|40/26mm|

길이 내부 단위는m다. 사용자 높이를 robot.yaml에 반영했고 파지 높이만 tuning.yaml에서 trial로 조정한다. center_height는 기하 중심, grip_height는 TCP 파지 기준점이며 서로 독립이다. 기존 size_m는 원래 제공된 크기 기록으로 보존하고 최신 수직 치수는 height_m/center_height로 구분했다. 머리카메라 bbox 중심이나 표면 depth가 곧 기하 중심이라고 가정하지 않는다.

최신 main commitddae58cb002ae533989bddd3f2b8147029a5ad0f의20261012 data.yaml/class_schema.py에서 출력ID를 확인했다:0 green_apple,1 peach,2 tomato,3 fanta_can,4 cup,5 banana. model_names는 이 **디텍션 출력 스키마**다. 한국어 이름은 display_name으로 관리한다. 토마토와 컵의 기존 파지 규칙을 해당ID로 활성화했다. 바나나는ID5를 반영했지만 몸통 높이/반경/장축/닫힘축 관계가 완비되기 전까지 물리 파지를 비활성화한다.

기존20261007 가중치 자체는7클래스 그대로다. 최신 Head YoloDetector가 raw IDs를 `{0:2,1:5,2:3,3:0,4:4,5:1,6:제외}`로 변환한다. 매니퓰레이션에는 변환된6클래스ID를 전달하며 다시 변환하지 않는다. 손목에서 기존 가중치의YOLO.predict()를 직접 호출한다면 손목 adapter도 같은class_schema.class_mapping을 적용해야 한다. 새6클래스모델을 학습해 쓰는 경우ID를 그대로 전달한다. raw7클래스 ID와 current6클래스 ID를 섞으면 다른물체의 규칙이 선택되므로 model_names_verified는 실제 두 인식모듈의 출력 스키마 확인 후 활성화한다. 일반COCO apple을토마토로 바꾸는 일반적인alias가 아니다.

## 실제 모듈 연결 계약

기존 HeadTarget + 퓨전 PointStamped 수신부와 접근 완료 후 자동 인계 옵션은 [EXTERNAL_INPUT.md](EXTERNAL_INPUT.md)를 따른다. 디텍션·퓨전·내비게이션은 기존 외부 모듈이 담당한다.

| 입력/출력 | 담당 | 확인 내용 |
|---|---|---|
| `/manipulation/target` Target | head/융합 adapter | 동일 작업의 실제class_id, coarse 3D(m), instance ID, 취득시각/frame |
| `/manipulation/approach_result` ApproachResult | navigation adapter | 같은task_id, 실제 접근 SUCCESS 및 base_stopped |
| `/manipulation/base_lock_request` → `/manipulation/base_lease` | coordinator → navigation | 모든 베이스 이동 명령 차단 후 실제 stopped/locked heartbeat |
| `/manipulation/wrist/request` ObservationRequest | coordinator | observation_id, 목표class/instance/coarse |
| `/manipulation/wrist/result` WristObservation | wrist detector adapter | 요청ID에 대응한 새 실제 관측, class/instance, 전체 후보 batch, confidence·좌표 품질 |
| `/manipulation/run` Manipulate | 상위 작업 client | 동일task_id의 실행 goal, feedback/result/cancel |

손목 결과는 camera optical frame의 CENTER 3D와 취득시각을 권장한다. depth 표면점을 보내면 SURFACE를 명시하고 상부 관측·물체 높이·테이블 기반 변환을 적용한다. depth/mask/CameraInfo를 보내는 경로도 지원한다. 기본3개 독립 취득 프레임이 안정 조건을 만족해야 한다. 바나나의 axis_yaw는 gravity frame 기준 장축 방향이다. 정확한 필드/단위는 [INTERFACES.md](INTERFACES.md)를 따른다.

디텍터가 이미 이 타입을 publish하면 새 adapter가 필요 없다. 다른 타입이면 외부 adapter가 포맷을 변환한다. bbox만 보내는 head 메시지를3D로 포장하지 않는다. 디텍션 완료만으로 Nav와 wrist 요청/응답 계약까지 연결됐다고 판단하지 않는다.

## 데모 실행 순서

선 연결, CAN 식별, 드라이버 피드백 읽기, 실측 파일 편집의 실제 명령은 [FIRST_CONNECTION.md](FIRST_CONNECTION.md)를 따른다. 아래 순서는 외부 계약 adapter와 TF 통합이 완료된 뒤의 운영 순서다. 현재 main의 실물 센서 실행기와 이 패키지의 bringup을 그대로 함께 실행하면 팔 TF/CAN 노드가 중복될 수 있다. 기존 Head/퓨전 출력 수신부는 제공한다. 실제 Nav의 task_id·잠금 응답과 손목 요청·결과 연결 및 TF 통합은 별도로 확인해야 한다.

1. Jetson에서 실제 CAN·Piper feedback 및 손목 카메라가 동작하는지 확인하고 firmware/관절 대응/그리퍼 폭·접촉 기준을 robot.measured.yaml에 채운다. 이미 저장된 hand–eye·CAD base/TCP는 사용한다.
2. 실물 테이블/지지 영역/주변 장애물과 복귀 자세를 입력하고 최종 detector names를 정확히 매핑한다. live 이동이 있으면 coarse_fixed_frame은 충분한 TF 이력이 있는odom 등 고정 frame을 사용한다. bench용 base_link 설정을 그대로 사용하지 않는다.
3. 실제 head/융합, navigation, wrist detector 및 계약 adapter를 각각 시작한다. 같은 ROS_DOMAIN_ID·시계 기준으로 연결한다. 같은 TF를 두 노드가 동시에 publish하지 않도록 소유권을 정한다.
4. 실물 coordinator/MoveIt/bridge를 기동한다. Piper driver가 이미 실행 중이면 start_driver=false다.

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
source "$ROBOT_REPO/SW/manipulation/manipulation_ws/install/setup.bash"
ros2 launch robocup_real_manipulation bringup.launch.py \
  hardware:=true config:="$HOME/robot_config/robot.measured.yaml" \
  tuning_file:="$ROBOT_REPO/SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/tuning.yaml" \
  repo:="$ROBOT_REPO" start_driver:=false
```

5. 상위 모듈에서 물체를 선택한다. 실제Target/접근 완료가 같은ID로 전달되고 실제베이스 잠금이 확인된 다음 goal을 보낸다. 상위 client가 action goal을 보내면 별도CLI goal을 중복 송신하지 않는다.

```bash
ros2 topic info /manipulation/target -v
ros2 topic info /manipulation/approach_result -v
ros2 topic info /manipulation/wrist/result -v
ros2 topic echo /manipulation/status
```

수동으로 goal만 시작해야 한다면 실제 외부 이벤트의ID를 사용한다:

```bash
ros2 action send_goal /manipulation/run \
  robocup_real_manipulation_msgs/action/Manipulate \
  '{task_id: "실제-상위모듈-작업-ID"}' --feedback
```

6. 받은coarse 기반 관측 이동 → 실제 손목 재측정 → 클래스 규칙의TCP 생성 → pick/place/복귀 결과를 확인한다. 첫 데모는 물체 한 개에서 시작한 뒤 검증한클래스를 늘린다. 설정상enabled인 다른클래스에도 높이/반경/ID가 완비돼야 hardware 검사를 통과한다.

핸드아이 보고 오차와 실제 픽앤플레이스 성공률은 별개다. 실제 위치/접촉 결과에 따라 파지 높이·폭·관측 거리·속도를 조정하며 설정snapshot/hash와 실패원인을 기록한다. 실행·복구 절차는 [DEPLOYMENT.md](DEPLOYMENT.md)를 따른다.
