# 검증 결과와 실물 확인 항목

2026-10-14 실제 손목 입력 연결부 반영 후 노트북 x86_64 / Ubuntu 22.04 / ROS 2 Humble / Python 3.10에서 빌드와 전체 단위 검사를 재실행했다. ROS mock 계약12개는2026-10-11 검증 기록이다. Jetson에 접속하거나 실제 Piper·그리퍼·베이스를 움직인 결과는 아니다.

| 검사 | 결과 | 근거 |
|---|---|---|
| 빌드·설치 | 메시지 패키지는 CMake 직접 빌드·설치, 실행 패키지는 setuptools 빌드·설치 성공 | [빌드 기록](../validation/build.log), [Python 패키지 설치 로그](../validation/phase1_python_install.log) |
| 단위 검사 | hand–eye 적용·TCP 보정·데모 점검·최신 클래스 매핑 포함58개 성공 | [unit_tests.log](../validation/unit_tests.log) |
| ROS 통신 계약 | 12개 시나리오 성공 | [ros_contract.log](../validation/ros_contract.log) |
| launch 인자 로딩 | config, tuning_file, hardware, start_driver 등 확인 | [launch_args.log](../validation/launch_args.log) |
| TCP CAD 점 확인 | 실제 STL/URDF의 손가락 끝 높이와 link6 기준점 재계산:142.5mm | [tcp_cad.json](../validation/tcp_cad.json), [기준점 그림](../validation/tcp_cad.png) |
| TCP ROS 실행 명령 설치 | cad/record/solve/apply 명령 진입점 확인 | [tcp_cli.log](../validation/tcp_cli.log) |
| 실물 hand–eye 반영 | 공개15개 PARK 계산 재현, 기본 설정과 원본 행렬의 방향/단위/회전 일치, 생성 URDF 적용 확인 | [wrist_handeye_import.json](../validation/wrist_handeye_import.json), test_calibration_integration.py |
| 읽기 전용 데모 설정 점검 | 전체 미정 값·flag 구분, 적용된 보정 변환 확인, disabled 클래스 처리·잘못된 회전 검출 | [demo_config_check.json](../validation/demo_config_check.json), test_demo_check.py |

check_live_demo.py의 실제 ROS graph/feedback/TF 조회는 Jetson에서 실행해야 한다. 로봇이 연결되지 않은 이 작업에서 실제 라이브 입력을 확인한 것은 아니다.

추가 TCP 검사는 저장된 기존 FK 관측과 새 FK의 일치, 자세 다양성 부족 거부, holdout 오차가 fit에 섞이지 않음, 회전된 프로브 오프셋 반영, 관절 안정 구간의 중복 시각/간격/이동 거부, 합성·미검증 결과의 적용 거부 및 구동 flag 보존을 확인했다. 실물 접촉·지그 치수·축 측정과 Jetson의 실제 피드백 수집은 아직 수행하지 않았다.

단위 검사는 이벤트 도착 순서·중복·재시작 replay, 클래스/폭 설정, 중심과 표면점 구분, 카메라 오프셋과 관측 자세, depth outlier, 같은 클래스 대상 연계, 경로 속도/가속도/범위, 각 단계 실패·취소의 후속 동작 차단을 검사한다. ROS 검사는 실제 topic/action 전송으로 target-first, approach-first, 오른쪽 계획 실패 후 왼쪽 선택, 접근 실패, 베이스 이동, 재관측 timeout, 오래된 영상, 잘못된 depth, 취득시각 TF 누락, 모호한 대상, 취소, place 성공 이후 복귀 실패를 확인한다.

ROS 검사의 모션·접촉·센서·베이스 응답은 합성이다. 실제 MoveIt 계획 성공, mesh 충돌 정확성, 물리 파지 성공률 또는 정지 성능을 증명하지 않는다. 테스트는 별도 ROS_DOMAIN_ID=187에서 수행하며 실제 로봇 연결용 adapter로 사용하지 않는다.

## Jetson ARM64와 실제 로봇에서 남은 검증

- 설치 기록과 현재 Jetson OS/JetPack/ROS/Python을 대조하고 ARM64에서 rosdep·colcon 빌드를 재실행한다. 센서·Piper SDK·CAN·RealSense 의존성과 고정 드라이버 SHA를 확인한다.
- [필수 입력 목록](REQUIRED_VALUES.md)의 미정 값을 실측한다. 카메라 optical frame/취득시각, hand–eye 기준 link6, link6↔TCP, 관절 sign/offset, 실제 그리퍼 폭/effort 대응을 확인한다.
- 외부 디텍션 adapter와 Tracer 접근 완료·실제 이동 잠금/heartbeat를 연결한다. 현재 저장소의 외부 모듈이 신규 계약을 이미 구현한 상태는 아니다.
- 전용 gripper feedback timestamp를 제공하는 [드라이버 패치](../tools/piper_feedback.patch)를 검토·적용·재빌드한 뒤 신선도 검사와 단절 동작을 확인한다. 이 작업에서 설치된 드라이버를 수정하지 않았다.
- RViz/MoveIt에서 실물 장착·자기 충돌·테이블·주변 장애물·attached object를 검증하고, 저속 무부하 경로부터 실제 경로 추종·CAN 지터·hold/stop을 확인한다.
- 카메라 최소 depth 거리, coarse Z 오차, 관측 시야/도달성을 측정하고 관측 높이·파지 높이·open/close 폭을 trial로 조정한다. 클래스별 실제 형상/높이/모델 ID를 확인한다.
- 실제 물체를 이용해 파지 확인, 운반 중 접촉 유지, 오른쪽/왼쪽 place, 팔 복귀, 취소·센서 단절·베이스 잠금 단절을 단계별 검증한다. [배포 절차](DEPLOYMENT.md)를 따른다.

## 현재 제약

최신6클래스 출력에 맞춰 토마토·컵 규칙을 활성화했다. 기존7클래스 가중치를 쓰는 손목 adapter는 Head와 동일한 매핑을 한 번 적용해야 한다. 바나나는 외부 인식의 장축 방향과 몸통 기하 설정이 필요해 물리 파지를 비활성화했다. 주변 환경은 설정한 정적 장애물 목록을 전제로 하며 새로운 장애물 감지는 구현하지 않는다. coarse 측정이 TF 이력보다 오래되면 재측정/외부 고정 좌표 처리가 필요하다. place 성공은 지지 영역과 개방 피드백에 근거하며 별도의 영상 기반 내려놓기 확인은 제공하지 않는다. 물리 접촉 판정과 CAN 단절 시 정지는 현장 검증이 필요하다.

2026-10-14: 기존 패키지에 phase1.launch/phase1_pick 및 실제 YOLO+SAM2/ROS aligned depth 연결부를 추가했다. 왜곡된 RGB 광선·depth 품질·전체 후보 batch·원본 취득시각·느린 추론 결과 거부·TF 소유권 검사를 포함한 단위58개가 통과했다. 모델 응답은 이 단위 검사에서 대역으로 제공했다. 실제 GPU 추론, Jetson ARM64 colcon build, 카메라/팔 동작은 미검증이다. 현재 노트북에 torch/sam2가 없어 실제 모델 로딩은 실행하지 않았다. 노트북 colcon 시도는 진행이 멈춰 중단했고, 같은 Python 소스를 setuptools로 빌드·설치하여 진입점을 확인했다. [1차 실행 절차](PHASE1_RUN.md).

추가 검사: 오목한 마스크의 빈 무게중심 대신 실제 몸통에서 depth 측정, 로봇 좌표계 PCA 방향, uint8/boolean 마스크, 분리·원형·invalid depth 거부, 전체 충돌 반경, CAD TCP Y 닫힘축의 직교, 관측 축의180도 부호/불안정 처리, 접근 전 좌표의odom 보존. 실제 GPU/로봇 실험은 별도다.
