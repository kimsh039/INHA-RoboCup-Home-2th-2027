# Detection

현재 인지는 Head의 클래스/위치 탐색과 Wrist의 근거리 재관측으로 나뉜다. 현재 실행 코드와 모델·연구 기록의 위치를 아래와 같이 구분한다.

| 기능 | 현재 코드 | 안내 |
|---|---|---|
| Head 6클래스 YOLO/TensorRT, 모델·학습 | `head/` | [Head](head/README.md) |
| Head 객체를 LiDAR/odom 좌표·책상과 연결 | `SW/control_center/head_lidar.py` | [Detection·도킹](../control_center/DETECTION.md) |
| 정밀 접근·도킹 인계 | `SW/control_center/detection_approach.py` | [도킹/팔 계약](../control_center/MANIPULATION.md) |
| 현재 Wrist SAM·depth·3D 관측 | `SW/manipulation/.../wrist_frontend.py` | [Wrist](wrist/README.md) |
| 구형 설계·시뮬레이션 상태 머신 | `head/head_detection_ws/` | [SIM_NAV_TEST](head/SIM_NAV_TEST.md) |

Head 클래스는 `green_apple, peach, tomato, fanta_can, cup, banana`. 현재 adapter는 confidence 필터 후 클래스별 최고 후보 하나를 남긴다. 이 confidence를 “어느 테이블에 있을 확률”로 직접 해석하지 않는다. 두 테이블의 동일 클래스 관측을 모두 보존하려면 top-1 정책의 한계를 검토해야 한다.

Wrist 기본값은 **SAM 2.1 Hiera Tiny + 자동 마스크**다. Head가 지정한 클래스·coarse 위치·depth·크기 조건으로 후보를 연결한다. SAM 자체는 클래스 분류기가 아니다. 원래 Small USB 카메라 실험은 현재 runtime에 연결되지 않아 활성 트리에서 정리했다.

[전체 Task](../control_center/TASK.md) · [발표 공부 가이드](../PRESENTATION_STUDY_GUIDE.md) · [현재 변경 기록](../CHANGES_20261016.md) · [기존 센서 역할 설계](SENSOR_ROLES.md) · [기존 상세 설계](PIPELINE.md)

과거 설계 문서의 Tiny prompt/GraspNet 계획이나 미구현 표시는 당시 설계다. 현재 구현 여부는 위 실행 코드와 최신 통합 문서를 기준으로 확인한다.
