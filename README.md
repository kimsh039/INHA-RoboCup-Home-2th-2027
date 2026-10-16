# INHA United @Home · 2027

TRACER × PiPER · Head D435 / Wrist D435IF · YDLIDAR G2 / Livox MID-360S · Jetson AGX Orin · ROS 2 Humble

**실물 실행과 관제:** [로봇·센서](SW/robot/README.md) · [웹 관제센터](SW/control_center/README.md) · [최신 구현 변경](SW/CHANGES_20261016.md)
**발표 준비:** [전체 구조·공부 순서·예상 질문](SW/PRESENTATION_STUDY_GUIDE.md) · [저장소 정리 기준과 복구](SW/REPOSITORY_MAINTENANCE.md)

## 현재 작업 흐름

Head YOLO11n → RGB–LiDAR 목표 대응 → 책상 탐색·closed approach → 베이스 잠금 → Wrist SAM 2.1 Tiny 재관측 → MoveIt/PiPER 집기 → 책상 위 접시 관측·배치 → 팔 수납 → 직선 후진·정지 → 회전·정렬·정지 → 홈 복귀를 연결한다.

전체 Task 코드는 통합됐지만 **접시 관측 자세와 실측 프로파일 commissioning이 남아 있으며, 전체 작업의 실물 성공을 확인한 상태는 아니다.** 과거 청사과 정차 실험, 오프라인 회귀, ROS 통신 검사와 전체 주행·파지 성공을 구분한다. [현재 전체 Task 조건](SW/control_center/TASK.md)

## 어디에서 무엇을 관리하는가

| 경로 | 역할 | 먼저 읽을 문서 |
|---|---|---|
| `HW/` | CAD·메시·질량/관성·실물 보정 원본 | [하드웨어](HW/README.md) |
| `SW/robot/` | 실물 센서·TF·오도메트리·충돌 보호·베이스 잠금 | [실물 로봇](SW/robot/README.md) |
| `SW/control_center/` | 관제·SLAM/Nav2·Head/LiDAR 대응·도킹·전체 Task | [관제센터](SW/control_center/README.md) |
| `SW/detection/head/` | YOLO 모델·공통 adapter·현재 데이터·학습/평가 | [Head](SW/detection/head/README.md) |
| `SW/manipulation/` | 현재 Wrist SAM·점군/기하·팔 제어·접시 배치 | [Manipulation](SW/manipulation/README.md) |
| `SW/detection/wrist/` | 현재 손목 실행 코드로 안내하는 진입 문서 | [Wrist](SW/detection/wrist/README.md) |
| `SW/simulation/` | Gazebo/MuJoCo·CAD 및 시뮬레이션 보정 | [Simulation](SW/simulation/README.md) |
| `SW/setup/` | Jetson 설치·버전·환경 기록 | [Setup](SW/setup/README.md) |

## 현재 인식 모델과 데이터

Head는 `head_yolo11n_20261013.pt`에서 만든 대상 장치의 TensorRT FP16 engine을 우선 사용한다. 클래스는 **green_apple, peach, tomato, fanta_can, cup, banana**이고 cup은 손잡이를 포함한 빨간 머그컵이다. 기본 confidence는 **cup/banana 0.50, 나머지 0.25**, NMS IoU 0.7이며 공통 adapter는 클래스별 최고 점수 박스 하나를 남긴다. 같은 클래스가 여러 테이블에 있을 때의 관측 누락은 별도 고려해야 한다.

현재 학습 데이터는 [20261013 스냅샷](SW/detection/head/datasets/20261013/README.md) 2,155장(train 1,786 / val 369)이다. [검증 mAP50 0.97474 / mAP50–95 0.70237](SW/detection/head/training/experiments/20261013/TRAINING_RESULTS.md)은 부분 수동 검토 라벨 기준 학습 검증 수치이며, 현재 후처리·실물 Task 성공률과 다르다.

## 모델과 좌표계 사용 기준

- CAD 원본: [robocup.urdf](SW/simulation/robot_description/robocup.urdf).
- 시뮬레이션/통합 보정 기록: [robocup.calibrated.urdf](SW/simulation/robot_description/robocup.calibrated.urdf).
- 실물: [부분 arm URDF + 실측 센서 TF](SW/robot/README.md). 전체 calibrated 모델을 실물에 중복 발행하지 않는다.
- RGB는 RealSense 원본 토픽을 사용한다. `calibrated_streams.py`는 CameraInfo를 발행하며 RGB를 중계하지 않는다.
- 모델·실행 패키지 변경은 실행 중 프로세스를 자동 갱신하지 않는다. [전체 Task 배포/필수 조건](SW/control_center/TASK.md)을 따른다.

최상위는 `HW/`, `SW/`, 이 README와 Git 설정만 유지한다. 오래된 실행 예시·중복 데이터는 활성 트리에서 정리하고, 과거 실험 지표와 복구 커밋은 남긴다.
