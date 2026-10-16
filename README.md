# INHA United @Home · 2027

**실물 로봇 실행:** [Jetson에서 실물 캘리브레이션을 적용하는 방법](SW/robot/README.md). 최신 G2·Base–Mid360·Head–Mid360·Wrist hand-eye·RGB 내부 보정·Tracer 오도메트리를 사용합니다.

**웹 관제센터:** [젯슨에서 실행하고 맥 localhost로 접속](SW/control_center/README.md). 실시간 SLAM·카메라·검출·점군 관측과 카메라/LiDAR 개별 실행·중지를 제공합니다.

**전체 Task 통합 (2026-10-16):** 관제센터 마지막 **전체 Task** 탭에서 물체를 선택하면 **SLAM → NAV → 탐지·closed approach → 집기 → 손목 SAM으로 책상 위 접시 관측·배치 → 팔 수납 → 직선 후진 → 정지 → 제자리 회전·Head 정렬 → 정지 → 지정 홈 복귀**를 연결합니다. 물체는 책상 위 접시에 놓고 로봇이 복귀합니다. [홈 등록·실물 설정·배포 방법 및 현재 한계](SW/control_center/TASK.md).

확장된 ROS 메시지와 실행용 패키지는 로컬에 설치하고 관제센터 및 읽기 전용 팔 모니터에 적용했습니다. SLAM 시작 시 첫 유효 좌표로 시작 위치·방향을 홈에 자동 등록합니다. 전체 Task 실물 실행에는 접시 관측 자세와 실물 프로파일 설정이 남아 있습니다. 자동 테스트와 전체 작업 실물 주행·파지는 수행하지 않았습니다.

인하 유나이티드 앳홈 2기 · TRACER × PiPER · ROS 2 Humble · Jetson AGX Orin

| 영역 | 내용 | 시작 문서 |
|---|---|---|
| **HW** | CAD, 메시, 하드웨어 URDF 원본 | [하드웨어](HW/README.md) |
| **SW** | 설치·운영, 시뮬레이션, 보정, Head/Wrist 인지 | [소프트웨어](SW/README.md) |

```text
HW/                         # 하드웨어 자료
├── URDF/                   # 로봇 모델·기구 구조
└── calibration/            # 캘리브레이션 결과
    └── camera_intrinsics/  # 카메라 내부 파라미터·오차 결과
        ├── head/           # 헤드 카메라
        └── wrist/          # 손목 카메라
SW/
├── control_center/         # Jetson 웹 관제 / Mac SSH 터널
├── manipulation/           # 실물 팔·손목 SAM·접시 배치 및 인계
├── setup/                  # Jetson 설치와 실행
├── simulation/             # Gazebo·MuJoCo·보정
└── detection/
    ├── head/               # YOLO11n 검출·추적·학습·평가
    └── wrist/              # D435·SAM 2.1·정밀 분할
```

새 코드와 자료는 위 구조에 맞춰 추가합니다. 저장소 루트에는 `HW/`, `SW/`, `README.md`를 유지하고, 카메라별 검출 코드·모델·실험은 `SW/detection/head/` 또는 `SW/detection/wrist/`에 둡니다. 공통 인지 문서는 `SW/detection/`에서 관리합니다. 가상환경과 빌드 산출물은 각 작업 폴더의 Git 제외 경로에 생성합니다.

Head와 Wrist 카메라는 모두 RealSense D435를 사용합니다. Head는 목표 탐색과 두 테이블의 관측 증거 수집, Wrist는 접근 후 재관측·분할·depth 점군을 담당합니다. [인지 구성](SW/detection/README.md) · [센서 역할](SW/detection/SENSOR_ROLES.md)

현재 최종 보정 모델은 [robocup.calibrated.urdf](SW/simulation/robot_description/robocup.calibrated.urdf)이며, [robocup.urdf](SW/simulation/robot_description/robocup.urdf)는 CAD 기준 원본입니다. 시뮬레이션 보정 기록과 실기 검증 범위는 [SW 안내](SW/README.md)에 있습니다.

[Head YOLO11n 7개 클래스 모델·실제 평가 결과](SW/detection/head/training/experiments/20261007/TRAINING_RESULTS.md): 100 epoch 완료, 두 보관 영상 739프레임 추론. 라벨 초안 기준 검증과 실제 누락을 함께 기록했습니다.

현재 검출 대상은 **green_apple, peach, tomato, fanta_can, cup, banana**입니다. [20261013 새 6클래스 모델·학습 결과](SW/detection/head/training/experiments/20261013/TRAINING_RESULTS.md)를 게시했습니다. cup은 빨간 머그컵만 대상으로 신뢰도 0.70, 다른 클래스는 0.25를 적용합니다. [우분투 / Jetson 작업 이어가기](SW/detection/head/UBUNTU_HANDOFF_20261013.md).
