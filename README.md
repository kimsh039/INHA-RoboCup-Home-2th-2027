# INHA United @Home · 2027

인하 유나이티드 앳홈 2기 · TRACER × PiPER · ROS 2 Humble · Jetson AGX Orin

| 영역 | 내용 | 시작 문서 |
|---|---|---|
| **HW** | CAD, 메시, 하드웨어 URDF 원본 | [하드웨어](HW/README.md) |
| **SW** | 설치·운영, 시뮬레이션, 보정, Head/Wrist 인지 | [소프트웨어](SW/README.md) |

```text
HW/                         # 하드웨어 자료
SW/
├── setup/                  # Jetson 설치와 실행
├── simulation/             # Gazebo·MuJoCo·보정
└── detection/
    ├── head/               # YOLO11n 검출·추적·학습·평가
    └── wrist/              # D435·SAM 2.1·정밀 분할
```

Head와 Wrist 카메라는 모두 RealSense D435를 사용합니다. Head는 목표 탐색과 두 테이블의 관측 증거 수집, Wrist는 접근 후 재관측·분할·depth 점군을 담당합니다. [인지 구성](SW/detection/README.md) · [센서 역할](SW/detection/SENSOR_ROLES.md)

현재 최종 보정 모델은 [robocup.calibrated.urdf](SW/simulation/robot_description/robocup.calibrated.urdf)이며, [robocup.urdf](SW/simulation/robot_description/robocup.urdf)는 CAD 기준 원본입니다. 시뮬레이션 보정 기록과 실기 검증 범위는 [SW 안내](SW/README.md)에 있습니다.

[Head YOLO11n 7개 클래스 모델·실제 평가 결과](SW/detection/head/training/experiments/20261007/TRAINING_RESULTS.md): 100 epoch 완료, 두 보관 영상 739프레임 추론. 라벨 초안 기준 검증과 실제 누락을 함께 기록했습니다.
