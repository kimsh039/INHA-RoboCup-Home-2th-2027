# Detection

카메라별 코드·모델·실험을 **head**와 **wrist**로 나눕니다. 공통 센서 역할과 접근 단계는 이 폴더에서 관리합니다.

| 경로 | 역할 | 안내 |
|---|---|---|
| `head/` | YOLO11n 목표 검출, OpenCV 추적, 학습·영상 평가 | [Head 구현](head/README.md) · [학습 실험](head/training/README.md) |
| `wrist/` | D435 재관측, SAM 2.1 분할, depth 점군·파지 입력 | [Wrist 안내](wrist/README.md) |

[전체 파이프라인](PIPELINE.md) · [센서 역할](SENSOR_ROLES.md) · [작업면 추출](SUPPORT_SURFACES.md) · [시뮬레이션 검출·주행](head/SIM_NAV_TEST.md)

현재 Head 검출 클래스는 `green_apple`, `peach`, `tomato`, `fanta_can`, `cup`, `banana`입니다. 2026-10-12에 기존 apple을 tomato로 정정하고 mug를 cup으로 통일하고 plate를 제외했습니다. [새 6클래스 재학습 모델·결과](head/training/experiments/20261013/TRAINING_RESULTS.md)를 게시했고 cup=0.70 / 나머지=0.25를 적용했습니다. [우분투 인수인계](head/UBUNTU_HANDOFF_20261013.md)를 참고합니다. YOLO의 bbox와 confidence를 테이블별 관측 증거로 사용합니다. 검출 confidence 자체를 “그 테이블에 물체가 있을 확률”로 해석하지 않으며, 테이블 대응·관측 이력·가림과 미관측 처리는 별도 단계입니다.
