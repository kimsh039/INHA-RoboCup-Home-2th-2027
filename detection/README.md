# Detection Workflow

> [!IMPORTANT]
> **설계 갱신: 2026-10-05.** 기본 알고리즘은 **전체 프레임 YOLO11n 검출 → OpenCV ROI 추적 → 주기적 YOLO 검증 → 추적 실패 시 전체 프레임 재검출 → 3D LiDAR 기반 근접 접근 → 손목 RGB-D 기반 파지**이다. 아래 내용은 구현할 파이프라인이며, 실시간 통합 완료를 뜻하지 않는다.

헤드 카메라는 목표 탐색과 접근 중 추적을 담당한다. YOLO를 매 프레임 실행하는 대신, 최초 검출과 추적 결과의 검증·보정 및 목표 재획득에 사용한다. 손목 카메라는 접근 후 목표를 다시 확인하고 정밀 분할·점군 생성·파지 후보 추정을 담당한다.

> **실기 환경과 설계를 구분한다.** [프로젝트 README](../README.md)와 [Jetson 운영 문서](../setup/jetson/README.md)의 기록은 Head **D435** / Wrist **D405**, YOLO11n·YOLO11n-seg, SAM 2.1 **Tiny GPU** 준비 상태이며, 초기 3D 실습은 RealSense aligned depth이다. 이 설계의 기본 검출 모델은 **YOLO11n (`yolo11n.pt`)**이다. 대화에서 지정한 주행용 **YDG2 2D LiDAR**와 접근용 **Mid-360S**를 역할 기준으로 표기한다. 기존 운영 문서는 2D 모델 미확정 / MID-360으로 기록되어 있으므로, 실제 제품명·드라이버·프레임은 연결 시 대조한다. 추적기, LiDAR 영상 투영, 자동 접근, GraspNet과 Pick & Place 통합은 구현·검증 대상이다.

이 문서의 범위는 **물체 검출 → ROI 추적·검증·복구 → 접근용 3D 위치 갱신 → 손목 재관측 → 정밀 분할·점군 → 파지 후보 전달**이다. 차체·팔 이동과 Pick & Place 실행은 외부 모듈과의 연결 조건을 설명한다. 관측 거리, 처리 주기와 품질 임계값은 실제 Jetson AGX Orin과 센서에서 측정한 뒤 확정한다.

## 1. 센서와 모델

| 단계 | 센서 / 모델 / 도구 | 역할 |
|---|---|---|
| 헤드 최초 검출·검증·재획득 | 헤드 **D435 RGB**, Ultralytics **YOLO11n** | 클래스, confidence, bbox 생성 및 추적 결과 보정 |
| 헤드 ROI 추적 | **OpenCV 추적기**, 원본 RGB | YOLO 실행 사이에 목표 bbox 갱신, ROI 표시와 상태 판정. 추적기 종류는 실측 비교 후 선택 |
| 주행 위치 추정·장애물 입력 | **YDG2 2D LiDAR** | 외부 주행 모듈의 입력. 물체의 3D 파지 위치 추정과 구분 |
| 접근용 3D 위치 | **Livox Mid-360S**, `livox_ros_driver2`, ROS 2 `tf2`, **PCL** | 목표 bbox에 대응하는 LiDAR 군집 선택과 대표 위치 갱신 |
| 손목 재검출 | 손목 **D405 RGB**, YOLO11n | 접근 후 달라진 시점에서 같은 목표를 다시 선택 |
| 정밀 분할 | **SAM 2.1 Hiera Tiny**, `SAM2ImagePredictor` | 손목 정지 이미지의 box/point prompt로 목표 마스크 생성 |
| 물체·장면 점군 | 손목 D405 depth, `realsense-ros` / `librealsense`, **Open3D** | RGB-depth 정합, 유효 depth 선택, 역투영·이상점 제거 |
| 파지 후보 | **GraspNet Baseline**, `graspnetAPI` | 6-DoF 파지 위치·방향, 그리퍼 폭, 접근 깊이, 점수 생성 |

초기 모델은 다음 조합으로 평가한다. 성능 수치는 실제 Jetson AGX Orin에서 측정하며, 다른 GPU나 로봇의 FPS를 목표 성능으로 사용하지 않는다.

| 모델 | 체크포인트 / 설정 | 선택 기준 |
|---|---|---|
| YOLO11n | `yolo11n.pt` | 헤드 전체 프레임 검출·주기 검증·재획득과 손목 재검출. bbox를 출력하며 인스턴스 마스크는 출력하지 않음 |
| SAM 2.1 Tiny | `sam2.1_hiera_tiny.pt` + `configs/sam2.1/sam2.1_hiera_t.yaml` | 손목 정지 이미지의 정밀 분할. 추론 시간과 마스크 품질을 함께 평가 |
| GraspNet Baseline | `checkpoint-rs.tar` | RealSense 데이터로 학습한 공식 baseline을 초기 비교 기준으로 사용 |

YOLO 사전학습 모델의 클래스에 없는 대회 물체는 자체 bbox 데이터로 검출 모델을 fine-tuning한다. 기본 헤드 경로에는 YOLO11n-seg와 SAM을 사용하지 않는다. **OpenCV 추적은 클래스 인식을 갱신하지 않으며, SAM 2.1은 클래스 인식이나 depth 복원을 대신하지 않는다.**

## 2. 전체 흐름

```mermaid
flowchart TD
    A[헤드 RGB 전체 프레임] --> B[YOLO11n 검출 · 목표 선택]
    B --> C{목표 검출?}
    C -- 아니오 --> A
    C -- 예 --> T[OpenCV 추적기 초기화]
    T --> U[새 프레임에서 bbox 갱신 · 확장 ROI 표시]
    U --> V{추적 실패?}
    V -- 예 --> LOST[위치 무효화 · 접근 보류 · 전체 프레임 재획득]
    LOST --> A
    V -- 아니오 --> Q{검증 주기 도달 또는 이상 징후?}
    Q -- 예 --> Y[YOLO11n 재검출 · 인스턴스 대응]
    Y --> Z{동일 목표 확인?}
    Z -- 아니오 --> LOST
    Z -- 예 --> FIX[bbox 보정 · 추적기 재초기화]
    Q -- 아니오 --> D[영상 투영 · 목표 LiDAR 군집 선택]
    FIX --> D
    L[Mid-360S 점군] --> D
    D --> E{3D 위치와 관측 나이 유효?}
    E -- 아니오 --> RH[접근 보류 · 헤드 재관측]
    RH --> U
    E -- 예 --> F[외부 접근 모듈에 위치 갱신 전달]
    NAV[2D LiDAR · 위치 추정 · 장애물 정보] --> F
    F --> ARR{손목 관측 위치 도달?}
    ARR -- 아니오 --> U
    ARR -- 예 --> G[차체와 팔 정지 · 손목 관측으로 전환]
    G --> H[손목 YOLO11n 재검출 · 목표 대응]
    H --> HM{목표 대응 성공?}
    HM -- 아니오 --> R[재관측 요청 · 실패 사유 반환]
    HM -- 예 --> I[SAM 2.1 Tiny 정밀 마스크]
    W[손목 정합 depth · CameraInfo] --> J[목표 점군 + 주변 장면 점군]
    I --> J
    J --> K{depth · 점군 품질 통과?}
    K -- 아니오 --> R
    R -. 관측 자세 변경 후 재시도 .-> G
    K -- 예 --> N[GraspNet 파지 후보 · 기하학적 필터]
    N --> NC{유효한 파지 후보 존재?}
    NC -- 아니오 --> R
    NC -- 예 --> O[외부 파지 모듈의 IK · 경로 검사]
    O --> OC{실행 가능?}
    OC -- 아니오 --> R
    OC -- 예 --> P[Pick & Place 실행 · 결과 확인]
```

### A. 전체 프레임 YOLO11n으로 목표 찾기

1. 탐색 상태에서 헤드 RGB **전체 프레임**에 YOLO11n을 실행해 `class_id`, confidence, bbox를 생성한다. 탐색 중 실행 주기는 연산 예산에 맞춰 정한다.
2. 임무에서 지정한 클래스와 관측 이력을 이용해 목표 인스턴스를 선택한다.
3. bbox를 **원본 RGB 좌표**로 복원하고 촬영 timestamp와 시스템이 부여한 `target_id`를 보존한다. 추론용 resize/letterbox 좌표를 그대로 추적이나 LiDAR 투영에 사용하지 않는다.
4. 선택한 bbox로 OpenCV 추적기를 초기화하고 추적 상태로 전환한다. tracker 내부 ID와 임무의 `target_id`를 구분한다.

같은 클래스의 물체가 여러 개면 클래스 이름만으로 목표를 선택하지 않는다. 접근 전후의 3D 위치와 영상 특징을 함께 사용하고, 구분이 모호하면 재관측한다.

### B. OpenCV ROI 추적과 주기적 YOLO 검증

**ROI를 표시하거나 잘라내는 작업 자체가 검출은 아니다.** YOLO 실행 사이의 프레임에서는 OpenCV 추적기가 선택한 목표의 bbox를 갱신한다. 목표 주변의 여유 영역을 포함한 확장 ROI는 시각화와 검증 검색 범위에 사용한다. LiDAR 대응에 사용하는 목표 bbox와 확장 ROI를 구분한다.

1. 새 헤드 프레임마다 추적기를 갱신하고 bbox 위치·크기, 추적 성공 여부와 이상 징후를 기록한다. 사용할 추적기와 처리 해상도는 이동·가림·크기 변화 조건에서 비교해 결정한다.
2. 검증 주기에 도달하면 YOLO11n을 다시 실행한다. **첫 구현은 전체 프레임 검증**으로 시작한다. 이후 연산 절감이 필요하면 확장 ROI 검증을 추가하되, 더 긴 주기의 전체 프레임 검증과 실패 시 전체 프레임 재획득을 유지한다.
3. YOLO 후보와 추적 bbox를 클래스, 겹침 정도(IoU), 중심 이동량, 크기 변화, 사용 가능한 3D 위치·영상 특징으로 대응시킨다. 가까운 박스나 같은 클래스라는 이유만으로 다른 물체로 전환하지 않는다.
4. 동일 목표가 확인되면 YOLO bbox로 추적기를 보정·재초기화하고 마지막 검증 시각을 갱신한다. 재검출 결과를 받은 시각이 아닌 **검증에 사용한 프레임의 촬영 시각**을 기록한다.
5. 추적기 실패, 검증 불일치 또는 검증 유효기간 초과 시 목표 위치를 무효화하고 외부 접근 모듈에 접근 보류를 알린 뒤, 전체 프레임 YOLO 재획득으로 돌아간다. 동일 인스턴스인지 불명확하면 재획득 성공으로 처리하지 않는다.

검증은 주기뿐 아니라 bbox 위치·크기의 급변, 화면 가장자리 접근, 가림 의심, 영상 bbox와 LiDAR 위치의 불일치에도 요청한다. 구체적인 판단 기준은 실측으로 정한다. 추적기가 성공을 반환해도 배경으로 이동할 수 있으므로 주기 검증을 생략하지 않는다.

비동기 추론을 사용할 경우 YOLO 결과는 해당 입력 프레임의 bbox이다. 오래된 결과를 현재 프레임의 bbox로 덮어쓰지 않는다. 입력 프레임 이후의 추적을 재적용하거나 최신 프레임에서 다시 검증하며, 지연 허용 범위를 넘긴 결과는 폐기한다. 대기열에 모든 프레임을 쌓는 대신 최신 입력을 우선하고, 진행 중인 추론 요청은 중복 발행하지 않는다.

| 상태 | 주요 처리 | 전환 조건 |
|---|---|---|
| `SEARCH` | 전체 프레임 YOLO, 목표 선택 | 선택 성공 시 `TRACK`; 미검출이면 탐색 유지 |
| `TRACK` | OpenCV bbox 갱신 | 주기·이상 징후 발생 시 `VERIFY`; 실패 시 `LOST` |
| `VERIFY` | YOLO 재검출과 동일 목표 확인 | 확인 시 `TRACK`; 불일치·기한 초과 시 `LOST` |
| `LOST` | 접근 보류, 기존 3D 위치 무효화, 전체 프레임 재획득 | 동일 목표 확인 후 `TRACK`; 모호하면 재관측·목표 재선택 요청 |
| `WRIST_OBSERVE` | 정지 후 손목 재검출·분할·점군·파지 후보 생성 | 목표·점군 품질 통과 및 유효 후보 확보 시 `GRASP_READY`; 실패 시 재관측 |
| `GRASP_READY` | 파지 후보 전달 | 외부 모듈 검사 후 실행; 장면 변경·관측 만료 시 재관측 |

근접 접근은 `TRACK` / `VERIFY` 중 유효한 3D 위치가 있을 때 병행한다. `VERIFY`에서 마지막 검증이 아직 유효한 동안만 위치를 갱신할 수 있다. 검증 불일치가 확인되면 즉시 접근을 보류한다. 주행 모듈이 손목 관측 위치 도달을 알리고 정지 조건을 통과하면 `WRIST_OBSERVE`로 전환한다.

### C. Mid-360S로 접근용 3D 위치를 갱신하고 근접 접근

이 설계는 **3D LiDAR를 접근용 위치 센서**로 사용한다. 초기 개발은 이미 준비한 RealSense aligned depth로 시작할 수 있으며, LiDAR 영상 투영·군집 선택이 검증된 뒤 연결한다. LiDAR 점이 부족할 때는 품질이 확인된 헤드 depth로 보완하거나 재관측하며, 사용한 위치 센서를 메타데이터에 기록한다.

1. 외부 보정값과 `tf2`를 이용해 LiDAR 점을 헤드 color optical frame으로 변환한다.
2. 해당 RGB/`CameraInfo`에 맞게 영상에 투영하고 **같은 관측 시각의 목표 bbox** 내부 점을 후보로 선택한다. 추적 중에는 YOLO 검증 유효기간과 추적 상태를 함께 확인한다.
3. 공간 범위 필터와 PCL 군집화를 적용한다. 테이블 등 **식별된 지지 평면**을 분리하고 목표 물체의 표면은 보존한다. YOLO11n은 마스크가 없으므로 bbox 안의 배경·인접 물체 혼입을 별도로 제거해야 한다.
4. 기존 목표 위치, 군집 크기, bbox와의 투영 대응 및 거리 일관성으로 목표 군집을 선택한다. bbox 내부 모든 점의 평균이나 가장 큰 군집만으로 결정하지 않는다.
5. 군집의 대표 위치, 점 수, 공간 분산, 위치 촬영 시각, 마지막 YOLO 검증 시각과 유효 여부를 반환한다. 이 위치는 보이는 표면의 대표값이며 물체의 정확한 기하학적 중심이라고 가정하지 않는다.
6. 외부 접근 모듈은 유효한 위치를 받아 장애물·차체 위치·팔 도달성을 고려해 손목 관측 위치로 이동한다. 접근 중에도 헤드 추적·검증과 3D 위치 갱신을 계속한다. **2D LiDAR는 주행 입력, Mid-360S는 목표의 접근용 3D 위치 입력**으로 역할을 구분한다.

정류된 핀홀 영상에서는 `u = fx·X/Z + cx`, `v = fy·Y/Z + cy`로 투영한다. 원시 왜곡 영상이라면 왜곡을 반영한 투영이 필요하다. 여기서 `Z`는 **카메라 전방 축 깊이**이며 LiDAR의 방사 거리와 구분한다.

작은 물체에 점이 부족하면 차체를 멈추고 짧게 누적해 재관측한다. 이동 중 누적은 odometry와 점별 시간 정보를 이용한 운동 보정이 필요하다. 최근 submap을 사용할 경우 좌표계·생성 시점·누적 구간을 확인하고, 오래된 점이나 같은 submap의 반복 사용을 새로운 독립 측정으로 취급하지 않는다.

**위치 품질이 부족하면 목표를 찾았다는 RGB 결과만 반환하고 3D 위치는 무효로 표시한다.** 필요하면 외부 접근 모듈이 지지면 형상을 기준으로 관측 위치를 바꾼다. LiDAR가 작은 물체를 항상 검출한다고 가정하지 않는다.

접근 완료는 임의의 고정 거리만으로 판단하지 않는다. 외부 모듈이 실제 손목 카메라의 검증된 관측 거리, RGB/depth 공통 시야, 팔 도달성과 정지 조건을 만족하는지 확인한 후 손목 관측으로 전환한다.

### D. 손목 카메라로 다시 관측

외부 팔 모듈에 목표 3D 위치와 관측 요구사항을 전달한다. 카메라–물체 거리, RGB/depth 공통 시야, 팔 도달성, 손가락·테이블 가림을 만족하는 **관측 자세**를 선택한 뒤 차체와 팔을 정지한다.

- 실제 손목 D405의 지원 profile과 관측 범위를 확인하고 거리별 RGB/depth 품질을 실측한다. 기존 손목 D435 설계의 30 / 40 / 50 cm 조건을 D405에 그대로 적용하지 않는다.
- 손목 RGB에서 YOLO11n을 다시 실행한다. 헤드에서 얻은 픽셀 bbox나 추적기 상태를 손목 이미지에 그대로 재사용하지 않는다.
- 헤드에서 추정한 3D 목표를 손목 영상에 투영해 재검출 후보와 연결한다. 시점 차이와 위치 불확실성을 고려한 ROI를 사용한다.
- 선택한 손목 bbox를 `SAM2ImagePredictor`의 box prompt로 사용한다. 필요하면 물체 내부/배경 point prompt로 마스크를 보정한다.

헤드→손목 전환에는 `target_id`, 클래스, 고정 좌표계의 목표 3D 위치, 위치 품질과 촬영 timestamp를 전달한다. 같은 목표인지 확인한 뒤 **SAM 2.1 Tiny**를 실행하며, 목표 대응이 실패하면 손목 재관측 또는 헤드 재획득을 요청한다.

첫 구현은 정지 이미지의 SAM 분할로 시작한다. 헤드의 고주기 추론을 줄이고 손목 관측과 파지 추론에 연산을 배분한다. 연속 영상 추적이 필요하면 별도의 video predictor 경로를 평가하되, 헤드와 손목 사이의 목표 ID 대응은 별도로 처리한다.

### E. 유효 depth로 점군 생성

손목 RGB, RGB에 정합된 depth, 해당 격자의 `CameraInfo`를 동기화한다. **SAM 마스크가 좋아도 물체의 depth가 없으면 정밀 3D 관측에 성공한 것으로 판단하지 않는다.**

1. 실제 driver의 depth scale·encoding·stride를 확인하고 미터로 변환한다. `16UC1`이라는 이유만으로 고정된 `/1000`을 적용하지 않는다. `32FC1` 입력은 단위 계약을 확인한다.
2. 검증한 거리 범위의 유효 픽셀만 사용한다. 배경으로 튀는 depth와 경계의 혼합 픽셀을 제거한다.
3. 필요하면 disparity-domain spatial filter를 적용한다. Temporal filter는 정지 관측에서만 평가하고, 팔/차체 이동 또는 목표 변경 시 이력을 초기화한다.
4. RGB 마스크와 같은 격자의 intrinsics로 역투영한다. 정합·crop·resize 후의 격자에 맞는 intrinsics를 사용한다.
5. Open3D로 점군을 정리하고, 관측 시점의 TF로 지정된 고정 좌표계에 변환한다.

역투영은 `X = (u−cx)·Z/fx`, `Y = (v−cy)·Z/fy`, `Z = depth_m`이다. 센서 CAD 원점 대신 실제 depth/color optical frame과 보정값을 사용한다.

점군은 두 종류를 유지한다.

| 출력 | 용도 |
|---|---|
| **목표 물체 점군** | SAM 마스크와 유효 depth로 만든 목표 표면. 파지 후보의 목표 물체 대응 확인 |
| **주변 장면 점군** | 테이블·인접 물체를 포함한 작업 영역. 파지 후보의 그리퍼·접근 구간 충돌 검사 |

Hole filling으로 추정한 값은 실제 측정값과 구분한다. 큰 구멍을 메운 결과나 과거 EMA 값만 남은 영역을 관측된 표면처럼 사용하지 않는다. 여러 자세의 점군을 합칠 때는 각 timestamp의 팔 TF와 hand–eye 보정이 필요하다.

### F. GraspNet 파지 후보와 Pick & Place 연결

GraspNet Baseline에 검증된 작업 영역 점군을 입력하고, SAM에서 얻은 목표 점군에 대응하는 후보를 선택한다. 공식 demo의 입력 점 수는 기본 **20,000개**이며 전처리·샘플링을 구현해야 한다. 부족한 점을 반복 샘플링해도 새로운 기하 정보가 생기는 것은 아니다.

- 출력은 단일 잡는 각도가 아니라 **6-DoF pose, 그리퍼 폭, 접근 깊이, score**를 포함한 후보 집합이다.
- `graspnetAPI.GraspGroup`의 NMS·점수 정렬과 baseline의 `ModelFreeCollisionDetector`를 활용한다. 충돌 검사는 목표 점군만이 아니라 주변 장면을 포함한다.
- 실제 Piper의 그리퍼 형상·허용 폭, 카메라 마운트와 접근 방향을 반영한다. Baseline의 기본 손가락 치수와 파지 좌표계가 Piper에 맞는다고 가정하지 않는다.
- 예측 grasp frame을 실제 TCP frame으로 변환한다. 외부 팔 모듈에서 IK와 전체 로봇·경로 충돌 검사를 통과해야 실행할 수 있다.
- 손목 depth의 최소 거리·가림 때문에 **최종 파지 직전에도 depth를 계속 얻을 수 있다고 가정하지 않는다.** 관측 시점의 후보를 고정 좌표계에 저장하고 timestamp·유효기간을 전달한다. 물체 이동이나 장면 변경이 감지되면 재관측한다.

외부 파지 모듈은 후보의 유효기간과 IK·충돌 검사 결과를 확인한 뒤 집기·이동·놓기를 실행한다. 실행 결과와 그리퍼 feedback 또는 재관측 결과로 성공 여부를 확인한다. 후보 생성만으로 Pick & Place 성공을 판정하지 않으며, 놓을 위치와 실행 경로의 계획은 외부 모듈이 담당한다.

## 3. 추적·검증 및 RGB-D 관측 조건: 실측 후 확정

헤드 D435와 손목 D405는 같은 거리·해상도 설정을 공유한다고 가정하지 않는다. 각 장치가 지원하는 RGB/depth profile, 정합 결과와 목표 물체의 관측 품질을 기록한 뒤 운영 조건을 정한다. CAD/시뮬레이션의 카메라 형상은 실제 depth 성능의 근거로 사용하지 않는다.

| 설계 파라미터 | 의미 / 결정 방법 |
|---|---|
| `search_period` | 목표가 없을 때 전체 프레임 YOLO 실행 간격 |
| `verify_period` | 추적 중 YOLO 검증 간격. 목표 유지율과 연산량을 함께 비교 |
| `full_verify_period` | ROI 검증을 추가했을 때 전체 프레임 검증 간격. 첫 구현은 `verify_period`마다 전체 프레임 사용 |
| `max_verify_age` | 마지막 성공한 YOLO 검증의 허용 나이. 초과 시 추적 bbox와 접근용 위치를 무효화 |
| `roi_margin` | 목표 bbox 주변의 검증 검색 여유 영역. 화면 경계에서 잘라낸 좌표도 보존 |
| 대응·추적 이상 기준 | confidence, IoU, 중심 이동량, 크기 변화, 3D 위치 불일치 기준 |
| 위치 품질·유효기간 | 최소 LiDAR 점 수, 허용 공간 분산, RGB–LiDAR 시간차, 3D 위치 관측 나이 |
| 손목 관측 조건 | 실제 거리, 정지 안정성, 유효 depth 비율과 점군 품질 |

모든 파라미터는 **TBD**이며 설정 파일로 관리한다. `max_verify_age`는 정상 검증 주기와 추론 지연을 고려해 정한다. 한 번의 미검출에도 첫 구현은 접근을 보류하고 재획득한다. 가림 중 추적 유지 등 완화 정책은 실패 사례를 수집한 뒤 별도로 평가한다.

| 실측 항목 | 방법 / 기록 |
|---|---|
| 관측 거리 | 센서별 지원 범위에서 같은 물체를 거리별 촬영하고 **실제 카메라 광학 원점 기준 거리**를 기록 |
| 대상 물체 | 대회 물체 중 무광·광택·어두운 물체와 작은 물체를 포함. 투명 물체는 별도 실패 조건으로 평가 |
| 유효 depth 비율 | 목표 마스크 내부에서 원래 유효하게 측정된 픽셀 비율. Hole filling·과거값 유지로 비율을 부풀리지 않음 |
| 노이즈·편향 | 정지 장면의 반복 측정 흔들림과 알려진 거리/형상에 대한 오차를 **따로** 기록 |
| 점군 품질 | 목표 점 수, 배경 혼입, 표면 구멍, 경계 이상점, 자세별 정합 오차 |
| 추적·복구 | 차체 이동, 가림, bbox 크기 변화, 같은 클래스 물체의 교차에서 추적 유지율·ID 전환·재획득 시간 기록 |
| 검증 정책 | 매 프레임 YOLO와 추적+주기 검증을 같은 영상에서 비교. YOLO 호출 횟수, 검증 지연, 드리프트 발생과 복구 시간 기록 |
| 분할·목표 연결 | 헤드/손목 YOLO bbox 품질, 손목 SAM 마스크 경계, 같은 클래스 물체 간 오인, 헤드→손목 대응 성공률 |
| 전체 지연 | 촬영 timestamp부터 추적 bbox·검증·3D 위치·손목 마스크·점군·파지 후보가 준비될 때까지 단계별 지연과 상위 지연(p95) |
| 동시 처리 부하 | 실제 주행 처리와 센서 수신을 켠 상태의 CPU/GPU·RAM·온도·프레임 누락·주행 주기 지연 |
| 설정 비교 | raw vs spatial/temporal filter, depth preset, 해상도, 노출·IR projector 설정을 동일 장면에서 비교 |

RealSense Viewer와 `rosbag2`로 원본 영상·depth·`CameraInfo`·TF·관절 상태를 저장한다. 추적 bbox, YOLO 검출·검증 결과, 상태 전환과 실패 사유를 함께 기록한다. LiDAR 시험은 원본 점군과 odometry, 선택된 군집을 함께 기록한다. 거리·조명·재질·카메라 설정·필터 상태, Jetson 메모리 용량·전력 모드·모델 실행 형식을 실험 로그에 남긴다.

**관측 거리, confidence, 최소 점 수, 허용 분산, 동기화 오차, 관측 유효기간은 모두 TBD이다.** 실측으로 정한 기준을 통과한 경우에만 다음 단계에 결과를 전달한다. 반복 측정이 안정적이거나 EKF 공분산이 작아도 체계적인 보정 오차가 작다는 뜻은 아니다.

## 4. ROS 2 데이터 계약

현재 저장소의 **ROS 2 Humble**을 기준으로 설계한다. 아래 이름은 제안이며 실제 센서 namespace와 remap한다.

| 제안 인터페이스 | 데이터 | 필수 조건 |
|---|---|---|
| 헤드/손목 RGB 입력 | `sensor_msgs/Image` | 센서 timestamp, optical frame, 영상 크기 |
| 손목 정합 depth 입력 | `sensor_msgs/Image` + `CameraInfo` | RGB 마스크와 동일한 투영 격자, depth 단위·유효값 정의 |
| LiDAR 입력 | `sensor_msgs/PointCloud2` | 실제 frame_id, timestamp. 이동 누적 시 점별 시간/운동 보정 |
| 2D LiDAR 주행 입력 | `sensor_msgs/LaserScan` | 실제 frame_id, timestamp. 외부 주행 모듈에서 사용 |
| `/detection/head/detections` | `vision_msgs/Detection2DArray` | YOLO 실행 프레임의 클래스·bbox·confidence·목표 대응 ID. 추적 결과와 구분 |
| `/detection/head/target_track` | 추적 결과 **메시지 정의 예정** | 원본 RGB 좌표 bbox, 목표 ID, 촬영 timestamp, 추적 상태, 마지막 YOLO confidence·검증 timestamp, 유효 여부 |
| `/detection/head/status` | 상태·실패 사유 **메시지 정의 예정** | `SEARCH` / `TRACK` / `VERIFY` / `LOST` / `WRIST_OBSERVE` / `GRASP_READY`, 상태 timestamp, 접근 보류 여부 |
| `/detection/target_position` | `geometry_msgs/PointStamped` + 품질 메타데이터 | 목표 ID, 위치 센서, 좌표계, 점 수·분산·관측 나이·최종 YOLO 검증 시각·유효 여부 |
| 손목 관측 요청 / 응답 | 외부 모듈과 **인터페이스 정의 예정** | 목표 ID·3D 위치·품질·관측 요구사항 전달, 접근 완료·정지 확인·재관측 응답 |
| `/detection/wrist/target_mask` | `sensor_msgs/Image` (`mono8`) | 손목 RGB의 해당 목표 마스크 |
| `/detection/wrist/object_points` / `scene_points` | `sensor_msgs/PointCloud2` | 목표 점군과 장면 점군 분리, frame·관측 ID 일치 |
| `/detection/grasp_candidates` | 파지 후보 배열 **메시지 정의 예정** | 후보 ID, pose, width, approach depth, score, 목표 ID, 관측 timestamp·좌표계 |

기본 헤드 경로에는 `/detection/head/target_mask`가 없다. tracker 점수·성공 여부와 YOLO confidence는 의미가 다르므로 하나의 confidence로 합치지 않는다. 추적 상태에서 클래스와 검출 confidence는 마지막 YOLO 검증값임을 명시한다.

`PoseArray`만으로는 그리퍼 폭과 score를 전달할 수 없으므로 파지 후보 메시지에 함께 담는다. bbox·마스크·점군·후보에는 해당 프레임/점군의 관측 ID를 연결하고, 서로 다른 헤드·손목 관측은 `target_id`로 연결한다. 실패는 `TARGET_NOT_FOUND`, `TRACK_LOST`, `VERIFY_MISMATCH`, `AMBIGUOUS_TARGET`, `INSUFFICIENT_LIDAR_POINTS`, `INVALID_DEPTH`, `STALE_OBSERVATION`, `NO_GRASP_CANDIDATE`처럼 원인을 구분해 반환한다. 무효화 이벤트는 외부 접근 모듈에 전달하며, 오래된 위치를 계속 접근 목표로 사용하지 않는다.

보정·동기화의 기본 조건은 다음과 같다.

- 헤드 RGB–LiDAR 외부 보정과 손목 **eye-in-hand calibration**을 각각 수행한다. CAD 배치값은 초기값이며 실측 보정값을 대체하지 않는다.
- RGB-depth는 RealSense의 공장 보정 TF와 해당 스트림 `CameraInfo`를 사용한다. 같은 TF를 URDF와 driver가 중복 발행하지 않는다.
- RGB·추적 bbox·depth·마스크와 LiDAR를 관측 시각 기준으로 연결하고 **추론 완료 시각 대신 촬영 시각**으로 TF를 조회한다. 서로 다른 장치의 clock offset도 확인한다. 검증 timestamp와 현재 추적 프레임 timestamp를 구분한다.
- 실측에서 정한 이동 안정성 기준을 통과한 뒤 정밀 관측한다. `message_filters`의 시간 허용 폭은 실측으로 정한다.

## 5. 참고 팀에서 반영한 구조

[Inha-United@home](https://github.com/inha-united-athome)의 공개 코드·설정을 참고했다. 아래 내용은 참고 구조이며 이 저장소에 해당 패키지를 설치·포팅했다는 의미는 아니다.

| 참고 저장소 | 반영할 부분 | 적용 시 확인 |
|---|---|---|
| `inha_perception` | YOLO 검출, box-prompt SAM 2.1 분할, 정합 depth의 점군 변환을 분리 | 실제 코드의 모델·토픽과 README 기본값에 차이가 있음. 경로·namespace를 parameter화하고 인스턴스별 마스크 출력을 추가 |
| `get_seg_3d_coord` | 관측 시각의 TF로 LiDAR/submap 투영 → 후보 점 선택 → PCL 군집화 → 위치 품질 기록 | 참고 구현의 마스크 선택을 기본 헤드의 bbox 선택·배경 제거로 수정해야 함. 군집 거리 `0.30 m`를 작은 파지 물체에 그대로 적용하지 않음 |
| `easy_handeye2` | ArUco 관측과 여러 팔 자세를 이용한 eye-in-hand 보정·검증 | 참고 팀의 frame과 보정값을 복사하지 않고 실제 Piper/손목 카메라에서 다시 측정 |
| `GraspGen_inha` | 필요할 때만 파지 추론을 실행하고, 점군을 기준 좌표계로 변환해 후보를 전달하는 구조 | 이번 기본 모델은 GraspNet. GraspGen은 향후 비교 후보이며 Piper 그리퍼에 대한 적합성 평가가 필요 |

기존 문서에 기록한 참고 팀의 하드웨어 구성은 **헤드 D435f + 팔 D405**이다. 우리 실기 운영 기록은 **헤드 D435 + 손목 D405**이며, 카메라 모델이 같더라도 장착·보정·관측 품질은 별도로 평가한다. `get_seg_3d_coord`와 파지 wrapper의 `inha_interfaces` 의존성도 이 저장소에 포함되어 있지 않다. OpenCV 추적과 YOLO 주기 검증 상태 머신은 이번 설계에서 추가하는 부분이다.

## 6. 현재 저장소와 구현 순서

2026-10-03의 최종 `main` 모델에는 Tracer·Piper·센서 랙과 손목 카메라가 기본 포함되어 있다.

| 입력 | 현재 상태 | detection 연결에 필요한 작업 |
|---|---|---|
| 헤드 RGB/depth·Mid-360S | Gazebo 센서 모델 존재, ROS 브리지 미포함 | Image·CameraInfo·PointCloud2 브리지 또는 실제 driver 연결 |
| 손목 RGB/depth·점군 | 최종 모델에 손목 카메라 기본 포함, 손목 URDF/광학 프레임과 `/wrist_camera/…` ROS 브리지 설정 존재 | Ubuntu 실구동·메시지 수신 검증, 실제 손목 D405 보정, RGB-depth 정합 |
| 검출·추적·분할·파지 후보 | YOLO/SAM GPU 샘플 추론 확인, 이 통합 파이프라인은 설계 단계 | 추적기·검증·복구 상태 머신, 모델 wrapper, 품질 판정, 데이터 계약 구현 |

[손목 카메라 가이드](../HW/URDF/wrist_camera_description/README.md)의 Gazebo 모델은 핀홀 근사이며 실제 depth 노이즈를 재현하지 않는다. `/wrist_camera/depth/image_raw`와 RGB 토픽이 존재한다는 것만으로 정합된 depth라는 뜻은 아니다. 기존 CAD/시뮬레이션의 D435f 표기와 실기 Head D435 / Wrist D405를 구분하고, 성능 판단은 실측 데이터로 수행한다. 헤드 센서의 상세 제약은 [센서 문서](../simulation/docs/SENSORS.md)를 참고한다.

구현은 다음 순서로 진행한다.

1. **센서 입력·보정:** 실제 driver 또는 헤드 센서 ROS 브리지 연결, 기존 손목 센서/TF·브리지 동작 확인, color-depth·LiDAR·hand–eye 검증.
2. **헤드 검출·추적:** 전체 프레임 YOLO11n → 목표 선택 → OpenCV 추적 → 주기·이상 징후 검증 → 실패 시 전체 프레임 재획득. 녹화 영상으로 상태 전환·ID 유지부터 확인.
3. **헤드 위치·접근 연결:** 초기 aligned depth 경로 확인 후 LiDAR 투영 → bbox 후보 점·군집 선택 → 3D 위치·품질·무효화 출력 → 외부 접근 모듈과 갱신·접근 보류·도달 응답 연결.
4. **센서·주기 실측:** D435/D405 거리·설정별 데이터를 수집하고 추적·검증 주기, 위치/점군 임계값과 손목 관측 조건 결정.
5. **손목 정밀 인식:** 헤드 목표 전달 → 손목 YOLO11n 재검출·목표 대응 → SAM 2.1 Tiny → 유효 목표/장면 점군 → 재관측 판정.
6. **파지 후보·실행 연결:** GraspNet 추론 → 그리퍼 형상·좌표 변환·장면 충돌 필터 → 후보 전달 → 외부 모듈의 IK·경로 검사·Pick & Place 결과 확인.
7. **통합 부하 검증:** 실제 주행 프로그램과 센서 입력을 함께 실행하고 전체 프레임 YOLO 기준선 대비 추적+주기 검증의 지연·목표 유지율·연산량 평가.

SAM 2는 현재 공식 요구사항이 Python ≥3.10, PyTorch ≥2.5.1이며 GraspNet Baseline은 PyTorch 1.6 기반 요구사항과 custom CUDA 연산자를 제시한다. 단일 Python 환경의 호환성을 가정하지 않고 **모델별 inference 프로세스/컨테이너를 분리**하는 구성을 검토한다. 실제 GPU·CUDA·ROS adapter의 호환성은 별도로 검증하고 사용한 commit과 환경 버전을 기록한다.

외부 접근 모듈은 목표 3D 위치와 품질을 받아 차체·팔을 관측 자세로 이동시킨다. 외부 파지 모듈은 후보를 받아 MoveIt 2 등으로 IK·전체 경로를 검사하고 Pick & Place를 실행한다. 주행 알고리즘, 배치 위치 계획, 그리퍼 제어는 이 문서의 구현 범위에 포함하지 않는다.

## 7. Jetson AGX Orin에서의 실행 정책

이 알고리즘은 YOLO 실행 횟수를 줄이고 정밀 추론을 손목 관측 단계에 모으는 설계이다. 성능 향상 폭은 추적 CPU 비용과 재검출 빈도에 따라 달라지므로, 전체 프레임 YOLO 기준선과 실제 통합 부하를 비교한다. 특정 FPS나 전체 실행 가능성을 설치·샘플 추론만으로 보장하지 않는다.

| 운영 단계 | 주요 처리 | 연산 배분 |
|---|---|---|
| 탐색 | 주행 센서·위치 추정·장애물 처리, 헤드 전체 프레임 YOLO | 탐색 YOLO 주기를 제한. 손목 SAM·GraspNet 대기 |
| 추적·근접 접근 | OpenCV 추적, 주기 YOLO 검증, 목표 주변 LiDAR 처리 | 전체 점군의 불필요한 복사·누적을 줄이고 공간 범위·점 수 제한. 손목 정밀 추론 대기 |
| 손목 관측 | 정지 확인, 손목 YOLO → SAM Tiny → RGB-D 점군 | 헤드 추론 빈도를 줄이거나 대기시키고 손목 처리를 순차 실행 |
| 파지 준비·실행 | GraspNet·충돌 필터, 외부 팔 모듈 검사·실행 | 요청 시 추론. 장면 변경이나 실패 시 재관측 |

- 센서 수신률, OpenCV 추적률, YOLO 검증률, 3D 위치 갱신률은 각각 설정한다. 카메라의 모든 프레임에 모든 모델을 실행하지 않는다.
- YOLO TensorRT FP16 등 최적화는 기본 PyTorch 경로의 정확도·입출력 계약과 지연을 확인한 뒤 비교한다. ROI를 작게 잘라도 모델 입력을 같은 크기로 resize하면 추론 비용이 비례해서 감소한다고 가정하지 않는다.
- 모델별 venv/프로세스/컨테이너는 의존성 분리 수단이다. 같은 Jetson의 GPU·메모리·CPU를 공유하므로 SAM과 GraspNet 요청이 무제한으로 겹치지 않게 관리한다.
- 손목 전체 depth 점군을 매 프레임 발행하기보다 필요한 관측에서 목표와 주변 작업 영역의 점군을 만든다. 주변 충돌 정보를 유지하면서 다운샘플링을 평가한다.
- 주행 모듈의 필요한 센서 입력·장애물 처리는 유지한다. `tegrastats`와 ROS timestamp/수신률로 CPU/GPU·RAM·온도·지연을 기록하고, 처리 적체 또는 관측 만료 시 접근을 보류한다.

## References

### 모델 · 센서 · 도구의 원본 GitHub

| 기술 | 원본 저장소 |
|---|---|
| YOLO11n / YOLO11-seg 비교 | [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) |
| SAM 2 / SAM 2.1 | [facebookresearch/sam2](https://github.com/facebookresearch/sam2) |
| GraspNet Baseline | [graspnet/graspnet-baseline](https://github.com/graspnet/graspnet-baseline) |
| GraspNet API | [graspnet/graspnetAPI](https://github.com/graspnet/graspnetAPI) |
| GraspGen — 향후 비교 후보 | [NVlabs/GraspGen](https://github.com/NVlabs/GraspGen) |
| RealSense SDK / Viewer / depth filters | [realsenseai/librealsense](https://github.com/realsenseai/librealsense) |
| RealSense ROS 2 driver | [realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros) |
| Livox ROS driver 2 / SDK 2 | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2), [Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) |
| PCL | [PointCloudLibrary/pcl](https://github.com/PointCloudLibrary/pcl) |
| Open3D | [isl-org/Open3D](https://github.com/isl-org/Open3D) |
| OpenCV — ROI 추적·투영·ArUco·hand–eye calibration | [opencv/opencv](https://github.com/opencv/opencv), [opencv/opencv_contrib](https://github.com/opencv/opencv_contrib) |
| easy_handeye2 원본 | [marcoesposito1988/easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2) |
| ROS 2 TF / 시간 동기화 | [ros2/geometry2](https://github.com/ros2/geometry2), [ros2/message_filters](https://github.com/ros2/message_filters) |
| ROS 메시지 / 기록 도구 | [ros2/common_interfaces](https://github.com/ros2/common_interfaces), [ros-perception/vision_msgs](https://github.com/ros-perception/vision_msgs), [ros2/rosbag2](https://github.com/ros2/rosbag2) |
| Gazebo–ROS 브리지 | [gazebosim/ros_gz](https://github.com/gazebosim/ros_gz) |
| MoveIt 2 — 외부 실행 모듈 | [moveit/moveit2](https://github.com/moveit/moveit2) |

### Inha-United@home 구현 참고

- [조직 및 하드웨어 구성](https://github.com/inha-united-athome)
- [inha_perception](https://github.com/inha-united-athome/inha_perception) — [YOLO 노드](https://github.com/inha-united-athome/inha_perception/blob/main/perception/perception/yolo.py), [SAM 2.1·점군 노드](https://github.com/inha-united-athome/inha_perception/blob/main/perception/perception/groundedsam2.py)
- [get_seg_3d_coord](https://github.com/inha-united-athome/get_seg_3d_coord/tree/v0.5) — [LiDAR 마스크 투영](https://github.com/inha-united-athome/get_seg_3d_coord/blob/v0.5/src/point_selector.cpp), [실측·EKF 기록 흐름](https://github.com/inha-united-athome/get_seg_3d_coord/blob/v0.5/README.md)
- [easy_handeye2 팀 구성](https://github.com/inha-united-athome/easy_handeye2)
- [GraspGen_inha ROS wrapper](https://github.com/inha-united-athome/GraspGen_inha/tree/v0.5/scripts_RBY1)

### 제조사 사양 · 공식 문서

- [D435 사양](https://www.realsenseai.com/products/stereo-depth-camera-d435/), [D435f 사양](https://www.realsenseai.com/products/d435f-3/)
- [D400 권장 depth preset·해상도](https://dev.realsenseai.com/docs/d400-series-visual-presets/), [depth 후처리](https://github.com/realsenseai/librealsense/blob/master/doc/post-processing-filters.md)
- [Mid-360S 사양·근접 검출 제한](https://www.livoxtech.com/mid-360s/specs)
- [YOLO11 지원 모델](https://docs.ultralytics.com/models/yolo11/), [SAM 2 설치 요구사항](https://github.com/facebookresearch/sam2/blob/main/INSTALL.md), [GraspNet RGB-D demo](https://github.com/graspnet/graspnet-baseline/blob/main/demo.py)
