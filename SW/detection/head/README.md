# Head YOLO11n · 현재 실행 기준

현재 모델은 [head_yolo11n_20261013.pt](models/head_yolo11n_20261013.pt)이며 대상 Jetson에서 만든 FP16 engine을 우선 사용한다. [TensorRT·실행/표시 모드](TENSORRT_JETSON.md) · [화면/FPS 빠른 실행](LIVE_JETSON.md).

## 현재 실행 경로

- 관제: [start-head-detection.sh](../../control_center/start-head-detection.sh) → [head-detection-node.py](../../control_center/head-detection-node.py).
- 공통 모델 로드/후처리: `head_detection_ws/src/robocup_head_detection/robocup_head_detection/backends.py`.
- Head/LiDAR 대응·odom 목표·책상 관측: [head_lidar.py](../../control_center/head_lidar.py).
- 실제 접근/도킹: [control_center/DETECTION.md](../../control_center/DETECTION.md).
- 현재 Wrist와 팔 연결: [manipulation](../../manipulation/README.md). Head 모델이 팔을 직접 움직이지 않는다.

입력은 `/head/camera/color/image_raw`, 객체 JSON은 `/detection/head/objects`, debug Image는 `/detection/head/debug_image`다. 원본 RGB를 읽고 실측 RGB CameraInfo는 별도 토픽을 사용한다. 예전 calibrated RGB relay 토픽은 발행하지 않는다.

## 모델·후처리

| ID | 이름 | 최소 confidence |
|---:|---|---:|
| 0 | green_apple | 0.25 |
| 1 | peach | 0.25 |
| 2 | tomato | 0.25 |
| 3 | fanta_can | 0.25 |
| 4 | cup | 0.50 |
| 5 | banana | 0.50 |

cup은 손잡이 포함 빨간 머그컵. imgsz 640, NMS IoU 0.7이다. 공통 adapter는 클래스별 최고 confidence 하나만 출력하므로 최대 6개다. 같은 클래스 여러 인스턴스를 모두 세는 모델 출력 정책은 아니다.

`model_paths.py`는 `ROBOCUP_MODEL` → 현재 .engine 존재 여부 → 현재 .pt 순으로 선택한다. TensorRT는 CUDA 장치가 필요하다. engine은 Git에 넣지 않으며 .pt는 재생성 원본이다.

## 학습과 영상 평가

현재 [데이터 20261013](datasets/20261013/README.md)은 train 1,786 / val 369, 합계 2,155장이다. [실제 학습 결과](training/experiments/20261013/TRAINING_RESULTS.md)와 [당시 cup=0.70 영상 비교](training/experiments/20261013/VIDEO_TEST.md)는 당시 설정의 결과다. 현재 cup/banana=0.50 + top-1의 정확도 지표로 바꾸어 해석하지 않는다.

[학습·평가 실행기](training/README.md). 이전 두 데이터셋의 1,262장이 현재 데이터에 바이트 동일하게 포함되는 것을 원본 영상/프레임 키로 확인한 뒤 중복본을 정리했다. [복구와 정리 기준](../../REPOSITORY_MAINTENANCE.md).

## 실시간과 선택형 추적

관제 기본은 전체 화면 YOLO다. 최신 프레임만 처리하는 worker와 debug 기본 6Hz 제한을 사용하며, 카메라/디텍션/화면 FPS를 구분한다. 25FPS 이상 현장 보고는 통제된 전후 벤치마크가 아니다.

선택형 `live_ros_viewer.py --yolo-hz 3`는 YOLO 사이 KCF 추적을 비교하는 별도 화면이다. 기본 미리보기는 관제 주석 영상/FPS를 재사용해 YOLO 중복 실행을 피한다. ROS 패키지의 기존 CSRT/KCF·ROI 재검증 상태 머신과 [시뮬레이션 실행](SIM_NAV_TEST.md)은 보존한다. `docker/`는 해당 선택형 x86 실험 구성으로 Jetson 기본 배포 경로가 아니다.

## 코드 확인

```bash
PYTHONPATH=SW/detection/head/head_detection_ws/src/robocup_head_detection \
  python -m unittest discover -s SW/detection/head/tests -v
```

NumPy와 OpenCV contrib가 필요하다. ROS/메시지 없는 환경의 ROS 테스트 skip과 실제 센서/모델 정확도 검증을 구분한다. 초기 직접 /cmd_vel 추종 데모는 현재 closed approach와 중복되는 비운영 코드라 정리했다.
