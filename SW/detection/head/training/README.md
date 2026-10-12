# Head YOLO11n · 현재 6클래스

**2026-10-13 재학습 완료.** [새 모델 결과](experiments/20261013/TRAINING_RESULTS.md), [실제 학습 데이터 2,155장](../datasets/20261013/README.md), [우분투 인수인계](../UBUNTU_HANDOFF_20261013.md)를 사용한다. 현재 가중치는 `../models/head_yolo11n_20261013.pt`이며 cup 신뢰도 0.70 / 나머지 0.25다. 아래 20261012 데이터·Colab 설명은 이전 준비 기록이다.

2026-10-12 정정한 검출 대상은 **green_apple, peach, tomato, fanta_can, cup, banana**다. 이전 `apple` 객체는 토마토였으며, `mug`는 `cup`으로 이름을 통일하고 접시는 제외한다.

| ID | 클래스 | 대상 |
|---:|---|---|
| 0 | green_apple | 청사과 |
| 1 | peach | 복숭아 |
| 2 | tomato | 토마토 |
| 3 | fanta_can | 환타 캔 |
| 4 | cup | 손잡이 포함 **빨간 머그컵만** |
| 5 | banana | 바나나 송이 |

[현재 데이터셋](../datasets/20261012/README.md)은 train 1,087장 / val 175장, bbox 7,202개다. 기존 접시 bbox 1,184개를 제외하고 좌표와 영상 단위 분할을 유지했다. 환타 캔은 로고가 보이지 않는 방향도 포함하고, 식별 가능한 모든 대상에 라벨을 달며 박스 겹침을 허용한다. 라벨 초안은 사용자 검토가 필요하다. 두 원본 테스트 영상은 학습 ZIP에 포함하지 않는다.

**20261012 준비 당시에는 학습 전이었으며 현재는 새 6클래스 학습이 완료됐다.** [상태](run_status.json)에 이전 7클래스 학습과 현재 재학습 준비 상태를 구분해 기록한다. 기존 [가중치](../models/head_yolo11n_20261007.pt)는 수정하지 않았다. Head `YoloDetector`와 `evaluate_holdout.py`가 이 모델의 정확한 7클래스 스키마를 인식하면 이름·ID를 현재 6클래스로 변환하고 접시 결과를 제외한다. 일반 COCO 모델의 `apple`은 그대로 유지한다. Ultralytics `YOLO(...).predict()`를 직접 호출하는 다른 코드는 자동 변환되지 않는다.

기존 모델로 두 보관 영상 **739프레임**을 다시 실행해 변환된 출력과 주석 영상을 확인했다. [현재 이름을 적용한 결과](experiments/20261012/holdout/TEST_RESULTS.md)는 기존 모델의 추론이며 새 학습 성능이 아니다. 영상 bbox 정답이 없어서 테스트 precision·recall·mAP와 테이블 선택 정확도를 계산하지 않는다.

## Colab 재학습

1. [현재 Colab 노트북](head_camera_yolo11n_colab.ipynb)을 업로드하고 GPU 런타임을 선택한다.
2. `head_camera_yolo11_dataset_20261012.zip`을 `/content`에 업로드한다. 로컬 생성 파일은 `RoboCup/outputs/head_camera_dataset_20261012/`에 있다. ZIP은 약 148 MB이며 Git에 별도로 중복 저장하지 않는다.
3. 셀을 순서대로 실행한다. SHA256과 6클래스 ID, train/val 이미지·라벨 대응을 검사한다.
4. YOLO11n (`yolo11n.pt`), detect, 100 epochs, imgsz 960, batch 8, seed 42로 **새 학습**을 시작한다. 원래 7클래스 run을 resume하지 않는다.
5. 완료 시 `head_camera_yolo11n_results_20261012.zip`에 best.pt / last.pt와 실제 로그·그래프를 내려받는다.

[패키지 명세](../datasets/20261012/colab_package_manifest.json)의 SHA256은 `ebfeadb315e613cf2376d20243e9f78b41a55f35797065ebfded55afe4a8bbcb`다. 데이터셋·ZIP 생성 명령은 [데이터셋 안내](../datasets/20261012/README.md)에 있다.

## 평가와 두 테이블 연결

저장소 루트에서 실행한다. 기본 출력은 현재 6클래스다. 기존 7클래스 실험을 재현할 때만 `--schema legacy`를 추가한다.

```bash
python SW/detection/head/training/evaluate_holdout.py \
  --model /absolute/path/best.pt \
  --videos /absolute/path/WIN_20261006_23_49_36_Pro.mp4 /absolute/path/WIN_20261006_23_52_09_Pro.mp4 \
  --output /absolute/path/holdout_results --device 0
```

기본값은 imgsz 960, conf 0.25, IoU 0.7이며 전체 프레임을 처리한다. 실제 실행 장치와 속도를 함께 기록하며 로컬 GPU FPS를 Jetson 실기 성능으로 해석하지 않는다.

[배치 초안](../datasets/20261012/video_table_inventory.json)과 객체별 테이블 대응에도 새 클래스명을 적용했다. 물리 테이블 ID는 최초 시야에서 이름 붙인 `table_left` / `table_right`로 유지한다. YOLO bbox를 작업면·깊이·지도 좌표에 연결하고 반복 관측은 추적 이력으로 관리한다. confidence를 테이블 존재 확률로 직접 사용하지 않는다.

## 이전 실험 기록

[20261007 학습 안내](LEGACY_20261007.md) · [원래 Colab 노트북](head_camera_yolo11n_colab_20261007.ipynb) · [원래 7클래스 데이터](../datasets/20261007/README.md) · [학습 결과](experiments/20261007/TRAINING_RESULTS.md) · [당시 영상 평가](experiments/20261007/holdout/TEST_RESULTS.md). 당시 클래스명과 검증 수치는 재현을 위해 보존하며 현재 6클래스 재학습 지표로 사용하지 않는다.
