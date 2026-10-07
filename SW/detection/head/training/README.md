# Head YOLO11n · 7개 클래스 학습

2026-10-07 촬영 영상으로 헤드 카메라 객체 검출 모델을 학습합니다. 학습은 **Colab Tesla T4**에서 시작했으며, 완료 후 최적 가중치와 검증 수치·두 보관 영상의 추론 결과를 이 폴더에 기록합니다. 현재 결과 상태는 [run_status.json](run_status.json)을 확인하세요.

| 항목 | 설정 |
|---|---|
| 모델 / task | Ultralytics YOLO11n (`yolo11n.pt`) / detect |
| 클래스 ID | 0 apple, 1 banana, 2 fanta_can, 3 green_apple, 4 mug, 5 peach, 6 plate |
| 학습 설정 | 100 epochs, imgsz 960, batch 8, seed 42, workers 2, AMP |
| Colab 환경 | Python 3.13.15, torch 2.11.0+cu130, Ultralytics 8.4.174, Tesla T4 14.6GiB |
| 초기 모델 | 181 layers, 2,591,205 parameters, 6.5 GFLOPs |
| 최적화 | optimizer=auto → AdamW, 초기 lr 0.000909 |
| 저장 | best.pt / last.pt, 10 epoch 간격 체크포인트 |

YOLO26n은 Ultralytics의 AMP 확인 과정에서만 다운로드됐으며 학습 모델은 YOLO11n입니다.

## 데이터 분리

원본 프레임 **0, 2, 4, …**를 1920×1080 JPEG로 추출했습니다. 연속 프레임 누출을 줄이기 위해 영상 전체 단위로 나눴습니다.

| 영상 | 용도 | 이미지 수 |
|---|---|---:|
| WIN_20261006_23_47_12_Pro.mp4 | train | 335 |
| WIN_20261006_23_48_36_Pro.mp4 | train | 370 |
| WIN_20261006_23_49_36_Pro.mp4 | 최종 테스트 영상 | 0 |
| WIN_20261006_23_50_08_Pro.mp4 | train | 173 |
| WIN_20261006_23_50_46_Pro.mp4 | val | 175 |
| WIN_20261006_23_51_34_Pro.mp4 | train | 209 |
| WIN_20261006_23_52_09_Pro.mp4 | 최종 테스트 영상 | 0 |

합계 **train 1,087장 / val 175장**, 8,386개 라벨 초안입니다. 테스트 영상은 학습 ZIP에 포함하지 않았습니다. [데이터셋](../datasets/20261007/README.md)에 이미지·라벨·두 원본 테스트 영상·설정·메타데이터를 Git으로 올렸습니다. 원본 학습 영상과 Colab 업로드 ZIP은 로컬에 보관합니다. ZIP의 SHA256과 구성은 [데이터 패키지 명세](data/colab_package_manifest.json)에 있습니다.

## 라벨 기준과 해석

- 바나나는 **송이 전체**를 하나의 객체로 라벨링합니다.
- 환타 캔은 로고가 보이지 않는 방향도 포함합니다.
- 머그컵은 보이는 손잡이를 포함합니다.
- 접시는 접시 자체의 외곽을 감쌉니다. 접시 위 과일은 별도 객체이며 박스 겹침을 허용합니다.
- 식별 가능한 대상은 모두 라벨링합니다. 테이블은 이번 7개 클래스에 포함하지 않습니다.

라벨은 기준 프레임에서 지정한 박스를 optical flow로 전달한 **검토용 초안**입니다. 사람이 모든 프레임을 확정한 정답은 아니므로 검증 mAP는 이 초안에 대한 수치입니다. 같은 공간·소품과 가까운 시점의 영상이어서 새 장소·새 개체의 일반화 성능과도 구분해야 합니다.

두 보관 영상에는 별도 bbox 정답이 없으므로 영상 추론 결과만으로 precision·recall·mAP를 만들지 않습니다. 테스트에서 프레임별 검출·confidence·처리 시간·대표 이미지·주석 영상을 저장하고, 관찰한 누락·오검출을 기록합니다. 정답 라벨을 추가하면 별도의 detection 지표를 계산할 수 있습니다.

## 두 테이블 선택과의 연결

목표는 지정받은 객체의 관측 증거를 두 물리 테이블에 연결해 탐색 우선순위를 정하는 것입니다. [배치 초안](data/video_table_inventory.json)의 `table_left` / `table_right`는 최초 방 시야에서 이름 붙인 물리 테이블이며, 매 프레임의 화면 좌우가 아닙니다.

YOLO는 클래스·bbox·confidence를 출력합니다. bbox를 작업면/깊이/지도 좌표에 연결하고, 같은 관측의 반복을 중복 증거로 세지 않도록 시간·추적 이력을 관리해야 합니다. confidence를 테이블 존재 확률로 직접 사용하지 않습니다. 이번 영상 추론 평가는 객체 검출을 확인하는 단계이며, 테이블 선택 정확도는 별도 대응 정답과 평가가 필요합니다.

## 재현

1. [Colab 노트북](head_camera_yolo11n_colab.ipynb)을 Colab에 업로드합니다.
2. 런타임을 T4 GPU로 선택합니다.
3. `head_camera_yolo11_dataset.zip`을 왼쪽 파일 패널에서 `/content`에 업로드하고 완료를 기다립니다.
4. 셀을 순서대로 실행합니다. ZIP SHA256, 클래스, 이미지·라벨 대응을 확인한 뒤 학습합니다.
5. 완료 시 가중치·로그·그래프 ZIP 다운로드가 시작됩니다.

학습한 모델로 로컬에서 두 영상을 평가하려면 저장소 루트에서:

```bash
python SW/detection/head/training/evaluate_holdout.py \
  --model /absolute/path/best.pt \
  --videos /absolute/path/WIN_20261006_23_49_36_Pro.mp4 /absolute/path/WIN_20261006_23_52_09_Pro.mp4 \
  --output /absolute/path/holdout_results --device 0
```

기본값은 imgsz 960, conf 0.25, IoU 0.7이며 전체 프레임을 처리합니다. 속도 수치는 평가를 실제 실행한 GPU와 함께 기록하며 Jetson 실기 FPS로 해석하지 않습니다.
