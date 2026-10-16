# Head 학습·평가

현재 기준은 **6클래스 native YOLO11n**, [데이터 20261013](../datasets/20261013/README.md), [학습 실험 20261013](experiments/20261013/TRAINING_RESULTS.md)이다. 데이터 2,155장, bbox 12,205개, 사용자 저장 315/검토 완료 표시 209장이다. 유효한 배경 이미지 1장은 빈 라벨로 유지한다.

## 재학습

설치된 CUDA/PyTorch·Ultralytics 환경에서 저장소 루트 기준:

```bash
python SW/detection/head/training/train_local.py --device 0
```

기본 COCO `yolo11n.pt`, imgsz 640, batch 4, 최대 100 epoch, patience 20, workers 2, seed 42다. pretrained 파일이 없으면 다운로드될 수 있다. 결과는 Git 제외된 `training/runs/`에 생기며 새 best를 검증한 뒤 배포 모델을 바꾼다. Jetson engine은 해당 장치에서 다시 생성한다.

기록된 학습은 RTX 5070 Laptop, 실제 32 epoch / best 12, mAP50 0.97474 / mAP50–95 0.70237이다. 부분 검토 라벨 기준 지표이며 현재 실물 Task 성공률과 다르다.

## 저장 영상 재현

`scripts/evaluate_video.py`는 **20261013 게시 영상의 재현용**이다. 기본 cup=0.70과 raw 다중 bbox를 쓰므로 현재 adapter의 cup/banana=0.50 + class별 top-1과 구분한다.

```bash
python SW/detection/head/scripts/evaluate_video.py \
  SW/detection/head/training/experiments/20261013/video_test/source.mp4 \
  --cup-conf 0.70 --device 0
```

OpenCV·Ultralytics·imageio-ffmpeg가 필요하다. 기존 출력 폴더를 덮어쓰지 않으므로 재실행에는 새 `--output` 경로를 지정한다. 해당 영상은 정답 bbox가 없어 테스트 mAP를 계산하지 않았다. 과거 원본 테스트 두 영상은 현재 학습 데이터에 들어가 독립 테스트가 아니다.

## 과거 기록

[7클래스 실험 기록](LEGACY_20261007.md), [20261007 지표](experiments/20261007/TRAINING_RESULTS.md), [클래스 정정 기록](experiments/20261012/CLASS_CORRECTION.md)은 역사적 결과다. 중복 이전 데이터·Colab notebooks·완료된 일회성 migration·7클래스 last 가중치는 활성 트리에서 정리했다. [복구 기준](../../../REPOSITORY_MAINTENANCE.md)을 따른다. 현재 파일을 과거 설정으로 되돌리거나 새 학습을 했다고 표시하지 않는다.
