# Head YOLO11n 학습·평가 결과 · 2026-10-07

Colab Tesla T4에서 YOLO11n의 7개 클래스 fine-tuning을 100 epoch 완료했다. 최적 가중치는 [head_yolo11n_20261007.pt](../../../models/head_yolo11n_20261007.pt)이며, 독립 보관 영상 두 개의 총 **739프레임**에 실제 추론을 실행했다. 현재 용도는 실험용 검출 초안이다.

| 항목 | 실제 결과 |
|---|---|
| 학습 | imgsz 960, batch 8, seed 42, AMP, workers 2, AdamW(auto lr 0.000909) |
| 환경 | Colab Tesla T4 14.6 GiB, Python 3.13.15, torch 2.11.0+cu130, Ultralytics 8.4.174 |
| 시간 | results.csv 마지막 누적 시간 3686.86초 / 로그 1.024시간 |
| 데이터 | train 1087 / val 175, 검증 1137 boxes; optical flow 전달 초안 |
| 선택 | epoch **6**, 최고 mAP50–95 기준; checkpoint train_metrics와 CSV 일치 |
| best 검증 | Precision **92.608%**, Recall **96.958%**, mAP50 **97.739%**, mAP50–95 **67.885%** |
| epoch 100 | mAP50 96.403%, mAP50–95 61.895%; 최적 모델보다 낮음 |
| 모델 SHA256 | `c5691631b1d8516d5c62e8cd93f35928f7b5e70e7dc8cce6bebc1d03706fc2c9` |
| 모델 크기 | 5,496,410 bytes |

가중치 정리 과정에서 checkpoint epoch는 -1로 바뀌었다. 선택 epoch 6은 저장된 train_metrics와 [100 epoch CSV](results.csv)의 일치로 확인했다. [실제 인자](args.yaml) · [Colab 출력 로그](colab_training_log.txt) · [검증 지표 JSON](validation_metrics.json) · [학습 곡선](results.png) · [혼동 행렬](confusion_matrix.png). [last.pt](weights/last.pt)는 최종 epoch의 optimizer 제거 가중치이며 optimizer 상태까지 포함한 resume checkpoint는 아니다.

| 클래스 | 검증 boxes | Precision | Recall | mAP50 | mAP50–95 |
|---|---:|---:|---:|---:|---:|
| apple | 169 | 0.984 | 0.994 | 0.995 | 0.605 |
| banana | 175 | 0.518 | 1.000 | 0.877 | 0.513 |
| fanta_can | 149 | 0.989 | 1.000 | 0.995 | 0.827 |
| green_apple | 175 | 1.000 | 0.828 | 0.994 | 0.561 |
| mug | 148 | 1.000 | 0.983 | 0.990 | 0.746 |
| peach | 146 | 0.995 | 0.986 | 0.993 | 0.836 |
| plate | 175 | 0.996 | 1.000 | 0.995 | 0.666 |

클래스별 수치는 최적 모델의 마지막 Colab 검증 출력에서 반올림된 값을 옮겼다. 라벨이 사람이 전수 확정한 정답이 아니며 동일 공간·소품·촬영 세션의 영상이라 위 mAP를 새 환경 정확도나 테이블 선택 확률로 해석하지 않는다. banana Precision 0.518과 작은 엄격 IoU AP, 후반 검증 하락은 라벨 품질·박스 중복·배치 변화 취약성을 검토할 신호다. 원인은 이 실험만으로 확정할 수 없다.

## 보관 영상 실제 추론

[전체 결과·대표 이미지·주석 영상](holdout/TEST_RESULTS.md) · [전체 수치](holdout/test_summary.json). 모델·imgsz 960 / conf 0.25 / IoU 0.7은 두 영상에서 동일하며 vid_stride=1로 모든 프레임을 처리했다.

| 영상 | 프레임 | 평균 추론 ms | 출력 저장 포함 FPS | 관찰 |
|---|---:|---:|---:|---|
| 23_49_36 | 308 | 6.92 | 42.50 | 머그컵 간헐 누락, 후반 바나나 검출 끊김 |
| 23_52_09 | 431 | 6.86 | 46.76 | 머그컵 0프레임, 가린 과일 누락·중복 박스 |

평가 GPU는 **RTX 5070 Laptop GPU**, Python 3.12.14 / torch 2.11.0+cu128이다. FPS는 디코딩·시각화·파일 저장을 포함하며 T4 또는 Jetson 실기 속도가 아니다. 영상에 독립 bbox 정답이 없어 테스트 precision·recall·mAP와 테이블 선택 정확도를 계산하지 않았다.

## 사용

저장소 루트에서 `pip install ultralytics==8.4.174` 후:

```bash
yolo predict model=SW/detection/head/models/head_yolo11n_20261007.pt source=SW/detection/head/datasets/20261007/test_videos/WIN_20261006_23_49_36_Pro.mp4 imgsz=960 conf=0.25 iou=0.7
```

ROS/Docker 연결은 [Head 실행 문서](../../../README.md)의 모델 경로 설정을 따른다. 영상 추론과 ROS 실기 통합은 별도 검증이다. 테이블 클래스는 없고, 두 물리 테이블의 작업면/깊이/지도 좌표와 객체를 연결하는 후속 단계가 필요하다. 검출 confidence를 테이블 존재 확률로 바로 사용하지 않는다.
