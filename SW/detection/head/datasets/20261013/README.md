# 20261013 실제 재학습 데이터 스냅샷

RTX 5070 Laptop에서 완료한 6클래스 학습에 사용한 이미지와 **사용자가 수정한 실제 YOLO 라벨**이다. 다운로드한 `head_camera_six_classes_review_snapshot.zip`과 학습 폴더의 라벨을 확인한 뒤 게시했다. 이 폴더가 이번 학습 데이터의 기준이다.

| 항목 | 수량 |
|---|---:|
| 전체 이미지 | 2,155 |
| train / val | 1,786 / 369 |
| 현재 bbox | 12,205 |
| 수정 저장 기록 / 검토 완료 표시 | 315 / 209 |

기존 1,262장에 추가 893장을 병합했다. 21개 원본 영상에서 새 프레임을 약 0.5초마다 추출하고, 원본 영상·프레임 번호가 같은 86장을 중복 제외했다. train/val은 영상 단위로 분리했다. `frames.csv`, `source_inventory.json`, `split_manifest.json`에 출처와 분할을 보관한다. 원본 21개 영상과 대형 ZIP은 이 폴더에 중복 저장하지 않는다. **이미지와 라벨은 전부 Git에 보관한다.**

| ID | 클래스 | 라벨 기준 |
|---:|---|---|
| 0 | green_apple | 청사과 |
| 1 | peach | 복숭아 |
| 2 | tomato | 토마토; 이전 apple 정정 |
| 3 | fanta_can | 환타 캔; 로고가 안 보이는 방향 포함 |
| 4 | cup | **빨간 머그컵만**, 손잡이 포함; 종이컵·테이크아웃 컵 제외 |
| 5 | banana | 바나나 송이 단위; 낱개 소품만 보이는 영상은 그 객체 |

접시는 제외한다. 식별 가능한 모든 대상에 라벨을 달며 서로 겹치는 박스를 허용한다. `review_state.json`은 저장 당시 사용자 수정·검토 기록이다. 검토 완료 표시가 없는 프레임도 학습에 포함됐으므로 전체를 수동 검증한 데이터로 해석하지 않는다. `dataset_summary.json`은 수정 후 라벨로 다시 집계했고 `file_hashes.json`에 실제 이미지·라벨 SHA256을 기록했다.

기존에 따로 보관했던 두 테스트 영상도 사용자의 “Camera Roll 전부” 지시에 따라 이번 데이터에 포함됐다. 그 두 영상은 이 모델의 독립 테스트로 사용할 수 없다. 추가 KakaoTalk 영상은 [별도 추론 결과](../../training/experiments/20261013/VIDEO_TEST.md)에 기록한다.

우분투 재학습: 저장소 루트에서 `python SW/detection/head/training/train_local.py`. 최초 COCO pretrained 모델은 다운로드될 수 있다. [학습 결과](../../training/experiments/20261013/TRAINING_RESULTS.md) · [우분투 인수인계](../../UBUNTU_HANDOFF_20261013.md).
