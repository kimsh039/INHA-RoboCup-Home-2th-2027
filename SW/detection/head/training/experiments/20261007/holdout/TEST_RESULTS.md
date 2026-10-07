# Two held-out video inference

Model SHA256: `c5691631b1d8516d5c62e8cd93f35928f7b5e70e7dc8cce6bebc1d03706fc2c9`

Evaluation device: **NVIDIA GeForce RTX 5070 Laptop GPU**; imgsz=960, conf=0.25, IoU=0.7, all frames.

These videos have no independent bounding-box ground truth. Precision, recall, mAP and table-selection accuracy were not computed. Frame occurrence is observation coverage, not detection recall.

| Video | Frames | Frames with detections | Mean inference ms | End-to-end FPS |
|---|---:|---:|---:|---:|
| WIN_20261006_23_49_36_Pro.mp4 | 308 | 308 | 6.92 | 42.50 |
| WIN_20261006_23_52_09_Pro.mp4 | 431 | 431 | 6.86 | 46.76 |

## WIN_20261006_23_49_36_Pro.mp4

| Class | Frames | Frame fraction | Boxes | Mean confidence |
|---|---:|---:|---:|---:|
| apple | 308 | 1.000 | 313 | 0.932 |
| banana | 266 | 0.864 | 283 | 0.877 |
| fanta_can | 308 | 1.000 | 308 | 0.955 |
| green_apple | 308 | 1.000 | 308 | 0.938 |
| mug | 159 | 0.516 | 160 | 0.625 |
| peach | 301 | 0.977 | 301 | 0.662 |
| plate | 308 | 1.000 | 308 | 0.879 |

[Annotated video](WIN_20261006_23_49_36_Pro/annotated.mp4) · [Raw detections](WIN_20261006_23_49_36_Pro/detections.csv)

![frame_000000.jpg](WIN_20261006_23_49_36_Pro/frame_000000.jpg)
![frame_000027.jpg](WIN_20261006_23_49_36_Pro/frame_000027.jpg)
![frame_000055.jpg](WIN_20261006_23_49_36_Pro/frame_000055.jpg)
![frame_000083.jpg](WIN_20261006_23_49_36_Pro/frame_000083.jpg)
![frame_000111.jpg](WIN_20261006_23_49_36_Pro/frame_000111.jpg)
![frame_000139.jpg](WIN_20261006_23_49_36_Pro/frame_000139.jpg)
![frame_000167.jpg](WIN_20261006_23_49_36_Pro/frame_000167.jpg)
![frame_000195.jpg](WIN_20261006_23_49_36_Pro/frame_000195.jpg)
![frame_000223.jpg](WIN_20261006_23_49_36_Pro/frame_000223.jpg)
![frame_000251.jpg](WIN_20261006_23_49_36_Pro/frame_000251.jpg)
![frame_000279.jpg](WIN_20261006_23_49_36_Pro/frame_000279.jpg)
![frame_000307.jpg](WIN_20261006_23_49_36_Pro/frame_000307.jpg)

## WIN_20261006_23_52_09_Pro.mp4

| Class | Frames | Frame fraction | Boxes | Mean confidence |
|---|---:|---:|---:|---:|
| apple | 13 | 0.030 | 13 | 0.388 |
| banana | 431 | 1.000 | 454 | 0.633 |
| fanta_can | 431 | 1.000 | 506 | 0.789 |
| green_apple | 118 | 0.274 | 150 | 0.438 |
| mug | 0 | 0.000 | 0 | - |
| peach | 161 | 0.374 | 161 | 0.391 |
| plate | 431 | 1.000 | 573 | 0.680 |

[Annotated video](WIN_20261006_23_52_09_Pro/annotated.mp4) · [Raw detections](WIN_20261006_23_52_09_Pro/detections.csv)

![frame_000000.jpg](WIN_20261006_23_52_09_Pro/frame_000000.jpg)
![frame_000039.jpg](WIN_20261006_23_52_09_Pro/frame_000039.jpg)
![frame_000078.jpg](WIN_20261006_23_52_09_Pro/frame_000078.jpg)
![frame_000117.jpg](WIN_20261006_23_52_09_Pro/frame_000117.jpg)
![frame_000156.jpg](WIN_20261006_23_52_09_Pro/frame_000156.jpg)
![frame_000195.jpg](WIN_20261006_23_52_09_Pro/frame_000195.jpg)
![frame_000234.jpg](WIN_20261006_23_52_09_Pro/frame_000234.jpg)
![frame_000273.jpg](WIN_20261006_23_52_09_Pro/frame_000273.jpg)
![frame_000312.jpg](WIN_20261006_23_52_09_Pro/frame_000312.jpg)
![frame_000351.jpg](WIN_20261006_23_52_09_Pro/frame_000351.jpg)
![frame_000390.jpg](WIN_20261006_23_52_09_Pro/frame_000390.jpg)
![frame_000430.jpg](WIN_20261006_23_52_09_Pro/frame_000430.jpg)

## 대표 프레임 시각 검토 · 2026-10-07

두 영상에서 균등 간격으로 추출한 12장씩, 총 24장과 세 장의 작은 글씨 상세 이미지를 확인했다. 전체 프레임 정답을 만든 평가는 아니다.

- **23_49_36**: 두 테이블에 분산된 사과·초록 사과·접시·환타 캔은 각 308프레임에서 검출됐다. 머그컵은 159/308프레임에서만 검출됐고, frame 111에서는 보이는 붉은 머그컵의 박스가 빠져 있다. 바나나는 266프레임에서 검출됐으며 후반 화면 가장자리/가림 구간에서 끊긴다.
- **23_52_09**: 물체들이 같은 테이블에서 앞뒤로 겹친다. 바나나·환타 캔·접시는 431프레임 모두에서 검출됐지만 중복 박스도 있다. 머그컵은 0/431프레임, 사과는 13, 초록 사과는 118, 복숭아는 161프레임에서만 검출됐다. frame 0에서 접시 아래쪽에 보이는 머그컵 손잡이에 대응하는 mug 박스가 없고, frame 273에서는 같은 초록 사과에 겹친 박스가 보인다.
- 숫자는 **검출이 발생한 프레임 수**다. 물체의 실제 가시성을 프레임별로 판정한 recall·정확도가 아니며 반복 관측을 독립 증거로 세어서는 안 된다.

현재 모델은 실험용 초안이다. 머그컵과 가림/물체 배치 변화에 취약하므로 두 테이블 탐색을 위한 실기 확정 모델로 사용하기 전에 라벨 전수 검토, 다양한 거리·배치·가림 및 빈 테이블 데이터 보강, 재학습이 필요하다. 테스트 영상은 향후 학습 데이터로 편입하면 더 이상 독립 테스트로 사용할 수 없으므로 별도 신규 테스트를 확보해야 한다.

[첫 영상 상세](WIN_20261006_23_49_36_Pro_111_detail.jpg) · [둘째 영상 시작 상세](WIN_20261006_23_52_09_Pro_0_detail.jpg) · [둘째 영상 겹침 상세](WIN_20261006_23_52_09_Pro_273_detail.jpg)
