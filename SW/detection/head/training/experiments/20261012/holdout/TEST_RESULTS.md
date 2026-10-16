> Current class correction replay; not a newly trained model. [Changes and visual findings](../CLASS_CORRECTION.md). Videos are H.264 delivery copies; timing excludes that subsequent conversion.

# Two held-out video inference

Model SHA256: `c5691631b1d8516d5c62e8cd93f35928f7b5e70e7dc8cce6bebc1d03706fc2c9`

Evaluation device: **NVIDIA GeForce RTX 5070 Laptop GPU**; imgsz=960, conf=0.25, IoU=0.7, all frames.

Output schema: current; legacy checkpoint adapter applied: True. This adapter does not retrain the model.

These videos have no independent bounding-box ground truth. Precision, recall, mAP and table-selection accuracy were not computed. Frame occurrence is observation coverage, not detection recall.

| Video | Frames | Frames with detections | Mean inference ms | End-to-end FPS |
|---|---:|---:|---:|---:|
| WIN_20261006_23_49_36_Pro.mp4 | 308 | 308 | 19.39 | 20.16 |
| WIN_20261006_23_52_09_Pro.mp4 | 431 | 431 | 19.02 | 21.56 |

## WIN_20261006_23_49_36_Pro.mp4

| Class | Frames | Frame fraction | Boxes | Mean confidence |
|---|---:|---:|---:|---:|
| green_apple | 308 | 1.000 | 308 | 0.938 |
| peach | 301 | 0.977 | 301 | 0.662 |
| tomato | 308 | 1.000 | 313 | 0.932 |
| fanta_can | 308 | 1.000 | 308 | 0.955 |
| cup | 159 | 0.516 | 160 | 0.625 |
| banana | 266 | 0.864 | 283 | 0.877 |

[Annotated video](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/annotated.mp4) · [Raw detections](WIN_20261006_23_49_36_Pro/detections.csv)

![frame_000000.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000000.jpg)
![frame_000027.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000027.jpg)
![frame_000055.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000055.jpg)
![frame_000083.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000083.jpg)
![frame_000111.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000111.jpg)
![frame_000139.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000139.jpg)
![frame_000167.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000167.jpg)
![frame_000195.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000195.jpg)
![frame_000223.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000223.jpg)
![frame_000251.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000251.jpg)
![frame_000279.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000279.jpg)
![frame_000307.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_49_36_Pro/frame_000307.jpg)

## WIN_20261006_23_52_09_Pro.mp4

| Class | Frames | Frame fraction | Boxes | Mean confidence |
|---|---:|---:|---:|---:|
| green_apple | 118 | 0.274 | 150 | 0.438 |
| peach | 161 | 0.374 | 161 | 0.391 |
| tomato | 13 | 0.030 | 13 | 0.388 |
| fanta_can | 431 | 1.000 | 506 | 0.789 |
| cup | 0 | 0.000 | 0 | - |
| banana | 431 | 1.000 | 454 | 0.633 |

[Annotated video](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/annotated.mp4) · [Raw detections](WIN_20261006_23_52_09_Pro/detections.csv)

![frame_000000.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000000.jpg)
![frame_000039.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000039.jpg)
![frame_000078.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000078.jpg)
![frame_000117.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000117.jpg)
![frame_000156.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000156.jpg)
![frame_000195.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000195.jpg)
![frame_000234.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000234.jpg)
![frame_000273.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000273.jpg)
![frame_000312.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000312.jpg)
![frame_000351.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000351.jpg)
![frame_000390.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000390.jpg)
![frame_000430.jpg](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/head/training/experiments/20261012/holdout/WIN_20261006_23_52_09_Pro/frame_000430.jpg)
