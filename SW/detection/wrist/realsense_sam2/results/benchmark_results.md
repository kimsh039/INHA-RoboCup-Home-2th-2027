# SAM 2.1 Benchmark Results

## Hardware

- Camera: Intel RealSense D435 / D435if
- GPU: NVIDIA GeForce RTX 5060 Laptop GPU
- OS: Ubuntu Linux
- Python: 3.10
- Resolution: 640x480
- Camera FPS: 30

## Final Configuration

```yaml
model: sam2.1_hiera_small
points_per_side: 12
points_per_batch: 64
pred_iou_thresh: 0.85
stability_score_thresh: 0.95
box_nms_thresh: 0.7
min_mask_region_area: 200
use_m2m: false
precision: bfloat16