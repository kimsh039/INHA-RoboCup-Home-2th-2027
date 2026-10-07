# RealSense D435 + SAM 2.1 Segmentation

Real-time segmentation pipeline using:

- Intel RealSense D435 / D435if
- SAM 2.1
- PyTorch + CUDA
- RGB + aligned depth
- BF16 inference
- OpenCV visualization

The project was tested on Ubuntu with an NVIDIA RTX 5060 Laptop GPU.

## Final SAM 2.1 configuration

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