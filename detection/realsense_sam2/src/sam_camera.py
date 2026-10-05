import cv2
import numpy as np
import pyrealsense2 as rs
import torch
import time
import random

from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator


# -----------------------------
# SAM 2.1 configuration
# -----------------------------
checkpoint = "third_party/sam2/checkpoints/sam2.1_hiera_small.pt"
model_cfg = "configs/sam2.1/sam2.1_hiera_s.yaml"

device = "cuda" if torch.cuda.is_available() else "cpu"

print("Loading SAM 2.1...")

model = build_sam2(
    model_cfg,
    checkpoint,
    device=device,
)

mask_generator = SAM2AutomaticMaskGenerator(
    model=model,
    points_per_side=12,
    points_per_batch=64,
    pred_iou_thresh=0.85,
    stability_score_thresh=0.95,
    box_nms_thresh=0.7,
    min_mask_region_area=200,
    use_m2m=False,
)

print("SAM 2.1 loaded.")
print("Device:", device)


# -----------------------------
# RealSense configuration
# -----------------------------
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30,
)

pipeline.start(config)

print("RealSense camera started.")
print("Continuous segmentation running.")
print("Press Q to quit.")


# -----------------------------
# Mask visualization
# -----------------------------
def draw_masks(image, masks):
    overlay = image.copy()

    masks = sorted(
        masks,
        key=lambda x: x["area"],
        reverse=True,
    )

    for mask_data in masks:
        mask = mask_data["segmentation"]

        color = np.array(
            [
                random.randint(0, 255),
                random.randint(0, 255),
                random.randint(0, 255),
            ],
            dtype=np.uint8,
        )

        overlay[mask] = (
            0.5 * overlay[mask]
            + 0.5 * color
        ).astype(np.uint8)

    return overlay


# -----------------------------
# Main loop
# -----------------------------
segmented_frame = None

try:
    while True:

        # Get newest camera frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(
            color_frame.get_data()
        )

        # Show raw camera continuously
        cv2.imshow(
            "RealSense RGB",
            frame
        )

        # Convert BGR -> RGB for SAM
        rgb = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        # Measure inference time
        torch.cuda.synchronize()
        start_time = time.time()

        with torch.inference_mode(), torch.autocast(
            device_type="cuda",
            dtype=torch.bfloat16,
        ):
            masks = mask_generator.generate(rgb)

        torch.cuda.synchronize()
        end_time = time.time()

        inference_time = end_time - start_time
        fps = 1.0 / inference_time

        print(
            f"Masks: {len(masks)} | "
            f"Time: {inference_time:.3f}s | "
            f"FPS: {fps:.2f}"
        )

        # Draw segmentation
        segmented_frame = draw_masks(
            frame,
            masks
        )

        # Add information text
        cv2.putText(
            segmented_frame,
            f"SAM FPS: {fps:.2f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            segmented_frame,
            f"Masks: {len(masks)}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

        cv2.imshow(
            "SAM 2.1 Segmentation",
            segmented_frame
        )

        # Press Q to quit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()