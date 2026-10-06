import cv2
import numpy as np
import pyrealsense2 as rs
import torch
import time
import random
import threading

from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator


# ============================================================
# Configuration
# ============================================================

CHECKPOINT = "third_party/sam2/checkpoints/sam2.1_hiera_small.pt"
MODEL_CFG = "configs/sam2.1/sam2.1_hiera_s.yaml"

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30


# ============================================================
# Shared data
# ============================================================

latest_frame = None
latest_segmented_frame = None

frame_lock = threading.Lock()
segmentation_lock = threading.Lock()

running = True


# ============================================================
# Load SAM 2.1
# ============================================================

device = "cuda" if torch.cuda.is_available() else "cpu"

print("Loading SAM 2.1...")
print("Device:", device)

model = build_sam2(
    MODEL_CFG,
    CHECKPOINT,
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

print("SAM 2.1 loaded successfully.")


# ============================================================
# Mask visualization
# ============================================================

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


# ============================================================
# Camera thread
# ============================================================

def camera_thread():
    global latest_frame
    global running

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(
        rs.stream.color,
        CAMERA_WIDTH,
        CAMERA_HEIGHT,
        rs.format.bgr8,
        CAMERA_FPS,
    )

    pipeline.start(config)

    print("RealSense camera started.")

    try:
        while running:

            frames = pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            frame = np.asanyarray(
                color_frame.get_data()
            )

            with frame_lock:
                latest_frame = frame.copy()

    finally:
        pipeline.stop()
        print("Camera stopped.")


# ============================================================
# SAM thread
# ============================================================

def segmentation_thread():
    global latest_segmented_frame
    global running

    # Warm-up status
    first_run = True

    while running:

        with frame_lock:

            if latest_frame is None:
                continue

            # Always copy the newest frame
            frame = latest_frame.copy()

        rgb = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB,
        )

        torch.cuda.synchronize()
        start_time = time.time()

        with torch.inference_mode(), torch.autocast(
            device_type="cuda",
            dtype=torch.bfloat16,
        ):

            masks = mask_generator.generate(rgb)

        torch.cuda.synchronize()

        inference_time = time.time() - start_time

        fps = 1.0 / inference_time

        segmented = draw_masks(
            frame,
            masks,
        )

        cv2.putText(
            segmented,
            f"SAM FPS: {fps:.2f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            segmented,
            f"Masks: {len(masks)}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

        with segmentation_lock:
            latest_segmented_frame = segmented

        if first_run:

            print(
                f"WARMUP | "
                f"Masks: {len(masks)} | "
                f"Time: {inference_time:.3f}s | "
                f"FPS: {fps:.2f}"
            )

            first_run = False

        else:

            print(
                f"Masks: {len(masks)} | "
                f"Time: {inference_time:.3f}s | "
                f"FPS: {fps:.2f}"
            )


# ============================================================
# Start threads
# ============================================================

camera_worker = threading.Thread(
    target=camera_thread,
)

sam_worker = threading.Thread(
    target=segmentation_thread,
)

camera_worker.start()
sam_worker.start()


# ============================================================
# Display loop
# ============================================================

print("System running.")
print("Press Q to quit.")

try:

    while running:

        with frame_lock:

            if latest_frame is not None:

                cv2.imshow(
                    "RealSense RGB - 30 FPS",
                    latest_frame,
                )

        with segmentation_lock:

            if latest_segmented_frame is not None:

                cv2.imshow(
                    "SAM 2.1 Segmentation",
                    latest_segmented_frame,
                )

        if cv2.waitKey(1) & 0xFF == ord("q"):

            running = False
            break

finally:

    running = False

    camera_worker.join()
    sam_worker.join()

    cv2.destroyAllWindows()

    print("Program finished.")