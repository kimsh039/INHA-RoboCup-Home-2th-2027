import cv2
import numpy as np
import pyrealsense2 as rs
import torch
import time
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

MIN_DEPTH = 0.25
MAX_DEPTH = 4.0
MIN_VALID_RATIO = 0.30


# ============================================================
# Shared data
# ============================================================

latest_frame = None
latest_depth = None
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

print("SAM 2.1 loaded.")


# ============================================================
# Camera thread
# ============================================================

def camera_thread():
    global latest_frame
    global latest_depth
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

    config.enable_stream(
        rs.stream.depth,
        CAMERA_WIDTH,
        CAMERA_HEIGHT,
        rs.format.z16,
        CAMERA_FPS,
    )

    profile = pipeline.start(config)

    # Convert RealSense raw depth values into meters
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Align depth pixels with RGB pixels
    align = rs.align(rs.stream.color)

    # Depth filtering
    spatial_filter = rs.spatial_filter()
    temporal_filter = rs.temporal_filter()
    hole_filter = rs.hole_filling_filter()

    print("RealSense RGB + Depth started.")
    print("Depth scale:", depth_scale)

    try:
        while running:

            frames = pipeline.wait_for_frames()

            # Make depth coordinates correspond to RGB coordinates
            aligned_frames = align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Filter depth
            filtered_depth = spatial_filter.process(depth_frame)
            filtered_depth = temporal_filter.process(filtered_depth)
            filtered_depth = hole_filter.process(filtered_depth)

            color_image = np.asanyarray(
                color_frame.get_data()
            )

            depth_raw = np.asanyarray(
                filtered_depth.get_data()
            )

            # Convert raw depth to meters
            depth_meters = (
                depth_raw.astype(np.float32)
                * depth_scale
            )

            # RGB and depth are updated together
            with frame_lock:
                latest_frame = color_image.copy()
                latest_depth = depth_meters.copy()

    finally:
        pipeline.stop()
        print("Camera stopped.")


# ============================================================
# Calculate robust depth inside one SAM mask
# ============================================================

def get_mask_depth(mask, depth_image):

    depth_values = depth_image[mask]

    total_pixels = depth_values.size

    if total_pixels == 0:
        return None, 0.0

    valid = (
        (depth_values > MIN_DEPTH)
        & (depth_values < MAX_DEPTH)
    )

    valid_values = depth_values[valid]

    valid_ratio = (
        len(valid_values) / total_pixels
    )

    # Not enough reliable depth data
    if (
        len(valid_values) < 50
        or valid_ratio < MIN_VALID_RATIO
    ):
        return None, valid_ratio

    # Median is more robust than mean
    median_depth = float(
        np.median(valid_values)
    )

    return median_depth, valid_ratio


# ============================================================
# Draw masks + distance
# ============================================================

def draw_masks_with_depth(
    image,
    masks,
    depth_image,
):

    overlay = image.copy()

    masks = sorted(
        masks,
        key=lambda x: x["area"],
        reverse=True,
    )

    colors = [
        (255, 80, 80),
        (80, 255, 80),
        (80, 80, 255),
        (255, 255, 80),
        (255, 80, 255),
        (80, 255, 255),
    ]

    for index, mask_data in enumerate(masks):

        mask = mask_data["segmentation"]

        color = np.array(
            colors[index % len(colors)],
            dtype=np.uint8,
        )

        overlay[mask] = (
            0.55 * overlay[mask]
            + 0.45 * color
        ).astype(np.uint8)

        distance, valid_ratio = get_mask_depth(
            mask,
            depth_image,
        )

        # SAM bounding box:
        # [x, y, width, height]
        x, y, w, h = mask_data["bbox"]

        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)

        if distance is not None:

            text = (
                f"{distance:.2f}m "
                f"{valid_ratio * 100:.0f}%"
            )

        else:

            text = "depth unreliable"

        cv2.putText(
            overlay,
            text,
            (x, max(y - 5, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
        )

    return overlay


# ============================================================
# SAM thread
# ============================================================

def segmentation_thread():
    global latest_segmented_frame
    global running

    first_run = True

    while running:

        with frame_lock:

            if (
                latest_frame is None
                or latest_depth is None
            ):
                continue

            frame = latest_frame.copy()
            depth = latest_depth.copy()

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

        inference_time = (
            time.time() - start_time
        )

        fps = 1.0 / inference_time

        segmented = draw_masks_with_depth(
            frame,
            masks,
            depth,
        )

        cv2.putText(
            segmented,
            f"SAM FPS: {fps:.2f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            segmented,
            f"Masks: {len(masks)}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        with segmentation_lock:
            latest_segmented_frame = segmented

        prefix = "WARMUP | " if first_run else ""

        print(
            f"{prefix}"
            f"Masks: {len(masks)} | "
            f"Time: {inference_time:.3f}s | "
            f"FPS: {fps:.2f}"
        )

        first_run = False


# ============================================================
# Start threads
# ============================================================

camera_worker = threading.Thread(
    target=camera_thread
)

sam_worker = threading.Thread(
    target=segmentation_thread
)

camera_worker.start()
sam_worker.start()


# ============================================================
# Display
# ============================================================

print("System running.")
print("Press Q to quit.")

try:

    while running:

        with frame_lock:

            if latest_frame is not None:

                cv2.imshow(
                    "RealSense RGB",
                    latest_frame,
                )

        with segmentation_lock:

            if latest_segmented_frame is not None:

                cv2.imshow(
                    "SAM 2.1 + Depth",
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