import cv2
import numpy as np
import pyrealsense2 as rs

# ==========================================
# STEP 1: Configure RealSense
# ==========================================

pipeline = rs.pipeline()
config = rs.config()

# Select our physical D435if
config.enable_device("243222070076")

# Enable RGB streaming
config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)

# ==========================================
# STEP 2: Start the camera
# ==========================================

try:
    profile = pipeline.start(config)

    print("D435if RGB camera started!")
    print("Press Q to quit.")

    # ======================================
    # STEP 3: Capture live images
    # ======================================

    while True:
        frames = pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert RealSense frame to NumPy
        color_image = np.asanyarray(
            color_frame.get_data()
        )

        # Display RGB image
        cv2.imshow(
            "PiPER Wrist Camera - D435if",
            color_image
        )

        # Press Q to exit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except RuntimeError as e:
    print("RealSense error:", e)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

    print("Camera stopped safely.")