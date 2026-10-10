import cv2
import numpy as np
import pyrealsense2 as rs
import yaml

from pathlib import Path


# ==========================================
# STEP 1: Configuration
# ==========================================

CAMERA_SERIAL = "243222070076"

TAG_SIZE = 0.075
TAG_PITCH = 0.114

BOARD_ROWS = 6
BOARD_COLS = 6

ROOT = Path(__file__).resolve().parents[4]

CALIBRATION_FILE = (
    ROOT / "HW/calibration/camera_intrinsics/wrist/ost.yaml"
)


# ==========================================
# STEP 2: Load camera intrinsics
# ==========================================

with open(CALIBRATION_FILE, "r") as f:
    data = yaml.safe_load(f)

K = np.array(
    data["camera_matrix"]["data"],
    dtype=np.float64
).reshape(3, 3)

D = np.array(
    data["distortion_coefficients"]["data"],
    dtype=np.float64
)


# ==========================================
# STEP 3: Configure AprilTag detector
# ==========================================

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_APRILTAG_36h11
)

detector = cv2.aruco.ArucoDetector(
    dictionary,
    cv2.aruco.DetectorParameters()
)


# ==========================================
# STEP 4: Define board geometry
# ==========================================

def get_marker_object_points(marker_id):

    column = marker_id % BOARD_COLS
    row = marker_id // BOARD_COLS

    # Board origin: center of ID 0
    # X points right
    # Y points upward

    center_x = column * TAG_PITCH
    center_y = row * TAG_PITCH

    half = TAG_SIZE / 2.0

    # OpenCV marker corner order:
    # top-left, top-right,
    # bottom-right, bottom-left

    points = np.array([
        [center_x + half, center_y - half, 0],  # Corner 0
        [center_x - half, center_y - half, 0],  # Corner 1
        [center_x - half, center_y + half, 0],  # Corner 2
        [center_x + half, center_y + half, 0]   # Corner 3
    ], dtype=np.float64)

    return points

# ==========================================
# STEP 5: Start RealSense camera
# ==========================================

pipeline = rs.pipeline()
config = rs.config()

config.enable_device(CAMERA_SERIAL)

config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)

started = False

try:
    pipeline.start(config)
    started = True

    print("Multi-AprilTag Board Pose Estimation")
    print("Press S to print board pose.")
    print("Press Q to quit.")

    while True:

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        image = np.asanyarray(
            color_frame.get_data()
        ).copy()

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, _ = detector.detectMarkers(gray)

        board_pose = None
        marker_count = 0

        if ids is not None:

            object_points = []
            image_points = []

            for marker_corners, marker_id in zip(
                corners,
                ids.flatten()
            ):

                marker_id = int(marker_id)

                if not (0 <= marker_id < BOARD_ROWS * BOARD_COLS):
                    continue

                obj = get_marker_object_points(marker_id)

                img = marker_corners.reshape(4, 2)

                object_points.extend(obj)
                image_points.extend(img)

                marker_count += 1

            if marker_count >= 4:

                object_points = np.array(
                    object_points,
                    dtype=np.float64
                )

                image_points = np.array(
                    image_points,
                    dtype=np.float64
                )

                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    K,
                    D,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )

                if success:
                    board_pose = (rvec, tvec)

                    # ==========================================
                    # BOARD REPROJECTION ERROR
                    # ==========================================

                    projected_points, _ = cv2.projectPoints(
                        object_points,
                        rvec,
                        tvec,
                        K,
                        D
                    )

                    projected_points = projected_points.reshape(-1, 2)

                    residuals = projected_points - image_points

                    reprojection_rms = np.sqrt(
                        np.mean(np.sum(residuals ** 2, axis=1))
                    )

                    cv2.putText(
                        image,
                        f"Board RMS: {reprojection_rms:.2f} px",
                        (20, 115),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 0),
                        2
                    )

                    cv2.drawFrameAxes(
                        image,
                        K,
                        D,
                        rvec,
                        tvec,
                        TAG_SIZE
                    )

                    x, y, z = tvec.flatten()

                    cv2.putText(
                        image,
                        f"Board X:{x:.3f} Y:{y:.3f} Z:{z:.3f} m",
                        (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

            cv2.aruco.drawDetectedMarkers(
                image,
                corners,
                ids
            )

        cv2.putText(
            image,
            f"Markers: {marker_count}/36",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.imshow(
            "D435if Multi-AprilTag Board Pose",
            image
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):

            if board_pose is not None:

                rvec, tvec = board_pose

                R, _ = cv2.Rodrigues(rvec)

                print("\n=== BOARD 6D POSE ===")
                print(
                    f"Reprojection RMS: {reprojection_rms:.4f} pixels"
                )
                print("Detected markers:", marker_count)
                print("Translation (m):", tvec.flatten())
                print("Rotation matrix:")
                print(R)

            else:
                print("Board pose unavailable.")

        elif key == ord("q"):
            break

finally:

    if started:
        pipeline.stop()

    cv2.destroyAllWindows()