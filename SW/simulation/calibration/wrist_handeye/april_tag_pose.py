import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from pathlib import Path
from collections import deque 

angle_history = deque(maxlen=30)  # Store the last 30 viewing angles

# ==========================================
# STEP 1: Configuration
# ==========================================

CAMERA_SERIAL = "243222070076"
TAG_SIZE = 0.075 # 8.8 cm

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

# Define marker corners in 3D
half = TAG_SIZE / 2

object_points = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float64)

# ==========================================
# STEP 4: Start RealSense
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

    print("D435if started!")
    print("Press S to print pose, Q to quit.")

    while True:
        frames = pipeline.wait_for_frames()
        frame = frames.get_color_frame()

        if not frame:
            continue

        image = np.asanyarray(frame.get_data())
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = detector.detectMarkers(gray)

        pose = None

        if ids is not None:
            for marker_corners, marker_id in zip(
                corners, ids.flatten()
            ):
                if marker_id != 0:
                    continue

                image_points = marker_corners.reshape(4, 2)

                # Measure detected marker width in pixels
                top_width = np.linalg.norm(
                    image_points[1] - image_points[0]
                )

                bottom_width = np.linalg.norm(
                    image_points[2] - image_points[3]
                )


                pixel_width = (top_width + bottom_width) / 2

                cv2.putText(
                    image,
                    f"Marker Width: {pixel_width:.2f} px",
                    (20, 160),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                print(
                    f"Detected marker width: {pixel_width:.2f} pixels",
                    end="\r",
                    flush=True
                )

                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    K,
                    D,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )

                # Alternative PnP method
                success_iter, rvec_iter, tvec_iter = cv2.solvePnP(
                    object_points,
                    image_points,
                    K,
                    D,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )

                if success and success_iter:
                    print(
                        f"IPPE Z: {float(tvec[2, 0]) * 100:.2f} cm | "
                        f"ITERATIVE Z: {float(tvec_iter[2, 0]) * 100:.2f} cm",
                        end="\r",
                        flush=True
                    )

                # Compare factory and team camera intrinsics
                K_factory = np.array([
                    [606.47009277, 0, 317.72854614],
                    [0, 606.55560303, 254.95840454],
                    [0, 0, 1]
                ], dtype=np.float64)

                D_factory = np.zeros(5, dtype=np.float64)

                if success:
                    success_factory, rvec_factory, tvec_factory = cv2.solvePnP(
                        object_points,
                        image_points,
                        K_factory,
                        D_factory,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if success_factory:
                        team_z = float(tvec[2, 0]) * 100
                        factory_z = float(tvec_factory[2, 0]) * 100

                        # Display both distances in the camera window
                        cv2.putText(
                            image,
                            f"Team Z: {team_z:.2f} cm",
                            (20, 75),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2
                        )

                        cv2.putText(
                            image,
                            f"Factory Z: {factory_z:.2f} cm",
                            (20, 105),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2
                        )

                        print(
                            f"Team Z: {team_z:.2f} cm | "
                            f"Factory Z: {factory_z:.2f} cm",
                            end="\r",
                            flush=True
                        )

                if success:
                    pose = (rvec, tvec)

                    # Calculate AprilTag viewing angle
                    R, _ = cv2.Rodrigues(rvec)

                    # AprilTag surface normal in camera coordinates
                    tag_normal = R[:, 2]

                    # Angle between camera optical axis and board normal
                    cos_angle = np.clip(abs(tag_normal[2]), 0.0, 1.0)

                    viewing_angle = np.degrees(
                        np.arccos(cos_angle)
                    )

                    # Store the latest 30 viewing angles
                    angle_history.append(viewing_angle)

                    # Calculate the median angle
                    stable_angle = np.median(angle_history)

                    cv2.putText(
                        image,
                        f"Viewing Angle: {stable_angle:.1f} deg",
                        (20, 190),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 0),
                        2
                    )


                    cv2.aruco.drawDetectedMarkers(
                        image,
                        [marker_corners],
                        np.array([[marker_id]])
                    )

                    cv2.drawFrameAxes(
                        image,
                        K,
                        D,
                        rvec,
                        tvec,
                        TAG_SIZE / 2
                    )

                    x, y, z = tvec.flatten()

                    cv2.putText(
                        image,
                        f"X:{x:.3f} Y:{y:.3f} Z:{z:.3f} m",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

        cv2.imshow("AprilTag 6D Pose", image)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s") and pose is not None:
            rvec, tvec = pose
            R, _ = cv2.Rodrigues(rvec)

            print("\n=== APRILTAG 6D POSE ===")
            print("Translation (m):", tvec.flatten())
            print("Rotation Matrix:")
            print(R)

        elif key == ord("q"):
            break

finally:
    if started:
        pipeline.stop()

    cv2.destroyAllWindows()