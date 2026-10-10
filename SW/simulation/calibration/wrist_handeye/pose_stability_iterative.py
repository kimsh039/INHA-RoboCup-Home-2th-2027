import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from pathlib import Path

# ==========================================
# STEP 1: Configuration
# ==========================================

CAMERA_SERIAL = "243222070076"

TAG_SIZE = 0.075  # meters
NUM_SAMPLES = 100

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
# STEP 3: AprilTag detector
# ==========================================

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_APRILTAG_36h11
)

detector = cv2.aruco.ArucoDetector(
    dictionary,
    cv2.aruco.DetectorParameters()
)

half = TAG_SIZE / 2

object_points = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float64)

# ==========================================
# STEP 4: Collect observations
# ==========================================

positions = []
reprojection_errors = []
rotations = []
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

    print("Camera started.")
    print("Keep the camera and AprilTag stationary.")
    print("Press S to begin collecting 100 valid samples.")
    print("Press Q to quit.")

    collecting = False

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

        detected_pose = None

        if ids is not None:
            for marker_corners, marker_id in zip(
                corners, ids.flatten()
            ):
                if marker_id != 0:
                    continue

                image_points = marker_corners.reshape(4, 2)

                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    K,
                    D,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )

                if not success:
                    continue

                detected_pose = (rvec, tvec)

                cv2.aruco.drawDetectedMarkers(
                    image,
                    [marker_corners],
                    np.array([[marker_id]], dtype=np.int32)
                )

                cv2.drawFrameAxes(
                    image,
                    K,
                    D,
                    rvec,
                    tvec,
                    TAG_SIZE / 2
                )

                if collecting:
                    positions.append(tvec.reshape(3).copy())
                    R, _ = cv2.Rodrigues(rvec)
                    rotations.append(R.copy())
                    projected, _ = cv2.projectPoints(
                        object_points,
                        rvec,
                        tvec,
                        K,
                        D
                    )

                    residuals = (
                        projected.reshape(-1, 2) - image_points
                    )

                    rms = np.sqrt(
                        np.mean(np.sum(residuals**2, axis=1))
                    )

                    reprojection_errors.append(float(rms))

                break

        count = len(positions)

        cv2.putText(
            image,
            f"Samples: {count}/{NUM_SAMPLES}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.imshow("AprilTag Stability Test", image)

        if count >= NUM_SAMPLES:
            break

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s") and detected_pose is not None:
            if not collecting:
                positions.clear()
                rotations.clear()
                reprojection_errors.clear()
                collecting = True
                print("Collecting samples...")

        elif key == ord("q"):
            break

finally:
    if started:
        pipeline.stop()

    cv2.destroyAllWindows()

# ==========================================
# STEP 5: Analyze results
# ==========================================

if len(positions) == NUM_SAMPLES:

    positions = np.array(positions)

    mean_xyz = np.mean(positions, axis=0)
    std_xyz = np.std(positions, axis=0)

    print("\n=== POSE STABILITY RESULTS ===")

    print("Valid samples:", len(positions))

    print("\nMean XYZ (mm):")
    print(mean_xyz * 1000)

    print("\nStandard Deviation XYZ (mm):")
    print(std_xyz * 1000)

    print("\nZ Range (mm):")
    print(
        np.min(positions[:, 2]) * 1000,
        "to",
        np.max(positions[:, 2]) * 1000
    )

    print("\nMean Reprojection RMS (pixels):")
    print(np.mean(reprojection_errors))

    # ==========================================
    # STEP A10: Rotation Stability Analysis
    # ==========================================

    from scipy.spatial.transform import Rotation

    # Convert rotation matrices into SciPy Rotation objects
    rotation_objects = Rotation.from_matrix(
        np.array(rotations)
    )

    # Calculate the mean orientation
    mean_rotation = rotation_objects.mean()

    # Calculate each observation's rotation relative to the mean
    relative_rotations = (
        mean_rotation.inv() * rotation_objects
    )

    # Convert angular differences from radians to degrees
    rotation_errors_deg = np.degrees(
        relative_rotations.magnitude()
    )

    mean_rotation_error = np.mean(rotation_errors_deg)
    std_rotation_error = np.std(rotation_errors_deg)
    max_rotation_error = np.max(rotation_errors_deg)

    print("\n=== ROTATION STABILITY RESULTS ===")

    print(f"Mean angular deviation: {mean_rotation_error:.6f} deg")
    print(f"Std angular deviation: {std_rotation_error:.6f} deg")
    print(f"Max angular deviation: {max_rotation_error:.6f} deg")

    # Save measured results
    import json

    results = {
        "camera": "RealSense D435if",
        "camera_serial": CAMERA_SERIAL,
        "marker_family": "AprilTag 36h11",
        "marker_id": 0,
        "tag_size_m": TAG_SIZE,
        "intrinsics_source": "HW/calibration/camera_intrinsics/wrist/ost.yaml",
        "num_samples": len(positions),
        "mean_xyz_mm": (mean_xyz * 1000).tolist(),
        "std_xyz_mm": (std_xyz * 1000).tolist(),
        "min_z_mm": float(np.min(positions[:, 2]) * 1000),
        "max_z_mm": float(np.max(positions[:, 2]) * 1000),
        "mean_reprojection_rms_px": float(np.mean(reprojection_errors)),
        "notes": "Stationary AprilTag repeatability test; not absolute accuracy validation.",
        "mean_rotation_deviation_deg": float(mean_rotation_error),
        "std_rotation_deviation_deg": float(std_rotation_error),
        "max_rotation_deviation_deg": float(max_rotation_error),
    }

    output_dir = Path(__file__).resolve().parent / "results"
    output_dir.mkdir(parents=True, exist_ok=True)

    output_file = output_dir / "pose_stability_iterative_result.json"

    with open(output_file, "w") as f:
        json.dump(results, f, indent=4)

    print("\nResults saved to:", output_file)

else:
    print(
        f"\nTest incomplete: collected {len(positions)} "
        f"of {NUM_SAMPLES} samples."
    )