import json
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from scipy.spatial.transform import Rotation


# ==========================================
# STEP 1: Configuration
# ==========================================

CAMERA_SERIAL = "243222070076"
TAG_SIZE = 0.075
NUM_SAMPLES = 100

ROOT = Path(__file__).resolve().parents[4]

CALIBRATION_FILE = (
    ROOT / "HW/calibration/camera_intrinsics/wrist/ost.yaml"
)

OUTPUT_FILE = (
    Path(__file__).resolve().parent
    / "results"
    / "pnp_comparison_tilted_25deg.json"
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

half = TAG_SIZE / 2.0

object_points = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float64)


# ==========================================
# STEP 4: Define pose estimation
# ==========================================

def estimate_pose(image_points, method):

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        K,
        D,
        flags=method
    )

    if not success:
        return None

    R, _ = cv2.Rodrigues(rvec)

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
        np.mean(np.sum(residuals ** 2, axis=1))
    )

    return {
        "position": tvec.reshape(3).copy(),
        "rotation": R.copy(),
        "rms": float(rms)
    }


# ==========================================
# STEP 5: Analyze stability
# ==========================================

def analyze_stability(poses):

    positions = np.array([
        pose["position"] for pose in poses
    ])

    rotations = Rotation.from_matrix(
        np.array([
            pose["rotation"] for pose in poses
        ])
    )

    mean_rotation = rotations.mean()

    angular_errors = np.degrees(
        (mean_rotation.inv() * rotations).magnitude()
    )

    return {
        "mean_xyz_mm": (
            np.mean(positions, axis=0) * 1000
        ).tolist(),

        "std_xyz_mm": (
            np.std(positions, axis=0) * 1000
        ).tolist(),

        "mean_rotation_deviation_deg": float(
            np.mean(angular_errors)
        ),

        "max_rotation_deviation_deg": float(
            np.max(angular_errors)
        ),

        "mean_reprojection_rms_px": float(
            np.mean([pose["rms"] for pose in poses])
        )
    }


# ==========================================
# STEP 6: Start RealSense camera
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

ippe_poses = []
iterative_poses = []
paired_translation_differences = []
paired_rotation_differences = []

started = False
collecting = False

try:
    pipeline.start(config)
    started = True

    print("Camera started.")
    print("Keep camera and AprilTag stationary.")
    print("Press S to collect 100 paired samples.")
    print("Press Q to quit.")

    while len(ippe_poses) < NUM_SAMPLES:

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

        detected = False

        if ids is not None:

            for marker_corners, marker_id in zip(
                corners, ids.flatten()
            ):

                if marker_id != 0:
                    continue

                detected = True

                image_points = marker_corners.reshape(4, 2)

                ippe = estimate_pose(
                    image_points,
                    cv2.SOLVEPNP_IPPE_SQUARE
                )

                iterative = estimate_pose(
                    image_points,
                    cv2.SOLVEPNP_ITERATIVE
                )

                cv2.aruco.drawDetectedMarkers(
                    image,
                    [marker_corners],
                    np.array([[marker_id]], dtype=np.int32)
                )

                if (
                    collecting
                    and ippe is not None
                    and iterative is not None
                ):

                    ippe_poses.append(ippe)
                    iterative_poses.append(iterative)

                    # Translation difference
                    translation_difference = np.linalg.norm(
                        ippe["position"] - iterative["position"]
                    ) * 1000

                    paired_translation_differences.append(
                        translation_difference
                    )

                    # Rotation difference
                    R_difference = (
                        ippe["rotation"].T
                        @ iterative["rotation"]
                    )

                    rotation_difference = np.degrees(
                        Rotation.from_matrix(
                            R_difference
                        ).magnitude()
                    )

                    paired_rotation_differences.append(
                        rotation_difference
                    )

                break

        cv2.putText(
            image,
            f"Paired samples: {len(ippe_poses)}/{NUM_SAMPLES}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.imshow(
            "IPPE vs ITERATIVE",
            image
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s") and detected:
            if not collecting:
                collecting = True
                print("Collecting paired samples...")

        elif key == ord("q"):
            break

finally:
    if started:
        pipeline.stop()

    cv2.destroyAllWindows()


# ==========================================
# STEP 7: Analyze and save results
# ==========================================

if len(ippe_poses) == NUM_SAMPLES:

    ippe_results = analyze_stability(ippe_poses)
    iterative_results = analyze_stability(iterative_poses)

    results = {
        "camera": "RealSense D435if",
        "camera_serial": CAMERA_SERIAL,
        "tag_family": "AprilTag 36h11",
        "tag_id": 0,
        "tag_size_m": TAG_SIZE,
        "num_paired_samples": NUM_SAMPLES,
        "intrinsics_source": str(CALIBRATION_FILE.relative_to(ROOT)),
        "ippe_square": ippe_results,
        "iterative": iterative_results,
        "paired_translation_difference_mm": {
            "mean": float(np.mean(paired_translation_differences)),
            "max": float(np.max(paired_translation_differences))
        },
        "paired_rotation_difference_deg": {
            "mean": float(np.mean(paired_rotation_differences)),
            "max": float(np.max(paired_rotation_differences))
        }
    }

    OUTPUT_FILE.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    with open(OUTPUT_FILE, "w") as f:
        json.dump(results, f, indent=4)

    print("\n=== IPPE_SQUARE ===")
    print(json.dumps(ippe_results, indent=4))

    print("\n=== ITERATIVE ===")
    print(json.dumps(iterative_results, indent=4))

    print("\n=== PAIRED DIFFERENCES ===")
    print(
        "Mean translation difference (mm):",
        results["paired_translation_difference_mm"]["mean"]
    )
    print(
        "Mean rotation difference (deg):",
        results["paired_rotation_difference_deg"]["mean"]
    )

    print("\nResults saved to:", OUTPUT_FILE)

else:
    print(
        f"\nIncomplete test: {len(ippe_poses)} "
        f"of {NUM_SAMPLES} paired observations."
    )