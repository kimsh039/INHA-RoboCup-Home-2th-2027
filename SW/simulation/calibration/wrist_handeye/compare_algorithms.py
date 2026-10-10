import json
from pathlib import Path

import cv2
import numpy as np


# ==========================================
# STEP 1: Load the calibration dataset
# ==========================================

ROOT = Path(__file__).resolve().parents[1]

DATASET_PATH = (
    ROOT
    / "records"
    / "20261006_wrist_d435"
    / "dataset.json"
)

with open(DATASET_PATH, "r") as f:
    dataset = json.load(f)

print("=== DATASET INFORMATION ===")
print("Calibration mode:", dataset["mode"])
print("Total samples:", len(dataset["samples"]))


# ==========================================
# STEP 2: Select training samples
# ==========================================

train_samples = [
    sample
    for sample in dataset["samples"]
    if sample["split"] == "train"
]

print("Training samples:", len(train_samples))


# ==========================================
# STEP 3: Extract transformations
# ==========================================

R_gripper2base = []
t_gripper2base = []

R_target2cam = []
t_target2cam = []

for sample in train_samples:

    T_robot = np.array(
        sample["T_base_flange"],
        dtype=np.float64
    )

    T_camera = np.array(
        sample["T_camera_target"],
        dtype=np.float64
    )

    # Robot rotation and translation
    R_gripper2base.append(T_robot[:3, :3])
    t_gripper2base.append(T_robot[:3, 3])

    # Camera rotation and translation
    R_target2cam.append(T_camera[:3, :3])
    t_target2cam.append(T_camera[:3, 3])


# ==========================================
# STEP 4: Define the calibration function
# ==========================================

def calibrate(method):

    R, t = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=method
    )

    T = np.eye(4)

    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)

    return T


# ==========================================
# STEP 5: Run both algorithms
# ==========================================

T_park = calibrate(cv2.CALIB_HAND_EYE_PARK)

T_tsai = calibrate(cv2.CALIB_HAND_EYE_TSAI)


# ==========================================
# STEP 6: Display results
# ==========================================

np.set_printoptions(
    precision=6,
    suppress=True
)

print("\n=== PARK RESULT ===")
print(T_park)

print("\n=== TSAI-LENZ RESULT ===")
print(T_tsai)

print("\n=== TRANSLATION COMPARISON ===")

print(
    "PARK (mm):",
    T_park[:3, 3] * 1000
)

print(
    "TSAI (mm):",
    T_tsai[:3, 3] * 1000
)

difference = np.linalg.norm(
    T_park[:3, 3] - T_tsai[:3, 3]
)

print(
    "Translation difference (mm):",
    difference * 1000
)

# ==========================================
# STEP 7: Validate calibration algorithms
# ==========================================

def get_target_poses(samples, T_flange_camera):
    """Transform observed target poses into the robot base frame."""

    target_poses = []

    for sample in samples:
        T_base_flange = np.array(
            sample["T_base_flange"],
            dtype=np.float64
        )

        T_camera_target = np.array(
            sample["T_camera_target"],
            dtype=np.float64
        )

        T_base_target = (
            T_base_flange
            @ T_flange_camera
            @ T_camera_target
        )

        target_poses.append(T_base_target)

    return target_poses


def rotation_error_deg(R1, R2):
    """Calculate angular difference between two rotations."""

    R_diff = R1.T @ R2

    cosine = (np.trace(R_diff) - 1.0) / 2.0
    cosine = np.clip(cosine, -1.0, 1.0)

    return np.degrees(np.arccos(cosine))


def validate(name, T_flange_camera):

    # Separate training and validation samples
    holdout_samples = [
        sample
        for sample in dataset["samples"]
        if sample["split"] == "holdout"
    ]

    # Estimate the fixed target pose from training samples
    training_target_poses = get_target_poses(
        train_samples,
        T_flange_camera
    )

    # Use first training observation as reference
    T_reference = training_target_poses[0]

    # Transform all validation observations
    validation_target_poses = get_target_poses(
        holdout_samples,
        T_flange_camera
    )

    translation_errors = []
    rotation_errors = []

    for T in validation_target_poses:

        translation_error = np.linalg.norm(
            T[:3, 3] - T_reference[:3, 3]
        )

        rotation_error = rotation_error_deg(
            T_reference[:3, :3],
            T[:3, :3]
        )

        translation_errors.append(
            translation_error * 1000
        )

        rotation_errors.append(rotation_error)

    print(f"\n=== {name} VALIDATION ===")

    print("Holdout samples:", len(holdout_samples))

    print(
        "Mean translation error (mm):",
        np.mean(translation_errors)
    )

    print(
        "Max translation error (mm):",
        np.max(translation_errors)
    )

    print(
        "Mean rotation error (deg):",
        np.mean(rotation_errors)
    )

    print(
        "Max rotation error (deg):",
        np.max(rotation_errors)
    )


# ==========================================
# STEP 8: Compare validation results
# ==========================================

validate("PARK", T_park)

validate("TSAI-LENZ", T_tsai)