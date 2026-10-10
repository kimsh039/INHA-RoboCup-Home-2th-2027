from pathlib import Path

import numpy as np
import yaml


# ==========================================
# STEP 1: Locate the calibration file
# ==========================================

REPO_ROOT = Path(__file__).resolve().parents[4]

CALIBRATION_FILE = (
    REPO_ROOT
    / "HW"
    / "calibration"
    / "camera_intrinsics"
    / "wrist"
    / "ost.yaml"
)


# ==========================================
# STEP 2: Load YAML
# ==========================================

with open(CALIBRATION_FILE, "r") as f:
    data = yaml.safe_load(f)


# ==========================================
# STEP 3: Extract camera matrix
# ==========================================

K = np.array(
    data["camera_matrix"]["data"],
    dtype=np.float64
).reshape(3, 3)


# ==========================================
# STEP 4: Extract distortion coefficients
# ==========================================

D = np.array(
    data["distortion_coefficients"]["data"],
    dtype=np.float64
)


# ==========================================
# STEP 5: Display results
# ==========================================

print("\n=== CAMERA INTRINSICS ===")

print("Resolution:",
      data["image_width"],
      "x",
      data["image_height"])

print("\nCamera Matrix K:")
print(K)

print("\nDistortion Coefficients:")
print(D)

print("\nDistortion Model:",
      data["distortion_model"])