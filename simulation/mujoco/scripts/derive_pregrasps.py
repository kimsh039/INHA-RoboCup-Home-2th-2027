"""Derive object-frame grasp preposes; no TCP mapping or robot feasibility claims."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def derive(rotation, translation, distance_m: float, tolerance: float = 1e-5) -> dict:
    R = np.asarray(rotation, dtype=float)
    p = np.asarray(translation, dtype=float)
    if R.shape != (3, 3) or p.shape != (3,) or not np.isfinite(R).all() or not np.isfinite(p).all():
        raise ValueError("Expected finite 3×3 rotation and 3-vector translation")
    if not np.isfinite(distance_m) or distance_m <= 0:
        raise ValueError("distance_m must be positive and finite")
    if not np.allclose(R.T @ R, np.eye(3), atol=tolerance, rtol=0) or abs(np.linalg.det(R)-1) > tolerance:
        raise ValueError("Rotation must be right-handed and orthonormal")
    # Official baseline loss_utils: axis_x is the approach vector.
    a_object = R[:, 0]/np.linalg.norm(R[:, 0])
    pre = p-distance_m*a_object
    T_object_grasp = np.eye(4)
    T_object_grasp[:3, :3] = R
    T_object_grasp[:3, 3] = p
    T_object_pregrasp = T_object_grasp.copy()
    T_object_pregrasp[:3, 3] = pre
    return {"approach_vector_object": a_object.tolist(),
            "T_object_grasp": T_object_grasp.tolist(),
            "T_object_pregrasp": T_object_pregrasp.tolist()}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=ROOT/"config/pregrasp.json")
    args = parser.parse_args()
    source = json.loads(args.input.read_text())
    cfg = json.loads(args.config.read_text())
    if source["units"] != "meters" or source["point_cloud_frame"] != "object":
        raise ValueError("Only object-frame meter inputs are supported")
    if source["approach_axis"] != "+X_grasp (rotation_matrix first column)":
        raise ValueError("Unexpected approach-axis convention")
    if not source["grasps"]:
        raise ValueError("NO_GRASP_CANDIDATE")
    derived = []
    for grasp in source["grasps"]:
        pre = derive(grasp["rotation_matrix"], grasp["translation"],
                     cfg["distance_m"], cfg["rotation_tolerance"])
        derived.append({**grasp, **pre})
    output = {**source, "grasps": derived, "pregrasp_distance_m": cfg["distance_m"],
              "pregrasp_frame": "object", "feasibility_checked": False,
              "tcp_mapping": "NOT_DEFINED", "selected_candidate_id": None,
              "pregrasp_note": "Geometric grasp-frame preposes only; not executable TCP targets"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, allow_nan=False)+"\n")
    print(f"Derived {len(derived)} object-frame pregrasps at {cfg['distance_m']} m → {args.output}")


if __name__ == "__main__":
    main()
