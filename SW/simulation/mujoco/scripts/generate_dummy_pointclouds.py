"""Generate deterministic surface clouds in an object-centered, Z-up frame."""
from __future__ import annotations

import argparse
import hashlib
import json
import zipfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def surface_points(kind: str, dimensions: dict, count: int, rng, exclude_support_face=False) -> np.ndarray:
    if count < 1:
        raise ValueError("num_points must be positive")
    if kind == "cube":
        size = np.asarray(dimensions["dimensions_m"], dtype=float)
        if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0):
            raise ValueError("Cube dimensions must be three positive finite lengths")
        areas = np.array([size[1]*size[2], size[0]*size[2], size[0]*size[1]])
        probability=np.repeat(areas,2)
        if exclude_support_face:probability[4]=0  # local -Z face resting on table
        faces = rng.choice(6, size=count, p=probability/probability.sum())
        points = rng.uniform(-0.5, 0.5, size=(count, 3))*size
        axes = faces//2
        points[np.arange(count), axes] = np.where(faces % 2 == 0, -0.5, 0.5)*size[axes]
        return points
    if kind not in ("pringles", "can"):
        raise ValueError(f"Unsupported object: {kind}")
    radius, height = dimensions["radius_m"], dimensions["height_m"]
    if not np.isfinite([radius, height]).all() or min(radius, height) <= 0:
        raise ValueError("Cylinder dimensions must be positive and finite")
    areas = np.array([2*np.pi*radius*height, np.pi*radius**2, np.pi*radius**2])
    if exclude_support_face:areas[2]=0
    surface = rng.choice(3, size=count, p=areas/areas.sum())
    angle = rng.uniform(0, 2*np.pi, count)
    radial = np.where(surface == 0, radius, radius*np.sqrt(rng.uniform(0, 1, count)))
    z = rng.uniform(-height/2, height/2, count)
    z[surface == 1] = height/2
    z[surface == 2] = -height/2
    return np.column_stack((radial*np.cos(angle), radial*np.sin(angle), z))


def generate(kind: str, cfg: dict) -> tuple[np.ndarray, dict]:
    if cfg["units"] != "meters" or cfg["source_frame"] != "object":
        raise ValueError("Expected meters and object source frame")
    noise = float(cfg["noise_std_m"])
    if not np.isfinite(noise) or noise < 0:
        raise ValueError("noise_std_m must be finite and nonnegative")
    rng = np.random.default_rng(cfg["seed"])
    points = surface_points(kind, cfg["objects"][kind], int(cfg["num_points"]), rng, cfg.get("exclude_support_face",False))
    if noise:
        points += rng.normal(0, noise, points.shape)
    return points.astype(np.float32), {
        "object": kind, "dimensions": cfg["objects"][kind],
        "source_frame": cfg["source_frame"], "units": cfg["units"],
        "origin": "geometric center", "axes": "right-handed; Z is cylinder upright axis",
        "num_points": len(points), "seed": cfg["seed"], "noise_std_m": noise,
        "sampling": "area-weighted exposed surface; support -Z face excluded" if cfg.get("exclude_support_face") else "area-weighted full surface; synthetic, not camera-visible partial cloud",
        "exclude_support_face":cfg.get("exclude_support_face",False),
        "support_face": "object -Z; object must be placed upright",
        "dtype": "float32", "shape": [len(points), 3]
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--object", choices=("all", "cube", "pringles", "can"), default="all")
    parser.add_argument("--config", type=Path, default=ROOT/"config/pointclouds.json")
    parser.add_argument("--output", type=Path, default=ROOT/"data/pointclouds")
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    kinds = ("cube", "pringles", "can") if args.object == "all" else (args.object,)
    for kind in kinds:
        points, metadata = generate(kind, cfg)
        directory = args.output/kind
        directory.mkdir(parents=True, exist_ok=True)
        cloud = directory/"points.npy"
        np.save(cloud, points, allow_pickle=False)
        metadata["npy_sha256"] = hashlib.sha256(cloud.read_bytes()).hexdigest()
        with (directory/"points.ply").open("w") as file:
            file.write(f"ply\nformat ascii 1.0\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
            np.savetxt(file, points, fmt="%.9g")
        (directory/"metadata.json").write_text(json.dumps(metadata, indent=2)+"\n")
        with zipfile.ZipFile(directory/"colab_input.zip","w",zipfile.ZIP_DEFLATED) as archive:
            for filename in ("points.npy","metadata.json"):archive.write(directory/filename,filename)
        print(f"{kind}: {len(points)} points → {directory}")


if __name__ == "__main__":
    main()
