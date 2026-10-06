"""Synthetic 4 cm cube: only the surface visible straight on from object -X.

This models a detection, not a detector or camera driver. points_2d_m.npy stores
metric coordinates on the face; the calibrated virtual view supplies depth.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from manipulation.synthetic_camera import camera_transform


def generate(count=20000, seed=42, distance_m=0.50):
    if count < 2048 or not np.isfinite(distance_m) or distance_m <= 0.02:
        raise ValueError('Need >=2048 points and camera farther than front face')
    face = np.random.default_rng(seed).uniform(-0.02, 0.02, (count, 2))
    # Object: Z up. Camera: X right, Y down, Z forward. Face is object -X.
    points = np.column_stack((np.full(count, -0.02), -face[:, 0], -face[:, 1])).astype(np.float32)
    T = camera_transform([-distance_m, 0, 0])
    camera = (points @ T[:3, :3].T + T[:3, 3]).astype(np.float32)
    meta = {
        'object': 'cube', 'dimensions': {'dimensions_m': [0.04]*3},
        'source_frame': 'object', 'units': 'meters', 'origin': 'geometric center',
        'num_points': count, 'shape': [count, 3], 'dtype': 'float32', 'seed': seed,
        'sampling': 'synthetic front face only; uniform metric plane samples',
        'view_policy': 'straight-on from object -X; one visible face; hidden faces absent',
        'T_camera_object': T.tolist(), 'camera_position_object_m': [-distance_m, 0, 0],
        'front_depth_m': distance_m-0.02,
        'depth_source': 'assumed virtual camera distance, not inferred from 2D detection',
        'plane_coordinates': 'camera X/Y in meters; not image pixels',
        'camera_axes': 'X right, Y down, Z forward', 'object_axes': 'Z up',
        'noise_std_m': 0.0, 'exclude_support_face': True,
    }
    return points, camera, face.astype(np.float32), meta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'data/pointclouds/cube_front')
    parser.add_argument('--count', type=int, default=20000)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--distance', type=float, default=0.50)
    args = parser.parse_args()
    dimensions = json.loads((ROOT/'config/pointclouds.json').read_text())['objects']['cube']['dimensions_m']
    if not np.allclose(dimensions, [0.04]*3, atol=1e-9, rtol=0):
        raise ValueError('Scene cube must remain 4 cm for this experiment')
    points, camera, plane, meta = generate(args.count, args.seed, args.distance)
    args.output.mkdir(parents=True, exist_ok=True)
    for filename, array in [('points.npy', points), ('points_camera.npy', camera), ('points_2d_m.npy', plane)]:
        np.save(args.output/filename, array, allow_pickle=False)
    meta['npy_sha256'] = hashlib.sha256((args.output/'points.npy').read_bytes()).hexdigest()
    (args.output/'metadata.json').write_text(json.dumps(meta, indent=2)+'\n')
    with zipfile.ZipFile(args.output/'colab_input.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in ('points.npy', 'points_camera.npy', 'points_2d_m.npy', 'metadata.json'):
            archive.write(args.output/name, name)
    print(f'{len(points)} front-face points, depth {meta["front_depth_m"]:.3f} m -> {args.output}')


if __name__ == '__main__':
    main()
