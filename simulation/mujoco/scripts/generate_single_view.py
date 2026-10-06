"""Create a synthetic partial cube cloud from one virtual camera viewpoint.

Only visible surfaces are retained; hidden surfaces are never model input.
Sampling is area based, not a rendered depth sensor measurement.
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
from scripts.generate_dummy_pointclouds import surface_points
from manipulation.synthetic_camera import camera_transform, visible_cube_surface


def generate(count=20000, seed=42, camera_position=(-0.50, 0, 0)):
    camera_position = np.asarray(camera_position, dtype=float)
    if (count < 2048 or camera_position.shape != (3,)
            or not np.isfinite(camera_position).all() or np.all(np.abs(camera_position) <= .02)):
        raise ValueError('Need >=2048 points and a finite camera outside the cube')
    rng = np.random.default_rng(seed)
    dimensions = {'dimensions_m': [.04]*3}
    full = surface_points('cube', dimensions, count*12, rng, exclude_support_face=True)
    view = visible_cube_surface(full, dimensions['dimensions_m'], camera_position)
    points = view[rng.choice(len(view), count, replace=len(view) < count)].astype(np.float32)
    T = camera_transform(camera_position)
    camera = (points@T[:3, :3].T+T[:3, 3]).astype(np.float32)
    metadata = {
        'object': 'cube', 'dimensions': dimensions, 'source_frame': 'object',
        'units': 'meters', 'origin': 'geometric center', 'object_axes': 'Z up',
        'camera_axes': 'X right, Y down, Z forward', 'num_points': count,
        'shape': [count, 3], 'dtype': 'float32', 'seed': seed, 'noise_std_m': 0,
        'sampling': 'area-weighted visible cube surface, synthetic, not a real depth image',
        'view_policy': 'one virtual viewpoint; visible faces only; support face excluded',
        'camera_position_object_m': camera_position.tolist(), 'T_camera_object': T.tolist(),
        'depth_source': 'known synthetic geometry and virtual camera pose',
        'exclude_support_face': True,
    }
    return points, camera, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', nargs=3, type=float, default=[-.50, 0, 0], metavar=('X', 'Y', 'Z'))
    parser.add_argument('--count', type=int, default=20000)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--output', type=Path, default=ROOT/'data/pointclouds/cube_single_view')
    args = parser.parse_args()
    scene_dimensions = json.loads((ROOT/'config/pointclouds.json').read_text())['objects']['cube']['dimensions_m']
    if not np.allclose(scene_dimensions, [.04]*3, atol=1e-9, rtol=0):
        raise ValueError('Scene cube must be 4 cm for this experiment')
    points, camera, metadata = generate(args.count, args.seed, args.camera)
    args.output.mkdir(parents=True, exist_ok=True)
    np.save(args.output/'points.npy', points, allow_pickle=False)
    np.save(args.output/'points_camera.npy', camera, allow_pickle=False)
    metadata['npy_sha256'] = hashlib.sha256((args.output/'points.npy').read_bytes()).hexdigest()
    (args.output/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    with zipfile.ZipFile(args.output/'cube_single_view_input.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in ('points.npy', 'points_camera.npy', 'metadata.json'):
            archive.write(args.output/name, name)
    print(f'{len(points)} visible points from {args.camera} -> {args.output}')


if __name__ == '__main__':
    main()
