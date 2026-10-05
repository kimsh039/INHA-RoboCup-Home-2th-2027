"""Generate cube returns from the URDF-mounted MID360 in the MuJoCo scene."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from manipulation.mid360_cloud import scan_cube
from manipulation.sim_model import config, load_model, body_transform


def live_status(timeout_s):
    import time
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node = rclpy.create_node('mid360_scene_snapshot')
    received = []
    subscription = node.create_subscription(String, '/simulation/grasp_status',
                                             lambda msg: received.append(json.loads(msg.data)), 10)
    try:
        end = time.monotonic()+timeout_s
        while not received and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        if not received:
            raise RuntimeError('START_MUJOCO_BRIDGE_FIRST: no /simulation/grasp_status')
        return received[-1]
    finally:
        node.destroy_node();rclpy.shutdown()


def apply_snapshot(model, data, status):
    import mujoco
    from scipy.spatial.transform import Rotation
    if status.get('failure'):
        raise ValueError(f'FAILED_SIMULATION_SNAPSHOT: {status["failure"]}')
    mujoco.mj_forward(model, data)
    if not np.allclose(body_transform(model, data, 'base_link'), status['T_world_base'], atol=1e-6):
        raise ValueError('LIVE_BASE_DIFFERS_FROM_COMPILED_SCENE')
    values = np.asarray(status['joint_positions'], dtype=float)
    T = np.asarray(status['T_world_object'], dtype=float)
    if values.shape != (8,) or T.shape != (4,4) or not np.isfinite(values).all() or not np.isfinite(T).all():
        raise ValueError('INVALID_SCENE_SNAPSHOT')
    for i, value in enumerate(values, 1):
        data.qpos[model.joint(f'joint{i}').qposadr[0]] = value
    start = int(model.joint('cube_free').qposadr[0])
    data.qpos[start:start+3] = T[:3, 3]
    data.qpos[start+3:start+7] = Rotation.from_matrix(T[:3, :3]).as_quat()[[3,0,1,2]]
    mujoco.mj_forward(model, data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'data/pointclouds/cube_mid360')
    parser.add_argument('--resolution', type=int, default=192)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--live', action='store_true', help='Capture current bridge poses/joints, read only')
    group.add_argument('--snapshot', type=Path, help='JSON from /simulation/grasp_status')
    args = parser.parse_args()
    status = live_status(10) if args.live else json.loads(args.snapshot.read_text()) if args.snapshot else None
    model, data = load_model()
    if status is not None:
        apply_snapshot(model, data, status)
    points, lidar, network, meta = scan_cube(model, data, config(), args.resolution)
    meta['snapshot_source'] = 'live bridge' if args.live else 'saved status' if args.snapshot else 'compiled initial MuJoCo scene'
    args.output.mkdir(parents=True, exist_ok=True)
    for name, array in [('points.npy', points), ('points_lidar.npy', lidar), ('points_network.npy', network)]:
        np.save(args.output/name, array, allow_pickle=False)
    meta['npy_sha256'] = hashlib.sha256((args.output/'points.npy').read_bytes()).hexdigest()
    (args.output/'metadata.json').write_text(json.dumps(meta, indent=2, allow_nan=False)+'\n')
    with zipfile.ZipFile(args.output/'cube_mid360_input.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in ['points.npy', 'points_lidar.npy', 'points_network.npy', 'metadata.json']:
            archive.write(args.output/name, name)
    print(f'MID360 world position: {meta["T_world_lidar"][0][3]:.6f}, {meta["T_world_lidar"][1][3]:.6f}, {meta["T_world_lidar"][2][3]:.6f}')
    print(f'Cube returns: {len(points)} / {meta["roi_rays_in_fov"]} rays; first hits: {meta["first_hit_counts"]}')
    print(args.output/'cube_mid360_input.zip')


if __name__ == '__main__':
    main()
