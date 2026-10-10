#!/usr/bin/env python3
"""Fit the real floor plane in the Mid-360S cloud of static poses.

Usage (ROS 2 Humble environment):
  python3 floor_real.py <bag_dir> <out_json> pose01 pose02 pose03

Each pose bag must contain /livox/lidar (PointCloud2). Points are expressed in base_link
with the CAD base_link <- livox_frame, a 0.8-3 m ring is searched for the dominant
near-horizontal plane (RANSAC 12 mm, then least squares 10 mm).
"""
import json, sys
from pathlib import Path
import numpy as np

sys.argv, ARGS = sys.argv[:1] + ['none', 'none'], sys.argv[1:]
TOOL = Path(__file__).resolve().parents[2] / 'base_2dlidar/vs_mid360/tools/calib_g2_livox.py'
exec(TOOL.read_text().split("if BAGDIR == 'prep':")[0])          # read_bag, voxel, R_BL, T_BL
bag_dir, out_path, names = Path(ARGS[0]), ARGS[1], ARGS[2:]

out = {}
for n in names:
    clouds, scans, acc = read_bag(str(bag_dir / n))
    P = np.vstack(clouds); del clouds
    P = P[np.linalg.norm(P, axis=1) > 0.3]
    P = voxel((R_BL @ P.T).T + T_BL, 0.015)
    rr = np.hypot(P[:, 0], P[:, 1])
    F = P[(rr > 0.8) & (rr < 3.0) & (P[:, 2] > -0.40) & (P[:, 2] < 0.05)]
    best = None; rng = np.random.default_rng(0)
    for _ in range(600):
        s = F[rng.choice(len(F), 3, replace=False)]; nrm = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(nrm) < 1e-9: continue
        nrm /= np.linalg.norm(nrm)
        if abs(nrm[2]) < np.cos(np.radians(8)): continue
        k = np.abs((F - s[0]) @ nrm) < 0.012
        if best is None or k.sum() > best.sum(): best = k
    for _ in range(3):
        G = F[best]; c = G.mean(0); _, _, vt = np.linalg.svd(G - c, full_matrices=False); nn = vt[2] * np.sign(vt[2][2])
        best = np.abs((F - c) @ nn) < 0.01
    roll = np.degrees(np.arctan2(nn[1], nn[2])); pitch = np.degrees(-np.arctan2(nn[0], nn[2]))
    z0 = c[2] + (nn[0] * c[0] + nn[1] * c[1]) / nn[2]               # floor height under the CAD base origin
    sensor_dist = float(abs((T_BL - c) @ nn))                         # Mid-360S origin to plane distance
    g = R_BL @ acc.mean(0); g /= np.linalg.norm(g)
    out[n] = dict(height_at_origin_mm=float(z0 * 1000), centroid_mm=(c * 1000).tolist(), normal=nn.tolist(),
                  roll_deg=float(roll), pitch_deg=float(pitch), sensor_to_plane_mm=sensor_dist * 1000,
                  n=int(best.sum()), inlier=float(best.mean()),
                  imu_gravity_base=g.tolist(), imu_angle_to_floor_normal_deg=float(np.degrees(np.arccos(abs(g @ nn)))))
    print(f"{n}: z@origin={z0*1000:+.1f} mm roll={roll:+.3f} pitch={pitch:+.3f} sensor->plane={sensor_dist*1000:.1f} mm "
          f"n={best.sum()} | IMU vs floor normal {out[n]['imu_angle_to_floor_normal_deg']:.2f} deg")
keys = ('height_at_origin_mm', 'roll_deg', 'pitch_deg', 'sensor_to_plane_mm')
res = dict(poses=names, per_pose=out, mean={k: float(np.mean([v[k] for v in out.values()])) for k in keys},
           std={k: float(np.std([v[k] for v in out.values()], ddof=1)) for k in keys})
json.dump(res, open(out_path, 'w'), indent=2)
print('mean', {k: round(v, 3) for k, v in res['mean'].items()}, 'std', {k: round(v, 3) for k, v in res['std'].items()})
