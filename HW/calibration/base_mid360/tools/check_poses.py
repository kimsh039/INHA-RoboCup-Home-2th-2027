#!/usr/bin/env python3
"""Check new static poses against the recorded G2 and Mid-360S calibration.

Usage (ROS 2 Humble):
  python3 check_poses.py <bag_dir> <out_json> pose11 pose12 ...

Each pose bag needs /livox/lidar and /scan, robot stationary (tools/rec.sh in
../../base_2dlidar/vs_mid360/tools records this). Per pose:
  - G2 -> Mid-360S wall distance with the recorded transforms (median / p90 / within 20 mm)
  - single-pose re-fit of base_link <- laser_frame x/y/yaw with the recorded Mid-360S fixed
  - floor plane in base_link with the recorded Mid-360S (roll / pitch / height vs ground)
"""
import sys, json, math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.argv, ARGS = sys.argv[:1] + ['none', 'none'], sys.argv[1:]
HERE = Path(__file__).resolve().parent
exec((HERE.parents[1] / 'base_2dlidar/vs_mid360/tools/calib_g2_livox.py').read_text().split("if BAGDIR == 'prep':")[0])
bag_dir, out_path, names = Path(ARGS[0]), ARGS[1], ARGS[2:]
g2 = json.load(open(HERE.parents[1] / 'base_2dlidar/calibration.json'))
mid = json.load(open(HERE.parent / 'base_mid360.json'))
T_bm = np.array(mid['matrix4x4']); T_bl = np.array(g2['matrix_4x4']); GROUND = mid['floor']['ground_z_m']
p_g2 = [T_bl[0, 3], T_bl[1, 3], math.atan2(T_bl[1, 0], T_bl[0, 0])]

def base_map(clouds):
    P = np.vstack(clouds); P = P[np.linalg.norm(P, axis=1) > 0.3]
    P = voxel((T_bm[:3, :3] @ P.T).T + T_bm[:3, 3], 0.015)
    rr = np.hypot(P[:, 0], P[:, 1])
    band = P[(P[:, 2] > 0.13) & (P[:, 2] < 0.56) & (rr < 6.5)]
    tree = cKDTree(band); normal = np.empty_like(band); planar = np.zeros(len(band), bool)
    for s in range(0, len(band), 50000):
        b = band[s:s + 50000]; _, nn = tree.query(b, k=40); Q = band[nn]; Q -= Q.mean(1, keepdims=True)
        w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', Q, Q) / 40)
        normal[s:s + len(b)] = v[:, :, 0]; planar[s:s + len(b)] = (w[:, 0] / w.sum(1) < 0.03) & (np.abs(v[:, 2, 0]) < 0.5)
    F = P[(rr > 0.8) & (rr < 3.0) & (np.abs(P[:, 2] - GROUND) < 0.08)]
    return dict(tree=cKDTree(band[planar]), pts=band[planar], nrm=normal[planar]), F

def floor_fit(F):
    best = None; rng = np.random.default_rng(0)
    for _ in range(400):
        s = F[rng.choice(len(F), 3, replace=False)]; n = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(n) < 1e-9: continue
        n /= np.linalg.norm(n)
        if abs(n[2]) < np.cos(np.radians(8)): continue
        k = np.abs((F - s[0]) @ n) < 0.012
        if best is None or k.sum() > best.sum(): best = k
    G = F[best]; c = G.mean(0); _, _, vt = np.linalg.svd(G - c, full_matrices=False); n = vt[2] * np.sign(vt[2][2])
    return dict(roll_deg=float(np.degrees(np.arctan2(n[1], n[2]))), pitch_deg=float(np.degrees(-np.arctan2(n[0], n[2]))),
                height_vs_ground_mm=float((c[2] + (n[0] * c[0] + n[1] * c[1]) / n[2] - GROUND) * 1000), n=int(best.sum()))

out = {}
for n in names:
    clouds, scans, acc = read_bag(str(bag_dir / n))
    M, F = base_map(clouds); del clouds
    L, _ = scan_points(scans)
    st = stats([(L, M)], p_g2); pp, _, _ = fit([(L, M)], p_g2); fl = floor_fit(F)
    out[n] = dict(residual=st, refit=dict(x_mm=pp[0] * 1000, y_mm=pp[1] * 1000, yaw_deg=math.degrees(pp[2]),
                                          dx_mm=(pp[0] - p_g2[0]) * 1000, dy_mm=(pp[1] - p_g2[1]) * 1000, dyaw_deg=math.degrees(pp[2] - p_g2[2])), floor=fl)
    print(f"{n}: wall med {st['med_cm']*10:.1f} mm p90 {st['p90_cm']*10:.1f} mm in20 {st['in2cm']*100:.0f}% | refit dx {out[n]['refit']['dx_mm']:+.1f} dy {out[n]['refit']['dy_mm']:+.1f} mm dyaw {out[n]['refit']['dyaw_deg']:+.3f} deg"
          f" | floor roll {fl['roll_deg']:+.2f} pitch {fl['pitch_deg']:+.2f} deg h {fl['height_vs_ground_mm']:+.1f} mm")
json.dump(dict(record=dict(g2=g2['translation_m'], g2_yaw_deg=g2['yaw_deg'], mid=mid['translation_xyz_m'], mid_rpy_deg=mid['rpy_deg']), poses=out),
          open(out_path, 'w'), indent=2, default=float)
