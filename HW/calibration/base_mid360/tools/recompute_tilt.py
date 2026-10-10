#!/usr/bin/env python3
"""Mid-360S base_link <- livox_frame from the floor plane and the G2 chain.

Usage:
  python3 recompute_tilt.py <prep_dir> <floor_json> <g2_calibration_json> <out_json>

prep_dir     per-pose npz from ../../base_2dlidar/vs_mid360/tools/calib_g2_livox.py prep
floor_json   output of floor_real.py (floor poses must not include the hold-out poses)
g2_...json   ../../base_2dlidar/calibration.json (base_link <- laser_frame, motion geometry)

1. Floor normal (mean of the floor poses) -> tilt correction R_d about the Mid-360S origin.
2. Re-fit G2 <-> Mid-360S x/y/yaw on the tilt-corrected maps (train poses).
3. Chain with the G2 base pose -> Mid-360S x/y/yaw; z = ground + mean sensor-to-floor distance.
4. Evaluate the hold-out poses with the final deployed transforms.
"""
import sys, json, math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.argv, ARGS = sys.argv[:1] + ['none', 'none'], sys.argv[1:]
TOOL = Path(__file__).resolve().parents[2] / 'base_2dlidar/vs_mid360/tools/calib_g2_livox.py'
exec(TOOL.read_text().split("if BAGDIR == 'prep':")[0])          # fit, stats, R_BL, T_BL, URDF_G2, CALIB, VALID, EXTRA
prep, floor_json, g2_json, out_json = Path(ARGS[0]), ARGS[1], ARGS[2], ARGS[3]
GROUND = -(0.082 + 0.0605)        # URDF: drive wheel axle z=-0.082 m, wheel radius 0.0605 m

fr = json.load(open(floor_json)); g2 = json.load(open(g2_json))
assert not set(fr['poses']) & set(VALID), 'hold-out poses must not be used for the floor'
n_mean = np.mean([v['normal'] for v in fr['per_pose'].values()], 0); n_mean /= np.linalg.norm(n_mean)
axis = np.cross(n_mean, [0, 0, 1.]); R_d = Rotation.from_rotvec(axis / np.linalg.norm(axis) * math.asin(np.linalg.norm(axis))).as_matrix()
s_mid = T_BL.copy()
def tilt(P): return (R_d @ (P - s_mid).T).T + s_mid

raw, tilted = {}, {}
for n in CALIB + VALID + EXTRA:
    z = np.load(prep / f'{n}.npz')
    raw[n] = (z['L'], dict(tree=cKDTree(z['pts']), pts=z['pts'], nrm=z['nrm']))
    pts = tilt(z['pts']); nrm = (R_d @ z['nrm'].T).T
    tilted[n] = (z['L'], dict(tree=cKDTree(pts), pts=pts, nrm=nrm))
p_untilted = np.radians(0) + np.array(g2.get('relative_untilted_xy_yaw', [0.024248171845592452, 0.00915943726488699, math.radians(2.836695401595045)]))
p_rel, _, _ = fit([tilted[n] for n in CALIB], p_untilted)

def T4(R, t): M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M
z_l = URDF_G2['z']
T_bpp_l = T4(Rotation.from_euler('z', p_rel[2]).as_matrix(), [p_rel[0], p_rel[1], z_l])        # base'' <- laser
T_bpp_m = T4(R_d @ R_BL, s_mid)                                                                 # base'' <- livox
T_b_l = np.array(g2['matrix_4x4'])
T_b_m = T_b_l @ np.linalg.inv(T_bpp_l) @ T_bpp_m
d = np.array([v['sensor_to_plane_mm'] for v in fr['per_pose'].values()]) / 1000
T_b_m[2, 3] = GROUND + d.mean()

# hold-out evaluation with the final deployed transforms
T_cad = T4(R_BL, T_BL); A = T_b_m @ np.linalg.inv(T_cad)      # CAD base -> final base, for Mid-360S points
p_g2 = [T_b_l[0, 3], T_b_l[1, 3], math.atan2(T_b_l[1, 0], T_b_l[0, 0])]
def final_map(n):
    L, M = raw[n]; pts = (A[:3, :3] @ M['pts'].T).T + A[:3, 3]; nrm = (A[:3, :3] @ M['nrm'].T).T
    return (L, dict(tree=cKDTree(pts), pts=pts, nrm=nrm))
final = {n: final_map(n) for n in raw}
ev = lambda data, p: stats(data, p)
res = dict(
    floor_poses=fr['poses'], floor_normal_cad_base=n_mean.tolist(),
    tilt_rpy_deg=np.degrees(Rotation.from_matrix(R_d).as_euler('xyz')).tolist(),
    g2_relative_tilt_corrected=dict(x_mm=p_rel[0] * 1000, y_mm=p_rel[1] * 1000, yaw_deg=math.degrees(p_rel[2])),
    per_pose_relative={n: dict(zip(('x_mm', 'y_mm', 'yaw_deg'), (lambda v: (v[0] * 1000, v[1] * 1000, math.degrees(v[2])))(fit([tilted[n]], p_rel)[0]))) for n in raw},
    holdout=dict(untilted_cad=ev([raw[n] for n in VALID], p_untilted), tilted_refit=ev([tilted[n] for n in VALID], p_rel),
                 final_deployed=ev([final[n] for n in VALID], p_g2),
                 final_deployed_per_pose={n: ev([final[n]], p_g2) for n in VALID + EXTRA}),
    train_final_deployed=ev([final[n] for n in CALIB], p_g2),
    six_dof_diag=dict(zip(('x_mm', 'y_mm', 'z_mm', 'roll_deg', 'pitch_deg', 'yaw_deg'),
                          (lambda v: (v[0] * 1000, v[1] * 1000, v[2] * 1000, math.degrees(v[3]), math.degrees(v[4]), math.degrees(v[5])))(
                              fit([tilted[n] for n in CALIB], [p_rel[0], p_rel[1], z_l, 0, 0, p_rel[2]], six=True)[0]))),
    z=dict(method='ground (URDF) + mean Mid-360S origin to floor-plane distance', ground_z_m=GROUND,
           sensor_to_plane_mm=(d * 1000).tolist(), mean_mm=float(d.mean() * 1000), std_mm=float(d.std(ddof=1) * 1000)),
    base_livox=T_b_m.tolist(), base_livox_rpy_deg=Rotation.from_matrix(T_b_m[:3, :3]).as_euler('xyz', degrees=True).tolist(),
)
json.dump(res, open(out_json, 'w'), indent=2, default=float)
e = res['base_livox_rpy_deg']; h = res['holdout']
print(f"floor poses {fr['poses']} tilt rpy {np.round(res['tilt_rpy_deg'],3).tolist()}")
print(f"G2 relative (tilt corrected): {res['g2_relative_tilt_corrected']}")
print(f"base<-livox xyz mm {np.round(T_b_m[:3,3]*1000,2).tolist()} rpy {np.round(e,3).tolist()} | z from {np.round(d*1000,1).tolist()} (std {d.std(ddof=1)*1000:.1f} mm)")
for k in ('untilted_cad', 'tilted_refit', 'final_deployed'):
    print(f"holdout {k:15s}: med {h[k]['med_cm']*10:.2f} mm p90 {h[k]['p90_cm']*10:.2f} mm in20mm {h[k]['in2cm']*100:.1f}% matched {h[k]['matched']*100:.0f}%")
print('per pose final:', {n: round(v['med_cm'] * 10, 2) for n, v in h['final_deployed_per_pose'].items()}, '| 6DoF diag', {k: round(v, 2) for k, v in res['six_dof_diag'].items()})
