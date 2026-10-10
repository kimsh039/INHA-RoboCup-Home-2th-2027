# Mid-360S 기울기(바닥 평면)를 반영해 G2<->Mid-360S 상대값을 다시 풀고, G2 운동 기하 결과와 이어 Mid-360S 6자유도를 구한다.
import sys, json, math, numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
sys.argv = ['x', 'none', 'none']
src = open('/mnt/c/Users/wwoo5/Desktop/INHA-RoboCup-Home-2th-2027/HW/calibration/base_2dlidar/vs_mid360/tools/calib_g2_livox.py').read().split("if BAGDIR == 'prep':")[0]
exec(src)
OUT = '/home/wwoo5241/livox_g2_calib/'
fr = json.load(open(OUT + 'floor_real.json')); mo = json.load(open(OUT + 'base_g2_motion.json'))
n_mean = np.mean([v['normal'] for v in fr['per_pose'].values()], 0); n_mean /= np.linalg.norm(n_mean)   # floor normal in base' (CAD Mid)
axis = np.cross(n_mean, [0, 0, 1.]); R_d = Rotation.from_rotvec(axis / np.linalg.norm(axis) * math.asin(np.linalg.norm(axis))).as_matrix()
s_mid = T_BL.copy()                                     # Mid-360S origin in base' (CAD)
def corr(P): return (R_d @ (P - s_mid).T).T + s_mid     # rotate cloud about the sensor origin
print('floor normal in base\':', n_mean.round(5).tolist(), '-> tilt correction', np.round(np.degrees(Rotation.from_matrix(R_d).as_euler('xyz')), 3).tolist(), 'deg')

poses = {}
for n in CALIB + VALID + EXTRA:
    z = np.load(OUT + f'prep/{n}.npz')
    pts = corr(z['pts']); nrm = (R_d @ z['nrm'].T).T
    poses[n] = (z['L'], dict(tree=cKDTree(pts), pts=pts, nrm=nrm))
calib = [poses[n] for n in CALIB]; valid = [poses[n] for n in VALID]
p_urdf = [URDF_G2['x'], URDF_G2['y'], URDF_G2['yaw']]
p_old = [0.024248171845592452, 0.00915943726488699, math.radians(2.836695401595045)]
p3, sol3, _ = fit(calib, p_old)
print(f'\n[기울기 반영] G2 relative (base\'\' <- laser): x={p3[0]*1000:.2f} y={p3[1]*1000:.2f} mm yaw={math.degrees(p3[2]):.3f} deg   (이전: 24.25, 9.16, 2.837)')
for tag, data in [('train', calib), ('holdout', valid)]:
    a = stats(data, p_old); b = stats(data, p3)
    print(f'  {tag}: 이전 값으로 med={a["med_cm"]*10:.2f} mm p90={a["p90_cm"]*10:.2f} | 다시 푼 값 med={b["med_cm"]*10:.2f} mm p90={b["p90_cm"]*10:.2f} mm in2cm={b["in2cm"]*100:.0f}%')
pp = {n: fit([poses[n]], p3)[0] for n in poses}
for n, v in pp.items(): print(f'   {n:20s} x={v[0]*1000:6.1f} y={v[1]*1000:6.1f} yaw={math.degrees(v[2]):6.3f}')
p6, sol6, _ = fit(calib, [p3[0], p3[1], URDF_G2['z'], 0, 0, p3[2]], six=True)
print(f'  6DoF 진단: z={p6[2]*1000:.1f} roll={math.degrees(p6[3]):.2f} pitch={math.degrees(p6[4]):.2f} deg (이전 진단 pitch -2.91)')

def T4(R, t): M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M
T_bpp_l = T4(Rotation.from_euler('z', p3[2]).as_matrix(), [p3[0], p3[1], 0.01611 + 0.3266])   # base'' <- laser
T_bpp_m = T4(R_d @ R_BL, s_mid)                                                                 # base'' <- livox
r = mo['result']
T_b_l = T4(Rotation.from_euler('z', math.radians(r['yaw_deg'])).as_matrix(), [r['x_mm'] / 1000, r['y_mm'] / 1000, 0.01611 + 0.3266])
T_b_m = T_b_l @ np.linalg.inv(T_bpp_l) @ T_bpp_m
# z: 바닥(보정 후)이 base_link 아래 URDF 바닥 높이(-0.1425 m)에 오도록
GROUND = -(0.082 + 0.0605)
fl_pts = []
c_mean = np.mean([v['centroid_mm'] for v in fr['per_pose'].values()], 0) / 1000
c_b = (T_b_m @ np.linalg.inv(T_bpp_m) @ np.r_[corr(c_mean[None])[0], 1])[:3]
n_b = T_b_m[:3, :3] @ np.linalg.inv(T_bpp_m[:3, :3]) @ (R_d @ n_mean)
z0 = c_b[2] + (n_b[0] * c_b[0] + n_b[1] * c_b[1]) / n_b[2]
dz = GROUND - z0; T_b_m[2, 3] += dz
e = Rotation.from_matrix(T_b_m[:3, :3]).as_euler('xyz', degrees=True)
print(f'\n=== base_link <- livox_frame ===\n xyz mm = {np.round(T_b_m[:3,3]*1000,1).tolist()}  rpy deg = {np.round(e,3).tolist()}')
print(f' vs CAD (-180, 0, 1199.1 mm; 180, 0, 0 deg): dx={(T_b_m[0,3]+0.18)*1000:+.1f} dy={T_b_m[1,3]*1000:+.1f} dz={(T_b_m[2,3]-1.19911)*1000:+.1f} mm')
print(f' 바닥 높이 보정 dz={dz*1000:+.1f} mm, 바닥 법선(base) = {np.round(n_b,5).tolist()}')
print(f' 줄자: Mid-360S 1.355 m 위 -> base 기준 z={1.355+GROUND:.4f} m  (바닥으로 구한 값 {T_b_m[2,3]:.4f} m, 차이 {(T_b_m[2,3]-(1.355+GROUND))*1000:+.1f} mm)')
print(f' 줄자: G2 0.4811 m 위 -> base 기준 z={0.4811+GROUND:.4f} m (CAD 0.3427 m)')
json.dump(dict(R_tilt_rpy_deg=np.degrees(Rotation.from_matrix(R_d).as_euler('xyz')).tolist(), floor_normal_cad_base=n_mean.tolist(),
               g2_relative_tilt_corrected=dict(x_mm=p3[0] * 1000, y_mm=p3[1] * 1000, yaw_deg=math.degrees(p3[2])),
               train=stats(calib, p3), holdout=stats(valid, p3), holdout_old=stats(valid, p_old),
               per_pose={n: dict(x_mm=v[0] * 1000, y_mm=v[1] * 1000, yaw_deg=math.degrees(v[2])) for n, v in pp.items()},
               six_dof_diag=dict(z_mm=p6[2] * 1000, roll_deg=math.degrees(p6[3]), pitch_deg=math.degrees(p6[4])),
               base_livox=T_b_m.tolist(), base_livox_rpy_deg=e.tolist(), floor_dz_mm=dz * 1000),
          open(OUT + 'recompute_tilt.json', 'w'), indent=2, default=float)
