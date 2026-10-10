# Mid-360S(기준) <-> G2 정지 다자세 외부 파라미터 추정
# - Mid-360S의 base_link 자세는 URDF 값으로 고정
# - G2: base_link -> laser_frame 의 x, y, yaw 추정 (z, roll, pitch는 URDF 고정) + 6DoF 관측성 진단
# 사용: python3 calib_g2_livox.py <bag_dir> <out_json>
import sys, os, json, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from sensor_msgs_py import point_cloud2 as pc2
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

BAGDIR, OUT = sys.argv[1], sys.argv[2]
CALIB = ['pose01', 'pose02', 'pose03', 'pose05', 'pose06', 'pose07']
VALID = ['pose04', 'pose08']
EXTRA = ['trial_pose01_first']

# URDF (tracer_sensor_rack_piper.urdf): base_link->rack_base_link z=0.01611
R_BL = R.from_euler('xyz', [np.pi, 0, 0]).as_matrix()          # base_link -> livox_frame
T_BL = np.array([-0.18, 0.0, 0.01611 + 1.183])
URDF_G2 = dict(x=0.0001749995366, y=0.0, z=0.01611 + 0.3266, roll=0.0, pitch=0.0, yaw=0.1537539717)


def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    clouds, scans, acc = [], [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/livox/lidar':
            a = pc2.read_points(deserialize_message(data, PointCloud2), field_names=('x', 'y', 'z'), skip_nans=True)
            clouds.append(np.c_[a['x'], a['y'], a['z']].astype(np.float64))
        elif topic == '/scan':
            scans.append(deserialize_message(data, LaserScan))
        elif topic == '/livox/imu':
            m = deserialize_message(data, Imu)
            acc.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    return clouds, scans, np.array(acc)


def voxel(P, s=0.01):
    k = np.floor(P / s).astype(np.int64)
    k -= k.min(0)
    h = (k[:, 0] * 73856093) ^ (k[:, 1] * 19349663) ^ (k[:, 2] * 83492791)
    _, idx = np.unique(h, return_index=True)
    return P[idx]


def build_map(clouds):
    P = np.vstack(clouds)
    P = P[np.linalg.norm(P, axis=1) > 0.3]
    P = (R_BL @ P.T).T + T_BL                     # base_link
    P = voxel(P, 0.015)
    floor = P[(np.abs(P[:, 2]) < 0.08) & (np.hypot(P[:, 0], P[:, 1]) > 0.8) & (np.hypot(P[:, 0], P[:, 1]) < 3.0)]
    band = P[(P[:, 2] > 0.13) & (P[:, 2] < 0.56) & (np.hypot(P[:, 0], P[:, 1]) < 6.5)]
    n_all = len(P); del P
    tree = cKDTree(band)
    normal = np.empty_like(band); planar = np.zeros(len(band), bool)
    for s in range(0, len(band), 50000):           # 메모리 절약: 나눠서 PCA (이웃 40점, 반경 ~6cm)
        b = band[s:s + 50000]
        _, nn = tree.query(b, k=40)
        Q = band[nn]; Q -= Q.mean(1, keepdims=True)
        w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', Q, Q) / 40)
        normal[s:s + len(b)] = v[:, :, 0]
        planar[s:s + len(b)] = (w[:, 0] / w.sum(1) < 0.03) & (np.abs(v[:, 2, 0]) < 0.5)   # 평면 + 대체로 수직면
    pts, nrm = band[planar], normal[planar]
    return dict(tree=cKDTree(pts), pts=pts, nrm=nrm, floor=floor, n_all=n_all, n_band=len(band))


def scan_points(scans):
    lens = {len(s.ranges) for s in scans}
    s0 = scans[0]
    if len(lens) == 1:
        Rg = np.array([s.ranges for s in scans], dtype=float)
        Rg[~np.isfinite(Rg) | (Rg <= 0)] = np.nan
        valid = np.sum(np.isfinite(Rg), 0) >= 0.5 * len(scans)
        rr = np.nanmedian(np.where(valid, Rg, np.nan), 0)
        aa = s0.angle_min + np.arange(len(rr)) * s0.angle_increment
    else:
        rr = np.concatenate([np.array(s.ranges, float) for s in scans])
        aa = np.concatenate([s.angle_min + np.arange(len(s.ranges)) * s.angle_increment for s in scans])
    ok = np.isfinite(rr) & (rr > 0.7) & (rr < 6.0)
    return np.c_[rr[ok] * np.cos(aa[ok]), rr[ok] * np.sin(aa[ok]), np.zeros(ok.sum())], len(lens) == 1


def T_from(p, six):
    if six:
        x, y, z, ro, pi, ya = p
    else:
        x, y, ya = p; z, ro, pi = URDF_G2['z'], URDF_G2['roll'], URDF_G2['pitch']
    return R.from_euler('xyz', [ro, pi, ya]).as_matrix(), np.array([x, y, z])


def match(data, p, six, dmax):
    out = []
    for L, M in data:
        Rm, t = T_from(p, six)
        X = (Rm @ L.T).T + t
        d, i = M['tree'].query(X, distance_upper_bound=dmax)
        k = np.isfinite(d)
        out.append((L[k], M['pts'][i[k]], M['nrm'][i[k]]))
    return out


def residuals(p, corr, six):
    Rm, t = T_from(p, six)
    return np.concatenate([np.einsum('ij,ij->i', ((Rm @ L.T).T + t) - q, n) for L, q, n in corr])


def fit(data, p0, six=False):
    p = np.array(p0, float)
    for dmax in [0.30, 0.20, 0.10, 0.10, 0.05, 0.05, 0.05, 0.03, 0.03]:
        corr = match(data, p, six, dmax)
        sol = least_squares(residuals, p, args=(corr, six), loss='huber', f_scale=0.01)
        p = sol.x
    return p, sol, corr


def stats(data, p, six=False):
    corr = match(data, p, six, 0.20)
    r = np.abs(residuals(p, corr, six))
    tot = sum(len(L) for L, _ in data)
    return dict(n=int(tot), matched=float(len(r) / max(tot, 1)), med_cm=float(np.median(r) * 100),
                p90_cm=float(np.percentile(r, 90) * 100), in2cm=float(np.mean(r < 0.02)))


def floor_check(F):
    c = F.mean(0); _, _, vt = np.linalg.svd(F - c, full_matrices=False); n = vt[2] * np.sign(vt[2][2])
    return dict(n_pts=int(len(F)), height_mm=float(c[2] * 1000), tilt_deg=float(np.degrees(np.arccos(n[2]))),
                roll_deg=float(np.degrees(np.arctan2(n[1], n[2]))), pitch_deg=float(np.degrees(-np.arctan2(n[0], n[2]))))


def imu_check(acc):
    g = R_BL @ acc.mean(0)
    g = g / np.linalg.norm(g)
    return dict(n=int(len(acc)), g_base=g.round(4).tolist(), tilt_deg=float(np.degrees(np.arccos(abs(g[2])))),
                roll_deg=float(np.degrees(np.arctan2(g[1], g[2]))), pitch_deg=float(np.degrees(-np.arctan2(g[0], g[2]))))


if BAGDIR == 'prep':                                 # 자세 1개 전처리 → npz (프로세스 분리로 메모리 누수 회피)
    path, out = sys.argv[2], sys.argv[3]
    clouds, scans, acc = read_bag(path)
    n_clouds = len(clouds); M = build_map(clouds); del clouds
    L, fixed_len = scan_points(scans)
    fl, im = floor_check(M['floor']), imu_check(acc)
    np.savez(out, L=L, pts=M['pts'], nrm=M['nrm'], meta=json.dumps(dict(fl=fl, imu=im)))
    print(f"{os.path.basename(path)}: clouds={n_clouds} scans={len(scans)} (fixed_len={fixed_len}) g2_pts={len(L)} map all/band/planar={M['n_all']}/{M['n_band']}/{len(M['pts'])} | "
          f"floor h={fl['height_mm']:+.1f}mm tilt={fl['tilt_deg']:.2f}deg | imu tilt={im['tilt_deg']:.2f}deg", flush=True)
    sys.exit(0)

poses = {}
for name in CALIB + VALID + EXTRA:
    f = os.path.join(BAGDIR, name + '.npz')
    if not os.path.isfile(f):
        print('skip', name); continue
    z = np.load(f); meta = json.loads(str(z['meta']))
    poses[name] = dict(L=z['L'], M=dict(tree=cKDTree(z['pts']), pts=z['pts'], nrm=z['nrm']), fl=meta['fl'], imu=meta['imu'])

p_urdf = [URDF_G2['x'], URDF_G2['y'], URDF_G2['yaw']]
calib = [(poses[n]['L'], poses[n]['M']) for n in CALIB if n in poses]
valid = [(poses[n]['L'], poses[n]['M']) for n in VALID if n in poses]

p3, sol3, _ = fit(calib, p_urdf)
J = sol3.jac; r = sol3.fun
s = 1.4826 * np.median(np.abs(r))
cov = np.linalg.inv(J.T @ J) * s ** 2

per_pose = {}
for n in poses:
    pp, _, _ = fit([(poses[n]['L'], poses[n]['M'])], p3)
    per_pose[n] = dict(x_mm=pp[0] * 1000, y_mm=pp[1] * 1000, yaw_deg=np.degrees(pp[2]))
loo = []
for i in range(len(calib)):
    pp, _, _ = fit(calib[:i] + calib[i + 1:], p3)
    loo.append(pp)
loo = np.array(loo)

p6_0 = [p3[0], p3[1], URDF_G2['z'], 0.0, 0.0, p3[2]]
p6, sol6, _ = fit(calib, p6_0, six=True)
sc = np.array([0.1, 0.1, 0.1, 0.1745, 0.1745, 0.1745])
sv = np.linalg.svd(sol6.jac * sc, compute_uv=False)

res = dict(
    estimate_3dof=dict(x_mm=p3[0] * 1000, y_mm=p3[1] * 1000, yaw_deg=np.degrees(p3[2]),
                       sigma_formal=dict(x_mm=np.sqrt(cov[0, 0]) * 1000, y_mm=np.sqrt(cov[1, 1]) * 1000, yaw_deg=np.degrees(np.sqrt(cov[2, 2]))),
                       loo_spread=dict(x_mm=np.ptp(loo[:, 0]) * 1000, y_mm=np.ptp(loo[:, 1]) * 1000, yaw_deg=np.degrees(np.ptp(loo[:, 2])))),
    urdf=dict(x_mm=URDF_G2['x'] * 1000, y_mm=0.0, yaw_deg=np.degrees(URDF_G2['yaw'])),
    per_pose=per_pose,
    calib_stats=dict(urdf=stats(calib, p_urdf), est=stats(calib, p3)),
    valid_stats=dict(urdf=stats(valid, p_urdf), est=stats(valid, p3)),
    valid_per_pose={n: dict(urdf=stats([(poses[n]['L'], poses[n]['M'])], p_urdf), est=stats([(poses[n]['L'], poses[n]['M'])], p3)) for n in VALID + EXTRA if n in poses},
    six_dof_diag=dict(x_mm=p6[0] * 1000, y_mm=p6[1] * 1000, z_mm=p6[2] * 1000, roll_deg=np.degrees(p6[3]), pitch_deg=np.degrees(p6[4]), yaw_deg=np.degrees(p6[5]),
                      scaled_singular_values=sv.tolist(), condition=float(sv[0] / sv[-1])),
    mid360_floor={n: poses[n]['fl'] for n in poses},
    mid360_imu={n: poses[n]['imu'] for n in poses},
)
json.dump(res, open(OUT, 'w'), indent=2, ensure_ascii=False, default=float)

e = res['estimate_3dof']
print("\n=== 3DoF 추정 (base_link -> laser_frame, z/roll/pitch=URDF) ===")
print(f"URDF : x={res['urdf']['x_mm']:.1f}mm y=0.0mm yaw={res['urdf']['yaw_deg']:.3f}deg")
print(f"추정 : x={e['x_mm']:.1f}mm y={e['y_mm']:.1f}mm yaw={e['yaw_deg']:.3f}deg")
print(f"  형식 σ : x={e['sigma_formal']['x_mm']:.2f}mm y={e['sigma_formal']['y_mm']:.2f}mm yaw={e['sigma_formal']['yaw_deg']:.3f}deg")
print(f"  LOO 폭 : x={e['loo_spread']['x_mm']:.1f}mm y={e['loo_spread']['y_mm']:.1f}mm yaw={e['loo_spread']['yaw_deg']:.3f}deg")
print("\n자세별 단독 추정:")
for n, v in per_pose.items():
    tag = 'calib' if n in CALIB else ('VALID' if n in VALID else 'extra')
    print(f"  {n:20s} [{tag}] x={v['x_mm']:6.1f} y={v['y_mm']:6.1f} yaw={v['yaw_deg']:6.3f}")
for k in ['calib_stats', 'valid_stats']:
    u, s_ = res[k]['urdf'], res[k]['est']
    print(f"\n{k}: URDF med={u['med_cm']:.2f}cm p90={u['p90_cm']:.2f}cm in2cm={u['in2cm']*100:.0f}% matched={u['matched']*100:.0f}%  ->  "
          f"추정 med={s_['med_cm']:.2f}cm p90={s_['p90_cm']:.2f}cm in2cm={s_['in2cm']*100:.0f}% matched={s_['matched']*100:.0f}%")
for n, v in res['valid_per_pose'].items():
    print(f"  {n}: URDF med={v['urdf']['med_cm']:.2f}cm -> 추정 med={v['est']['med_cm']:.2f}cm p90={v['est']['p90_cm']:.2f}cm in2cm={v['est']['in2cm']*100:.0f}%")
d6 = res['six_dof_diag']
print(f"\n6DoF 진단: x={d6['x_mm']:.1f} y={d6['y_mm']:.1f} z={d6['z_mm']:.1f}mm roll={d6['roll_deg']:.2f} pitch={d6['pitch_deg']:.2f} yaw={d6['yaw_deg']:.3f}deg")
print(f"  scaled singular values (x,y,z,roll,pitch,yaw 혼합) = {np.round(sv, 2).tolist()}  condition={d6['condition']:.1f}")
