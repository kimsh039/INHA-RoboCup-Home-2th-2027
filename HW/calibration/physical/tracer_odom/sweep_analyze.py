#!/usr/bin/env python3
"""Analyse an odom_speed_sweep.py session: ground truth from G2 and Mid-360S, IMU gyro, firmware deficit per speed.

Usage (ROS 2 Humble): python3 sweep_analyze.py <bag_without_lidar> <mid_npz> <out_json>

bag: /scan /odom /tracer_status /livox/imu (sweep_extract_mid.py made <mid_npz> from the full bag)
Per stop-to-stop segment:
  G2   : base motion from scan_utils.chain (base_link <- laser_frame of ../base_2dlidar)
  Mid  : 3-DoF point-to-plane ICP between still clouds already in base_link (T_base_mid360)
  IMU  : gyro rotated to base_link, bias from the surrounding still periods, integrated
  firmware: /tracer_status linear/angular velocity (uncorrected), integrated in SE(2)
"""
import sys, json, math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
BAG, MIDNPZ, OUT = sys.argv[1:4]
bag = Path(BAG).resolve()
sys.argv = [sys.argv[0], str(bag.parent)]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); import scan_utils; from scan_utils import se2, inv, yaw, read, stops, chain
scan_utils.B = str(bag.parent).rstrip('/') + '/'
cal = json.load(open(HERE.parent / 'base_2dlidar/calibration.json'))
T_bl = se2(cal['translation_m'][0], cal['translation_m'][1], math.radians(cal['yaw_deg']))
T_bm = np.loadtxt(HERE.parent / 'base_mid360/T_base_mid360.csv', delimiter=',')
EPS = 1e-3
import os
DEADBAND = os.environ.get('DEADBAND') == '1'
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from tracer_msgs.msg import TracerStatus

# ---------- read ----------
od, sc = read(bag.name)
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/livox/imu', '/tracer_status']))
imu, fw = [], []
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/livox/imu':
        m = deserialize_message(d, Imu); g = m.angular_velocity
        imu.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, g.x, g.y, g.z))
    else:
        m = deserialize_message(d, TracerStatus)
        fw.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.linear_velocity, m.angular_velocity, m.battery_voltage, m.control_mode))
imu = np.array(imu); fw = np.array(fw)
gz = (imu[:, 1:4] @ T_bm[:3, :3].T)[:, 2]                       # yaw rate about base_link z

# ---------- stops, G2 ----------
st = stops(od, sc); poses, q = chain(st, bag.name)
mid = np.load(MIDNPZ)
mid_t = sorted([(float(mid[k][0]), float(mid[k][1]), k[1:]) for k in mid.files if k.startswith('t')])
def mid_cloud(t):
    for a, b, k in mid_t:
        if a - 0.6 <= t <= b + 0.6: return mid['c' + k].astype(np.float64)
    return None

# ---------- Mid 3-DoF ICP ----------
def normals3(P, k=12):
    _, nn = cKDTree(P).query(P, k=k); Q = P[nn] - P[nn].mean(1, keepdims=True)
    w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', Q, Q)); return v[:, :, 0], w[:, 0] / (w.sum(1) + 1e-12)
def se2_3(p):
    c, s = math.cos(p[2]), math.sin(p[2]); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]]), np.array([p[0], p[1], 0.])
def icp3(src, dst, p0, gates=(0.6, 0.3, 0.15, 0.08, 0.05, 0.03, 0.03)):
    n, curv = normals3(dst); good = (curv < 0.05) & (np.abs(n[:, 2]) < 0.9)     # planar, not floor/ceiling
    D, N = dst[good], n[good]; tree = cKDTree(D); p = np.array(p0, float)
    rng = np.random.default_rng(0); S = src[rng.choice(len(src), min(25000, len(src)), replace=False)]
    for g in gates:
        R, t = se2_3(p); d, i = tree.query(S @ R.T + t, distance_upper_bound=g); k = np.isfinite(d)
        s, qq, nn = S[k], D[i[k]], N[i[k]]
        f = lambda x: np.einsum('ij,ij->i', s @ se2_3(x)[0].T + se2_3(x)[1] - qq, nn)
        sol = least_squares(f, p, loss='huber', f_scale=0.01); p = sol.x
    return p, float(k.mean()), float(np.median(np.abs(sol.fun)) * 1000)

rows = []
for i in range(1, len(st)):
    Tg = T_bl @ inv(poses[i - 1]) @ poses[i] @ inv(T_bl)                    # base_{i-1} <- base_i from G2
    A, Bc = mid_cloud(st[i - 1]['t']), mid_cloud(st[i]['t'])
    pm, fm, mm = icp3(Bc, A, [Tg[0, 2], Tg[1, 2], yaw(Tg)]) if A is not None and Bc is not None else ([np.nan] * 3, 0, np.nan)
    t0, t1 = st[i - 1]['t'], st[i]['t']
    # firmware integration (SE(2), like the driver)
    k = np.where((fw[:, 0] >= t0) & (fw[:, 0] < t1))[0]; dt = np.diff(fw[k, 0], append=fw[k[-1], 0])
    v, w = fw[k, 1], fw[k, 2]
    # IMU: bias from the two still periods
    ki = (imu[:, 0] >= t0) & (imu[:, 0] < t1); still_b = [(imu[:, 0] >= a) & (imu[:, 0] <= b) for a, b, _ in mid_t if a - 0.6 <= t0 <= b + 0.6 or a - 0.6 <= t1 <= b + 0.6]
    bias = float(np.mean(np.concatenate([gz[m] for m in still_b]))) if still_b else 0.0
    dti = np.diff(imu[ki, 0], append=imu[ki, 0][-1]); yaw_imu = float(np.sum((gz[ki] - bias) * dti))
    moving = (np.abs(v) > EPS) | (np.abs(w) > EPS); Tm = float(np.sum(dt[moving]))
    rows.append(dict(i=i, t0=t0, t1=t1, v=v, w=w, dt=dt, T_move=Tm, ti=fw[k, 0], imu_t=imu[ki, 0], imu_gz=gz[ki] - bias,
                     g2=dict(x=Tg[0, 2], y=Tg[1, 2], yaw=yaw(Tg), match=q[i - 1][0], ref_match=q[i - 1][2]),
                     mid=dict(x=pm[0], y=pm[1], yaw=pm[2], match=fm, med_mm=mm), imu_yaw=yaw_imu, imu_bias=bias,
                     cmd_v=float(np.median(v[moving])) if moving.any() else 0.0, cmd_w=float(np.median(w[moving])) if moving.any() else 0.0))

def integ(v, w, dt):
    th = np.concatenate([[0.0], np.cumsum(w * dt)]); return float(np.sum(v * np.cos(th[:-1]) * dt)), float(np.sum(v * np.sin(th[:-1]) * dt)), float(th[-1])
print(f'\n{"#":>2} {"kind":4s} {"fw v/w med":>10s} | {"G2 d/yaw":>16s} {"Mid d/yaw":>16s} {"IMU yaw":>8s} | {"fw d/yaw":>16s} | G2-Mid mm/deg  match G2/Mid')
for r in rows:
    x, y, th = integ(r['v'], r['w'], r['dt']); r['fw'] = dict(x=x, y=y, yaw=th)
    rot = abs(r['mid']['yaw']) > math.radians(20); r['kind'] = 'rot' if rot else 'str'
    dg = math.hypot(r['g2']['x'], r['g2']['y']); dm = math.hypot(r['mid']['x'], r['mid']['y'])
    r['gm_diff_mm'] = math.hypot(r['g2']['x'] - r['mid']['x'], r['g2']['y'] - r['mid']['y']) * 1000; r['gm_diff_deg'] = math.degrees(r['g2']['yaw'] - r['mid']['yaw'])
    sp = r['cmd_w'] if rot else r['cmd_v']
    print(f"{r['i']:2d} {r['kind']:4s} {sp:+10.3f} | {dg*1000:7.1f} {math.degrees(r['g2']['yaw']):+8.2f} {dm*1000:7.1f} {math.degrees(r['mid']['yaw']):+8.2f} {math.degrees(r['imu_yaw']):+8.2f} | "
          f"{math.hypot(x, y)*1000:7.1f} {math.degrees(th):+8.2f} | {r['gm_diff_mm']:6.1f} {r['gm_diff_deg']:+6.2f}   {r['g2']['match']*100:3.0f}%/{r['mid']['match']*100:3.0f}%")

# ---------- ground truth = Mid (360°, long range); G2 as cross-check ----------
gm_mm = np.array([r['gm_diff_mm'] for r in rows]); gm_deg = np.array([r['gm_diff_deg'] for r in rows])
imu_vs_mid = np.array([math.degrees(r['imu_yaw'] - r['mid']['yaw']) for r in rows if r['kind'] == 'rot'])
imu_scale = sum(abs(r['mid']['yaw']) for r in rows if r['kind'] == 'rot') / sum(abs(r['imu_yaw']) for r in rows if r['kind'] == 'rot')
print(f'\nG2 vs Mid: translation diff median {np.median(gm_mm):.1f} / max {gm_mm.max():.1f} mm, yaw diff RMS {np.sqrt(np.mean(gm_deg**2)):.3f} deg')
print(f'IMU vs Mid (rotations): yaw diff RMS {np.sqrt(np.mean(imu_vs_mid**2)):.2f} deg, Mid/IMU total ratio {imu_scale:.4f}')

# ---------- firmware deficit per speed level ----------
levels = {}
for r in rows:
    if r['kind'] == 'rot':
        true, rep = r['mid']['yaw'], r['fw']['yaw']; key = ('rot', round(abs(r['cmd_w']), 2))
    else:
        true = r['mid']['x']; rep = r['fw']['x']; key = ('str', round(abs(r['cmd_v']), 2))
    r['ratio'] = rep / true; r['deficit'] = (true - rep) / r['T_move'] * np.sign(true)
    levels.setdefault(key, []).append(r)
print('\nfirmware vs Mid per speed level (rep/true, deficit per moving second):')
lev_out = []
for key in sorted(levels):
    R = levels[key]; u = 'rad/s' if key[0] == 'rot' else 'm/s'
    rat = [x['ratio'] for x in R]; de = [x['deficit'] for x in R]
    lev_out.append(dict(kind=key[0], fw_speed=key[1], n=len(R), ratio_mean=float(np.mean(rat)), ratio_std=float(np.std(rat)), deficit_mean=float(np.mean(de)), deficit_std=float(np.std(de)),
                        pos=[x['ratio'] for x in R if x['mid']['yaw' if key[0] == 'rot' else 'x'] > 0], neg=[x['ratio'] for x in R if x['mid']['yaw' if key[0] == 'rot' else 'x'] < 0]))
    print(f"  {key[0]} fw {key[1]:.2f} {u}: n={len(R)} rep/true {np.mean(rat):.4f} ± {np.std(rat):.4f}  deficit {np.mean(de):+.4f} ± {np.std(de):.4f} {u}   (+dir {np.mean(lev_out[-1]['pos']):.4f}, -dir {np.mean(lev_out[-1]['neg']):.4f})")

# ---------- models, rep-1 vs rep-2 cross-validation ----------
def correct(r, p, tw):
    kv, cv, kw, cw = p; v, w = r['v'], r['w']
    vc = np.where(np.abs(v) > 0.0035, kv * v + cv * np.sign(v), v); wc = np.where(np.abs(w) > tw, kw * w + cw * np.sign(w), w)
    if DEADBAND: wc = np.where((np.abs(w) <= 0.0186) & (np.abs(v) > 0.05), 0.0, wc)     # one-step yaw-rate blips while driving straight
    return vc, wc
def errs(R, p, tw):
    e = []
    for r in R:
        x, y, th = integ(*correct(r, p, tw), r['dt']); g = r['mid']
        e.append((math.hypot(x - g['x'], y - g['y']), math.atan2(math.sin(th - g['yaw']), math.cos(th - g['yaw']))))
    return np.array(e)
def resid(R, p, tw):
    e = []
    for r in R:
        x, y, th = integ(*correct(r, p, tw), r['dt']); g = r['mid']
        e += [x - g['x'], y - g['y'], 0.5 * math.atan2(math.sin(th - g['yaw']), math.cos(th - g['yaw']))]
    return np.array(e)
MODELS = {'none': ([], [1, 0, 1, 0]), 'offset': ([1, 3], [1, 0.0076, 1, 0.053]), 'scale': ([0, 2], [1.03, 0, 1.1, 0]), 'scale+offset': ([0, 1, 2, 3], [1.02, 0.005, 1.0, 0.04])}
def fit(R, model, tw):
    free, p0 = MODELS[model]; p0 = np.array(p0, float)
    if not free: return p0
    def f(x): p = p0.copy(); p[free] = x; return resid(R, p, tw)
    s = least_squares(f, p0[free], loss='soft_l1', f_scale=0.02); p = p0.copy(); p[free] = s.x; return p
# split: within each (level, direction) the first occurrence -> A, second -> B
seen = {}; A, Bs = [], []
for r in rows:
    key = (r['kind'], round(abs(r['cmd_w'] if r['kind'] == 'rot' else r['cmd_v']), 2), np.sign(r['mid']['yaw'] if r['kind'] == 'rot' else r['mid']['x']))
    (A if seen.get(key, 0) == 0 else Bs).append(r); seen[key] = seen.get(key, 0) + 1
def summ(R, p, tw):
    E = errs(R, p, tw); kr = np.array([r['kind'] == 'rot' for r in R])
    return dict(rot_yaw_rms_deg=float(np.degrees(np.sqrt(np.mean(E[kr, 1] ** 2)))), str_trans_rms_mm=float(np.sqrt(np.mean(E[~kr, 0] ** 2)) * 1000),
                str_yaw_rms_deg=float(np.degrees(np.sqrt(np.mean(E[~kr, 1] ** 2)))), rot_trans_rms_mm=float(np.sqrt(np.mean(E[kr, 0] ** 2)) * 1000))
model_out = {}
print('\nmodels (fit on repetition 1, test on repetition 2, and reverse; then fit on all):')
for tw in [0.02, 0.06]:
    for model in MODELS:
        pA, pB, pall = fit(A, model, tw), fit(Bs, model, tw), fit(rows, model, tw)
        tA, tB = summ(Bs, pA, tw), summ(A, pB, tw)
        model_out[f'{model}|tw={tw}'] = dict(fit_rep1=pA.tolist(), test_rep2=tA, fit_rep2=pB.tolist(), test_rep1=tB, fit_all=pall.tolist(), all=summ(rows, pall, tw))
        print(f"  {model:13s} tw={tw:.2f} all-fit kv {pall[0]:.4f} cv {pall[1]:+.5f} kw {pall[2]:.4f} cw {pall[3]:+.4f} | "
              f"test rot {tA['rot_yaw_rms_deg']:.2f}/{tB['rot_yaw_rms_deg']:.2f}°  str {tA['str_trans_rms_mm']:.1f}/{tB['str_trans_rms_mm']:.1f} mm  str-yaw {tA['str_yaw_rms_deg']:.2f}/{tB['str_yaw_rms_deg']:.2f}°")

# ---------- direction-aware lookup table (deficit vs |speed|, linear interpolation, flat outside) ----------
LW = np.array(sorted({round(abs(r['cmd_w']), 2) for r in rows if r['kind'] == 'rot'})); LV = np.array(sorted({round(abs(r['cmd_v']), 2) for r in rows if r['kind'] == 'str'}))
LW = np.array([LW[0]] + [x for a, x in zip(LW, LW[1:]) if x - a > 0.03]); LV = np.array([LV[0]] + [x for a, x in zip(LV, LV[1:]) if x - a > 0.03])
def lut_correct(r, q, tw=0.02):
    nw, nv = len(LW), len(LV); dwp, dwn, dvp, dvn = q[:nw], q[nw:2 * nw], q[2 * nw:2 * nw + nv], q[2 * nw + nv:]
    v, w = r['v'], r['w']
    dw = np.where(w > 0, np.interp(np.abs(w), LW, dwp), np.interp(np.abs(w), LW, dwn)); dv = np.where(v > 0, np.interp(np.abs(v), LV, dvp), np.interp(np.abs(v), LV, dvn))
    return np.where(np.abs(v) > 0.0035, v + np.sign(v) * dv, v), np.where(np.abs(w) > tw, w + np.sign(w) * dw, w)
def lut_resid(R, q):
    e = []
    for r in R:
        x, y, th = integ(*lut_correct(r, q), r['dt']); g = r['mid']
        e += [x - g['x'], y - g['y'], 0.5 * math.atan2(math.sin(th - g['yaw']), math.cos(th - g['yaw']))]
    return np.array(e)
def lut_fit(R):
    q0 = np.r_[np.full(2 * len(LW), 0.03), np.full(2 * len(LV), 0.01)]
    return least_squares(lambda q: lut_resid(R, q), q0, loss='soft_l1', f_scale=0.02).x
def lut_summ(R, q):
    out = {}
    for r in R:
        x, y, th = integ(*lut_correct(r, q), r['dt']); g = r['mid']
        key = f"{r['kind']} {abs(r['cmd_w'] if r['kind'] == 'rot' else r['cmd_v']):.2f}"
        out.setdefault(key, []).append((math.hypot(x - g['x'], y - g['y']) * 1000, math.degrees(math.atan2(math.sin(th - g['yaw']), math.cos(th - g['yaw'])))))
    return {k: dict(trans_rms_mm=float(np.sqrt(np.mean([a[0] ** 2 for a in v]))), yaw_rms_deg=float(np.sqrt(np.mean([a[1] ** 2 for a in v])))) for k, v in sorted(out.items())}
qA, qB, qall = lut_fit(A), lut_fit(Bs), lut_fit(rows)
tA, tB = lut_summ(Bs, qA), lut_summ(A, qB)
print('\nlookup table (direction-aware), test per level  [fit rep1 -> test rep2 | fit rep2 -> test rep1]:')
for k in tA: print(f"  {k}: trans {tA[k]['trans_rms_mm']:6.1f} / {tB.get(k, {}).get('trans_rms_mm', float('nan')):6.1f} mm   yaw {tA[k]['yaw_rms_deg']:5.2f} / {tB.get(k, {}).get('yaw_rms_deg', float('nan')):5.2f} deg")
nw = len(LW); print('  all-fit nodes  w', LW.tolist(), '+', np.round(qall[:nw], 4).tolist(), '-', np.round(qall[nw:2 * nw], 4).tolist())
print('                 v', LV.tolist(), '+', np.round(qall[2 * nw:2 * nw + len(LV)], 4).tolist(), '-', np.round(qall[2 * nw + len(LV):], 4).tolist())
# same per-level table for the best parametric model, for comparison
for model, tw in [('offset', 0.06), ('scale+offset', 0.02)]:
    pA = fit(A, model, tw); pB = fit(Bs, model, tw)
    def per_level(R, p):
        o = {}
        for r in R:
            x, y, th = integ(*correct(r, p, tw), r['dt']); g = r['mid']; key = f"{r['kind']} {abs(r['cmd_w'] if r['kind'] == 'rot' else r['cmd_v']):.2f}"
            o.setdefault(key, []).append((math.hypot(x - g['x'], y - g['y']) * 1000, math.degrees(math.atan2(math.sin(th - g['yaw']), math.cos(th - g['yaw'])))))
        return {k: (float(np.sqrt(np.mean([a[0] ** 2 for a in v]))), float(np.sqrt(np.mean([a[1] ** 2 for a in v])))) for k, v in sorted(o.items())}
    a, b = per_level(Bs, pA), per_level(A, pB)
    print(f'  [{model} tw={tw}] ' + '  '.join(f"{k}: {a[k][0]:.0f}/{b.get(k, (np.nan,))[0]:.0f}mm {a[k][1]:.2f}/{b.get(k, (0, np.nan))[1]:.2f}��" for k in a))
lut_out = dict(levels_w=LW.tolist(), levels_v=LV.tolist(), fit_all=qall.tolist(), test_rep2=tA, test_rep1=tB)
# ---------- wheel speed + IMU yaw (what the EKF will do): yaw from gyro, distance from corrected wheel v ----------
def fused(r, p):
    vc, _ = correct(r, p, 0.02)
    th_imu = np.concatenate([[0.0], np.cumsum(r['imu_gz'] * np.diff(r['imu_t'], append=r['imu_t'][-1]))])[:-1]
    th = np.interp(r['ti'], r['imu_t'], th_imu); th_end = float(th_imu[-1] + r['imu_gz'][-1] * 0) if len(th_imu) else 0.0
    th_end = float(np.sum(r['imu_gz'] * np.diff(r['imu_t'], append=r['imu_t'][-1])))
    return float(np.sum(vc * np.cos(th) * r['dt'])), float(np.sum(vc * np.sin(th) * r['dt'])), th_end
fus_out = {}
print('\nwheel v (corrected) + IMU gyro yaw, per level  [linear fit rep1 -> test rep2 | rep2 -> rep1]:')
for name, pfn in [('fw v raw', lambda R: np.array([1, 0, 1, 0.])), ('v: scale+offset', lambda R: fit(R, 'scale+offset', 0.02))]:
    pA, pB = pfn(A), pfn(Bs)
    def lv(R, p):
        o = {}
        for r in R:
            x, y, th = fused(r, p); g = r['mid']; key = f"{r['kind']} {abs(r['cmd_w'] if r['kind'] == 'rot' else r['cmd_v']):.2f}"
            o.setdefault(key, []).append((math.hypot(x - g['x'], y - g['y']) * 1000, math.degrees(math.atan2(math.sin(th - g['yaw']), math.cos(th - g['yaw'])))))
        return {k: (float(np.sqrt(np.mean([a[0] ** 2 for a in v]))), float(np.sqrt(np.mean([a[1] ** 2 for a in v])))) for k, v in sorted(o.items())}
    a, b = lv(Bs, pA), lv(A, pB); fus_out[name] = dict(test_rep2=a, test_rep1=b)
    print(f'  [{name}] ' + '  '.join(f"{k}: {a[k][0]:.0f}/{b[k][0]:.0f}mm {a[k][1]:.2f}/{b[k][1]:.2f}��" for k in a))
clean = lambda r: {k: (v if not isinstance(v, np.ndarray) else None) for k, v in r.items() if k not in ('v', 'w', 'dt', 'ti', 'imu_t', 'imu_gz')}
json.dump(dict(bag=bag.name, battery_v=[float(fw[:, 3].min()), float(fw[:, 3].max())], control_modes=sorted(set(fw[:, 4].astype(int).tolist())),
               g2_vs_mid=dict(trans_median_mm=float(np.median(gm_mm)), trans_max_mm=float(gm_mm.max()), yaw_rms_deg=float(np.sqrt(np.mean(gm_deg ** 2)))),
               imu_vs_mid=dict(yaw_rms_deg=float(np.sqrt(np.mean(imu_vs_mid ** 2))), mid_over_imu=imu_scale),
               levels=lev_out, models=model_out, lookup=lut_out, fused=fus_out, segments=[clean(r) for r in rows]), open(OUT, 'w'), indent=2, default=float)
