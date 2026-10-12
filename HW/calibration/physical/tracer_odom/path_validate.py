#!/usr/bin/env python3
"""Analyse an odom_path_test.py session: yaw drift while still, arcs, continuous loops (Mid-360S ground truth).

Usage (ROS 2 Humble): python3 path_validate.py <bag_without_lidar> <mid_npz> <out_json>

bag: /odom /odom_fused /odom_ekf /livox/imu (mid_npz from sweep_extract_mid.py on the full bag)
Segments between consecutive still periods are labelled in plan order (still, 16 arcs, CCW loops, CW loops).
"""
import sys, json, math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tracer_msgs.msg import TracerStatus
BAG, MIDNPZ, OUT = sys.argv[1:4]
HERE = Path(__file__).resolve().parent
R_BM = np.loadtxt(HERE.parent / 'base_mid360/T_base_mid360.csv', delimiter=',')[:3, :3]
TOP = ['/odom', '/odom_fused', '/odom_ekf']
NAMES = {'/odom': 'driver v4', '/odom_fused': 'fusion node', '/odom_ekf': 'EKF'}

def se2(x, y, th): c, s = math.cos(th), math.sin(th); return np.array([[c, -s, x], [s, c, y], [0, 0, 1.]])
def inv(T): o = np.eye(3); o[:2, :2] = T[:2, :2].T; o[:2, 2] = -T[:2, :2].T @ T[:2, 2]; return o
def yaw(T): return math.atan2(T[1, 0], T[0, 0])
def wrap(a): return math.atan2(math.sin(a), math.cos(a))

r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=str(Path(BAG).resolve()), storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=TOP + ['/livox/imu', '/tracer_status']))
od = {k: [] for k in TOP}; imu = []; mode = []
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/tracer_status':
        m = deserialize_message(d, TracerStatus); mode.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.control_mode))
    elif t == '/livox/imu':
        m = deserialize_message(d, Imu); g = m.angular_velocity; imu.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, *(R_BM @ [g.x, g.y, g.z])))
    else:
        m = deserialize_message(d, Odometry); q = m.pose.pose.orientation
        od[t].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w)))
od = {k: np.array(v) for k, v in od.items()}; imu = np.array(imu); mode = np.array(mode)
rc = lambda a, b: bool(np.any(mode[(mode[:, 0] >= a) & (mode[:, 0] <= b), 1] == 3))   # remote control took over
for k in od: od[k][:, 3] = np.unwrap(od[k][:, 3])
def pose_at(a, t):
    k = min(max(np.searchsorted(a[:, 0], t), 0), len(a) - 1); return se2(*a[k, 1:4]), a[k, 3]
mid = np.load(MIDNPZ); still = sorted([(float(mid[k][0]), float(mid[k][1]), k[1:]) for k in mid.files if k.startswith('t')])

def normals3(P, k=12):
    _, nn = cKDTree(P).query(P, k=k); Q = P[nn] - P[nn].mean(1, keepdims=True)
    w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', Q, Q)); return v[:, :, 0], w[:, 0] / (w.sum(1) + 1e-12)
def R3(p): c, s = math.cos(p[2]), math.sin(p[2]); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]]), np.array([p[0], p[1], 0.])
def icp3(src, dst, p0, gates=(0.6, 0.3, 0.15, 0.08, 0.05, 0.03, 0.03)):
    n, curv = normals3(dst); good = (curv < 0.05) & (np.abs(n[:, 2]) < 0.9); D, N = dst[good], n[good]; tree = cKDTree(D); p = np.array(p0, float)
    S = src[np.random.default_rng(0).choice(len(src), min(25000, len(src)), replace=False)]
    for g in gates:
        R, t = R3(p); d, i = tree.query(S @ R.T + t, distance_upper_bound=g); k = np.isfinite(d); s, q, nn = S[k], D[i[k]], N[i[k]]
        sol = least_squares(lambda x: np.einsum('ij,ij->i', s @ R3(x)[0].T + R3(x)[1] - q, nn), p, loss='huber', f_scale=0.01); p = sol.x
    return p, float(k.mean()), float(np.median(np.abs(sol.fun)) * 1000)

# ---------- 1. long still: yaw drift and gyro bias stability ----------
a0, b0, _ = max(still, key=lambda s: s[1] - s[0])
drift = dict(duration_s=b0 - a0)
for k in TOP:
    _, y0 = pose_at(od[k], a0); _, y1 = pose_at(od[k], b0); drift[NAMES[k]] = dict(yaw_change_deg=math.degrees(y1 - y0), rate_deg_per_min=math.degrees(y1 - y0) / (b0 - a0) * 60)
ki = (imu[:, 0] >= a0) & (imu[:, 0] <= b0); gz = imu[ki, 3]; ti = imu[ki, 0]
w30 = 30.0; first = gz[ti < a0 + w30].mean(); last = gz[ti > b0 - w30].mean()
drift['raw_gyro_bias'] = dict(first_30s=float(first), last_30s=float(last), change_rad_s=float(last - first),
                              yaw_if_bias_frozen_deg=float(math.degrees(np.sum((gz - first) * np.diff(ti, append=ti[-1])))))
print(f"1) still {drift['duration_s']:.0f} s:")
for k in TOP: print(f"   {NAMES[k]:12s} yaw change {drift[NAMES[k]]['yaw_change_deg']:+7.3f} deg ({drift[NAMES[k]]['rate_deg_per_min']:+.3f} deg/min)")
print(f"   raw gyro z bias {first:+.5f} -> {last:+.5f} rad/s (change {last - first:+.5f}); yaw if the start bias were frozen: {drift['raw_gyro_bias']['yaw_if_bias_frozen_deg']:+.2f} deg")

# ---------- 2/3. segments ----------
rows = []
segs = [(s0, s1) for s0, s1 in zip(still[:-1], still[1:])]
segs = [s for s in segs if s[0][1] >= b0 - 1e-6]           # after the long still
for n, ((x0, y0_, k0), (x1, y1_, k1)) in enumerate(segs):
    t0, t1 = (x0 + y0_) / 2, (x1 + y1_) / 2
    if rc(t0, t1): print(f'   segment {n}: remote control (mode 3) active, skipped'); continue
    Te = inv(pose_at(od['/odom_ekf'], t0)[0]) @ pose_at(od['/odom_ekf'], t1)[0]
    p, fr, md = icp3(mid['c' + k1].astype(float), mid['c' + k0].astype(float), [Te[0, 2], Te[1, 2], yaw(Te)])
    turn = math.degrees(pose_at(od['/odom_ekf'], t1)[1] - pose_at(od['/odom_ekf'], t0)[1]); dur = t1 - t0
    lab = (f'loop {"CCW" if turn > 0 else "CW"} {abs(turn) / 360:.1f} laps' if abs(turn) > 300 else f'arc {"fwd" if p[0] > 0 else "back"} {"L" if p[2] * (1 if p[0] > 0 else -1) > 0 else "R"} {math.hypot(p[0], p[1]):.2f} m')
    G = se2(*p); row = dict(label=lab, duration_s=dur, t0=t0, t1=t1, mid=dict(x=p[0], y=p[1], yaw_deg=math.degrees(p[2]), match=fr, med_mm=md), err={})
    for k in TOP:
        (P0, Y0), (P1, Y1) = pose_at(od[k], t0), pose_at(od[k], t1); T = inv(P0) @ P1
        path = float(np.sum(np.hypot(*np.diff(od[k][(od[k][:, 0] >= t0) & (od[k][:, 0] <= t1), 1:3], axis=0).T)))
        row['err'][NAMES[k]] = dict(trans_mm=math.hypot(T[0, 2] - G[0, 2], T[1, 2] - G[1, 2]) * 1000,
                                    yaw_deg=math.degrees(wrap(yaw(T) - p[2])), total_turn_deg=math.degrees(Y1 - Y0), path_m=path)
    rows.append(row)
print('\n2) arcs and 3) loops (error vs Mid-360S):')
print(f"   {'segment':22s} {'truth dx/dy mm, yaw':>26s} | " + ' | '.join(f'{NAMES[k]:>20s}' for k in TOP) + '   Mid match')
for rw in rows:
    m = rw['mid']; cells = ' | '.join(f"{rw['err'][NAMES[k]]['trans_mm']:7.1f} mm {rw['err'][NAMES[k]]['yaw_deg']:+6.2f}°" for k in TOP)
    print(f"   {rw['label']:22s} {m['x']*1000:+8.1f} {m['y']*1000:+8.1f} {m['yaw_deg']:+7.2f}° | {cells}   {m['match']*100:3.0f}%")
summ = {}
for grp, sel in [('arcs', lambda r: not r['label'].startswith('loop')), ('loops', lambda r: r['label'].startswith('loop'))]:
    R = [r for r in rows if sel(r)]
    if not R: continue
    summ[grp] = {NAMES[k]: dict(trans_rms_mm=float(np.sqrt(np.mean([r['err'][NAMES[k]]['trans_mm'] ** 2 for r in R]))),
                                yaw_rms_deg=float(np.sqrt(np.mean([r['err'][NAMES[k]]['yaw_deg'] ** 2 for r in R]))),
                                path_m=float(np.mean([r['err'][NAMES[k]]['path_m'] for r in R]))) for k in TOP}
    print(f'   {grp} RMS: ' + ' | '.join(f"{n}: {v['trans_rms_mm']:.1f} mm {v['yaw_rms_deg']:.2f}° (path {v['path_m']:.2f} m)" for n, v in summ[grp].items()))
json.dump(dict(bag=Path(BAG).name, still_drift=drift, segments=rows, summary=summ), open(OUT, 'w'), indent=2, default=float)
