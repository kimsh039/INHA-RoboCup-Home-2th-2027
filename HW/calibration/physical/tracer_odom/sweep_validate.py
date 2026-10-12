#!/usr/bin/env python3
"""Validate recorded odometry topics against Mid-360S ground truth on an odom_speed_sweep.py session.

Usage (ROS 2 Humble): python3 sweep_validate.py <bag_without_lidar> <mid_npz> <out_json>

Compares, per stop-to-stop segment, the pose change of /odom (driver as recorded), /odom_fused (wheel v +
IMU yaw), /odom_ekf (robot_localization, if recorded), and the firmware twist from /tracer_status integrated without correction and with the v3 model,
with the 3-DoF Mid-360S ICP motion between still clouds (sweep_extract_mid.py, already in base_link).
"""
import sys, json, math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from tracer_msgs.msg import TracerStatus
BAG, MIDNPZ, OUT = sys.argv[1:4]
bag = str(Path(BAG).resolve())

def se2(x, y, th): c, s = math.cos(th), math.sin(th); return np.array([[c, -s, x], [s, c, y], [0, 0, 1.]])
def inv(T): o = np.eye(3); o[:2, :2] = T[:2, :2].T; o[:2, 2] = -T[:2, :2].T @ T[:2, 2]; return o
def yaw(T): return math.atan2(T[1, 0], T[0, 0])
def wrap(a): return math.atan2(math.sin(a), math.cos(a))

r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/odom', '/odom_fused', '/odom_ekf', '/tracer_status']))
topics = {'/odom': [], '/odom_fused': [], '/odom_ekf': []}; fw = []
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/tracer_status':
        m = deserialize_message(d, TracerStatus); fw.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.linear_velocity, m.angular_velocity))
    else:
        m = deserialize_message(d, Odometry); q = m.pose.pose.orientation
        topics[t].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w), m.twist.twist.linear.x, m.twist.twist.angular.z))
od = {k: np.array(v) for k, v in topics.items()}; fw = np.array(fw)
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
    return p, float(k.mean())
def pose_at(a, t):
    k = min(max(np.searchsorted(a[:, 0], t), 0), len(a) - 1); return se2(*a[k, 1:4])
def integ(t, v, w):
    dt = np.diff(t, append=t[-1]); th = np.concatenate([[0.0], np.cumsum(w * dt)])
    return se2(float(np.sum(v * np.cos(th[:-1]) * dt)), float(np.sum(v * np.sin(th[:-1]) * dt)), float(th[-1]))
def v3(v, w):
    return np.where(np.abs(v) > 1e-3, v + 0.00756 * np.sign(v), v), np.where(np.abs(w) > 0.06, w + 0.0527 * np.sign(w), w)

rows = []
for (a0, b0, k0), (a1, b1, k1) in zip(still[:-1], still[1:]):
    t0, t1 = (a0 + b0) / 2, (a1 + b1) / 2
    To = inv(pose_at(od['/odom'], t0)) @ pose_at(od['/odom'], t1)
    p, fr = icp3(mid['c' + k1].astype(float), mid['c' + k0].astype(float), [To[0, 2], To[1, 2], yaw(To)])
    G = se2(*p); k = (fw[:, 0] >= t0) & (fw[:, 0] < t1)
    cand = {'driver /odom (v4)': To, 'fused /odom_fused': inv(pose_at(od['/odom_fused'], t0)) @ pose_at(od['/odom_fused'], t1) if len(od['/odom_fused']) else None,
            'EKF /odom_ekf': inv(pose_at(od['/odom_ekf'], t0)) @ pose_at(od['/odom_ekf'], t1) if len(od['/odom_ekf']) else None,
            'firmware raw': integ(fw[k, 0], fw[k, 1], fw[k, 2]), 'v3 (offline)': integ(fw[k, 0], *v3(fw[k, 1], fw[k, 2]))}
    mv = (np.abs(fw[k, 1]) > 1e-3) | (np.abs(fw[k, 2]) > 1e-3)
    rot = abs(p[2]) > math.radians(20); sp = float(np.median(np.abs(fw[k, 2] if rot else fw[k, 1])[mv])) if mv.any() else 0.0
    row = dict(kind='rot' if rot else 'str', fw_speed=sp, mid=dict(x=p[0], y=p[1], yaw=p[2], match=fr), err={})
    for name, T in cand.items():
        if T is None: continue
        row['err'][name] = dict(trans_mm=math.hypot(T[0, 2] - G[0, 2], T[1, 2] - G[1, 2]) * 1000, yaw_deg=math.degrees(wrap(yaw(T) - p[2])),
                                dist_ratio=math.hypot(T[0, 2], T[1, 2]) / max(math.hypot(*p[:2]), 1e-6))
    rows.append(row)
LV = {'rot': [0.11, 0.26, 0.44, 0.73], 'str': [0.09, 0.19, 0.34, 0.48]}
lev = lambda r: min(LV[r['kind']], key=lambda x: abs(x - r['fw_speed']))
names = list(rows[0]['err'])
summary = {}
print(f"Mid ICP match {np.mean([r['mid']['match'] for r in rows])*100:.0f}% mean, {len(rows)} segments\n")
print(f"{'level':10s} " + ' | '.join(f'{n:>20s}' for n in names))
for kind in ['rot', 'str']:
    for L in LV[kind]:
        R = [r for r in rows if r['kind'] == kind and lev(r) == L]
        if not R: continue
        cells = []
        for n in names:
            yr = float(np.sqrt(np.mean([r['err'][n]['yaw_deg'] ** 2 for r in R]))); tr = float(np.sqrt(np.mean([r['err'][n]['trans_mm'] ** 2 for r in R])))
            summary.setdefault(n, {})[f'{kind} {L}'] = dict(n=len(R), yaw_rms_deg=yr, trans_rms_mm=tr)
            cells.append(f'{yr:5.2f}° {tr:6.1f} mm')
        print(f"{kind} {L:.2f}   " + ' | '.join(f'{c:>20s}' for c in cells))
print('\noverall RMS (rotations: yaw per 90 deg turn; straights: position per 1 m)')
for n in names:
    Rr = [r for r in rows if r['kind'] == 'rot']; Rs = [r for r in rows if r['kind'] == 'str']; Rs2 = [r for r in Rs if lev(r) <= 0.19]
    o = dict(rot_yaw_rms_deg=float(np.sqrt(np.mean([r['err'][n]['yaw_deg'] ** 2 for r in Rr]))),
             str_trans_rms_mm=float(np.sqrt(np.mean([r['err'][n]['trans_mm'] ** 2 for r in Rs]))),
             str_slow_trans_rms_mm=float(np.sqrt(np.mean([r['err'][n]['trans_mm'] ** 2 for r in Rs2]))),
             str_yaw_rms_deg=float(np.sqrt(np.mean([r['err'][n]['yaw_deg'] ** 2 for r in Rs]))))
    summary[n]['overall'] = o
    print(f"  {n:20s} rot yaw {o['rot_yaw_rms_deg']:5.2f}°  straight {o['str_trans_rms_mm']:6.1f} mm (<=0.19 m/s: {o['str_slow_trans_rms_mm']:5.1f} mm)  straight yaw {o['str_yaw_rms_deg']:4.2f}°")
json.dump(dict(bag=Path(bag).name, summary=summary, segments=rows), open(OUT, 'w'), indent=2)
