#!/usr/bin/env python3
"""G2 scan utilities for stop-and-go bags (imported by sweep_analyze.py).

read(name)       /odom and /scan of the rosbag2 folder B + name
stops(od, sc)    still periods (|v| < 0.004, |w| < 0.008 for >= 1 s): per-beam median scan, mean odom pose
chain(st, label) laser_0 <- laser_i by 2D point-to-line ICP (odometry only as the initial guess)
"""
import sys, json, math, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
B = './'                        # bag root; set scan_utils.B (with trailing /) before read()
RMIN, RMAX = 0.45, 8.0

def se2(x, y, th):
    c, s = np.cos(th), np.sin(th); return np.array([[c, -s, x], [s, c, y], [0, 0, 1.]])
def inv(T):
    o = np.eye(3); o[:2, :2] = T[:2, :2].T; o[:2, 2] = -T[:2, :2].T @ T[:2, 2]; return o
def apply(T, X): return X @ T[:2, :2].T + T[:2, 2]
def yaw(T): return math.atan2(T[1, 0], T[0, 0])

def read(name):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=B + name, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    od, sc = [], []
    while r.has_next():
        t, d, _ = r.read_next()
        if t == '/odom':
            m = deserialize_message(d, Odometry); q = m.pose.pose.orientation
            od.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                       2 * math.atan2(q.z, q.w), m.twist.twist.linear.x, m.twist.twist.angular.z))
        elif t == '/scan':
            m = deserialize_message(d, LaserScan)
            sc.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.array(m.ranges, float), m.angle_min, m.angle_increment))
    od = np.array(od); od[:, 3] = np.unwrap(od[:, 3])
    return od, sc

def stops(od, sc, settle=0.4, min_len=1.0):
    still = (np.abs(od[:, 4]) < 0.004) & (np.abs(od[:, 5]) < 0.008)
    segs, i = [], 0
    while i < len(od):
        if still[i]:
            j = i
            while j + 1 < len(od) and still[j + 1]: j += 1
            if od[j, 0] - od[i, 0] >= min_len: segs.append((od[i, 0] + settle, od[j, 0] - 0.1, i, j))
            i = j + 1
        else: i += 1
    out = []
    for t0, t1, i, j in segs:
        S = [s for s in sc if t0 <= s[0] <= t1]
        if len(S) < 5: continue
        Rg = np.array([s[1] for s in S]); Rg[~np.isfinite(Rg) | (Rg <= 0)] = np.nan
        ok = np.sum(np.isfinite(Rg), 0) >= 0.6 * len(S)
        rr = np.nanmedian(np.where(ok, Rg, np.nan), 0); a = S[0][2] + np.arange(len(rr)) * S[0][3]
        k = np.isfinite(rr) & (rr > RMIN) & (rr < RMAX)
        L = np.c_[rr[k] * np.cos(a[k]), rr[k] * np.sin(a[k])]
        o = od[i:j + 1].mean(0)
        out.append(dict(L=L, n_scans=len(S), odom=se2(o[1], o[2], o[3]), t=(t0 + t1) / 2))
    return out

def normals(L, k=7):
    _, nn = cKDTree(L).query(L, k=k); Q = L[nn] - L[nn].mean(1, keepdims=True)
    w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', Q, Q)); return v[:, :, 0], w[:, 0] / (w.sum(1) + 1e-12) < 0.05

def icp(src, dst, T0, gates=(0.3, 0.15, 0.08, 0.05, 0.03, 0.03, 0.02)):
    n, lin = normals(dst); D, N = dst[lin], n[lin]; tree = cKDTree(D); T = T0.copy()
    for g in gates:
        d, i = tree.query(apply(T, src), distance_upper_bound=g); k = np.isfinite(d)
        s, q, nn = src[k], D[i[k]], N[i[k]]
        p0 = [T[0, 2], T[1, 2], yaw(T)]
        sol = least_squares(lambda p: np.einsum('ij,ij->i', apply(se2(*p), s) - q, nn), p0, loss='huber', f_scale=0.01)
        T = se2(*sol.x)
    return T, float(k.mean()), float(np.median(np.abs(sol.fun)) * 1000)

def T_bl_guess():   # rough base_link <- laser_frame (CAD) to turn the odom initial guess into laser motion
    return se2(0.000175, 0.0, 0.15375)

def chain(st, label):
    """laser_0 <- laser_i: registered to the previous stop, then refined against the first stop"""
    Tb = T_bl_guess(); poses = [np.eye(3)]; q = []
    for i in range(1, len(st)):
        d_odom = inv(st[i - 1]['odom']) @ st[i]['odom']
        T0 = inv(Tb) @ d_odom @ Tb
        Ti, fr, md = icp(st[i]['L'], st[i - 1]['L'], T0)
        Tg = poses[-1] @ Ti
        Tg2, fr2, md2 = icp(st[i]['L'], st[0]['L'], Tg)          # refine directly against the first stop (limits drift)
        use = Tg2 if fr2 > 0.6 else Tg
        poses.append(use); q.append((fr, md, fr2, md2, math.degrees(yaw(d_odom)), math.degrees(yaw(Ti))))
    print(f'[{label}] stops={len(st)}  (previous-stop match / median mm, first-stop match / median mm, odom yaw, scan yaw)')
    for k, e in enumerate(q, 1):
        print(f'   {k:2d}: {e[0]*100:3.0f}%/{e[1]:4.1f}  {e[2]*100:3.0f}%/{e[3]:4.1f}  odom {e[4]:+6.1f}°  scan {e[5]:+6.1f}°')
    return poses, q

def center(Ts):
    A = np.vstack([np.hstack([T[:2, :2], -np.eye(2)]) for T in Ts]); b = np.concatenate([-T[:2, 2] for T in Ts])
    sol, *_ = np.linalg.lstsq(A, b, rcond=None); r = (A @ sol - b).reshape(-1, 2)
    return sol[:2], np.sqrt((r ** 2).sum(1))

