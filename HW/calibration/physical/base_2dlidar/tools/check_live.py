#!/usr/bin/env python3
"""Live check of the deployed base <- G2 and base <- Mid-360S calibration while the robot stands still.

Usage (on the robot, drivers running): python3 check_live.py <T_base_lidar.csv> <T_base_mid360.csv> [seconds]

Collects /scan and /livox/lidar for a few seconds (aborts if /odom shows motion) and reports:
  1. G2 -> Mid-360S vertical-plane distance (median, p90, signed mean) with the deployed transforms, and the small
     x / y / yaw correction that would minimise it (should be ~0)
  2. Mid-360S floor plane in base_link: height at the base origin (URDF floor -0.1425 m) and roll / pitch (should be ~0)
"""
import sys, time, math
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
T_bl = np.loadtxt(sys.argv[1], delimiter=','); T_bm = np.loadtxt(sys.argv[2], delimiter=',')
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
GROUND = -(0.082 + 0.0605)
S, M, moving = [], [], [False]
rclpy.init(); n = rclpy.create_node('calib_check_live')
n.create_subscription(LaserScan, '/scan', lambda m: S.append(m), qos_profile_sensor_data)
n.create_subscription(PointCloud2, '/livox/lidar', lambda m: M.append(m), qos_profile_sensor_data)
def od(m):
    if abs(m.twist.twist.linear.x) > 1e-3 or abs(m.twist.twist.angular.z) > 1e-3: moving[0] = True
n.create_subscription(Odometry, '/odom', od, 10)
t0 = time.time()
while time.time() - t0 < DUR: rclpy.spin_once(n, timeout_sec=0.05)
if moving[0]: sys.exit('robot moved during the check; keep it still and run again')
print(f'{len(S)} scans, {len(M)} Mid-360S frames in {DUR:.0f} s (robot still)')
# G2: per-beam median, to base_link
R = np.array([s.ranges for s in S], float); R[~np.isfinite(R) | (R <= 0)] = np.nan
ok = np.sum(np.isfinite(R), 0) >= 0.6 * len(R); r = np.where(ok, np.nanmedian(np.where(ok, R, np.nan), 0), np.nan)
a = S[0].angle_min + np.arange(len(r)) * S[0].angle_increment; k = np.isfinite(r) & (r > 0.6) & (r < 8)
L = np.c_[r[k] * np.cos(a[k]), r[k] * np.sin(a[k]), np.zeros(k.sum())]; G = L @ T_bl[:3, :3].T + T_bl[:3, 3]
# Mid-360S: merged frames to base_link
P = np.vstack([np.stack([p['x'], p['y'], p['z']], 1) for p in (point_cloud2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True) for m in M)]).astype(float)
P = P[(np.linalg.norm(P, axis=1) > 0.3) & (np.linalg.norm(P, axis=1) < 12)]; B = P @ T_bm[:3, :3].T + T_bm[:3, 3]
_, idx = np.unique(np.floor(B / 0.03).astype(np.int64), axis=0, return_index=True); B = B[idx]
# 1. G2 vs vertical Mid planes around the G2 height
band = B[np.abs(B[:, 2] - T_bl[2, 3]) < 0.25]
_, nn = cKDTree(band).query(band, k=12); Q = band[nn] - band[nn].mean(1, keepdims=True)
w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', Q, Q)); nrm = v[:, :, 0]; flat = (w[:, 0] / (w.sum(1) + 1e-12) < 0.03) & (np.abs(nrm[:, 2]) < 0.2)
D2 = band[flat, :2]; N2 = nrm[flat, :2] / np.linalg.norm(nrm[flat, :2], axis=1, keepdims=True); tree = cKDTree(D2)
def tf(p, X): c, s = math.cos(p[2]), math.sin(p[2]); return X @ np.array([[c, -s], [s, c]]).T + p[:2]
d, i = tree.query(G[:, :2], distance_upper_bound=0.2); m = np.isfinite(d)
res = np.einsum('ij,ij->i', G[m, :2] - D2[i[m]], N2[i[m]])
p = np.zeros(3)
for gate in (0.1, 0.05, 0.03, 0.03):
    d, i = tree.query(tf(p, G[:, :2]), distance_upper_bound=gate); mm = np.isfinite(d); X, Dq, Nq = G[mm, :2], D2[i[mm]], N2[i[mm]]
    p = least_squares(lambda q: np.einsum('ij,ij->i', tf(q, X) - Dq, Nq), p, loss='huber', f_scale=0.01).x
print(f'1) G2 -> Mid-360S planes: {m.sum()} matched of {len(G)} G2 points; |distance| median {np.median(np.abs(res))*1000:.1f} mm, '
      f'p90 {np.percentile(np.abs(res), 90)*1000:.1f} mm, within 2 cm {np.mean(np.abs(res) < 0.02)*100:.0f} %')
print(f'   correction that would fit best: dx {p[0]*1000:+.1f} mm, dy {p[1]*1000:+.1f} mm, dyaw {math.degrees(p[2]):+.3f} deg (should be ~0)')
# 2. floor
rr = np.hypot(B[:, 0], B[:, 1]); C = B[(np.abs(B[:, 2] - GROUND) < 0.12) & (rr > 0.8) & (rr < 3.0)]
rng = np.random.default_rng(0); best = (0, None)
for _ in range(400):
    q3 = C[rng.choice(len(C), 3, replace=False)]; nv = np.cross(q3[1] - q3[0], q3[2] - q3[0])
    if np.linalg.norm(nv) < 1e-9: continue
    nv /= np.linalg.norm(nv); kk = np.abs((C - q3[0]) @ nv) < 0.012
    if kk.sum() > best[0]: best = (kk.sum(), kk)
F = C[best[1]]; c0 = F.mean(0); nf = np.linalg.svd(F - c0, full_matrices=False)[2][2]; nf = nf if nf[2] > 0 else -nf
h0 = c0[2] - (nf[0] * (0 - c0[0]) + nf[1] * (0 - c0[1])) / nf[2]
print(f'2) floor (Mid-360S, {len(F)} pts): height at base origin {h0*1000:.1f} mm (URDF floor {GROUND*1000:.1f}, diff {(h0-GROUND)*1000:+.1f} mm); '
      f'tilt roll {math.degrees(math.atan2(nf[1], nf[2])):+.2f} deg, pitch {math.degrees(math.atan2(-nf[0], nf[2])):+.2f} deg (includes the room floor slope)')
