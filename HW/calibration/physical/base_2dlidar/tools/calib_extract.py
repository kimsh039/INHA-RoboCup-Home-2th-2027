#!/usr/bin/env python3
"""Per still period of a calib_drive.py bag: Mid-360S cloud (sensor frame), G2 median scan, IMU gravity, odom.

Usage (ROS 2 Humble, on the robot): python3 calib_extract.py <bag_dir> <out_npz>

Still periods come from /odom twist (|v| < 0.004, |w| < 0.008 for >= 1.5 s, 0.5 s settle). Mid-360S frames in a
still period are merged in livox_frame (0.3-30 m) and voxel-downsampled to 3 cm; the G2 scan is the per-beam
median (beams valid in >= 60 % of scans). Nothing is transformed with a calibration here.
"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from sensor_msgs_py import point_cloud2
BAG, OUT = sys.argv[1:3]
def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r
st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
od = []; r = reader(['/odom'])
while r.has_next():
    _, d, _ = r.read_next(); m = deserialize_message(d, Odometry); od.append((st(m), m.twist.twist.linear.x, m.twist.twist.angular.z))
od = np.array(od); still = (np.abs(od[:, 1]) < 0.004) & (np.abs(od[:, 2]) < 0.008)
wins, i = [], 0
while i < len(od):
    if still[i]:
        j = i
        while j + 1 < len(od) and still[j + 1]: j += 1
        if od[j, 0] - od[i, 0] >= 1.5: wins.append((od[i, 0] + 0.5, od[j, 0] - 0.1))
        i = j + 1
    else: i += 1
win_of = lambda t: next((n for n, (a, b) in enumerate(wins) if a <= t <= b), None)
mid = [[] for _ in wins]; scans = [[] for _ in wins]; acc = [[] for _ in wins]; meta = None
r = reader(['/livox/lidar', '/scan', '/livox/imu'])
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/livox/lidar':
        m = deserialize_message(d, PointCloud2); n = win_of(st(m))
        if n is None: continue
        a = point_cloud2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True); mid[n].append(np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64))
    elif t == '/scan':
        m = deserialize_message(d, LaserScan); n = win_of(st(m))
        if n is None: continue
        scans[n].append(np.array(m.ranges, float)); meta = (m.angle_min, m.angle_increment)
    else:
        m = deserialize_message(d, Imu); n = win_of(st(m))
        if n is None: continue
        acc[n].append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
res = dict(windows=np.array(wins), scan_angle=np.array(meta))
for n in range(len(wins)):
    P = np.vstack(mid[n]); rg = np.linalg.norm(P, axis=1); P = P[(rg > 0.3) & (rg < 30)]
    _, idx = np.unique(np.floor(P / 0.03).astype(np.int64), axis=0, return_index=True)
    S = np.array(scans[n]); S[~np.isfinite(S) | (S <= 0)] = np.nan
    ok = np.sum(np.isfinite(S), 0) >= 0.6 * len(S); med = np.where(ok, np.nanmedian(np.where(ok, S, np.nan), 0), np.nan)
    res[f'mid{n:03d}'] = P[idx].astype(np.float32); res[f'scan{n:03d}'] = med; res[f'imu{n:03d}'] = np.mean(acc[n], 0)
    print(f'{n:2d} {wins[n][1] - wins[n][0]:4.1f} s  Mid {len(mid[n])} frames {len(idx)} pts  G2 {len(scans[n])} scans {int(np.isfinite(med).sum())} beams  IMU {len(acc[n])}')
np.savez_compressed(OUT, **res)
