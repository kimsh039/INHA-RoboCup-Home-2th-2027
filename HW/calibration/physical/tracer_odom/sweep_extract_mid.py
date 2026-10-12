#!/usr/bin/env python3
"""Extract one Mid-360S cloud per still period of a stop-and-go bag (for odometry ground truth).

Usage (ROS 2 Humble, on the robot): python3 sweep_extract_mid.py <bag_dir> <T_base_mid360.csv> <out_npz>

Still periods are found from /odom twist like scan_utils.stops(). All /livox/lidar frames inside a
still period (after a 0.4 s settle) are merged, moved to base_link with T_base_mid360, cropped
(0.6-25 m, z -0.1..2.2 m above base_link) and voxel-downsampled to 4 cm.
"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
bag, tcsv, out = sys.argv[1:4]
T = np.loadtxt(tcsv, delimiter=',')
def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r
od = []
r = reader(['/odom'])
while r.has_next():
    _, d, _ = r.read_next(); m = deserialize_message(d, Odometry)
    od.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.twist.twist.linear.x, m.twist.twist.angular.z))
od = np.array(od); still = (np.abs(od[:, 1]) < 0.004) & (np.abs(od[:, 2]) < 0.008)
wins, i = [], 0
while i < len(od):
    if still[i]:
        j = i
        while j + 1 < len(od) and still[j + 1]: j += 1
        if od[j, 0] - od[i, 0] >= 1.0: wins.append((od[i, 0] + 0.4, od[j, 0] - 0.1))
        i = j + 1
    else: i += 1
print(len(wins), 'still periods')
acc = [[] for _ in wins]
r = reader(['/livox/lidar'])
while r.has_next():
    _, d, _ = r.read_next(); m = deserialize_message(d, PointCloud2); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    k = [n for n, (a, b) in enumerate(wins) if a <= t <= b]
    if not k: continue
    a = point_cloud2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
    p = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
    acc[k[0]].append(p)
res = {}
for n, (w, A) in enumerate(zip(wins, acc)):
    if not A: continue
    P = np.vstack(A); rng = np.linalg.norm(P, axis=1); P = P[(rng > 0.6) & (rng < 25)]
    Q = P @ T[:3, :3].T + T[:3, 3]; Q = Q[(Q[:, 2] > -0.1) & (Q[:, 2] < 2.2)]
    key = np.floor(Q / 0.04).astype(np.int64); _, idx = np.unique(key, axis=0, return_index=True)
    res[f'c{n:03d}'] = Q[idx].astype(np.float32); res[f't{n:03d}'] = np.array([w[0], w[1], len(A)])
    print(n, f'{w[0] - od[0, 0]:7.1f}s', len(A), 'frames', len(idx), 'pts')
np.savez_compressed(out, **res)
