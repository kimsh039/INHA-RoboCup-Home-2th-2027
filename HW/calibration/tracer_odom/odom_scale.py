# 사용: python3 odom_scale.py <bag_root> <base_g2_motion.json> <out_odom_scale.json>  (tracer_msgs 필요)
# Tracer odom 배율 보정: 정지 구간 사이의 스캔 기반 base 운동 vs odom(펌웨어 v, w 적분) / 모터 RPM
import math, json, numpy as np
import sys
from pathlib import Path
ARGS = sys.argv[1:]
sys.argv = [sys.argv[0], ARGS[0]]          # base_g2_motion.py reads the bag root from argv[1]
src = (Path(__file__).resolve().parents[1] / 'base_2dlidar/tools/base_g2_motion.py').read_text().split('res = {}')[0]
exec(src)
import rosbag2_py
from rclpy.serialization import deserialize_message
from tracer_msgs.msg import TracerStatus
m = json.load(open(ARGS[1]))
pc = np.array(m['p_center_laser_mm']) / 1000                       # base 원점 (laser 좌표)
yb = math.radians(m['result']['yaw_deg'])
T_bl = se2(m['result']['x_mm'] / 1000, m['result']['y_mm'] / 1000, yb)   # base <- laser

def status(name):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=B + name, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    out = []
    while r.has_next():
        t, d, ts = r.read_next()
        if t == '/tracer_status':
            s = deserialize_message(d, TracerStatus)
            out.append((ts * 1e-9, s.linear_velocity, s.angular_velocity, s.actuator_states[0].rpm, s.actuator_states[1].rpm))
    return np.array(out)

rows = []
for name in ['spin_cw', 'spin_ccw', 'straight_forward', 'straight_reverse']:
    od, sc = read(name); st = stops(od, sc); poses, _ = chain(st, name)
    S = status(name)
    for i in range(1, len(st)):
        # 스캔 기반 base 상대운동: base_{i-1} <- base_i
        Tb = T_bl @ inv(poses[i - 1]) @ poses[i] @ inv(T_bl)
        d_scan = math.hypot(Tb[0, 2], Tb[1, 2]); th_scan = yaw(Tb)
        Td = inv(st[i - 1]['odom']) @ st[i]['odom']
        d_odom = math.hypot(Td[0, 2], Td[1, 2]); th_odom = yaw(Td)
        # 같은 구간의 모터 RPM 적분 (각 모터 회전수)
        t0, t1 = st[i - 1]['t'], st[i]['t']; k = (S[:, 0] >= t0) & (S[:, 0] <= t1)
        dt = np.diff(S[k, 0], prepend=S[k, 0][0]) if k.any() else np.zeros(0)
        rev0 = float(np.sum(S[k, 3] * dt) / 60); rev1 = float(np.sum(S[k, 4] * dt) / 60)
        rows.append((name, d_scan, th_scan, d_odom, th_odom, rev0, rev1))

R = np.array([r[1:] for r in rows]); names = [r[0] for r in rows]
spin = np.array([n.startswith('spin') for n in names]); strt = ~spin
k_w = np.sum(np.abs(R[spin, 1])) / np.sum(np.abs(R[spin, 3]))
k_v = np.sum(R[strt, 0]) / np.sum(R[strt, 2])
print('\n구간별 (스캔 vs odom):')
for n, r in zip(names, R):
    print(f'  {n:17s} dist {r[0]*1000:7.1f} / {r[2]*1000:7.1f} mm | yaw {math.degrees(r[1]):+7.2f} / {math.degrees(r[3]):+7.2f}° | motor rev {r[4]:+.3f} {r[5]:+.3f}')
for nm in ['spin_cw', 'spin_ccw']:
    s = np.array([n == nm for n in names]); print(f'  k_w[{nm}] = {np.sum(np.abs(R[s,1]))/np.sum(np.abs(R[s,3])):.4f}  (구간별 {np.round(np.abs(R[s,1])/np.maximum(np.abs(R[s,3]),1e-9),3).tolist()})')
for nm in ['straight_forward', 'straight_reverse']:
    s = np.array([n == nm for n in names]); print(f'  k_v[{nm}] = {np.sum(R[s,0])/np.sum(R[s,2]):.4f}  (구간별 {np.round(R[s,0]/np.maximum(R[s,2],1e-9),3).tolist()})')
print(f'\n=> 각속도 배율 k_w = {k_w:.4f}, 선속도 배율 k_v = {k_v:.4f}')
# 모터 회전수 기반 유효 바퀴 반지름·간격 (모터 0/1의 좌우·부호는 데이터로 판단)
a = R[strt]; b = R[spin]
rev_lin = np.abs(a[:, 4] - a[:, 5]) / 2 if np.mean(np.sign(a[:, 4]) != np.sign(a[:, 5])) > 0.5 else np.abs(a[:, 4] + a[:, 5]) / 2
rev_rot = np.abs(b[:, 4] + b[:, 5]) / 2 if np.mean(np.sign(a[:, 4]) != np.sign(a[:, 5])) > 0.5 else np.abs(b[:, 4] - b[:, 5]) / 2
if rev_lin.sum() > 0 and rev_rot.sum() > 0:
    r_eff = np.sum(a[:, 0]) / (2 * math.pi * np.sum(rev_lin))                    # m per wheel radian
    track = 2 * r_eff * 2 * math.pi * np.sum(rev_rot) / np.sum(np.abs(b[:, 1]))  # b = 2 r Δφ / Δθ
    print(f'모터 RPM 기준(기어비 포함 등가): 유효 반지름 {r_eff*1000:.2f} mm/rad-of-motor, 유효 바퀴 간격 {track*1000:.1f} mm (URDF 바퀴 간격 340 mm)')
json.dump(dict(k_w=k_w, k_v=k_v, segments=[dict(bag=n, scan_dist_m=r[0], scan_yaw_rad=r[1], odom_dist_m=r[2], odom_yaw_rad=r[3], motor_rev=[r[4], r[5]]) for n, r in zip(names, R)]),
          open(ARGS[2], 'w'), indent=2)
