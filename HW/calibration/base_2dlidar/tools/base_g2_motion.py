# 팀원 stop-and-go bag(G2 + odom)으로 base_link <- laser_frame 추정
# 사용 (ROS 2 Humble): python3 base_g2_motion.py <bag_root> <out_json>
#   bag_root 아래 spin_cw, spin_ccw, straight_forward, straight_reverse rosbag2 폴더
#  - 정지 구간마다 빔별 중앙값 스캔 (모션 왜곡 없음)
#  - 정지 스캔끼리 2D point-to-line ICP (odom은 초기값으로만 사용)
#  - spin: 회전 중심 = base 원점 -> laser 좌표의 base 위치 p
#  - straight: base 원점 이동 방향 -> yaw
import sys, json, math, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
B = (sys.argv[1].rstrip('/') + '/') if len(sys.argv) > 1 else './'
OUT_JSON = sys.argv[2] if len(sys.argv) > 2 else 'base_g2_motion.json'
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

def T_bl_guess():   # odom 초기값을 laser 운동으로 바꿀 때 쓰는 대략값 (CAD)
    return se2(0.000175, 0.0, 0.15375)

def chain(st, label):
    """laser_0 <- laser_i : 직전 정지에 정합 후, 누적값을 다시 기준(0) 및 직전 2개에 대해 정제"""
    Tb = T_bl_guess(); poses = [np.eye(3)]; q = []
    for i in range(1, len(st)):
        d_odom = inv(st[i - 1]['odom']) @ st[i]['odom']
        T0 = inv(Tb) @ d_odom @ Tb
        Ti, fr, md = icp(st[i]['L'], st[i - 1]['L'], T0)
        Tg = poses[-1] @ Ti
        Tg2, fr2, md2 = icp(st[i]['L'], st[0]['L'], Tg)          # 기준 스캔에 직접 정제 (드리프트 억제)
        use = Tg2 if fr2 > 0.6 else Tg
        poses.append(use); q.append((fr, md, fr2, md2, math.degrees(yaw(d_odom)), math.degrees(yaw(Ti))))
    print(f'[{label}] stops={len(st)}  (직전정합 대응/중앙mm, 기준정합 대응/중앙mm, odom회전, 스캔회전)')
    for k, e in enumerate(q, 1):
        print(f'   {k:2d}: {e[0]*100:3.0f}%/{e[1]:4.1f}  {e[2]*100:3.0f}%/{e[3]:4.1f}  odom {e[4]:+6.1f}°  scan {e[5]:+6.1f}°')
    return poses, q

def center(Ts):
    A = np.vstack([np.hstack([T[:2, :2], -np.eye(2)]) for T in Ts]); b = np.concatenate([-T[:2, 2] for T in Ts])
    sol, *_ = np.linalg.lstsq(A, b, rcond=None); r = (A @ sol - b).reshape(-1, 2)
    return sol[:2], np.sqrt((r ** 2).sum(1))

res = {}
for name in ['spin_cw', 'spin_ccw']:
    od, sc = read(name); st = stops(od, sc); poses, q = chain(st, name)
    p, r = center(poses)
    tot = math.degrees(sum(math.radians(e[5]) for e in q)); odt = math.degrees(sum(math.radians(e[4]) for e in q))
    # bootstrap (정지 자세 재표본)
    rng = np.random.default_rng(0); bs = []
    for _ in range(300):
        idx = rng.choice(len(poses), len(poses), replace=True)
        if len(set(idx)) < 4: continue
        bs.append(center([poses[k] for k in idx])[0])
    bs = np.array(bs) * 1000
    res[name] = dict(p_laser_mm=(p * 1000).tolist(), resid_mm_med=float(np.median(r) * 1000), resid_mm_max=float(r.max() * 1000),
                     scan_total_deg=tot, odom_total_deg=odt, n=len(poses), boot_std_mm=bs.std(0).tolist())
    print(f'  -> base 원점 (laser 좌표) = ({p[0]*1000:.1f}, {p[1]*1000:.1f}) mm | 원 맞춤 잔차 중앙 {np.median(r)*1000:.1f} / 최대 {r.max()*1000:.1f} mm | '
          f'bootstrap σ ({bs[:,0].std():.1f}, {bs[:,1].std():.1f}) mm | 총회전 스캔 {tot:+.1f}° vs odom {odt:+.1f}° (비 {odt/tot:.3f})')

pc = np.mean([res[n]['p_laser_mm'] for n in ['spin_cw', 'spin_ccw']], 0) / 1000
print(f'\n회전 중심 평균 (laser 좌표) = ({pc[0]*1000:.1f}, {pc[1]*1000:.1f}) mm, CW-CCW 차이 {np.linalg.norm(np.subtract(res["spin_cw"]["p_laser_mm"], res["spin_ccw"]["p_laser_mm"])):.1f} mm')

yaws = {}
for name, sign in [('straight_forward', 1), ('straight_reverse', -1)]:
    od, sc = read(name); st = stops(od, sc); poses, q = chain(st, name)
    phis, ws = [], []
    for i in range(1, len(poses)):
        Tij = inv(poses[i - 1]) @ poses[i]                       # laser_{i-1} <- laser_i
        d = apply(Tij, pc[None])[0] - pc                           # base 원점 이동 (laser_{i-1} 좌표)
        if np.linalg.norm(d) < 0.05: continue
        d *= sign; phis.append(math.atan2(d[1], d[0])); ws.append(np.linalg.norm(d))
    phis, ws = np.array(phis), np.array(ws)
    phi = math.atan2((ws * np.sin(phis)).sum(), (ws * np.cos(phis)).sum())
    # 시작->끝 전체 이동 방향 (heading 변화 보정: 시작 laser 좌표)
    Tend = poses[-1]; dtot = (apply(Tend, pc[None])[0] - pc) * sign
    yaws[name] = dict(yaw_deg=-math.degrees(phi), yaw_total_deg=-math.degrees(math.atan2(dtot[1], dtot[0])),
                      seg_std_deg=float(np.degrees(np.std(phis))), n_seg=len(phis), dist_m=float(np.linalg.norm(dtot)),
                      heading_change_deg=math.degrees(yaw(Tend)))
    print(f'  -> yaw(base<-laser) = {-math.degrees(phi):.3f}° (구간 {len(phis)}개, σ {np.degrees(np.std(phis)):.2f}°) | 전체 이동 기준 {yaws[name]["yaw_total_deg"]:.3f}° '
          f'| 이동 {yaws[name]["dist_m"]:.2f} m, 스캔 heading 변화 {yaws[name]["heading_change_deg"]:+.2f}°')

yb = math.radians(np.mean([yaws[n]['yaw_deg'] for n in yaws]))
Rb = np.array([[math.cos(yb), -math.sin(yb)], [math.sin(yb), math.cos(yb)]])
t = -(Rb @ pc)
print(f'\n=== base_link <- laser_frame (G2 + 운동 기하, odom 크기 미사용) ===')
print(f'x = {t[0]*1000:.1f} mm, y = {t[1]*1000:.1f} mm, yaw = {math.degrees(yb):.3f}°  (전진 {yaws["straight_forward"]["yaw_deg"]:.3f}°, 후진 {yaws["straight_reverse"]["yaw_deg"]:.3f}°)')
json.dump(dict(spin=res, straight=yaws, result=dict(x_mm=t[0] * 1000, y_mm=t[1] * 1000, yaw_deg=math.degrees(yb)), p_center_laser_mm=(pc * 1000).tolist()),
          open(OUT_JSON, 'w'), indent=2)
