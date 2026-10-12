#!/usr/bin/env python3
"""Solve base <- G2, base <- Mid-360S and the G2 range scale from one calib_drive.py session.

Usage: python3 calib_solve.py <calib_npz> <out_json> [<straights_npz> ...]   (calib_extract.py output; numpy + scipy)
       extra npz: calib_drive.py --straights N sessions, pooled into the driving-direction (yaw) estimate

Still order (calib_drive.py): 0 start | 1-8 after +45 deg spins | 9-16 after -45 deg spins | 17-22 forward | 23-28 reverse.

1. Mid-360S floor (every still, 0.8-3 m ring, RANSAC 12 mm): mean normal over all 16 headings cancels the room
   floor slope -> roll, pitch; mean sensor-to-floor distance -> z (floor at base_link z = -0.1425 m).
2. Mid-360S x, y, yaw from motion geometry in the levelled sensor frame: 3-DoF point-to-plane ICP between stills,
   rotation centre of the spins = base origin, base-origin direction on the straights = base x. Independent of G2.
3. G2 range scale s and G2 pose from the static stills against vertical Mid-360S planes (in the step 1-2 base).
4. G2 x, y, yaw from motion geometry (ranges x s), z / roll / pitch from the existing record (CAD). Independent of Mid.
5. Closure: G2 pose from 3 (static, through the Mid-360S) vs 4 (motion); residual G2-to-Mid-plane distances.
Fit / check: every estimate is made from the CCW and the CW spins (and forward / reverse) separately as well.
"""
import sys, json, math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
NPZ, OUT = sys.argv[1:3]; EXTRA = sys.argv[3:]
HERE = Path(__file__).resolve().parent
G2REC = json.load(open(HERE.parent / 'calibration.json', encoding='utf-8'))
MIDREC = json.load(open(HERE.parents[1] / 'base_mid360/base_mid360.json', encoding='utf-8'))
GROUND = -(0.082 + 0.0605)                  # URDF: drive wheel axle z = -0.082 m, wheel radius 0.0605 m
D = np.load(NPZ); N = len(D['windows'])
DX = [np.load(f) for f in EXTRA]
SRC = [(D, i) for i in range(N)] + [(E, i) for E in DX for i in range(len(E['windows']))]   # global still id -> (data, index)
XGROUPS = []; _o = N
for E in DX: XGROUPS.append(list(range(_o, _o + len(E['windows'])))); _o += len(E['windows'])
MIDRAW = lambda i: SRC[i][0][f'mid{SRC[i][1]:03d}'].astype(float)
SCANRAW = lambda i: SRC[i][0][f'scan{SRC[i][1]:03d}']
assert N == 29, f'expected 29 still periods, got {N}'
CCW, CW, FWD, REV = list(range(0, 9)), list(range(8, 17)), list(range(16, 23)), list(range(22, 29))

def se2(x, y, th): c, s = math.cos(th), math.sin(th); return np.array([[c, -s, x], [s, c, y], [0, 0, 1.]])
def inv(T): o = np.eye(3); o[:2, :2] = T[:2, :2].T; o[:2, 2] = -T[:2, :2].T @ T[:2, 2]; return o
def yaw(T): return math.atan2(T[1, 0], T[0, 0])
def apply(T, X): return X @ T[:2, :2].T + T[:2, 2]

# ---------- 1. floor ----------
T_bm0 = np.array(MIDREC['matrix4x4'])
rng = np.random.default_rng(0)
def floor_plane(P):
    B = P @ T_bm0[:3, :3].T + T_bm0[:3, 3]; r = np.hypot(B[:, 0], B[:, 1])
    C = P[(np.abs(B[:, 2] - GROUND) < 0.12) & (r > 0.8) & (r < 3.0)]
    best = (0, None)
    for _ in range(400):
        a, b, c = C[rng.choice(len(C), 3, replace=False)]; n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9: continue
        n /= np.linalg.norm(n); k = np.abs((C - a) @ n) < 0.012
        if k.sum() > best[0]: best = (k.sum(), k)
    Q = C[best[1]]; m = Q.mean(0); n = np.linalg.svd(Q - m, full_matrices=False)[2][2]
    if n @ (T_bm0[:3, :3].T @ [0, 0, 1]) < 0: n = -n                 # normal pointing up (base +z) in sensor frame
    return n, float(-(m @ n)), int(len(Q))                           # plane n.x + d = 0 -> sensor-to-floor distance = d
fl = [floor_plane(D[f'mid{i:03d}'].astype(float)) for i in range(N)]
nrm = np.array([f[0] for f in fl]); dist = np.array([f[1] for f in fl])
n_mean = nrm.mean(0); n_mean /= np.linalg.norm(n_mean)
# levelled frame L: Rx(pi) (upside-down mount) followed by the small tilt that maps the floor normal to +z
R_flip = Rotation.from_euler('x', math.pi).as_matrix(); u = R_flip @ n_mean
ax = np.cross(u, [0, 0, 1.]); R_tilt = Rotation.from_rotvec(ax / np.linalg.norm(ax) * math.asin(np.linalg.norm(ax))).as_matrix()
R_level = R_tilt @ R_flip
z_mid = float(dist.mean()) + GROUND
per_still_tilt = np.degrees(np.array([[math.atan2((R_level @ n)[1], (R_level @ n)[2]), math.atan2(-(R_level @ n)[0], (R_level @ n)[2])] for n in nrm]))

# ---------- 2. Mid-360S motion geometry in L ----------
def normals3(P, k=12):
    _, nn = cKDTree(P).query(P, k=k); Q = P[nn] - P[nn].mean(1, keepdims=True)
    w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', Q, Q)); return v[:, :, 0], w[:, 0] / (w.sum(1) + 1e-12)
def R3(p): c, s = math.cos(p[2]), math.sin(p[2]); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]]), np.array([p[0], p[1], 0.])
LC = [None] * len(SRC)
def Lcloud(i):
    if LC[i] is None:
        P = MIDRAW(i) @ R_level.T; n, curv = normals3(P)
        LC[i] = (P, P[(curv < 0.05) & (np.abs(n[:, 2]) < 0.9)], n[(curv < 0.05) & (np.abs(n[:, 2]) < 0.9)])
    return LC[i]
def icp3(i_src, i_dst, T0, gates=(0.5, 0.25, 0.12, 0.06, 0.04, 0.03, 0.03)):
    S = Lcloud(i_src)[0]; _, Dd, Nn = Lcloud(i_dst); tree = cKDTree(Dd); p = np.array([T0[0, 2], T0[1, 2], yaw(T0)])
    S = S[np.random.default_rng(1).choice(len(S), min(30000, len(S)), replace=False)]
    for g in gates:
        R, t = R3(p); d, k = tree.query(S @ R.T + t, distance_upper_bound=g); m = np.isfinite(d); s, q, nn = S[m], Dd[k[m]], Nn[k[m]]
        sol = least_squares(lambda x: np.einsum('ij,ij->i', s @ R3(x)[0].T + R3(x)[1] - q, nn), p, loss='huber', f_scale=0.01); p = sol.x
    return se2(*p), float(m.mean()), float(np.median(np.abs(sol.fun)) * 1000)

def chain(group, icp, T_guess, label):
    """poses ref <- i for every still of the group (consecutive ICP, then refined against the group reference)"""
    poses, q = [np.eye(3)], []
    for a, b in zip(group[:-1], group[1:]):
        Ti, f1, m1 = icp(b, a, T_guess(a, b)); Tg = poses[-1] @ Ti
        Tr, f2, m2 = icp(b, group[0], Tg); poses.append(Tr if f2 > 0.4 else Tg); q.append((f1, m1, f2, m2))
    print(f'  {label:14s} match {np.mean([x[0] for x in q])*100:3.0f}% / ref {np.mean([x[2] for x in q])*100:3.0f}%, median {np.mean([x[1] for x in q]):.1f} mm')
    return poses, q
def centre(Ts):
    A = np.vstack([np.hstack([T[:2, :2], -np.eye(2)]) for T in Ts]); b = np.concatenate([-T[:2, 2] for T in Ts])
    sol, *_ = np.linalg.lstsq(A, b, rcond=None); r = (A @ sol - b).reshape(-1, 2); return sol[:2], np.sqrt((r ** 2).sum(1))
def step_dirs(Tsteps, c, sign):
    """base-origin motion per step in the frame of the previous still; a step with a small heading change dyaw
    is an arc whose chord lies dyaw/2 off the start heading, so dyaw/2 is removed"""
    d = np.array([(apply(T, c[None])[0] - c) * sign for T in Tsteps]); dy = np.array([yaw(T) for T in Tsteps])
    return np.arctan2(d[:, 1], d[:, 0]) - dy / 2, np.linalg.norm(d, axis=1)
def pooled(phis, w):
    return math.atan2((w * np.sin(phis)).sum(), (w * np.cos(phis)).sum()), float(np.degrees(np.std(phis))), w
def extra_steps(icp, fwd_guess):
    """consecutive still pairs of the extra straight sessions; direction from the better of the two initial guesses"""
    out = []
    for grp in XGROUPS:
        for a, b in zip(grp[:-1], grp[1:]):
            best = max((icp(b, a, se2(*(sg * fwd_guess), 0.0)) for sg in (+1, -1)), key=lambda r: r[1])
            out.append(best[0])
    return out

def solve_motion(icp, c_guess, label):
    """c_guess: approximate base origin in the sensor (planar) frame; returns centre, yaw, consistency"""
    def g_spin(sgn): return lambda a, b: se2(*(c_guess - se2(0, 0, sgn * math.pi / 4)[:2, :2] @ c_guess), sgn * math.pi / 4)
    out = {}
    for name, grp, sgn in [('ccw', CCW, +1), ('cw', CW, -1)]:
        P, _ = chain(grp, icp, g_spin(sgn), f'{label} spin {name}'); c, r = centre(P)
        out[name] = dict(centre=c, resid_mm=r * 1000, turn_deg=math.degrees(sum(wrap(yaw(inv(P[k - 1]) @ P[k])) for k in range(1, len(P)))))
    c = (out['ccw']['centre'] + out['cw']['centre']) / 2
    steps = {}
    for name, grp, sgn in [('fwd', FWD, +1), ('rev', REV, -1)]:
        P, _ = chain(grp, icp, lambda a, b, s=sgn: se2(*(np.array([0.15 * s, 0.0])), 0.0), f'{label} {name}')
        steps[name] = [inv(P[k - 1]) @ P[k] for k in range(1, len(P))]
    if XGROUPS:
        xs = extra_steps(icp, np.array([0.15, 0.0])); n_x = {'fwd': 0, 'rev': 0, 'pause': 0}   # pause = not a single 0.15 m step
        for T in xs:
            dx = (apply(T, c[None])[0] - c)[0]
            L = np.linalg.norm(apply(T, c[None])[0] - c)
            k = 'pause' if not 0.10 < L < 0.20 else ('fwd' if dx > 0 else 'rev'); n_x[k] += 1   # one 0.15 m step; merged or partial steps dropped
            if k != 'pause': steps[k].append(T)
        print(f'  {label} extra straight steps: {n_x}')
    for name, sgn in [('fwd', +1), ('rev', -1)]:
        ph, w = step_dirs(steps[name], c, sgn); phi, sd, w = pooled(ph, w)
        print(f'  {label} {name} steps (deg@mm, dyaw): ' + ' '.join(f'{math.degrees(a):+.2f}@{b*1000:.0f}({math.degrees(yaw(T)):+.2f})' for a, b, T in zip(ph, w, steps[name])))
        out[name] = dict(phi=phi, seg_std_deg=sd, step_m=w.tolist(), n=len(w), sem_deg=sd / math.sqrt(len(w)),
                         first_session_phi=pooled(ph[:6], w[:6])[0])
    phi = math.atan2(math.sin(out['fwd']['phi']) + math.sin(out['rev']['phi']), math.cos(out['fwd']['phi']) + math.cos(out['rev']['phi']))
    yb = -phi; Rb = np.array([[math.cos(yb), -math.sin(yb)], [math.sin(yb), math.cos(yb)]]); t = -(Rb @ c)
    cons = dict(cw_ccw_centre_difference_mm=float(np.linalg.norm(out['ccw']['centre'] - out['cw']['centre']) * 1000),
                forward_reverse_yaw_difference_deg=float(abs(math.degrees(wrap(out['fwd']['phi'] - out['rev']['phi'])))),
                spin_fit_resid_max_mm=float(max(out['ccw']['resid_mm'].max(), out['cw']['resid_mm'].max())),
                straight_seg_std_deg=float(max(out['fwd']['seg_std_deg'], out['rev']['seg_std_deg'])))
    return dict(t=t, yaw=yb, centre=c, detail=out, consistency=cons)
def wrap(a): return math.atan2(math.sin(a), math.cos(a))

print('2) Mid-360S motion geometry (levelled frame):')
T_lb0 = np.linalg.inv(np.vstack([np.hstack([T_bm0[:3, :3] @ R_level.T, T_bm0[:3, 3:]]), [0, 0, 0, 1]]))   # L <- base (current calibration)
c_guess_mid = T_lb0[:2, 3]
MID = solve_motion(icp3, c_guess_mid, 'Mid')
R_bL = Rotation.from_euler('z', MID['yaw']).as_matrix()
T_bm = np.eye(4); T_bm[:3, :3] = R_bL @ R_level; T_bm[:2, 3] = MID['t']; T_bm[2, 3] = z_mid

# ---------- 4 (needs s from 3) / 3. G2 ----------
ang0, dang = D['scan_angle']
def g2_points(i, s=1.0, rmin=0.45, rmax=8.0):
    r = SCANRAW(i); a = ang0 + np.arange(len(r)) * dang; k = np.isfinite(r) & (r > rmin) & (r < rmax)
    return np.c_[r[k] * np.cos(a[k]), r[k] * np.sin(a[k])] * s
def normals2(L, k=7):
    _, nn = cKDTree(L).query(L, k=k); Q = L[nn] - L[nn].mean(1, keepdims=True)
    w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', Q, Q)); return v[:, :, 0], w[:, 0] / (w.sum(1) + 1e-12) < 0.05
def make_icp2(s):
    def icp2(i_src, i_dst, T0, gates=(0.3, 0.15, 0.08, 0.05, 0.03, 0.03, 0.02)):
        src, dst = g2_points(i_src, s), g2_points(i_dst, s); n, lin = normals2(dst); Dd, Nn = dst[lin], n[lin]; tree = cKDTree(Dd); T = T0.copy()
        for g in gates:
            d, k = tree.query(apply(T, src), distance_upper_bound=g); m = np.isfinite(d); ss, q, nn = src[m], Dd[k[m]], Nn[k[m]]
            sol = least_squares(lambda p: np.einsum('ij,ij->i', apply(se2(*p), ss) - q, nn), [T[0, 2], T[1, 2], yaw(T)], loss='huber', f_scale=0.01); T = se2(*sol.x)
        return T, float(m.mean()), float(np.median(np.abs(sol.fun)) * 1000)
    return icp2

# vertical Mid planes in the base band around the G2 height, per still
BAND = []
for i in range(N):
    P = D[f'mid{i:03d}'].astype(float) @ T_bm[:3, :3].T + T_bm[:3, 3]; P = P[(P[:, 2] > 0.15) & (P[:, 2] < 0.55)]
    n, curv = normals3(P); k = (curv < 0.03) & (np.abs(n[:, 2]) < 0.2)
    BAND.append((P[k, :2], n[k, :2] / np.linalg.norm(n[k, :2], axis=1, keepdims=True), cKDTree(P[k, :2])))
def g2_static(stills, p0, free_scale=True):
    def tf(p, L): s, x, y, th = p; return apply(se2(x, y, th), s * L)
    p = np.array(p0, float)
    for gate in (0.2, 0.1, 0.05, 0.03, 0.03, 0.03):
        A = []
        for i in stills:
            L = g2_points(i, 1.0, 0.6); Dd, Nn, tree = BAND[i]; d, k = tree.query(tf(p, L), distance_upper_bound=gate); m = np.isfinite(d); A.append((L[m], Dd[k[m]], Nn[k[m]]))
        f = lambda q: np.concatenate([np.einsum('ij,ij->i', tf(np.r_[q[0] if free_scale else 1.0, q[1:]] if True else q, L) - Dd, Nn) for L, Dd, Nn in A])
        p = least_squares(f, p, loss='huber', f_scale=0.01).x
        if not free_scale: p[0] = 1.0
    return p
def residual_stats(stills, p, gate=0.2):
    r = []
    for i in stills:
        L = g2_points(i, 1.0, 0.6); Dd, Nn, tree = BAND[i]; X = apply(se2(*p[1:]), p[0] * L); d, k = tree.query(X, distance_upper_bound=gate); m = np.isfinite(d)
        r.append(np.abs(np.einsum('ij,ij->i', X[m] - Dd[k[m]], Nn[k[m]])))
    r = np.concatenate(r); return dict(med_cm=float(np.median(r) * 100), p90_cm=float(np.percentile(r, 90) * 100), within_2cm=float(np.mean(r < 0.02)), n=int(len(r)))
p_prev = [1.0, G2REC['translation_m'][0], G2REC['translation_m'][1], math.radians(G2REC['yaw_deg'])]
ALL = list(range(N)); even, odd = ALL[0::2], ALL[1::2]
st_all = g2_static(ALL, p_prev); st_even = g2_static(even, p_prev); st_odd = g2_static(odd, p_prev)
S_G2 = float(st_all[0])
print(f'3) G2 static vs Mid planes: s = {S_G2:.5f} (even {st_even[0]:.5f}, odd {st_odd[0]:.5f}), x {st_all[1]*1000:+.1f} y {st_all[2]*1000:+.1f} mm yaw {math.degrees(st_all[3]):+.3f} deg')

print('4) G2 motion geometry (ranges x s):')
G2 = solve_motion(make_icp2(S_G2), np.array([-p_prev[1], -p_prev[2]]), 'G2')
G2_raw = solve_motion(make_icp2(1.0), np.array([-p_prev[1], -p_prev[2]]), 'G2 s=1')
p_motion = [S_G2, G2['t'][0], G2['t'][1], G2['yaw']]

# ---------- 5. closure and residuals ----------
closure = dict(dx_mm=(st_all[1] - G2['t'][0]) * 1000, dy_mm=(st_all[2] - G2['t'][1]) * 1000, dyaw_deg=math.degrees(wrap(st_all[3] - G2['yaw'])))
res_final = residual_stats(ALL, p_motion); res_final_s1 = residual_stats(ALL, [1.0, *p_motion[1:]]); res_prev = residual_stats(ALL, p_prev)          # previous G2 record, against the new Mid-360S base
# previous Mid-360S calibration as the base for comparison
e_mid = Rotation.from_matrix(T_bm[:3, :3]).as_euler('xyz', degrees=True); e_prev = Rotation.from_matrix(T_bm0[:3, :3]).as_euler('xyz', degrees=True)
out = dict(
    floor=dict(normal_sensor=n_mean.tolist(), n=N, z_m=z_mid, distance_mean_m=float(dist.mean()), distance_std_mm=float(dist.std() * 1000),
               per_still_tilt_std=dict(roll_deg=float(per_still_tilt[:, 0].std()), pitch_deg=float(per_still_tilt[:, 1].std())),
               note='per-still floor normal in the levelled frame; its spread over 16 headings includes the room floor slope'),
    mid=dict(matrix4x4=T_bm.tolist(), translation_m=T_bm[:3, 3].tolist(), rpy_deg=e_mid.tolist(), consistency=MID['consistency'],
             spins={k: dict(centre_L_mm=(v['centre'] * 1000).tolist(), resid_max_mm=float(v['resid_mm'].max()), turn_deg=v['turn_deg']) for k, v in MID['detail'].items() if k in ('ccw', 'cw')},
             straights={k: dict(phi_deg=math.degrees(v['phi']), seg_std_deg=v['seg_std_deg'], n=v['n'], sem_deg=v['sem_deg'], first_session_phi_deg=math.degrees(v['first_session_phi']), steps_m=v['step_m']) for k, v in MID['detail'].items() if k in ('fwd', 'rev')},
             previous=dict(translation_m=T_bm0[:3, 3].tolist(), rpy_deg=e_prev.tolist())),
    g2=dict(translation_xy_m=G2['t'].tolist(), yaw_deg=math.degrees(G2['yaw']), range_scale=S_G2, consistency=G2['consistency'],
            spins={k: dict(centre_laser_mm=(v['centre'] * 1000).tolist(), resid_max_mm=float(v['resid_mm'].max()), turn_deg=v['turn_deg']) for k, v in G2['detail'].items() if k in ('ccw', 'cw')},
            straights={k: dict(phi_deg=math.degrees(v['phi']), seg_std_deg=v['seg_std_deg'], n=v['n'], sem_deg=v['sem_deg'], first_session_phi_deg=math.degrees(v['first_session_phi']), steps_m=v['step_m']) for k, v in G2['detail'].items() if k in ('fwd', 'rev')},
            with_scale_1=dict(translation_xy_m=G2_raw['t'].tolist(), yaw_deg=math.degrees(G2_raw['yaw']), consistency=G2_raw['consistency']),
            previous=dict(translation_xy_m=p_prev[1:3], yaw_deg=G2REC['yaw_deg'])),
    g2_static=dict(all=st_all.tolist(), even=st_even.tolist(), odd=st_odd.tolist(), note='[range scale, x m, y m, yaw rad] of the G2 in the Mid-derived base_link'),
    closure_static_minus_motion=closure,
    residual_g2_to_mid_planes=dict(final=res_final, final_scale_1=res_final_s1, previous_calibrations=res_prev,
                                   note='all 29 stills; neither final transform was fitted to these residuals (both are motion geometry)'))
print('\n=== results ===')
print(f"floor: z {z_mid*1000:.1f} mm (sensor-floor {dist.mean()*1000:.1f} ± {dist.std()*1000:.1f} mm), per-still tilt std roll {per_still_tilt[:,0].std():.2f} pitch {per_still_tilt[:,1].std():.2f} deg")
print(f"Mid  base<-livox: xyz ({T_bm[0,3]*1000:+.1f}, {T_bm[1,3]*1000:+.1f}, {T_bm[2,3]*1000:.1f}) mm  rpy ({e_mid[0]:+.3f}, {e_mid[1]:+.3f}, {e_mid[2]:+.3f}) deg   [prev ({T_bm0[0,3]*1000:+.1f}, {T_bm0[1,3]*1000:+.1f}, {T_bm0[2,3]*1000:.1f}) rpy ({e_prev[0]:+.3f}, {e_prev[1]:+.3f}, {e_prev[2]:+.3f})]")
print(f"     consistency: {json.dumps({k: round(v, 3) for k, v in MID['consistency'].items()})}")
print(f"G2   base<-laser: x {G2['t'][0]*1000:+.1f} y {G2['t'][1]*1000:+.1f} mm yaw {math.degrees(G2['yaw']):+.3f} deg  (s=1: {G2_raw['t'][0]*1000:+.1f}, {G2_raw['t'][1]*1000:+.1f}, {math.degrees(G2_raw['yaw']):+.3f})  [prev {p_prev[1]*1000:+.1f}, {p_prev[2]*1000:+.1f}, {G2REC['yaw_deg']:+.3f}]")
print(f"     consistency: {json.dumps({k: round(v, 3) for k, v in G2['consistency'].items()})}")
print(f"closure (G2 static via Mid - G2 motion): dx {closure['dx_mm']:+.1f} dy {closure['dy_mm']:+.1f} mm dyaw {closure['dyaw_deg']:+.3f} deg")
print(f"G2->Mid plane residual: final med {res_final['med_cm']*10:.2f} mm p90 {res_final['p90_cm']*10:.1f} mm ({res_final['within_2cm']*100:.0f}% < 2 cm) | s=1 med {res_final_s1['med_cm']*10:.2f} p90 {res_final_s1['p90_cm']*10:.1f} | previous calib med {res_prev['med_cm']*10:.2f} p90 {res_prev['p90_cm']*10:.1f}")
json.dump(out, open(OUT, 'w'), indent=2, default=float)
