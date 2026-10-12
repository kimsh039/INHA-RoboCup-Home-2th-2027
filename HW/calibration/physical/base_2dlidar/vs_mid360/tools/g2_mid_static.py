#!/usr/bin/env python3
"""G2 <-> Mid-360S (2D <-> 3D) from static poses: G2 points against Mid-360S planes, leave-one-session-out.

Usage: python3 g2_mid_static.py <calib_solve.json> <out_json> <label>=<calib_npz> [<label>=<calib_npz> ...]
       (calib_extract.py output; numpy + scipy only)

Per still: Mid-360S cloud in livox_frame, planar points (PCA curvature < 0.03) with normals; G2 median scan in
laser_frame with ranges x s. Residual: point-to-plane distance of s*G2 moved by T_livox<-laser.
  fit A (deployed form): G2 x, y, yaw in base_link (z, roll, pitch fixed, CAD) + range scale s, Mid-360S fixed at
         the motion-geometry + floor result of calib_solve.json  -> directly comparable with the G2 motion result
  fit B (diagnostic): full 6-DoF T_livox<-laser + s; parameter standard deviations from J^T J show what a
         horizontal 2D scan can observe (z, roll, pitch are expected to be weak)
Hold-out: fit on all sessions but one, residual statistics on the held-out session (correspondences within 20 cm).
Combined result: the G2 <-> Mid-360S relative pose is taken from fit A (static, most precise); the base_link frame
(origin = rotation centre, +x = driving direction) is the mean of the two motion-geometry estimates, i.e. the
G2 x, y, yaw are the mean of fit A and the G2 motion result, and the Mid-360S is moved by the same planar frame
correction. Closure is then zero by construction; the size of the correction is reported.
"""
import sys, json, math
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
SOLVE, OUT = sys.argv[1:3]; SESS = [a.split('=', 1) for a in sys.argv[3:]]
sol = json.load(open(SOLVE))
T_bm = np.array(sol['mid']['matrix4x4']); T_mb = np.linalg.inv(T_bm)
G2Z, G2RP = 0.34271, (0.0, 0.0)                       # base_link <- laser_frame z, roll, pitch (CAD, as in calibration.json)
p_motion = [sol['g2']['range_scale'], *sol['g2']['translation_xy_m'], math.radians(sol['g2']['yaw_deg'])]

def normals3(P, k=12):
    _, nn = cKDTree(P).query(P, k=k); Q = P[nn] - P[nn].mean(1, keepdims=True)
    w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', Q, Q)); return v[:, :, 0], w[:, 0] / (w.sum(1) + 1e-12)
STILLS = []
for label, f in SESS:
    D = np.load(f); a0, da = D['scan_angle']
    for i in range(len(D['windows'])):
        P = D[f'mid{i:03d}'].astype(float); P = P[np.linalg.norm(P, axis=1) < 12]
        n, c = normals3(P); k = c < 0.03
        r = D[f'scan{i:03d}']; a = a0 + np.arange(len(r)) * da; m = np.isfinite(r) & (r > 0.6) & (r < 8.0)
        L = np.c_[r[m] * np.cos(a[m]), r[m] * np.sin(a[m]), np.zeros(m.sum())]
        STILLS.append(dict(session=label, P=P[k], N=n[k], tree=cKDTree(P[k]), L=L))
print({lb: sum(s['session'] == lb for s in STILLS) for lb, _ in SESS}, 'stills')

def T_ml_A(p):          # p = [s, x, y, yaw] of base_link <- laser_frame
    T = np.eye(4); T[:3, :3] = Rotation.from_euler('xyz', [G2RP[0], G2RP[1], p[3]]).as_matrix(); T[:3, 3] = [p[1], p[2], G2Z]
    return T_mb @ T
def T_ml_B(q):          # q = [s, rx, ry, rz (rotvec), tx, ty, tz] of livox_frame <- laser_frame
    T = np.eye(4); T[:3, :3] = Rotation.from_rotvec(q[1:4]).as_matrix(); T[:3, 3] = q[4:7]; return T
def assoc(S, T, s, gate):
    out = []
    for st in S:
        X = (s * st['L']) @ T[:3, :3].T + T[:3, 3]; d, k = st['tree'].query(X, distance_upper_bound=gate); m = np.isfinite(d)
        out.append((st['L'][m], st['P'][k[m]], st['N'][k[m]]))
    return out
def resid(A, T, s): return np.concatenate([np.einsum('ij,ij->i', (s * L) @ T[:3, :3].T + T[:3, 3] - Q, N) for L, Q, N in A])
def fit(S, form, x0):
    x = np.array(x0, float); Tf = T_ml_A if form == 'A' else T_ml_B
    for gate in (0.2, 0.1, 0.05, 0.03, 0.03, 0.03):
        A = assoc(S, Tf(x), x[0], gate); r = least_squares(lambda y: resid(A, Tf(y), y[0]), x, loss='huber', f_scale=0.01); x = r.x
    J = r.jac; dof = max(len(r.fun) - len(x), 1); sig2 = np.sum(np.minimum(np.abs(r.fun), 0.01) ** 2) / dof
    cov = np.linalg.pinv(J.T @ J) * sig2
    return x, np.sqrt(np.diag(cov)), int(len(r.fun))
def stats(S, T, s, gate=0.2):
    r = np.abs(resid(assoc(S, T, s, gate), T, s))
    return dict(med_cm=float(np.median(r) * 100), p90_cm=float(np.percentile(r, 90) * 100), within_2cm=float(np.mean(r < 0.02)), n=int(len(r)))
def base_of(T_ml):      # base_link <- laser_frame as x, y, z, roll, pitch, yaw
    T = T_bm @ T_ml; e = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    return dict(x_mm=T[0, 3] * 1000, y_mm=T[1, 3] * 1000, z_mm=T[2, 3] * 1000, roll_deg=e[0], pitch_deg=e[1], yaw_deg=e[2])

res = dict(sessions={lb: f for lb, f in SESS}, mid_fixed_from=SOLVE, g2_motion=dict(zip(['s', 'x_m', 'y_m', 'yaw_rad'], p_motion)))
xA, sdA, nA = fit(STILLS, 'A', p_motion)
res['fit_A_all'] = dict(s=xA[0], x_mm=xA[1] * 1000, y_mm=xA[2] * 1000, yaw_deg=math.degrees(xA[3]), sd=dict(s=sdA[0], x_mm=sdA[1] * 1000, y_mm=sdA[2] * 1000, yaw_deg=math.degrees(sdA[3])), n=nA)
T0 = T_ml_A(xA); q0 = np.r_[xA[0], Rotation.from_matrix(T0[:3, :3]).as_rotvec(), T0[:3, 3]]
xB, sdB, nB = fit(STILLS, 'B', q0)
res['fit_B_all'] = dict(s=xB[0], base=base_of(T_ml_B(xB)), sd=dict(s=sdB[0], rot_deg=np.degrees(sdB[1:4]).tolist(), t_mm=(sdB[4:7] * 1000).tolist()), n=nB,
                        note='6-DoF diagnostic; the standard deviations show which G2 parameters the static scans constrain')
print(f"fit A (all): s {xA[0]:.5f}  x {xA[1]*1000:+.2f} ± {sdA[1]*1000:.2f}  y {xA[2]*1000:+.2f} ± {sdA[2]*1000:.2f} mm  yaw {math.degrees(xA[3]):+.3f} ± {math.degrees(sdA[3]):.3f} deg")
b = res['fit_B_all']['base']
print(f"fit B (6-DoF diag): base x {b['x_mm']:+.1f} y {b['y_mm']:+.1f} z {b['z_mm']:.1f} mm  roll {b['roll_deg']:+.2f} pitch {b['pitch_deg']:+.2f} yaw {b['yaw_deg']:+.3f}  "
      f"sd rot {np.round(np.degrees(sdB[1:4]), 3).tolist()} deg  t {np.round(sdB[4:7] * 1000, 2).tolist()} mm")
print(f"motion (calib_solve): x {p_motion[1]*1000:+.2f} y {p_motion[2]*1000:+.2f} mm yaw {math.degrees(p_motion[3]):+.3f} deg ->"
      f" static - motion: dx {(xA[1]-p_motion[1])*1000:+.2f} dy {(xA[2]-p_motion[2])*1000:+.2f} mm dyaw {math.degrees(xA[3]-p_motion[3]):+.3f} deg")
# leave-one-session-out
loso = {}
print('\nleave-one-session-out (fit A on the others; residuals on the held-out session):')
for lb, _ in SESS:
    tr = [s for s in STILLS if s['session'] != lb]; te = [s for s in STILLS if s['session'] == lb]
    x, sd, _ = fit(tr, 'A', p_motion)
    loso[lb] = dict(params=dict(s=x[0], x_mm=x[1] * 1000, y_mm=x[2] * 1000, yaw_deg=math.degrees(x[3])), test=stats(te, T_ml_A(x), x[0]),
                    test_motion_result=stats(te, T_ml_A(p_motion), p_motion[0]))
    t = loso[lb]['test']; tm = loso[lb]['test_motion_result']
    print(f"  {lb:8s} params s {x[0]:.5f} x {x[1]*1000:+.2f} y {x[2]*1000:+.2f} yaw {math.degrees(x[3]):+.3f} | held-out med {t['med_cm']*10:.2f} mm p90 {t['p90_cm']*10:.1f} mm "
          f"| motion result on it: med {tm['med_cm']*10:.2f} p90 {tm['p90_cm']*10:.1f}")
res['leave_one_session_out'] = loso
res['final_deployed'] = dict(all=stats(STILLS, T_ml_A(p_motion), p_motion[0]), all_scale_1=stats(STILLS, T_ml_A([1.0, *p_motion[1:]]), 1.0),
                             note='deployed transforms (G2 motion geometry, Mid-360S motion geometry + floor) with the fitted G2 range scale; nothing fitted to these residuals')
f = res['final_deployed']['all']; f1 = res['final_deployed']['all_scale_1']
print(f"\ndeployed (motion results) on all stills: med {f['med_cm']*10:.2f} mm p90 {f['p90_cm']*10:.1f} mm ({f['within_2cm']*100:.0f}% < 2 cm); with s = 1: med {f1['med_cm']*10:.2f} p90 {f1['p90_cm']*10:.1f}")

def planar(x, y, th): M = np.eye(4); M[:3, :3] = Rotation.from_euler('z', th).as_matrix(); M[:2, 3] = [x, y]; return M
def combine(xs):
    """xs = static fit A params -> (final G2 base<-laser, final Mid base<-livox, frame correction)"""
    gx, gy, gth = [(a + b) / 2 for a, b in zip(xs[1:], p_motion[1:])]
    T_static = np.linalg.inv(T_mb) @ T_ml_A(xs)                     # base <- laser in the motion-Mid base (= T_bm @ T_ml)
    T_final_g2 = T_static.copy(); T_final_g2[:3, :3] = Rotation.from_euler('xyz', [G2RP[0], G2RP[1], gth]).as_matrix(); T_final_g2[:3, 3] = [gx, gy, G2Z]
    C = T_final_g2 @ np.linalg.inv(T_static)                       # planar frame correction (rotation about z + xy shift)
    return T_final_g2, C @ T_bm, C
Tg, Tm, C = combine(xA)
res['combined'] = dict(g2_base_laser=Tg.tolist(), mid_base_livox=Tm.tolist(), s=xA[0],
                       frame_correction=dict(x_mm=C[0, 3] * 1000, y_mm=C[1, 3] * 1000, yaw_deg=math.degrees(math.atan2(C[1, 0], C[0, 0]))))
T_ml_comb = np.linalg.inv(Tm) @ Tg
res['combined']['all'] = stats(STILLS, T_ml_comb, xA[0])
hold = {}
for lb, _ in SESS:
    x = np.array([loso[lb]['params']['s'], loso[lb]['params']['x_mm'] / 1000, loso[lb]['params']['y_mm'] / 1000, math.radians(loso[lb]['params']['yaw_deg'])])
    g, m, _ = combine(x); hold[lb] = stats([s for s in STILLS if s['session'] == lb], np.linalg.inv(m) @ g, x[0])
res['combined']['holdout_per_session'] = hold
med = float(np.median([h['med_cm'] for h in hold.values()])); p90 = float(np.max([h['p90_cm'] for h in hold.values()]))
res['combined']['holdout_summary'] = dict(med_cm=med, p90_cm=p90, note='median of per-session held-out medians; worst per-session p90')
eg = Rotation.from_matrix(Tg[:3, :3]).as_euler('xyz', degrees=True); em = Rotation.from_matrix(Tm[:3, :3]).as_euler('xyz', degrees=True)
print(f"\ncombined: G2 x {Tg[0,3]*1000:+.2f} y {Tg[1,3]*1000:+.2f} mm yaw {eg[2]:+.3f} deg | Mid x {Tm[0,3]*1000:+.2f} y {Tm[1,3]*1000:+.2f} z {Tm[2,3]*1000:.1f} rpy {np.round(em, 3).tolist()} "
      f"| frame correction {C[0,3]*1000:+.2f}, {C[1,3]*1000:+.2f} mm {math.degrees(math.atan2(C[1,0], C[0,0])):+.3f} deg")
print(f"combined residual all: med {res['combined']['all']['med_cm']*10:.2f} p90 {res['combined']['all']['p90_cm']*10:.1f} mm; held-out per session: " +
      ' '.join(f"{k} {v['med_cm']*10:.2f}/{v['p90_cm']*10:.1f}" for k, v in hold.items()))

# ---------- 6-DoF (fit B) applied: G2 = final Mid-360S o T_livox<-laser(fit B); leave-one-session-out ----------
T_ml_Ball = T_ml_B(xB); Tg6 = Tm @ T_ml_Ball
loso6, hold6 = {}, {}
print('\nfit B (6-DoF) leave-one-session-out:')
for lb, _ in SESS:
    tr = [s for s in STILLS if s['session'] != lb]; te = [s for s in STILLS if s['session'] == lb]
    xb, sdb, _ = fit(tr, 'B', xB); T = T_ml_B(xb); bb = base_of(T)
    loso6[lb] = dict(s=xb[0], base=bb); hold6[lb] = stats(te, T, xb[0])
    print(f"  {lb:8s} z {bb['z_mm']:.1f} roll {bb['roll_deg']:+.2f} pitch {bb['pitch_deg']:+.2f} x {bb['x_mm']:+.1f} y {bb['y_mm']:+.1f} yaw {bb['yaw_deg']:+.3f} "
          f"| held-out med {hold6[lb]['med_cm']*10:.2f} p90 {hold6[lb]['p90_cm']*10:.1f} mm (4-DoF: {hold[lb]['med_cm']*10:.2f} / {hold[lb]['p90_cm']*10:.1f})")
e6 = Rotation.from_matrix(Tg6[:3, :3]).as_euler('xyz', degrees=True)
res['combined_6dof'] = dict(g2_base_laser=Tg6.tolist(), livox_laser=T_ml_Ball.tolist(), s=xB[0], all=stats(STILLS, T_ml_Ball, xB[0]),
                            leave_one_session_out=loso6, holdout_per_session=hold6,
                            holdout_summary=dict(med_cm=float(np.median([h['med_cm'] for h in hold6.values()])), p90_cm=float(np.max([h['p90_cm'] for h in hold6.values()]))))
print(f"6-DoF applied: G2 base x {Tg6[0,3]*1000:+.2f} y {Tg6[1,3]*1000:+.2f} z {Tg6[2,3]*1000:.1f} mm rpy {np.round(e6, 3).tolist()} | all med {res['combined_6dof']['all']['med_cm']*10:.2f} p90 {res['combined_6dof']['all']['p90_cm']*10:.1f} mm")
json.dump(res, open(OUT, 'w'), indent=2, default=float)
