#!/usr/bin/env python3
# Mid-360S만으로 제자리 회전 중심 → Mid-360S x, y (G2 체인과 독립). 같은 자리에서 방향만 바꾼 정지 자세, 기울기 보정 맵.
# 사용: python3 mid_rotation_check.py <prep_dir> <solver_output.json> <out_json> pose01 pose02 pose03 pose04
import sys, json, math, numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
R_BL = Rotation.from_euler('xyz', [math.pi, 0, 0]).as_matrix(); T_BL = np.array([-0.18, 0.0, 0.01611 + 1.183])
ARGS = sys.argv[1:]; PREP, POSES = ARGS[0], ARGS[3:]
rt = json.load(open(ARGS[1]))
n_mean = np.array(rt['floor_normal_cad_base']); axis = np.cross(n_mean, [0, 0, 1.]); R_d = Rotation.from_rotvec(axis / np.linalg.norm(axis) * math.asin(np.linalg.norm(axis))).as_matrix()
def se2(x, y, th): c, s = np.cos(th), np.sin(th); return np.array([[c, -s, x], [s, c, y], [0, 0, 1.]])
def inv(T): o = np.eye(3); o[:2, :2] = T[:2, :2].T; o[:2, 2] = -T[:2, :2].T @ T[:2, 2]; return o
def apply(T, X): return X @ T[:2, :2].T + T[:2, 2]
def icp(src, dst, dn, T0, gates=(0.5, 0.3, 0.15, 0.08, 0.05, 0.03, 0.03)):
    tree = cKDTree(dst); T = T0.copy()
    for g in gates:
        d, i = tree.query(apply(T, src), distance_upper_bound=g); k = np.isfinite(d); s, q, n = src[k], dst[i[k]], dn[i[k]]
        sol = least_squares(lambda p: np.einsum('ij,ij->i', apply(se2(*p), s) - q, n), [T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])], loss='huber', f_scale=0.01); T = se2(*sol.x)
    return T, float(k.mean()), float(np.median(np.abs(sol.fun)) * 1000)
def global_init(src, dst):
    rng = np.random.default_rng(0); s = src[rng.choice(len(src), min(1500, len(src)), replace=False)]; tree = cKDTree(dst); best = []
    for yd in np.arange(0, 360, 2.0):
        th = math.radians(yd); X = s @ np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]]).T
        for tx in np.arange(-0.5, 0.501, 0.05):
            for ty in np.arange(-0.5, 0.501, 0.05):
                d, _ = tree.query(X + [tx, ty], distance_upper_bound=0.05); best.append((np.isfinite(d).mean(), yd, tx, ty))
    best.sort(reverse=True); return best[:5]
M = {}
for n in POSES:
    z = np.load(f'{PREP}/{n}.npz')
    P = (R_d @ (z['pts'] - T_BL).T).T + T_BL                      # tilt-corrected, base'' (CAD x/y of Mid kept)
    N = (R_d @ z['nrm'].T).T; nn = N[:, :2] / np.linalg.norm(N[:, :2], axis=1, keepdims=True)
    M[n] = (P[:, :2], nn)
ref = POSES[0]; Ts = [np.eye(3)]; reg = {}
for n in POSES[1:]:
    best = None
    for fr, yd, tx, ty in global_init(M[n][0], M[ref][0]):
        T, f, md = icp(M[n][0], M[ref][0], M[ref][1], se2(tx, ty, math.radians(yd)))
        if best is None or f > best[1]: best = (T, f, md)
    Ts.append(best[0]); reg[n] = dict(yaw_deg=math.degrees(math.atan2(best[0][1, 0], best[0][0, 0])), matched=best[1], med_mm=best[2]); print(f'{n}: yaw {math.degrees(math.atan2(best[0][1,0], best[0][0,0])):+.2f} deg matched {best[1]*100:.0f}% med {best[2]:.1f} mm')
A = np.vstack([np.hstack([T[:2, :2], -np.eye(2)]) for T in Ts]); b = np.concatenate([-T[:2, 2] for T in Ts])
sol, *_ = np.linalg.lstsq(A, b, rcond=None); c = sol[:2]; r = (A @ sol - b).reshape(-1, 2)
lo = []
for i in range(len(POSES)):
    idx = [j for j in range(len(POSES)) if j != i]; Ai = np.vstack([A[2 * j:2 * j + 2] for j in idx]); bi = np.concatenate([b[2 * j:2 * j + 2] for j in idx])
    lo.append(np.linalg.lstsq(Ai, bi, rcond=None)[0][:2])
lo = np.array(lo) * 1000
mid_rc = np.array([-0.18, 0.0]) - c                                 # Mid-360S position relative to the rotation centre
print(f'rotation centre in base\'\' = ({c[0]*1000:+.1f}, {c[1]*1000:+.1f}) mm, fit residual {np.sqrt((r**2).sum(1)).max()*1000:.1f} mm max, LOO x {lo[:,0].min():.1f}..{lo[:,0].max():.1f}')
print(f'=> Mid-360S relative to rotation centre (Mid-only, independent of G2): x = {mid_rc[0]*1000:.1f} mm, y = {mid_rc[1]*1000:.1f} mm')
print('   compare: G2-chain calibration x = -158.8 mm, tape (axle) x = -173.5 mm, CAD -180 mm')
json.dump(dict(poses=POSES, registration=reg, rotation_centre_base_mm=(c * 1000).tolist(), fit_residual_max_mm=float(np.sqrt((r ** 2).sum(1)).max() * 1000),
               loo_x_mm=[float(lo[:, 0].min()), float(lo[:, 0].max())], mid_relative_to_rotation_centre_mm=(mid_rc * 1000).tolist()), open(ARGS[2], 'w'), indent=2)
