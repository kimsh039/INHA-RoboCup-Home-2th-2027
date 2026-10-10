#!/usr/bin/env python3
"""Fit the Tracer odometry offset model by integrating the deployed formula in SE(2).

Usage (ROS 2 Humble): python3 odom_fit_se2.py <team_bag_root> <validation_bag> <out_json>

Model (as in the driver):  v = v_rep + c_v·sign(v_rep) if |v_rep| > 1e-3
                           w = w_rep + c_w·sign(w_rep) if |w_rep| > thr
Per stop-to-stop segment the corrected (v, w) are integrated exactly like the driver
(x += v cosθ dt, y += v sinθ dt, θ += w dt) and compared with the scan-matched base motion.
c_w (closed form on yaw) and c_v (1-D search on translation) are fitted per threshold on one
session and tested on the other, then on both. The validation bag was recorded with the
previous scale model active (v=1.112 v_rep, w=1.319 w_rep + 0.00378 while moving); it is inverted.
"""
import sys, json, math
from pathlib import Path
import numpy as np
from scipy.optimize import minimize_scalar
ARGS = sys.argv[1:]
sys.argv = [sys.argv[0], ARGS[0]]
exec((Path(__file__).resolve().parents[1] / 'base_2dlidar/tools/base_g2_motion.py').read_text().split('res = {}')[0])
cal = json.load(open(Path(__file__).resolve().parents[1] / 'base_2dlidar/calibration.json'))
T_bl = se2(cal['translation_m'][0], cal['translation_m'][1], math.radians(cal['yaw_deg']))
EPS = 1e-3; THRS = [0.02, 0.04, 0.06, 0.08, 0.10]

def segments(root, name, invert):
    global B
    B = str(root).rstrip('/') + '/'
    od, sc = read(name); st = stops(od, sc); poses, _ = chain(st, name)
    v, w = od[:, 4].copy(), od[:, 5].copy()
    if invert:
        v /= 1.112; mv = (np.abs(v) > EPS) | (np.abs(w) > EPS); w = (w - 0.00378 * mv) / 1.319
    out = []
    for i in range(1, len(st)):
        k = np.where((od[:, 0] >= st[i - 1]['t']) & (od[:, 0] < st[i]['t']))[0]
        Tb = T_bl @ inv(poses[i - 1]) @ poses[i] @ inv(T_bl)
        dt = np.diff(od[k, 0], append=od[k[-1], 0] if len(k) else 0)
        out.append(dict(src=name, Tb=Tb, v=v[k], w=w[k], dt=dt))
    return out

def integrate(s, cv, cw, thr):
    v = s['v'] + cv * np.sign(s['v']) * (np.abs(s['v']) > EPS)
    w = s['w'] + cw * np.sign(s['w']) * (np.abs(s['w']) > thr)
    x = y = th = 0.0
    for vi, wi, dti in zip(v, w, s['dt']):
        x += vi * math.cos(th) * dti; y += vi * math.sin(th) * dti; th += wi * dti
    return x, y, th

def errors(S, cv, cw, thr):
    et, ey = [], []
    for s in S:
        x, y, th = integrate(s, cv, cw, thr); Tb = s['Tb']
        et.append(math.hypot(x - Tb[0, 2], y - Tb[1, 2])); ey.append(math.atan2(math.sin(th - yaw(Tb)), math.cos(th - yaw(Tb))))
    return np.array(et), np.array(ey)

def fit(S, thr):
    a = np.array([np.sum(s['w'] * s['dt']) for s in S]); T = np.array([np.sum(np.sign(s['w']) * (np.abs(s['w']) > thr) * s['dt']) for s in S])
    y = np.array([yaw(s['Tb']) for s in S]); cw = float(T @ (y - a) / (T @ T))
    r = minimize_scalar(lambda cv: float(np.mean(errors(S, cv, cw, thr)[0] ** 2)), bounds=(-0.02, 0.03), method='bounded', options=dict(xatol=1e-6))
    return float(r.x), cw

team = sum([segments(ARGS[0], n, False) for n in ['spin_cw', 'spin_ccw', 'straight_forward', 'straight_reverse']], [])
vp = Path(ARGS[1]).resolve(); val = segments(vp.parent, vp.name, True)
kind = lambda s: 'rot' if abs(yaw(s['Tb'])) > math.radians(10) else ('str' if math.hypot(s['Tb'][0, 2], s['Tb'][1, 2]) > 0.10 else 'small')
def report(S, cv, cw, thr):
    et, ey = errors(S, cv, cw, thr); ks = np.array([kind(s) for s in S])
    f = lambda m, e, u: float(np.sqrt(np.mean(e[m] ** 2)) * u) if m.any() else float('nan')
    return dict(trans_rms_mm=f(ks != 'small', et, 1000), trans_rms_straight_mm=f(ks == 'str', et, 1000),
                yaw_rms_rot_deg=f(ks == 'rot', ey, 180 / math.pi), yaw_rms_straight_deg=f(ks == 'str', ey, 180 / math.pi))
res = dict(uncorrected=dict(team=report(team, 0, 0, 1), val=report(val, 0, 0, 1)), thresholds={})
print('uncorrected:', res['uncorrected'])
for thr in THRS:
    r = {}
    for a_name, A, b_name, Bset in [('team', team, 'val', val), ('val', val, 'team', team)]:
        cv, cw = fit(A, thr); r[f'{a_name}_to_{b_name}'] = dict(c_v=cv, c_w=cw, train=report(A, cv, cw, thr), test=report(Bset, cv, cw, thr))
    cv, cw = fit(team + val, thr); r['both'] = dict(c_v=cv, c_w=cw, team=report(team, cv, cw, thr), val=report(val, cv, cw, thr), all=report(team + val, cv, cw, thr))
    res['thresholds'][str(thr)] = r
    t1, t2 = r['team_to_val'], r['val_to_team']
    print(f"thr {thr:.2f}: team->val c_v={t1['c_v']:.5f} c_w={t1['c_w']:.4f} test trans {t1['test']['trans_rms_mm']:.1f} mm rot {t1['test']['yaw_rms_rot_deg']:.2f} str-yaw {t1['test']['yaw_rms_straight_deg']:.2f} | "
          f"val->team c_v={t2['c_v']:.5f} c_w={t2['c_w']:.4f} test trans {t2['test']['trans_rms_mm']:.1f} mm rot {t2['test']['yaw_rms_rot_deg']:.2f} str-yaw {t2['test']['yaw_rms_straight_deg']:.2f} | "
          f"both c_v={r['both']['c_v']:.5f} c_w={r['both']['c_w']:.4f} all trans {r['both']['all']['trans_rms_mm']:.1f} rot {r['both']['all']['yaw_rms_rot_deg']:.2f} str-yaw {r['both']['all']['yaw_rms_straight_deg']:.2f}")
json.dump(res, open(ARGS[2], 'w'), indent=2)
