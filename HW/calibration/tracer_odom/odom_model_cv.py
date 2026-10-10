#!/usr/bin/env python3
"""Compare odometry correction models with cross-validation between two stop-and-go sessions.

Usage (ROS 2 Humble): python3 odom_model_cv.py <team_bag_root> <validation_bag> <out_json>

team_bag_root: spin_cw, spin_ccw, straight_forward, straight_reverse (uncorrected /odom)
validation_bag: one bag recorded with the corrected driver (correction is inverted here)

Per stop-to-stop segment: reported increments  dθ_rep = ∫w_rep dt, d_rep = ∫|v_rep| dt,
signed moving times  T_w = ∫sign(w_rep)·[|w_rep|>0] dt,  T_v = ∫[|v_rep|>0] dt.
Models (yaw):  scale k·dθ_rep | scale+bias k·dθ_rep + b·T_move | offset dθ_rep + c·T_w | scale+offset k·dθ_rep + c·T_w
Models (dist): scale k·d_rep | offset d_rep + c·T_v | scale+offset.
"""
import sys, json, math
from pathlib import Path
import numpy as np
ARGS = sys.argv[1:]
sys.argv = [sys.argv[0], ARGS[0]]
exec((Path(__file__).resolve().parents[1] / 'base_2dlidar/tools/base_g2_motion.py').read_text().split('res = {}')[0])
cal = json.load(open(Path(__file__).resolve().parents[1] / 'base_2dlidar/calibration.json'))
T_bl = se2(cal['translation_m'][0], cal['translation_m'][1], math.radians(cal['yaw_deg']))
EPS = 1e-3
THRS = [1e-3, 0.02, 0.03, 0.04]          # |w_rep| threshold for the angular offset (steps of ~0.0185 rad/s)

def segments(root, name, invert):
    global B
    B = str(root).rstrip('/') + '/'
    od, sc = read(name); st = stops(od, sc); poses, _ = chain(st, name)
    v, w = od[:, 4].copy(), od[:, 5].copy()
    if invert:                                    # recorded with the corrected driver: v=1.112 v_rep, w=1.319 w_rep + 0.00378 (moving)
        v /= 1.112; mv = (np.abs(v) > EPS) | (np.abs(w) > EPS); w = (w - 0.00378 * mv) / 1.319
    dt = np.diff(od[:, 0], append=od[-1, 0])
    out = []
    for i in range(1, len(st)):
        k = (od[:, 0] >= st[i - 1]['t']) & (od[:, 0] < st[i]['t'])
        Tb = T_bl @ inv(poses[i - 1]) @ poses[i] @ inv(T_bl)
        mvw = np.abs(w[k]) > EPS; mvv = np.abs(v[k]) > EPS
        out.append(dict(src=name, th_scan=yaw(Tb), d_scan=math.hypot(Tb[0, 2], Tb[1, 2]),
                        th_rep=float(np.sum(w[k] * dt[k])), d_rep=float(np.sum(np.abs(v[k]) * dt[k])),
                        Tw=float(np.sum(np.sign(w[k]) * mvw * dt[k])), Tmove=float(np.sum(((np.abs(v[k]) > EPS) | mvw) * dt[k])),
                        Tv=float(np.sum(mvv * dt[k])),
                        Tw_thr={str(t): float(np.sum(np.sign(w[k]) * (np.abs(w[k]) > t) * dt[k])) for t in THRS},
                        th_rep_thr={str(t): float(np.sum(w[k] * (np.abs(w[k]) > t) * dt[k])) for t in THRS}))
    return out

team = sum([segments(ARGS[0], n, False) for n in ['spin_cw', 'spin_ccw', 'straight_forward', 'straight_reverse']], [])
vp = Path(ARGS[1]).resolve(); val = segments(vp.parent, vp.name, True)

def design(S, kind, model):
    if kind == 'yaw':
        y = np.array([s['th_scan'] for s in S]); a = np.array([s['th_rep'] for s in S])
        cols = {'scale': [a], 'scale+bias': [a, np.array([s['Tmove'] for s in S])], 'offset': [np.array([s['Tw'] for s in S])],
                'scale+offset': [a, np.array([s['Tw'] for s in S])]}[model]
        base = a if model == 'offset' else 0 * a
    else:
        y = np.array([s['d_scan'] for s in S]); a = np.array([s['d_rep'] for s in S])
        cols = {'scale': [a], 'offset': [np.array([s['Tv'] for s in S])], 'scale+offset': [a, np.array([s['Tv'] for s in S])]}[model]
        base = a if model == 'offset' else 0 * a
    return np.column_stack(cols), y - base, base

def fit(S, kind, model):
    X, y, _ = design(S, kind, model); p, *_ = np.linalg.lstsq(X, y, rcond=None); return p
def err(S, kind, model, p):
    X, y, base = design(S, kind, model); r = (X @ p) - y
    return r
sel = lambda S, kind: [s for s in S if (abs(s['th_scan']) > math.radians(10) if kind == 'yaw' else s['d_scan'] > 0.10)] if kind == 'dist' else S
res = {}
for kind, models in [('yaw', ['scale', 'scale+bias', 'offset', 'scale+offset']), ('dist', ['scale', 'offset', 'scale+offset'])]:
    for model in models:
        row = {}
        for tr_name, tr, te_name, te in [('team', team, 'val', val), ('val', val, 'team', team)]:
            p = fit(sel(tr, kind), kind, model)
            r_in = err(sel(tr, kind), kind, model, p); r_out = err(sel(te, kind), kind, model, p)
            unit = (lambda x: np.degrees(x)) if kind == 'yaw' else (lambda x: x * 1000)
            row[f'fit_{tr_name}'] = dict(params=p.tolist(), rms_in=float(unit(np.sqrt(np.mean(r_in ** 2)))), rms_out=float(unit(np.sqrt(np.mean(r_out ** 2)))))
        pj = fit(sel(team + val, kind), kind, model); rj = err(sel(team + val, kind), kind, model, pj)
        row['fit_both'] = dict(params=pj.tolist(), rms=float((np.degrees if kind == 'yaw' else (lambda x: x * 1000))(np.sqrt(np.mean(rj ** 2)))))
        res[f'{kind}:{model}'] = row
        u = '°' if kind == 'yaw' else 'mm'
        print(f"{kind:4s} {model:13s} | team->val: params {np.round(row['fit_team']['params'],5).tolist()} in {row['fit_team']['rms_in']:.2f}{u} out {row['fit_team']['rms_out']:.2f}{u} "
              f"| val->team: params {np.round(row['fit_val']['params'],5).tolist()} in {row['fit_val']['rms_in']:.2f}{u} out {row['fit_val']['rms_out']:.2f}{u} | both {np.round(pj,5).tolist()} rms {row['fit_both']['rms']:.2f}{u}")

# angular offset only above a |w_rep| threshold (straight-line blips of one step excluded); per motion type
print()
print('angular offset with threshold (fit on one session, test on the other), RMS per segment [rotation | straight]:')
thr_res = {}
for t in THRS:
    out = {}
    for tr_name, tr, te in [('team', team, val), ('val', val, team)]:
        x = np.array([s['Tw_thr'][str(t)] for s in tr]); y = np.array([s['th_scan'] - s['th_rep'] for s in tr]); c = float(x @ y / (x @ x))
        rot = [s for s in te if abs(s['th_scan']) > math.radians(10)]; stg = [s for s in te if s['d_scan'] > 0.10 and abs(s['th_scan']) < math.radians(5)]
        e = lambda S: float(np.degrees(np.sqrt(np.mean([(s['th_rep'] + c * s['Tw_thr'][str(t)] - s['th_scan']) ** 2 for s in S])))) if S else float('nan')
        e0 = lambda S: float(np.degrees(np.sqrt(np.mean([(s['th_rep'] - s['th_scan']) ** 2 for s in S])))) if S else float('nan')
        out[tr_name] = (c, e(rot), e(stg), e0(rot), e0(stg))
    thr_res[str(t)] = {k: dict(zip(('c_w', 'rot_rms_deg', 'straight_rms_deg', 'uncorr_rot_rms_deg', 'uncorr_straight_rms_deg'), v)) for k, v in out.items()}
    print(f"  thr {t:.3f}: team->val c={out['team'][0]:.4f} rot {out['team'][1]:.2f}° str {out['team'][2]:.2f}°  |  val->team c={out['val'][0]:.4f} rot {out['val'][1]:.2f}° str {out['val'][2]:.2f}°"
          f"   (uncorrected: val rot {out['team'][3]:.2f}° str {out['team'][4]:.2f}° / team rot {out['val'][3]:.2f}° str {out['val'][4]:.2f}°)")

# Note: the distance models above compare the path length with the net scan displacement and are only
# indicative; the deployed linear offset is fitted by SE(2) integration in odom_fit_se2.py.
json.dump(dict(team_segments=team, val_segments=val, models=res, angular_offset_thresholds=thr_res), open(ARGS[2], 'w'), indent=2)
