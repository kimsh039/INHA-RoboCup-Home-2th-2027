#!/usr/bin/env python3
"""Validate corrected Tracer odometry against G2 scan matching on a stop-and-go bag.

Usage (ROS 2 Humble): python3 odom_validate.py <bag_dir> <out_json> [offset|scale|none]

The last argument names the correction that was active in the driver while recording
(default: offset, the current model; scale = previous model c10aa90; none = uncorrected).
Stops are detected from /odom twist. Base motion between consecutive stops comes from G2
scan matching (base_link <- laser_frame of ../base_2dlidar). The recorded /odom is compared
with it, and so is the uncorrected firmware odometry reconstructed by inverting the
active correction on the recorded twist.
"""
import sys, json, math
from pathlib import Path
import numpy as np
ARGS = sys.argv[1:]
MODEL = ARGS[2] if len(ARGS) > 2 else 'offset'
bag = Path(ARGS[0]).resolve()
sys.argv = [sys.argv[0], str(bag.parent)]
exec((Path(__file__).resolve().parents[1] / 'base_2dlidar/tools/base_g2_motion.py').read_text().split('res = {}')[0])
cal = json.load(open(Path(__file__).resolve().parents[1] / 'base_2dlidar/calibration.json'))
oc = json.load(open(Path(__file__).resolve().parent / 'odom_calibration.json'))
T_bl = se2(cal['translation_m'][0], cal['translation_m'][1], math.radians(cal['yaw_deg']))

od, sc = read(bag.name)
st = stops(od, sc)
poses, _ = chain(st, bag.name)

# uncorrected firmware odometry from the recorded twist
v_c, w_c = od[:, 4], od[:, 5]
if MODEL == 'scale':
    pm = oc['previous_scale_model']; KV, KW, BW = pm['linear_velocity_scale'], pm['angular_velocity_scale'], pm['angular_velocity_bias_rad_s']
    v_rep = v_c / KV
    moving = (np.abs(v_rep) > 1e-3) | (np.abs(w_c) > 1e-3)
    w_rep = (w_c - BW * moving) / KW
elif MODEL == 'offset':
    OV, OW, TW = oc['linear_velocity_offset_m_s'], oc['angular_velocity_offset_rad_s'], oc['angular_offset_threshold_rad_s']
    v_rep = np.where(np.abs(v_c) > OV + 1e-3, v_c - np.sign(v_c) * OV, v_c)
    w_rep = np.where(np.abs(w_c) > OW + max(TW, 1e-3), w_c - np.sign(w_c) * OW, w_c)
else:
    v_rep, w_rep = v_c.copy(), w_c.copy()
raw = np.zeros((len(od), 3))
for i in range(1, len(od)):
    dt = od[i, 0] - od[i - 1, 0]; th = raw[i - 1, 2]
    raw[i] = raw[i - 1] + [v_rep[i - 1] * math.cos(th) * dt, v_rep[i - 1] * math.sin(th) * dt, w_rep[i - 1] * dt]
def raw_pose(t):
    k = np.searchsorted(od[:, 0], t); k = min(max(k, 0), len(od) - 1); return se2(*raw[k])

rows = []
for i in range(1, len(st)):
    Tb = T_bl @ inv(poses[i - 1]) @ poses[i] @ inv(T_bl)                       # base_{i-1} <- base_i (scan)
    Tc = inv(st[i - 1]['odom']) @ st[i]['odom']                                  # corrected odom
    Tr = inv(raw_pose(st[i - 1]['t'])) @ raw_pose(st[i]['t'])                    # uncorrected
    d = [math.hypot(T[0, 2], T[1, 2]) for T in (Tb, Tc, Tr)]; y = [yaw(T) for T in (Tb, Tc, Tr)]
    kind = 'rotate' if abs(y[0]) > math.radians(10) and d[0] < 0.05 else ('straight' if d[0] > 0.10 and abs(y[0]) < math.radians(5) else 'mixed')
    if d[0] < 0.01 and abs(y[0]) < math.radians(1): kind = 'none'
    rows.append(dict(kind=kind, scan_dist_m=d[0], odom_dist_m=d[1], raw_dist_m=d[2], scan_yaw_deg=math.degrees(y[0]),
                     odom_yaw_deg=math.degrees(y[1]), raw_yaw_deg=math.degrees(y[2]),
                     pos_err_odom_mm=float(np.hypot(*(Tc[:2, 2] - Tb[:2, 2])) * 1000), pos_err_raw_mm=float(np.hypot(*(Tr[:2, 2] - Tb[:2, 2])) * 1000)))
print(f'{"#":>2} {"kind":8s} {"dist scan/odom/raw (mm)":>28s} {"yaw scan/odom/raw (deg)":>30s} {"pos err odom/raw (mm)":>22s}')
for k, r in enumerate(rows, 1):
    print(f"{k:2d} {r['kind']:8s} {r['scan_dist_m']*1000:8.1f} {r['odom_dist_m']*1000:8.1f} {r['raw_dist_m']*1000:8.1f}   "
          f"{r['scan_yaw_deg']:+8.2f} {r['odom_yaw_deg']:+8.2f} {r['raw_yaw_deg']:+8.2f}   {r['pos_err_odom_mm']:8.1f} {r['pos_err_raw_mm']:8.1f}")
summ = {}
R = [r for r in rows if r['kind'] == 'rotate']; L = [r for r in rows if r['kind'] == 'straight']; M = [r for r in rows if r['kind'] == 'mixed']
if R:
    s = sum(abs(r['scan_yaw_deg']) for r in R)
    summ['rotation'] = dict(n=len(R), scan_total_deg=s, odom_ratio=sum(abs(r['odom_yaw_deg']) for r in R) / s, raw_ratio=sum(abs(r['raw_yaw_deg']) for r in R) / s,
                            odom_yaw_err_rms_deg=float(np.sqrt(np.mean([(r['odom_yaw_deg'] - r['scan_yaw_deg']) ** 2 for r in R]))),
                            raw_yaw_err_rms_deg=float(np.sqrt(np.mean([(r['raw_yaw_deg'] - r['scan_yaw_deg']) ** 2 for r in R]))))
if L:
    s = sum(r['scan_dist_m'] for r in L)
    summ['straight'] = dict(n=len(L), scan_total_m=s, odom_ratio=sum(r['odom_dist_m'] for r in L) / s, raw_ratio=sum(r['raw_dist_m'] for r in L) / s,
                            odom_yaw_drift_rms_deg=float(np.sqrt(np.mean([(r['odom_yaw_deg'] - r['scan_yaw_deg']) ** 2 for r in L]))),
                            raw_yaw_drift_rms_deg=float(np.sqrt(np.mean([(r['raw_yaw_deg'] - r['scan_yaw_deg']) ** 2 for r in L]))),
                            odom_pos_err_mean_mm=float(np.mean([r['pos_err_odom_mm'] for r in L])), raw_pos_err_mean_mm=float(np.mean([r['pos_err_raw_mm'] for r in L])))
if M:
    summ['mixed'] = dict(n=len(M), odom_pos_err_mean_mm=float(np.mean([r['pos_err_odom_mm'] for r in M])), raw_pos_err_mean_mm=float(np.mean([r['pos_err_raw_mm'] for r in M])),
                         odom_yaw_err_rms_deg=float(np.sqrt(np.mean([(r['odom_yaw_deg'] - r['scan_yaw_deg']) ** 2 for r in M]))),
                         raw_yaw_err_rms_deg=float(np.sqrt(np.mean([(r['raw_yaw_deg'] - r['scan_yaw_deg']) ** 2 for r in M]))))
# whole-run endpoint
Tb = T_bl @ inv(poses[0]) @ poses[-1] @ inv(T_bl); Tc = inv(st[0]['odom']) @ st[-1]['odom']; Tr = inv(raw_pose(st[0]['t'])) @ raw_pose(st[-1]['t'])
summ['endpoint'] = dict(scan=dict(x_mm=Tb[0, 2] * 1000, y_mm=Tb[1, 2] * 1000, yaw_deg=math.degrees(yaw(Tb))),
                        odom_err=dict(pos_mm=float(np.hypot(*(Tc[:2, 2] - Tb[:2, 2])) * 1000), yaw_deg=math.degrees(yaw(inv(Tb) @ Tc))),
                        raw_err=dict(pos_mm=float(np.hypot(*(Tr[:2, 2] - Tb[:2, 2])) * 1000), yaw_deg=math.degrees(yaw(inv(Tb) @ Tr))))
print(json.dumps(summ, indent=1, ensure_ascii=False))
json.dump(dict(bag=str(bag.name), stops=len(st), segments=rows, summary=summ, recorded_with=MODEL), open(ARGS[1], 'w'), indent=2)
