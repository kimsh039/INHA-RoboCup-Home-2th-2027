# 이전 배율+치우침 모델(c10aa90) 계산용. 이동 판정 0.004/0.008은 그 모델의 값이며 현재 offset 모델과 다릅니다.
# 사용: python3 odom_bias.py <bag_root> <odom_scale.json> <out_odom_bias.json>
import math, json, numpy as np
import sys
from pathlib import Path
ARGS = sys.argv[1:]
sys.argv = [sys.argv[0], ARGS[0]]          # base_g2_motion.py reads the bag root from argv[1]
src = (Path(__file__).resolve().parents[1] / 'base_2dlidar/tools/base_g2_motion.py').read_text().split('res = {}')[0]
exec(src)
seg = json.load(open(ARGS[1]))['segments']
# 구간별 이동 시간(odom 기준 moving) 다시 계산
tm = []
for name in ['spin_cw', 'spin_ccw', 'straight_forward', 'straight_reverse']:
    od, sc = read(name); st = stops(od, sc)
    mv = (np.abs(od[:, 4]) > 0.004) | (np.abs(od[:, 5]) > 0.008); dt = np.diff(od[:, 0], prepend=od[0, 0])
    for i in range(1, len(st)):
        k = (od[:, 0] >= st[i - 1]['t']) & (od[:, 0] <= st[i]['t']); tm.append(float(np.sum(dt[k & mv])))
th_s = np.array([s['scan_yaw_rad'] for s in seg]); th_o = np.array([s['odom_yaw_rad'] for s in seg]); T = np.array(tm)
A = np.c_[th_o, T]; (k_w, b_w), *_ = np.linalg.lstsq(A, th_s, rcond=None); r = th_s - A @ [k_w, b_w]
k0 = np.sum(np.abs(th_s[:16])) / np.sum(np.abs(th_o[:16])); r0 = th_s - k0 * th_o
print(f'배율만      : k_w={k0:.4f}                      -> 잔차 RMS {np.degrees(np.sqrt(np.mean(r0**2))):.2f}°  (spin {np.degrees(np.sqrt(np.mean(r0[:16]**2))):.2f}°, straight {np.degrees(np.sqrt(np.mean(r0[16:]**2))):.2f}°)')
print(f'배율+치우침 : k_w={k_w:.4f}, bias={b_w:+.5f} rad/s ({math.degrees(b_w):+.3f}°/s, 이동 중) -> 잔차 RMS {np.degrees(np.sqrt(np.mean(r**2))):.2f}°  (spin {np.degrees(np.sqrt(np.mean(r[:16]**2))):.2f}°, straight {np.degrees(np.sqrt(np.mean(r[16:]**2))):.2f}°)')
# 치우침을 odom 단위(펌웨어 보고값)로: w_true = k_w*(w_rep + b_rep) -> b_rep = b_w/k_w
print(f'  펌웨어 보고 각속도 기준 치우침 = {b_w/k_w:+.5f} rad/s  (보정식: w = k_w * w_rep + {b_w:+.5f}  [이동 중])')
cw = np.arange(8); ccw = np.arange(8, 16)
for nm, ix in [('cw', cw), ('ccw', ccw)]:
    print(f'  spin_{nm}: 치우침 반영 시 구간별 k = {np.round((th_s[ix]-b_w*T[ix])/th_o[ix],3).tolist()}')
json.dump(dict(k_w=k_w, bias_w_rad_s=b_w, k_w_scale_only=k0, moving_time_s=tm), open(ARGS[2], 'w'), indent=2)
