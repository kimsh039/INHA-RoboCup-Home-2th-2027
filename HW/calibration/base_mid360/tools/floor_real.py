# 진짜 바닥 찾기: 공칭 base' 좌표 z 히스토그램 + 넓은 범위 RANSAC 평면 (자세 1~4)
import sys, json, numpy as np
sys.argv = ['x', 'none', 'none']
src = open('/mnt/c/Users/wwoo5/Desktop/INHA-RoboCup-Home-2th-2027/HW/calibration/base_2dlidar/vs_mid360/tools/calib_g2_livox.py').read().split("if BAGDIR == 'prep':")[0]
exec(src)
out = {}
for n in ['pose01', 'pose02', 'pose03', 'pose04']:
    clouds, scans, acc = read_bag('/home/wwoo5241/livox_g2_calib/bags/' + n)
    P = np.vstack(clouds); del clouds
    P = P[np.linalg.norm(P, axis=1) > 0.3]; P = (R_BL @ P.T).T + T_BL; P = voxel(P, 0.015)
    rr = np.hypot(P[:, 0], P[:, 1]); ring = P[(rr > 0.8) & (rr < 3.0)]
    h, e = np.histogram(ring[:, 2], bins=np.arange(-0.40, 0.40, 0.01))
    top = np.argsort(h)[::-1][:4]
    print(n, 'z peaks (m, count):', [(round(float(e[i] + 0.005), 3), int(h[i])) for i in top])
    # 기울어진 바닥도 들어오도록 넓은 범위 + RANSAC(거의 수평, 가장 많은 inlier)
    zf = None
    F = ring[(ring[:, 2] > -0.40) & (ring[:, 2] < 0.05)]
    best = None; rng = np.random.default_rng(0)
    for _ in range(600):
        s = F[rng.choice(len(F), 3, replace=False)]; nrm = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(nrm) < 1e-9: continue
        nrm /= np.linalg.norm(nrm)
        if abs(nrm[2]) < np.cos(np.radians(8)): continue
        k = np.abs((F - s[0]) @ nrm) < 0.012
        if best is None or k.sum() > best.sum(): best = k
    for _ in range(3):   # 최소제곱 재정련
        G = F[best]; c = G.mean(0); _, _, vt = np.linalg.svd(G - c, full_matrices=False); nn = vt[2] * np.sign(vt[2][2])
        best = np.abs((F - c) @ nn) < 0.01
    roll = np.degrees(np.arctan2(nn[1], nn[2])); pitch = np.degrees(-np.arctan2(nn[0], nn[2]))
    z0 = c[2] + (nn[0] * c[0] + nn[1] * c[1]) / nn[2]          # base' 원점 바로 아래의 바닥 높이
    out[n] = dict(height_at_origin_mm=float(z0 * 1000), centroid_mm=(c * 1000).tolist(), normal=nn.tolist(), roll_deg=float(roll), pitch_deg=float(pitch),
                  n=int(best.sum()), inlier=float(best.mean()))
    print(f'   floor: z@origin={z0*1000:+.1f} mm roll={roll:+.3f} pitch={pitch:+.3f} n={best.sum()} ({best.mean()*100:.0f}% of band)')
m = {k: float(np.mean([v[k] for v in out.values()])) for k in ('height_at_origin_mm', 'roll_deg', 'pitch_deg')}
s = {k: float(np.std([v[k] for v in out.values()], ddof=1)) for k in ('height_at_origin_mm', 'roll_deg', 'pitch_deg')}
print('mean:', {k: round(v, 3) for k, v in m.items()}, ' std:', {k: round(v, 3) for k, v in s.items()})
json.dump(dict(per_pose=out, mean=m, std=s), open('/home/wwoo5241/livox_g2_calib/floor_real.json', 'w'), indent=2)
