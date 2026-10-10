# Base–Mid360 실측 캘리브레이션 (Livox Mid-360S)

2026-10-10 · 실제 로봇 · 결과 방향 `base_link ← livox_frame`, 거리 m

| | x (mm) | y (mm) | z (mm) | roll (°) | pitch (°) | yaw (°) |
|---|---:|---:|---:|---:|---:|---:|
| CAD URDF | −180.0 | 0 | 1199.1 | 180 | 0 | 0 |
| 이전 runtime (Gazebo) | −180.0 | 0.2 | 1198.9 | ≈180 | ≈0 | ≈0 |
| **이번 실측** | **−158.8** | **0.8** | **1190.6** | **−179.92** | **2.847** | **−1.609** |

센서가 **pitch 약 2.85°** 기울어져 있고, CAD보다 21 mm 앞, 8.5 mm 아래, yaw −1.6°입니다.

## 방법

1. **roll/pitch:** 같은 자리에서 방향만 바꾼 정지 자세 pose01–03의 Mid-360S 바닥 평면(0.8–3 m 고리, RANSAC 12 mm)을 맞추고 법선을 평균했습니다. pitch +2.85° (σ 0.22°), roll +0.08° (σ 0.18°). hold-out(pose04, pose08)은 쓰지 않았습니다.
2. **z:** 기울기 보정 후 바닥이 URDF 바닥 높이(구동 바퀴 r=60.5 mm, 축 z=−0.082 m → −0.1425 m)에 오도록, 센서 원점-바닥 평면 거리의 평균(1333.1 mm, σ 1.7 mm)을 썼습니다.
3. **x, y, yaw:** 기울기를 보정한 Mid-360S 맵에 G2 스캔을 다시 맞춰 G2↔Mid-360S 상대값을 구하고(train pose01, 02, 03, 05, 06, 07), [G2의 base 실측값](../base_2dlidar/README.md)과 이었습니다. `base_mid360.json`의 `g2_record_sha256`이 그 G2 기록을 가리키며, 통합 스크립트가 일치를 확인합니다.

## 검증 (hold-out pose04, pose08)

G2 점과 Mid-360S 벽면 사이 점-평면 거리(20 cm 이내 대응점):

| 조건 | 중앙값 | p90 | 20 mm 이내 |
|---|---:|---:|---:|
| 기울기 미보정 CAD Mid-360S + 그때 맞춘 G2 ([vs_mid360](../base_2dlidar/vs_mid360/README.md)) | 6.87 mm | 18.8 mm | 91 % |
| 기울기 보정 + 다시 맞춘 상대값 | 6.75 mm | 16.2 mm | 94 % |
| **최종 적용 값 (G2 운동 기하 + 이 Mid-360S)** | **6.58 mm** | **16.4 mm** | **94 %** |

- 기울기 보정 후 6자유도 진단: roll −0.07°, pitch −0.07° (보정 전 pitch −2.91°).
- 최종 값으로 hold-out 바닥을 다시 맞추면 roll/pitch 0.1–0.25°, 바닥 높이 +1.8 / +5.1 mm ([tools/check_poses.py](tools/check_poses.py)).

## 한계

- **절대 z는 독립적으로 검증되지 않았습니다.** 줄자(하우징 중앙, 바닥 위 1.355 m → z 1.2125 m)와 −21.9 mm 다릅니다. Livox 점 원점의 하우징 내 위치를 확인하면 이 차이를 설명하거나 고칠 수 있습니다.
- 바닥 기울기는 한 자리에서 잰 값입니다. 다른 장소에서 [tools/check_poses.py](tools/check_poses.py)로 확인할 수 있습니다.
- 내장 IMU 중력 방향은 바닥 법선과 같은 쪽으로 기울지만 0.5–0.9° 다릅니다. 방향 확인용으로만 썼습니다.
- 처음에는 바닥 탐색 범위를 CAD `base_link` ±8 cm로 잡아 바닥보다 약 14 cm 높은 다른 수평면을 맞췄습니다(기울기 0.2–1.2°로 잘못 보고). 이 값은 쓰지 않습니다.
- Gazebo의 Head–Mid360 평면 기록과 이 실측 Mid-360S를 섞은 "LiDAR 경로" Head 값은 arm 경로와 차이가 커졌습니다. runtime은 arm 경로를 쓰므로 Head 위치는 바뀌지 않습니다.

## 재현

```bash
python3 tools/floor_real.py ~/livox_g2_calib/bags floor_output.json pose01 pose02 pose03
python3 tools/recompute_tilt.py <prep_dir> floor_output.json ../base_2dlidar/calibration.json solver_output.json
python3 tools/check_poses.py ~/livox_g2_calib/bags check.json pose11 pose12 ...
```

`<prep_dir>`는 `../base_2dlidar/vs_mid360/tools/calib_g2_livox.py prep`의 자세별 npz입니다. 원본 bag(Jetson `~/livox_g2_calib/bags`)의 SHA-256은 [../base_2dlidar/vs_mid360/calibration.json](../base_2dlidar/vs_mid360/calibration.json)에 있습니다.

파일: [base_mid360.json](base_mid360.json) · [T_base_mid360.csv](T_base_mid360.csv) · [solver_output.json](solver_output.json) · [floor_output.json](floor_output.json)
