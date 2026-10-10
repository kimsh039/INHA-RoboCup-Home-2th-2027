# Base–Mid360 실측 캘리브레이션 (Livox Mid-360S)

2026-10-10 · 실제 로봇 · 결과 방향 `base_link ← livox_frame`, 거리 m

| | x (mm) | y (mm) | z (mm) | roll (°) | pitch (°) | yaw (°) |
|---|---:|---:|---:|---:|---:|---:|
| CAD URDF | −180.0 | 0 | 1199.1 | 180 | 0 | 0 |
| 이전 runtime (Gazebo) | −180.0 | 0.2 | 1198.9 | ≈180 | ≈0 | ≈0 |
| **이번 실측** | **−158.0** | **0.1** | **1188.5** | **−179.88** | **2.906** | **−1.607** |

센서가 **pitch 약 2.9°** 기울어져 있고, CAD보다 22 mm 앞, 10.6 mm 아래, yaw −1.6°입니다.

## 방법

1. **roll/pitch:** 같은 자리에서 4방향으로 돌려 녹화한 정지 자세(pose01–04)의 Mid-360S 바닥 평면(0.8–3 m 고리, RANSAC 12 mm)을 맞추고 법선을 평균했습니다. pitch +2.91° (σ 0.22°), roll +0.12° (σ 0.17°)이며, 내장 IMU 중력 방향(pitch +2.3°)과 같은 쪽입니다.
2. **z:** 기울기 보정 후 바닥 평면이 URDF 바닥 높이(구동 바퀴 r=60.5 mm, 축 z=−0.082 m → 바닥 z=−0.1425 m)에 오도록 했습니다. 자세별 바닥 높이 σ 1.4 mm.
3. **x, y, yaw:** 기울기를 보정한 Mid-360S 맵에 G2 스캔을 다시 맞춰 G2↔Mid-360S 상대값을 구하고(8자세, train 6 / holdout 2), [G2의 base 실측값](../base_2dlidar/README.md)과 이었습니다.

## 검증

| 항목 | 값 |
|---|---|
| G2→Mid-360S 벽 거리 중앙 / p90 (holdout 2자세) | 6.8 / 16.0 mm, 2 cm 이내 94 % |
| 같은 자료에서 기울기 미보정 ([vs_mid360](../base_2dlidar/vs_mid360/README.md)) | 6.9 / 18.8 mm (그리고 6자유도 진단 pitch −2.91°) |
| 기울기 보정 후 6자유도 진단 pitch | 0.02° |
| 자세별 G2 상대 x 흩어짐 | pose08 외 ±3 mm, pose08 +10 mm |
| 줄자 Mid-360S 높이 (하우징 중앙) | 바닥 위 1.355 m → z=1.2125 m. 바닥 평면 값과 −24 mm 차이 |

줄자는 하우징 중앙까지 잰 값이고 Livox 점 원점과 같지 않을 수 있습니다. 점군을 `base_link`에 맞게 쓰는 목적이므로 바닥 평면에서 구한 z를 적용했습니다.

## 기록

- 처음에는 바닥 탐색 범위를 CAD `base_link` ±8 cm로 잡아, 바닥보다 약 14 cm 높은 다른 수평면을 바닥으로 잘못 맞췄습니다(기울기 0.2–1.2°로 잘못 보고). 이 값은 쓰지 않았습니다.
- Gazebo의 Head–Mid360 평면 기록과 이 실측 Mid-360S를 섞은 "LiDAR 경로" Head 값은 arm 경로와 20.8 mm / 3.66° 차이가 납니다. runtime은 arm 경로를 쓰므로 Head 위치는 바뀌지 않습니다.

## 파일

[base_mid360.json](base_mid360.json) · [T_base_mid360.csv](T_base_mid360.csv) · [solver_output.json](solver_output.json) · [tools/floor_real.py](tools/floor_real.py) · [tools/recompute_tilt.py](tools/recompute_tilt.py) · 원본 bag: Jetson `~/livox_g2_calib/bags` (SHA-256은 [../base_2dlidar/vs_mid360/calibration.json](../base_2dlidar/vs_mid360/calibration.json))
