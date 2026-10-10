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

## 독립 확인과 실측 (2026-10-11)

| 항목 | 값 | 판단 |
|---|---|---|
| Mid-360S만으로 구한 제자리 회전 중심 기준 x / y ([tools/mid_rotation_check.py](tools/mid_rotation_check.py)) | −157.0 / +3.4 mm | G2 경유 값 −158.8 / +0.8 mm와 1.8 / 2.6 mm 차이 |
| 줄자 직선거리 (바닥 기준점 → 하우징 중앙) | 136 cm | 예상 136.4 cm |
| 줄자 앞뒤 거리 (**로봇을 옆으로 눕힌 상태**) | −173.5 mm | 서 있는 상태 값과 약 15 mm 차이 → 아래 기둥 기울기 |
| 휴대폰 수평계: 기둥 앞쪽 기울기 / Mid-360S 기울기 | 약 2° / 약 2° | IMU 2.29°(중력), 바닥 평면 2.85°(바닥 기준, 방 바닥 약 0.5° 경사) |

**해석:** Mid-360S가 달린 약 1.2 m 기둥이 앞으로 기울어 있습니다. 랙 아래를 축으로 1° 기울면 센서가 −159.5 mm(이 캘리브레이션)로, 0.3°면 −173.5 mm(눕혀서 잰 값)로 옵니다. 기울기 일부는 하중에 따라 달라지는 것으로 보여, **서 있는 상태에서 잰 이 캘리브레이션 값을 운용 값으로 씁니다.** Mid-360S 자체 기울기(2.85°)가 기둥 기울기보다 큰 것은 브래킷 장착 각도가 더해진 것으로 보입니다.

**권고:** 기둥 보강(대각 보강재, 연결부 볼트). 기계 구조를 바꾸면 Mid-360S를 다시 캘리브레이션합니다. 주행 중 흔들림은 `/livox/imu`, `/livox/lidar`를 함께 녹화해 확인할 수 있습니다.

## 한계

- **절대 z는 줄자와 약 5 mm 안에서 맞습니다.** Livox 매뉴얼(Mid-360 v1.2 부록, Mid-360S v1.0 p.16)상 점군 원점은 장착면에서 47.0 mm(하우징 높이 60 mm)이므로, 거꾸로 장착된 이 로봇에서는 하우징 중앙보다 17 mm 아래입니다. 줄자(하우징 중앙 바닥 위 1.355 m) → 원점 1.338 m → z 1.1955 m, 바닥 평면 값 1.1906 m와 −4.9 mm 차이입니다(줄자 중심 잡기·평면 맞춤 오차 수준).
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
