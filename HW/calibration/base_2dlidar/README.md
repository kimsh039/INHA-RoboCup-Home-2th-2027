# Base–2D LiDAR 실측 캘리브레이션 (YDLIDAR G2, 운동 기하)

2026-10-10 · 실제 로봇 · 결과 방향 `base_link ← laser_frame`, 거리 m

**결과: x=3.05 mm, y=6.76 mm, yaw=1.2176°** (z=0.34271 m, roll=pitch=0은 CAD 고정)

| | x (mm) | y (mm) | yaw (°) |
|---|---:|---:|---:|
| CAD URDF | 0.175 | 0 | 8.8095 |
| 이전 runtime (Gazebo) | 0.124 | 0.521 | 8.8448 |
| **이번 실측** | **3.05** | **6.76** | **1.2176** |

## 정의와 방법

- `base_link` 원점 = 차동 구동의 제자리 회전 중심(구동 바퀴 축 중점), `+x` = 직진 방향으로 정의해 측정했습니다.
- 팀이 녹화한 stop-and-go bag 4개(`spin_cw`, `spin_ccw`, `straight_forward`, `straight_reverse`)를 썼습니다. 정지 구간마다 G2 빔별 중앙값 스캔을 만들어 모션 왜곡을 없앴습니다.
- 정지 스캔끼리 2D point-to-line ICP로 상대 자세를 구했습니다. odometry는 ICP 초기값으로만 쓰고, 크기 값은 쓰지 않았습니다([odom은 회전 약 25%, 거리 약 10% 작게 보고](../tracer_odom/README.md)).
- 회전: 모든 정지 자세에서 움직이지 않는 점 `R_i p + t_i = q`를 최소제곱으로 풀어 laser 좌표의 base 원점 `p`를 구했습니다.
- 직진: 정지 사이 base 원점 이동 방향(laser 좌표)으로 yaw를 구했습니다.

## 일관성

| 항목 | 값 |
|---|---|
| 회전 중심 (laser 좌표) CW / CCW | (−2.1, −7.5) / (−4.3, −5.9) mm, 차이 2.7 mm |
| 원 맞춤 잔차 중앙값 CW / CCW | 2.0 / 2.5 mm (정지 9자세씩, bootstrap σ < 1 mm) |
| yaw 전진 / 후진 | 1.208° / 1.227° (구간 σ 0.18° / 0.39°) |
| 정지 스캔 정합 | 대응 65–76 %, 중앙 잔차 0.9–2.9 mm |
| 팀 odom hand-eye 후보 yaw | 약 1.0–1.15° |
| 줄자 G2 높이 | 바닥 위 0.4811 m (CAD 0.4852 m) |

## 한계

- 제자리 회전 중심이 구동 바퀴 축 중점이라고 가정했습니다. 회전 중 미끄러짐이 커서(odom 대비 약 32 %) 중심이 수 mm 이동했을 수 있습니다. CW/CCW 차이는 2.7 mm였습니다.
- z/roll/pitch는 평면 운동으로 정해지지 않아 CAD 값을 유지했습니다.

## 파일

- [calibration.json](calibration.json), [T_base_lidar.csv](T_base_lidar.csv), [solver_output.json](solver_output.json), [tools/base_g2_motion.py](tools/base_g2_motion.py)
- 원본 bag은 Jetson `~/tracer_ws/calibration_bags/g2_scan_only_20261010_230526`에 있고 SHA-256은 `calibration.json`의 `bags`에 있습니다.
- [vs_mid360/](vs_mid360/README.md): 같은 날 G2를 기울기 보정 없는 CAD Mid-360S 기준으로 맞춘 첫 결과입니다. Mid-360S가 2.9° 기울어져 있어 `base_link` 기준값으로는 맞지 않으며 기록으로만 남깁니다. 기울기 보정한 상대값은 [base_mid360](../base_mid360/README.md)에 있습니다.
- 적용: `SW/simulation/calibration/integrate_calibration.py --replace`가 `calibration.json`을 읽어 `laser_frame_joint`를 생성합니다.
