# Tracer odometry 보정

2026-10-10 · 실제 로봇 · Tracer 펌웨어가 보고하는 선속도·각속도에 대한 배율과 치우침

`tracer_ros2`의 odom은 펌웨어가 보내는 `v`, `w`를 그대로 적분합니다. 정지 구간 사이의 실제 base 운동(G2 스캔 정합 + [base_2dlidar](../base_2dlidar/README.md))과 비교하면 회전은 약 25 %, 거리는 약 10 % 작게 보고됩니다.

**보정식:** `v = 1.112 · v_rep`, `w = 1.319 · w_rep + 0.00378 rad/s` (이동 중, |v_rep| 또는 |w_rep| > 1e-3)

| 항목 | 값 |
|---|---|
| 선속도 배율 | 1.112 (전진 1.108, 후진 1.116, 구간 1.09–1.16) |
| 각속도 배율 | 1.319 (치우침과 함께 적합, 구간 1.23–1.47) |
| 각속도 치우침 | +0.00378 rad/s (이동 중) |
| 정지 구간당 yaw 잔차 RMS (배율만 → 배율+치우침) | 직진 1.31° → 0.34°, 회전 2.10° → 2.01° |

회전 잔차가 남는 것은 회전할 때마다 미끄러짐이 달라서입니다. 모터 RPM 기준 유효 바퀴 간격은 약 287 mm(URDF 340 mm)입니다.

## 적용

Jetson `~/tracer_ws/src/tracer_ros2`(agilexrobotics 원본)의 로컬 브랜치 `odom-correction`, 커밋 `c10aa90`. 변경 내용은 [0001-tracer-odom-correction.patch](0001-tracer-odom-correction.patch)에 있습니다.

- 파라미터 `linear_velocity_scale`, `angular_velocity_scale`, `angular_velocity_bias`. C++ 기본값은 1.0 / 1.0 / 0.0이고, `tracer_base.launch.py`의 기본값을 위 측정값으로 넣었습니다.
- `/odom`의 pose와 twist, `odom → base_link` TF에 적용합니다. `/tracer_status`는 펌웨어 원래 값입니다.
- 보정 끄기: `ros2 launch tracer_base tracer_base.launch.py ... linear_velocity_scale:=1.0 angular_velocity_scale:=1.0 angular_velocity_bias:=0.0`
- 다른 Tracer에 적용하기: `git -C ~/tracer_ws/src/tracer_ros2 am <patch>` 후 `colcon build --packages-select tracer_base`

`/cmd_vel`은 그대로 펌웨어에 전달되므로, 로봇은 명령보다 약 11 % 빠르게 가고 약 32 % 더 회전합니다. Nav2 속도 한계를 정할 때 고려해야 합니다.

## 파일

[odom_calibration.json](odom_calibration.json) · [odom_scale.py](odom_scale.py) · [odom_bias.py](odom_bias.py) · 원본 bag: Jetson `~/tracer_ws/calibration_bags/g2_scan_only_20261010_230526`
