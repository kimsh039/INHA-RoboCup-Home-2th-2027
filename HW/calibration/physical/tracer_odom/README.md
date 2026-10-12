# Tracer odometry 보정

2026-10-11 · 실제 로봇 · Tracer 펌웨어가 보고하는 선속도·각속도 보정, 그리고 IMU yaw odometry

`tracer_ros2`의 odom은 펌웨어가 보내는 `v`, `w`를 적분합니다. 이 값은 계단식(각속도 약 0.0185 rad/s, 선속도 0.003 m/s 단위)이고 실제보다 작습니다.

`/cmd_vel` 자동 주행([odom_speed_sweep.py](odom_speed_sweep.py))으로 보정했습니다.
- 회전: 속도 4단계 × ±90° × 2번
- 직진: 속도 4단계 × ±1 m × 2번
- 기준값: 정지 구간 사이의 실제 base 운동을 Mid-360S 점군 정합으로 구했습니다.

## 결론
- **직진 거리:** 보정 모델로 잘 맞고(0.2 m/s 이하에서 1 m당 5–7 mm), 세션이 바뀌어도 같습니다.
- **회전:** 펌웨어가 덜 보고하는 양이 세션마다 달라서, 바퀴 odom만으로는 90°당 2.5–4°가 한계입니다. **방향이 중요하면 IMU yaw를 쓰는 `/odom_ekf` 또는 `/odom_fused`를 씁니다**(90°당 0.1°).
- **0.34 m/s 이상 급가속 직진:** 바퀴가 미끄러져 보정이 안 됩니다. 전진과 후진의 실제 거리가 최대 ±5 % 다르고, 펌웨어가 모르는 방향 변화가 최대 4° 생깁니다. 내비게이션에서 가속을 0.5 m/s² 정도로 제한합니다.

## 결과

| 검증 주행 | 회전 90°당 yaw | 직진 1 m당 (≤0.19 m/s) | 직진 1 m당 (전체) | 직진 yaw |
|---|---:|---:|---:|---:|
| 펌웨어 그대로 (sweep 4) | 13.3° | 78 mm | 67 mm | 1.6° |
| 드라이버 `/odom` v4 (sweep 3 / 4) | 3.6° / 2.5° | 22 / 20 mm | 40 / 31 mm | 2.0° / 1.1° |
| `/odom_fused` (sweep 3 / 4) | **0.10° / 0.11°** | **5 / 7 mm** | 31 / 26 mm | 0.10° / 0.08° |
| `/odom_ekf` (sweep 4) | **0.11°** | **7 mm** | 26 mm | 0.07° |

**경로 테스트** ([odom_path_test.py](odom_path_test.py), [path_validate.py](path_validate.py)):

| | 5분 정지 yaw 변화 | 곡선 14개 (0.4–2.1 m, 90°) yaw / 위치 RMS |
|---|---:|---:|
| 드라이버 `/odom` v4 | 0 | 8.1° / 132 mm |
| `/odom_fused` | +0.06° | 0.17° / 32 mm |
| `/odom_ekf` | +0.07° | 0.17° / 31 mm |

- 곡선의 위치 오차는 경로 길이의 약 3 %로 직진보다 크고, 후진 곡선이 더 나쁩니다.
- 2 m 루프 테스트는 공간이 부족해 리모컨으로 중단했습니다. 리모컨 모드 구간은 분석에서 뺐습니다.

## 모델 v4 (드라이버 `/odom`)

- `v = 1.0362·v_rep + 0.00632·sign(v_rep)` (|v_rep| > 0.0035, 펌웨어 한 단계보다 큰 값만)
- `w = 0.9924·w_rep + 0.0343·sign(w_rep)` (|w_rep| > 0.02)
- 직진 데드밴드: |v_rep| > 0.05 m/s일 때 |w_rep| ≤ 0.0186 rad/s(한 단계)는 0으로 둡니다. 펌웨어가 직진 중 방향과 상관없이 −1 단계를 자주 보고하기 때문입니다.

sweep 1에서 1회차로 맞추고 2회차로 시험해 골랐습니다. scale / offset / scale+offset / 속도별 표를 비교했습니다([sweep1_analysis.json](sweep_20261011/sweep1_analysis.json)).

펌웨어 보고값 / 실제:

| 직진 | 0.09 m/s | 0.19 m/s | 0.34 m/s | 0.48 m/s |
|---|---:|---:|---:|---:|
| sweep 1 | 0.903 | 0.936 | 0.948 | 0.964 ± 0.05 |
| sweep 2 | 0.905 | 0.937 | 0.949 | 0.965 ± 0.05 |

| 회전 | 0.11 rad/s | 0.26 rad/s | 0.44 rad/s | 0.73 rad/s |
|---|---:|---:|---:|---:|
| sweep 1 | 0.772 | 0.890 | 0.935 | 0.968 |
| sweep 2 | 0.749 | 0.854 | 0.898 | 0.92–0.94 |

회전은 같은 명령에 대한 펌웨어 적분값이 두 sweep에서 같았는데, 실제 회전량이 달랐습니다. 펌웨어가 보지 못하는 바퀴-바닥 쪽 변화로 보입니다.

## 기준값

- **Mid-360S:** 정지 구간마다 점군을 모아 `base_link`로 옮기고, 이웃 정지 구간끼리 3자유도 점-평면 ICP로 맞춥니다([sweep_extract_mid.py](sweep_extract_mid.py), [sweep_analyze.py](sweep_analyze.py)).
- **G2와 교차 확인:** 회전은 0.07–0.11° RMS로 일치합니다([scan_utils.py](scan_utils.py)). 직진 거리는 G2 거리값이 약 0.6 % 짧아서([../base_2dlidar](../base_2dlidar/README.md)) ToF인 Mid-360S를 기준으로 씁니다.
- **IMU:** Mid-360S 자이로를 `base_link` z에 맞추면 회전 yaw가 Mid-360S와 0.09–0.11° RMS, 배율 0.999로 일치합니다.

## IMU yaw odometry

방향은 Mid-360S 자이로, 거리는 드라이버 `/odom`의 v4 선속도로 씁니다. 두 방식의 정확도가 같습니다.

**`/odom_ekf` (권장, Nav2 연동)**
- [imu_base_relay.py](imu_base_relay.py): IMU를 `base_link`로 돌리고 자이로 기준값(bias)을 빼서 `/imu/base`로 냅니다. `robot_localization`이 자이로 기준값을 추정하지 않는데, Livox 자이로 z 기준값이 약 0.013 rad/s(분당 약 45°)라 이 단계가 필요합니다.
- [ekf_odom_imu.yaml](ekf_odom_imu.yaml): `/odom`에서는 vx, vy만, `/imu/base`에서는 yaw rate만 씁니다. 2D 모드이고 TF는 내지 않습니다.

**`/odom_fused`**
- [odom_imu_fusion.py](odom_imu_fusion.py): 같은 계산을 노드 하나로 합니다. 출력은 `odom_fused → base_link`이고 TF는 내지 않습니다.

**자이로 기준값 처리**
- 처음 2초 정지 평균으로 잡습니다.
- 이후에는 바퀴가 1.5초 이상 정지해 있고 |gz − bias| < 0.004 rad/s일 때만 20초 평균으로 갱신합니다.
- **정지 중에도 yaw를 고정하지 않습니다.** 펌웨어가 0을 보고한 뒤에도 로봇이 잠시 미세하게 돕니다.

**실행** (Jetson; Mid-360S 드라이버와 `tracer_base`가 떠 있어야 하고, 시작 후 2초 정지):

```bash
python3 imu_base_relay.py
```
```bash
ros2 run robot_localization ekf_node --ros-args --params-file ekf_odom_imu.yaml -r odometry/filtered:=/odom_ekf
```

Jetson에서는 tmux `imurelay`, `ekf`, `fusion`에서 돌고 있으며, 재부팅하면 다시 띄워야 합니다. TF로 쓰려면 `publish_tf`를 켜고 드라이버 TF와 겹치지 않게 정리해야 합니다.

## 적용 (드라이버)

- Jetson `~/tracer_ws/src/tracer_ros2`(agilexrobotics 원본), 로컬 브랜치 `odom-correction`, HEAD `97a1788`
- 원본 대비 변경은 [tracer-odom-correction.patch](tracer-odom-correction.patch)에 있습니다. 다른 Tracer에는 `git -C ~/tracer_ws/src/tracer_ros2 apply <patch>` 후 `colcon build --packages-select tracer_base`로 적용합니다.
- **파라미터:** `linear_velocity_scale`, `linear_velocity_offset`, `linear_offset_threshold`, `angular_velocity_scale`, `angular_velocity_offset`, `angular_offset_threshold`, `straight_yaw_deadband`, `straight_yaw_deadband_min_linear`. C++ 기본값은 모두 보정 없음입니다.
- launch 기본값이 이 로봇의 측정값이라 평소 실행 명령으로 v4가 켜집니다. 시작 로그에 `odom linear velocity scale: 1.0362`가 보이면 정상입니다.
- **계수를 바꿀 때:** 이 기록을 고친 뒤 로봇에서 `python3 set_launch_defaults.py odom_calibration.json ~/tracer_ws/src/tracer_ros2/tracer_base/launch/tracer_base.launch.py` → 커밋 → `tracer_base` 재시작 순서로 합니다.
- **보정 끄기:** `linear_velocity_scale:=1 angular_velocity_scale:=1 linear_velocity_offset:=0 angular_velocity_offset:=0 straight_yaw_deadband:=0`
- `/odom`의 pose, twist, `odom → base_link` TF에 적용됩니다. `/tracer_status`는 펌웨어 원래 값이고, `/cmd_vel`은 그대로 펌웨어에 전달됩니다.

## 재현 (Jetson, 2 m × 2 m 빈 공간, 비상정지 옆에 사람)

```bash
~/livox_g2_calib/odomval.sh start lidar && python3 odom_speed_sweep.py all && ~/livox_g2_calib/odomval.sh stop
```

분석:
1. bag에서 `/livox/lidar`를 뺀 사본을 만듭니다(`ros2 bag convert`).
2. `python3 sweep_extract_mid.py <bag> ../base_mid360/T_base_mid360.csv mid.npz`
3. 모델 맞춤: `DEADBAND=1 python3 sweep_analyze.py <bag_without_lidar> mid.npz out.json`
4. 기록된 `/odom`, `/odom_fused`, `/odom_ekf` 검증: `python3 sweep_validate.py <bag_without_lidar> mid.npz out.json`
5. 경로 테스트: `python3 odom_path_test.py` 후 `python3 path_validate.py`

원본 bag은 분석 후 지웠습니다. 구간별 결과는 [sweep_20261011/](sweep_20261011/)에, 기록은 [odom_calibration.json](odom_calibration.json)에 있습니다.
