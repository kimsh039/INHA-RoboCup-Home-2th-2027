# Tracer odometry 보정

2026-10-10 · 실제 로봇 · Tracer 펌웨어가 보고하는 선속도·각속도 보정

`tracer_ros2`의 odom은 펌웨어가 보내는 `v`, `w`를 적분합니다. 이 값은 계단식(각속도 약 0.0185 rad/s, 선속도 0.003 m/s 단위)이고, **움직이는 동안 거의 일정한 양만큼 작게** 보고됩니다. 정지 구간 사이의 실제 base 운동(G2 스캔 정합 + [base_2dlidar](../base_2dlidar/README.md))과 비교해 보정했습니다.

**적용 모델 (offset, v3):**
- `v = v_rep + 0.00756·sign(v_rep)` (|v_rep| > 1e-3)
- `w = w_rep + 0.0527·sign(w_rep)` (|w_rep| > 0.06)

계수는 드라이버와 같은 방식(SE(2) 적분)으로 맞췄습니다([odom_fit_se2.py](odom_fit_se2.py), [model_fit_se2.json](model_fit_se2.json)). v2(0.00443 / 0.0523 / 0.04)는 선속도를 이동 거리와 순변위로 비교한 잘못된 목적함수로 맞춘 값이라 대체했습니다.

## 모델 선택 (두 세션 교차 검증)

| 세션 | 회전 속도 | 회전 단위 |
|---|---|---|
| 팀 `g2_scan_only_20261010_230526` | 약 0.148 rad/s | 약 40° |
| 검증 `val_20261010_235250` | 약 0.241 rad/s | 약 90° |

한 세션으로 맞추고 다른 세션에 적용한 구간당 RMS 오차:

| 모델 | 회전 yaw (팀→검증 / 검증→팀) | 직진 yaw | 위치 |
|---|---|---|---|
| 보정 없음 | 15.9° / 11.0° | 1.12° / 0.98° | 19.5 / 20.2 mm |
| 배율 (k_w 1.32, k_v 1.11) | 6.17° / 3.28° | — | — |
| 배율 + 이동 중 치우침 (c10aa90) | 6.00° / 3.19° | — | — |
| offset, 임계값 0.04 | 1.69° / 1.46° | 1.38° / 1.20° | 6.6 / 4.6 mm |
| **offset, 임계값 0.06 (적용)** | **1.33° / 1.30°** | **1.20° / 1.08°** | **6.3 / 4.5 mm** |
| offset, 임계값 0.08 | 1.30° / 1.39° | 1.13° / 0.98° | 6.1 / 4.4 mm |

두 세션을 합쳐 맞춘 계수가 위 값이며, 합친 자료에서 회전 1.28°, 직진 yaw 1.13°, 위치 5.0 mm입니다. 배율 모델은 회전 속도가 다른 세션에서 과보정했습니다(검증 세션에서 회전 +9 %, 거리 +5 %; [validation_scale_model.json](validation_scale_model.json)).

## 적용

Jetson `~/tracer_ws/src/tracer_ros2`(agilexrobotics 원본)의 로컬 브랜치 `odom-correction`, 커밋 `c10aa90`(배율·치우침 파라미터) + `ad78440`(offset 파라미터, launch 타입). 변경 내용은 [0001-tracer-odom-correction.patch](0001-tracer-odom-correction.patch)에 있습니다.

> **현재 로봇의 launch 기본값은 v2(0.00443 / 0.0523 / 0.04)입니다.** v3로 바꾸려면 로봇에서 `python3 set_launch_defaults.py odom_calibration.json ~/tracer_ws/src/tracer_ros2/tracer_base/launch/tracer_base.launch.py` 후 커밋하고 `tracer_base`를 재시작합니다.

- 파라미터: `linear_velocity_offset`, `angular_velocity_offset`, `angular_offset_threshold` (+ `linear_velocity_scale`, `angular_velocity_scale`, `angular_velocity_bias`). C++ 기본값은 모두 보정 없음입니다.
- `tracer_base.launch.py` 기본값은 **이 로봇의 측정값**이라 평소 실행 명령으로 보정이 켜집니다(로봇 전용 작업공간). 공용 기본값을 끄고 로봇별 설정으로 켜는 방식으로 바꿀 수도 있습니다. launch 인자는 `float`로 고정해 `:=0`, `:=1`도 쓸 수 있습니다.
- `/odom`의 pose와 twist, `odom → base_link` TF에 적용하고, `/tracer_status`는 펌웨어 원래 값입니다.
- 보정 끄기: `... linear_velocity_offset:=0 angular_velocity_offset:=0`
- 다른 Tracer에 적용하기: `git -C ~/tracer_ws/src/tracer_ros2 am <patch>` 후 `colcon build --packages-select tracer_base`

`/cmd_vel`은 그대로 펌웨어에 전달됩니다. 명령 대비 실제 운동은 측정하지 않았습니다.

## 검증과 재현

- offset 모델로 실제 주행한 검증은 아직 없습니다. 기록 후 `python3 odom_validate.py <bag> out.json offset`으로 확인합니다(녹화: Jetson `~/livox_g2_calib/odomval.sh start|stop|trash|status`).
- `odom_model_cv.py <팀 bag 폴더> <검증 bag> out.json`: 모델 교차 검증 ([model_cross_validation.json](model_cross_validation.json))
- `odom_scale.py`, `odom_bias.py`: 이전 배율·치우침 모델 계산
- 기록: [odom_calibration.json](odom_calibration.json)
