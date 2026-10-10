# Tracer odometry 보정

2026-10-10 · 실제 로봇 · Tracer 펌웨어가 보고하는 선속도·각속도 보정

`tracer_ros2`의 odom은 펌웨어가 보내는 `v`, `w`를 적분합니다. 이 값은 계단식(각속도 약 0.0185 rad/s, 선속도 0.003 m/s 단위)이고, **움직이는 동안 거의 일정한 양만큼 작게** 보고됩니다. 정지 구간 사이의 실제 base 운동(G2 스캔 정합 + [base_2dlidar](../base_2dlidar/README.md))과 비교해 보정했습니다.

**적용 모델 (offset):**
- `v = v_rep + 0.00443·sign(v_rep)` (|v_rep| > 1e-3)
- `w = w_rep + 0.0523·sign(w_rep)` (|w_rep| > 0.04)

## 모델 선택 (두 세션 교차 검증)

| 세션 | 회전 속도 | 회전 단위 |
|---|---|---|
| 팀 `g2_scan_only_20261010_230526` | 약 0.148 rad/s | 약 40° |
| 검증 `val_20261010_235250` | 약 0.241 rad/s | 약 90° |

한 세션으로 맞추고 다른 세션에 적용한 구간당 RMS 오차:

| 모델 | yaw (팀→검증 / 검증→팀) | 거리 |
|---|---|---|
| 보정 없음 | 회전 15.9° / 11.0° | — |
| 배율 (k_w 1.32, k_v 1.11) | 6.17° / 3.28° | 23.7 / 15.5 mm |
| 배율 + 이동 중 치우침 (이전 적용 c10aa90) | 6.00° / 3.19° | — |
| **offset, 각속도 임계값 0.04** | **회전 1.69° / 1.46°, 직진 1.38° / 1.20°** | **7.7 / 7.6 mm** |

두 세션을 합쳐 맞춘 계수가 위 값이며, 합친 자료에서 회전 1.35°, 직진 yaw 1.27°, 거리 5.5 mm입니다. 배율 모델은 회전 속도가 다른 세션에서 과보정했습니다(검증 세션에서 회전 +9 %, 거리 +5 %; [validation_scale_model.json](validation_scale_model.json)).

## 적용

Jetson `~/tracer_ws/src/tracer_ros2`(agilexrobotics 원본)의 로컬 브랜치 `odom-correction`, 커밋 `c10aa90`(배율·치우침 파라미터) + `ad78440`(offset 파라미터, launch 타입). 변경 내용은 [0001-tracer-odom-correction.patch](0001-tracer-odom-correction.patch)에 있습니다.

- 파라미터: `linear_velocity_offset`, `angular_velocity_offset`, `angular_offset_threshold` (+ `linear_velocity_scale`, `angular_velocity_scale`, `angular_velocity_bias`). C++ 기본값은 모두 보정 없음입니다.
- `tracer_base.launch.py` 기본값은 **이 로봇의 측정값(offset 0.00443 / 0.0523 / 0.04, scale 1, bias 0)**이라 평소 실행 명령으로 보정이 켜집니다. launch 인자는 `float`로 고정해 `:=0`, `:=1`도 쓸 수 있습니다.
- `/odom`의 pose와 twist, `odom → base_link` TF에 적용하고, `/tracer_status`는 펌웨어 원래 값입니다.
- 보정 끄기: `... linear_velocity_offset:=0 angular_velocity_offset:=0`
- 다른 Tracer에 적용하기: `git -C ~/tracer_ws/src/tracer_ros2 am <patch>` 후 `colcon build --packages-select tracer_base`

`/cmd_vel`은 그대로 펌웨어에 전달됩니다. 명령 대비 실제 운동은 측정하지 않았습니다.

## 검증과 재현

- offset 모델로 실제 주행한 검증은 아직 없습니다. 기록 후 `python3 odom_validate.py <bag> out.json offset`으로 확인합니다(녹화: Jetson `~/livox_g2_calib/odomval.sh start|stop|trash|status`).
- `odom_model_cv.py <팀 bag 폴더> <검증 bag> out.json`: 모델 교차 검증 ([model_cross_validation.json](model_cross_validation.json))
- `odom_scale.py`, `odom_bias.py`: 이전 배율·치우침 모델 계산
- 기록: [odom_calibration.json](odom_calibration.json)
