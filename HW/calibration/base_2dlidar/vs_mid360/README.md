# (대체됨) G2 ↔ CAD Mid-360S 상대 캘리브레이션

> **이 결과는 runtime에 쓰지 않습니다.** Mid-360S를 CAD 자세(기울기 0)로 가정해 구한 G2 값입니다. 이후 Mid-360S가 pitch 약 2.9° 기울어진 것이 확인됐습니다(아래 "참고"의 IMU 기울기와 6자유도 진단 pitch −2.91°가 그 신호였습니다). 아래의 바닥 기울기 0.18–1.21° 진단은 진짜 바닥이 아닌 다른 수평면을 맞춘 잘못된 값입니다.
> 현재 값: G2는 [../README.md](../README.md)(운동 기하), Mid-360S와 기울기 보정한 G2↔Mid-360S 상대값은 [../../base_mid360/README.md](../../base_mid360/README.md). 이 폴더의 bag 8자세는 Mid-360S 계산에 그대로 쓰였습니다.

2026-10-10 · 실제 로봇 · 정지 다자세 · 결과 방향 `base_link ← laser_frame`, 거리 m

**결과: x=24.248 mm, y=9.159 mm, yaw=2.8367°** (z=0.34271 m, roll=pitch=0은 CAD 고정)

| | x (mm) | y (mm) | yaw (°) |
|---|---:|---:|---:|
| CAD URDF (`robocup.urdf`) | 0.175 | 0 | 8.8095 |
| 이전 runtime (Gazebo 기록 `20261005_base_2dlidar`) | 0.124 | 0.521 | 8.8448 |
| **이번 실측** | **24.248** | **9.159** | **2.8367** |
| 자세별 단독 추정 표준편차 (train 6개) | 3.3 | 4.7 | 0.16 |
| leave-one-out 범위 | 1.8 | 2.6 | 0.12 |

CAD 대비 yaw가 약 6.0° 작고, 센서 원점이 앞쪽 약 24 mm·왼쪽 약 9 mm에 있습니다.

## 방법

- 기준 센서는 Mid-360S입니다. 추정 중 `base_link ← livox_frame`은 CAD 공칭값(−0.18, 0, 1.19911 m, roll 180°)으로 고정했습니다. 현재 runtime의 Mid360 값과 차이는 0.29 mm / 0.004°입니다.
- 로봇을 세운 상태로 자세마다 약 10초씩 `/livox/lidar`, `/scan`, `/odom`, `/livox/imu`를 rosbag2로 저장했습니다.
- Mid-360S 약 110프레임을 누적해 15 mm voxel로 줄이고, G2 높이 부근(base z 0.13–0.56 m)에서 평면이면서 거의 수직인 점과 법선(PCA, 이웃 40점)을 뽑았습니다.
- G2는 약 105스캔의 빔별 중앙값을 쓰고, 0.7 m 미만(랙 기둥 반사)과 6 m 초과를 제외했습니다.
- train 자세 전체를 함께 쓰는 point-to-plane ICP(Huber 10 mm, 대응 거리 300→30 mm)로 x, y, yaw를 풀었습니다.

## 데이터와 검증

| 자세 | 용도 | odom x, y (m) | 방향 (°) | 단독 추정 x / y (mm), yaw (°) |
|---|---|---|---:|---|
| pose01 | train | −1.005, −1.638 | 143 | 19.7 / 8.8, 2.738 |
| pose02 | train | 같은 자리 | 205 | 24.3 / 10.2, 2.749 |
| pose03 | train | 같은 자리 | 303 | 22.9 / 2.5, 3.132 |
| pose05 | train | −0.850, −1.953 | 96 | 26.3 / 6.0, 2.911 |
| pose06 | train | −0.671, −1.765 | 42 | 21.9 / 12.2, 2.715 |
| pose07 | train | −0.731, −2.766 | 63 | 28.9 / 15.9, 2.800 |
| pose04 | **holdout** | −1.009, −1.637 | 75 | 19.7 / 7.1, 2.651 |
| pose08 | **holdout** | −1.646, −1.674 | 47 | 38.1 / 8.8, 2.606 |
| trial_pose01_first | 추가 holdout | −1.130, −1.545 | 144 | 20.2 / 7.0, 2.679 |

G2 점에서 Mid-360S 벽면까지 거리(점-평면, 20 cm 이내 대응):

| 자료 | CAD URDF 중앙값 / p90 | 이번 결과 중앙값 / p90 | 2 cm 이내 |
|---|---|---|---:|
| train 6자세 | 4.94 / 10.98 cm | 0.65 / 1.75 cm | 92% |
| holdout pose04 | 4.25 / 9.68 cm | 0.70 / 1.76 cm | 92% |
| holdout pose08 | 7.00 / 15.52 cm | 0.67 / 1.97 cm | 90% |
| 추가 holdout | 4.34 / 10.11 cm | 0.72 / 1.56 cm | 95% |

holdout은 계산에 넣지 않았습니다. 세부 수치는 [calibration.json](calibration.json), [per_pose.csv](per_pose.csv), [solver_output.json](solver_output.json)에 있습니다.

## 한계

- **Mid-360S 기준 상대값**입니다. Mid-360S 장착 자체는 실측 보정하지 않았으므로, `base_link` 기준 절대 정확도는 Mid-360S 장착 오차만큼 제한됩니다.
- z, roll, pitch는 수직 벽만으로는 관측되지 않아 CAD 값을 유지했습니다. 6자유도 진단값(z=361 mm, pitch −2.9°)은 관측성이 낮아 적용하지 않았습니다.
- 한 방(사방 벽) 안에서 수집했고, x는 자세별로 약 ±3 mm, y는 약 ±5 mm 흩어집니다. 독립 구현 교차 검증(GPT-6 Astra)의 자세 단위 bootstrap 95% 구간은 x 21.6–26.7 mm, y 5.8–12.5 mm, yaw 2.73–2.99°입니다(같은 방 6자세 조건부). 형식 σ(0.3–0.4 mm)는 절대 정확도가 아닙니다.
- holdout pose08의 단독 x=38.1 mm는 공동 추정보다 13.9 mm 큽니다. 이 자세는 바닥 기울기 진단값도 1.21°로 가장 크지만, 공동 추정값을 적용해도 잔차 중앙값은 6.7 mm로 다른 자세와 같습니다. 원인은 특정하지 못했습니다.
- pose04를 잡을 때 케이블에 걸린 일이 있었습니다. 그 전(pose01–03)과 후(pose04–08) 단독 yaw 평균 차이는 0.13°로, 자세별 흩어짐(0.16°)과 같은 수준입니다.
- 참고: Mid-360S 내장 IMU의 중력 방향은 1.9–2.8° 기울어져 나오지만, 같은 점군의 바닥 평면 기울기는 0.18–1.21°(pose08만 1.21°, 나머지 0.74° 이하)입니다. 이번 결과에는 쓰지 않았고 원인은 확인하지 않았습니다.
- G2는 0.3 m 안쪽에서 로봇 자기 랙을 봅니다. 내비게이션에서는 `range_min`이나 필터로 제외해야 합니다.

## 재현과 적용

- 원본 bag(약 0.5 GB)은 커밋하지 않았습니다. Jetson `sparo@ubuntu:~/livox_g2_calib/bags`에 있고 SHA-256은 `calibration.json`의 `poses[].bag`에 있습니다.
- 수집: [tools/rec.sh](tools/rec.sh) `poseNN` (정지 확인 후 10초 녹화, 점군 메시지 수 검사)
- 계산: [tools/calib_g2_livox.py](tools/calib_g2_livox.py) `prep <bag> <npz>`로 자세별 전처리 후 `<npz 폴더> <출력 json>`
- 적용: `SW/simulation/calibration/integrate_calibration.py --replace`가 이 폴더의 `calibration.json`을 읽어 `robocup.calibrated.urdf`의 `laser_frame_joint`를 생성합니다. 이전 Gazebo 기록은 `SW/simulation/calibration/records/20261005_base_2dlidar`에 그대로 남아 있습니다.
