# Base–2D LiDAR 실측 캘리브레이션 (YDLIDAR G2)

2026-10-12 · 실제 로봇 (헤드 카메라 연결 후 구성) · 결과 방향 `base_link ← laser_frame`, 거리 m

| | x (mm) | y (mm) | z (mm) | roll (°) | pitch (°) | yaw (°) |
|---|---:|---:|---:|---:|---:|---:|
| CAD URDF | 0.175 | 0 | 342.7 | 0 | 0 | 8.81 |
| **적용 값 (2D-3D 6자유도 정합)** | **5.03** | **7.79** | **344.3** | **−1.28** | **−0.95** | **0.881** |

## 방법

1. **base 좌표계 정의**
   - `base_link` 원점 = 차동 구동의 제자리 회전 중심
   - `+x` = 전진과 후진 주행 방향의 평균
2. **자동 정지-주행 4세션** ([tools/calib_drive.py](tools/calib_drive.py)), CAN 제어 모드만 사용
   - 회전 +45° × 8, −45° × 8
   - 0.15 m 직진: 전진 36번, 후진 36번
   - 위치·방향을 바꾼 정지 자세 16개
   - 정지 구간마다 빔별 중앙값 G2 스캔과 Mid-360S 점군을 뽑습니다([tools/calib_extract.py](tools/calib_extract.py)).
3. **운동 기하** ([tools/calib_solve.py](tools/calib_solve.py))
   - G2와 Mid-360S가 **각각 독립적으로** 회전 중심(회전)과 주행 방향(직진, 걸음마다 호 보정)을 구합니다.
   - base 좌표계는 두 센서 추정의 평균입니다(Mid-360S 쪽 보정량 x −1.9 mm, y +0.1 mm, yaw +0.05°, [../base_mid360](../base_mid360/README.md)).
4. **2D-3D 정합** ([vs_mid360](vs_mid360/README.md))
   - 정지 자세 108개에서 G2 점을 Mid-360S 평면에 맞춰 `livox_frame ← laser_frame`을 6자유도와 G2 거리 배율로 구합니다.
   - 적용 값 = `base_link ← livox_frame` ∘ 이 상대 자세

## 검증

| 항목 | 값 | 통합 기준 |
|---|---|---|
| 시계/반시계 회전 중심 차이 (G2) | 1.7 mm | ≤ 5 mm |
| 전진/후진 주행 방향 차이 | 0.20° (Mid-360S 0.09°) | ≤ 0.3° |
| G2 운동 기하만 vs 적용 값 | x 1.8 mm, y 1.0 mm, yaw 0.05° | — |
| 세션 하나씩 뺀 6자유도 정합 범위 | z 341.7–344.7 mm, roll −1.20~−1.53°, pitch −0.81~−1.01°, yaw 0.825–0.837° | — |
| 세션 하나씩 뺀 G2 → Mid-360S 평면 거리 | 중앙값 4.1–4.9 mm, p90 13.5–21.4 mm | 중앙값 ≤ 10 mm, p90 ≤ 25 mm |

- **전진/후진 주행 방향 차이:** 실제 주행 특성으로 측정마다 0.1–0.25° 사이에서 나옵니다(전진·후진 각 36걸음). 그래서 통합 기준을 0.3°로 정했습니다.
- **z, roll, pitch 주의**
  - 수평 스캔이 수직인 벽에 닿으면 높이가 정해지지 않습니다. z는 수직이 아닌 면에서만 정해지고, roll/pitch와 Mid-360S 기울기에 엮여 있습니다.
  - 이번 값은 줄자(바닥에서 481.1 mm → 338.6 mm)와 5.7 mm 차이입니다. 같은 G2인데 직전 측정(Mid-360S 기울기가 달랐을 때)은 z 351.2 mm, roll −0.87°였습니다. z는 ±5 mm, roll은 ±0.4° 정도로 보아야 합니다.
  - 팀 결정으로 정합 값을 적용했습니다. 기울어진 판을 두고 측정하면 더 확실해집니다.
- **G2 거리 배율 1.007:** G2 거리값이 ToF인 Mid-360S보다 약 0.6 % 짧습니다. 드라이버에 배율 설정이 없으니, 절대 거리가 필요한 곳에서 따로 반영합니다(1 m에서 약 6 mm).

## 다시 하기 (Jetson, 비상정지 옆에 사람, 앞뒤 1 m 빈 공간)

```bash
~/livox_g2_calib/odomval.sh start lidar && python3 calib_drive.py && ~/livox_g2_calib/odomval.sh stop
```
```bash
~/livox_g2_calib/odomval.sh start lidar && python3 calib_drive.py --straights 3 && ~/livox_g2_calib/odomval.sh stop
```
```bash
~/livox_g2_calib/odomval.sh start lidar && python3 calib_drive.py --poses && ~/livox_g2_calib/odomval.sh stop
```

세션마다 `python3 calib_extract.py <bag> calib_<bag>.npz`를 실행한 뒤 다음을 실행합니다.

```bash
python3 tools/calib_solve.py <spin.npz> solve.json <straights.npz>...
python3 vs_mid360/tools/g2_mid_static.py solve.json static.json spins=<...> str1=<...> poses=<...>
```

전체 약 12분입니다. 구성이 바뀌면 먼저 `python3 tools/check_live.py T_base_lidar.csv ../base_mid360/T_base_mid360.csv`로 정지 상태에서 4초 점검하고, 보정량이 몇 mm 이상이면 다시 합니다.

## 파일

- [calibration.json](calibration.json): 적용 기록, 원본 bag SHA-256 (bag은 분석 후 삭제)
- [T_base_lidar.csv](T_base_lidar.csv)
- [solver_output.json](solver_output.json): `calib_solve.py` 결과
- [vs_mid360/](vs_mid360/README.md): G2 ↔ Mid-360S 정지 자세 캘리브레이션
