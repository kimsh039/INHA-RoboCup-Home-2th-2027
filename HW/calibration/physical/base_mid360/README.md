# Base–Mid360 실측 캘리브레이션 (Livox Mid-360S)

2026-10-12 · 실제 로봇 (헤드 카메라 연결 후 구성) · 결과 방향 `base_link ← livox_frame`, 거리 m

| | x (mm) | y (mm) | z (mm) | roll (°) | pitch (°) | yaw (°) |
|---|---:|---:|---:|---:|---:|---:|
| CAD URDF | −180.0 | 0 | 1199.1 | 180 | 0 | 0 |
| **실측 (적용)** | **−156.5** | **5.4** | **1187.3** | **179.77** | **1.130** | **−1.926** |

센서가 **pitch 약 1.1°** 기울어져 있고, CAD보다 24 mm 앞, 12 mm 아래에 있습니다. 기울기는 로봇 구성에 따라 바뀝니다(아래 '기둥 기울기와 재측정').

## 방법 (G2와 독립)

자동 정지-주행 세션([../base_2dlidar/tools/calib_drive.py](../base_2dlidar/tools/calib_drive.py), [calib_solve.py](../base_2dlidar/tools/calib_solve.py)):

1. **roll, pitch, z:** 회전 세션의 정지 자세 29개(방향 16가지)마다 바닥 평면을 맞춥니다(RANSAC 12 mm, 0.8–3 m 고리).
   - 법선을 평균하면 방 바닥의 경사가 상쇄됩니다.
   - z는 센서-바닥 거리 평균 1329.8 mm(σ 0.9 mm)에 URDF 바닥 높이 −0.1425 m를 더한 값입니다.
2. **x, y, yaw:** 수평을 맞춘 센서 좌표계에서 정지 점군끼리 3자유도 점-평면 ICP를 합니다.
   - 회전 고정점 = base 원점
   - 전진 36번, 후진 36번 걸음의 이동 방향 = base `+x`
3. **결합:** G2 운동 기하 추정과 평균한 base 좌표계로 옮깁니다(x −1.9 mm, y +0.1 mm, yaw +0.05°; [../base_2dlidar](../base_2dlidar/README.md)).

## 검증

| 항목 | 값 |
|---|---|
| 정지 자세별 바닥 기울기 산포 (29개) | roll 0.05°, pitch 0.12° |
| 시계/반시계 회전 중심 차이 | 1.3 mm |
| 전진/후진 방향 차이 | 0.09° |
| 세션 하나씩 뺀 G2→Mid-360S 평면 거리 | 중앙값 4.1–4.9 mm, p90 13.5–21.4 mm ([../base_2dlidar/vs_mid360](../base_2dlidar/vs_mid360/README.md)) |
| z vs 줄자 | 줄자 기준 하우징 중앙 1.355 m − Livox 원점 오프셋 17 mm → 1195.5 mm. 바닥 평면 값과 −8 mm 차이 |

**Livox 점군 원점:** 매뉴얼(Mid-360 v1.2 부록, Mid-360S v1.0 p.16)상 장착면에서 47.0 mm(하우징 높이 60 mm)입니다. 거꾸로 장착된 이 로봇에서는 하우징 중앙보다 17 mm 아래입니다.

## 기둥 기울기와 재측정

- Mid-360S가 달린 기둥과 브래킷의 기울기는 로봇 구성에 따라 바뀝니다.
  - 2026-10-11 21시 측정(헤드 카메라 연결 전): pitch 2.91°, x −152.2 mm
  - 이번 측정(헤드 카메라 연결 후): pitch 1.13°, x −156.5 mm
  - 위치는 4 mm 정도만 바뀌었지만, 기울기 1.8°는 G2 높이(센서보다 0.85 m 아래)의 벽을 약 27 mm 밀어 보이게 합니다.
- **구성을 바꾸면 먼저 정지 점검을 합니다**([../base_2dlidar/tools/check_live.py](../base_2dlidar/tools/check_live.py), 4초).
  - 이번 측정 전 점검(이전 값 기준): 보정량 dx −31 mm, 바닥 pitch −1.59° → 다시 측정
  - 새 값 적용 후 점검: 바닥 높이 −1.4 mm, pitch +0.21°, 보정량 dx +3.9 mm, dy +10.0 mm(정지 자세 하나만 본 값, 벽이 한쪽에 몰린 위치)
- 보정량이 몇 mm 이상이면 자동 캘리브레이션을 다시 합니다(약 12분, [../base_2dlidar/README.md](../base_2dlidar/README.md) '다시 하기'). 기둥을 보강하면 이 변화가 줄어듭니다.

## 파일

- [base_mid360.json](base_mid360.json): 적용 기록, 원본 bag SHA-256(bag은 분석 후 삭제), 줄자·휴대폰 수평계 확인
- [T_base_mid360.csv](T_base_mid360.csv)
- [solver_output.json](solver_output.json): `calib_solve.py` 결과
