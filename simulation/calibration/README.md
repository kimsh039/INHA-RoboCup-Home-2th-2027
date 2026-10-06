# 전체 캘리브레이션 요약

**2026-10-06 기준 · Gazebo 시뮬레이션 · Head/Wrist 모두 D435**

## 보정 순서와 현재 상태

**Base–2D LiDAR → Base–Mid360 → Head–Mid360 → Base–PiPER → Head–PiPER 검증 → Link6–Wrist → TCP → 전체 교차 검증**

현재 **두 LiDAR의 계산·별도 자세 평가·URDF 반영**, **Head–Mid360과 Link6–Wrist의 계산·별도 자세 평가·기록 업로드**까지 진행했다. 두 카메라 보정값은 JSON으로 등록했으며 최종 URDF에는 아직 반영하지 않았다. 팔 베이스·TCP·전체 교차 검증은 남아 있다.

## 현재 결과

변환은 `parent ← child`, 즉 센서 좌표를 부모 좌표로 옮기는 방향이다. 아래 거리는 평가 대상이 서로 다르며, 별도 관측 오차와 GT 장착 변환 오차를 구분한다.

| 항목 | 계산 결과 | 별도 자세 평가 | GT 장착 오차 · 위치 / 방향 | 적용 상태 |
|---|---|---|---|---|
| **Base–2D LiDAR · 10/5** | `base_link ← laser_frame`: x=0.124mm, y=0.521mm, yaw=8.844783°. z/roll/pitch 고정 | 001/002 외벽 거리 RMS **6.737/6.846mm** · 임시 기준 통과 | **0.523mm (xy) / 0.0353° (yaw)** | 10/6 최종 URDF 반영 |
| **Base–Mid360 · 10/6** | `base_link ← livox_frame`: xyz≈(-0.179962, 0.000156, 1.198867)m, roll≈180° | A/B 최대 평면 위치 **3.594/3.167mm**, 방향 **0.2100/0.1892°** · 임시 기준 통과 | **0.291mm / 0.00404°** | 10/6 최종 URDF 반영 |
| **Head D435–Mid360 · 10/6** | `camera_optical_frame ← livox_frame`: xyz≈(0.001345, 0.087028, -0.043408)m | 25개 학습, 10개 평가. 최대 평면 거리 **4.247mm**, 방향 **0.814°** · 기준 10mm/1° 통과 | **11.720mm / 0.169°** · 위치 개선 필요 | JSON·원자료·평가 업로드; Base–Head 합성·URDF 미반영 |
| **Link6–Wrist D435 · 10/6** | `piper_link6 ← wrist_camera_optical_frame`: xyz≈(-0.066255, -0.002195, 0.036703)m | 25개 학습, 10개 평가. 최대 고정 태그 위치 **0.339mm**, 방향 **0.294°** · 기준 5mm/1° 통과 | **0.598mm / 0.195°** | JSON·원자료·평가 업로드; URDF 미반영 |

Head는 별도 평면 일관성 기준을 통과했어도 GT 장착 위치 오차가 11.72mm다. 원인 진단과 개선이 남아 있다. 표의 수치는 저장된 시뮬레이션 결과이며 실물 정확도를 뜻하지 않는다. 두 LiDAR 결과는 [robocup.calibrated.urdf](../robot_description/robocup.calibrated.urdf)에 반영돼 있다.

## 진행 과정

1. **기록:** 실험 설정·world·모델·코드를 보관하고 실제 base/팔 pose, 시각, 사진·점군을 저장한다.
2. **계산:** 학습 자료만으로 변환을 추정하고 결과 JSON과 대응 자료를 만든다.
3. **평가:** 별도 자세의 관측에 계산값을 고정해 오차를 구한다. GT 비교와 관측 일관성 판정을 구분한다.
4. **적용:** 적용할 결과를 최신 URDF에 누적하고 ROS·RViz에서 해당 모델을 선택한다. 결과 JSON 업로드만으로 TF가 바뀌지는 않는다.
5. **보관:** 계산값·평가 숫자·원자료·적용 여부·날짜를 기록한다.

RViz에서 축이 보이는 것은 적용 확인이다. 잘 보정됐는지는 별도 관측과 GT 비교 보고서로 판단한다. 계산 JSON의 `computed_validation_pending`은 계산 당시 상태로 남아 있으며 후속 판정은 별도 평가 JSON에 있다. 이번 카메라 업로드에서는 기존 평가를 보관했고 추가 테스트·재평가를 하지 않았다.

## 저장 위치와 읽는 순서

| 자료 | 위치 |
|---|---|
| 새 측정·계산·평가·일지 | `simulation/calibration/data/` 아래 날짜별 세션. 상세 가이드가 경로·파일을 생성하며 관찰 메모는 직접 기록 |
| 2D 결과와 평가 | [계산 요약](records/20261005_base_2dlidar/calibration/calibration_report.md) · [001/002 평가](records/20261005_base_2dlidar/validation/validation_report.md) |
| Mid360 원자료와 결과 | [학습·계산 기록](records/20261006_base_mid360/README.md) · [A/B 정확도 평가](records/20261006_base_mid360/validation_20261006_201607/README.md) |
| Head–Mid360 원자료와 결과 | [과정·수치·파일·Ubuntu 열기 명령](records/20261006_head_mid360/README.md) |
| Wrist D435 원자료와 결과 | [과정·수치·파일·Ubuntu 열기 명령](records/20261006_wrist_d435/README.md) |
| Head/Wrist 공통 실행 증거 | [명령·목표각·측정 모델·계산 코드 스냅샷](records/20261006_camera_run/README.md) |

각 기록의 README에서 보정 JSON → 별도 평가 → GT 비교 → 원자료 순서로 링크를 연다. 사진은 `images/`, Head 점군은 각 `train/`·`holdout/`의 압축 JSON/NPZ, 실제 자세와 촬영 연결은 dataset·`capture_record.json`에 있다. 원본 경로와 Git 경로의 대응 및 SHA-256은 각 manifest에 있다.

- [Mid360 Ubuntu 실행 가이드](BASE_MID360.md): 측정 → 계산 → 적용 → 이동·평가 → 기록까지 상세 명령.
- [보정 모델 적용 방법](../robot_description/README.md#보정-urdf-적용하기): ROS·RViz 실행과 모델 선택.
- [모델 변경·업로드 이력](../robot_description/README.md#모델-변경보정업로드-이력): 반영 날짜와 커밋.
- [Mac 환경 차이](MAC.md): 기존 Mac 자료를 이어갈 때 참고.
