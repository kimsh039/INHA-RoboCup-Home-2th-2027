# 전체 캘리브레이션 요약

**2026-10-06 기준 · Gazebo 시뮬레이션 · Head/Wrist 모두 D435**

## 보정 순서와 현재 상태

**Base–2D LiDAR → Base–Mid360 → Head–Mid360 → Base–PiPER → Head–PiPER 검증 → Link6–Wrist → TCP → 전체 교차 검증**

현재 **두 LiDAR의 계산·별도 자세 평가·URDF 반영**까지 진행했다. 카메라·팔·TCP의 보정 결과는 아직 등록되지 않았다. CAD 장착값과 실제 보정 결과는 구분한다.

## 현재 결과

변환은 센서 좌표를 `base_link` 좌표로 옮기는 방향이다.

| 항목 | 계산 결과 | 별도 자세 평가 |
|---|---|---|
| **Base–2D LiDAR · 10/5** | x=0.124mm, y=0.521mm, yaw=8.844783°. z/roll/pitch는 고정 입력 | 001/002 외벽 거리 RMS **6.737/6.846mm**. 임시 기준 통과 |
| **Base–Mid360 · 10/6** | xyz≈(-0.179962, 0.000156, 1.198867)m, roll≈180° | A/B 최대 평면 위치 오차 **3.594/3.167mm**, 각도 오차 **0.2100/0.1892°**. 임시 기준 통과 |

두 결과는 10/6 최신 랙·PiPER 구조의 [robocup.calibrated.urdf](../robot_description/robocup.calibrated.urdf)에 반영했다. 위 오차는 저장된 시뮬레이션 관측의 평가이며 실물 정확도를 뜻하지 않는다.

## 진행 과정

1. **기록:** 실험 설정·world·코드 버전을 보관하고, 실제 base/팔 pose와 센서 관측을 저장한다.
2. **계산:** 학습 자료만으로 변환을 추정하고 결과 JSON과 대응 자료를 만든다.
3. **적용:** 최신 URDF에 보정값을 누적하고 ROS·RViz에서 해당 모델을 선택한다.
4. **평가:** GUI를 켜둔 채 다른 자세로 이동하고 실제 pose·새 관측을 저장한다. 기존 보정값을 고정해 오차를 계산한다.
5. **보관:** 계산값·평가 숫자·적용 파일·날짜를 기록한다. 실패 결과도 남긴다.

RViz에서 축이 보이는 것은 적용 확인이다. 잘 보정됐는지는 **별도 관측의 평가 보고서**로 판단한다. 계산 JSON의 `computed_validation_pending`은 계산 당시 상태로 남아 있으며 후속 판정은 별도 보고서에 있다.

## 저장 위치와 실행 문서

| 자료 | 위치 |
|---|---|
| 새 측정·계산·평가·일지 | `simulation/calibration/data/` 아래 날짜별 세션. 상세 가이드가 경로·파일을 생성하며 관찰 메모는 직접 기록 |
| 2D 결과와 평가 | [계산 요약](records/20261005_base_2dlidar/calibration/calibration_report.md) · [001/002 평가](records/20261005_base_2dlidar/validation/validation_report.md) |
| Mid360 원자료와 결과 | [학습·계산 기록](records/20261006_base_mid360/README.md) · [A/B 정확도 평가](records/20261006_base_mid360/validation_20261006_201607/README.md) |

- [Mid360 Ubuntu 실행 가이드](BASE_MID360.md): 측정 → 계산 → 적용 → 이동·평가 → 기록까지 상세 명령.
- [보정 모델 적용 방법](../robot_description/README.md#보정-urdf-적용하기): ROS·RViz 실행과 모델 선택.
- [모델 변경·업로드 이력](../robot_description/README.md#모델-변경보정업로드-이력): 반영 날짜와 커밋.
- [Mac 환경 차이](MAC.md): 기존 Mac 자료를 이어갈 때 참고.
