# Head D435–Mid360 / Wrist D435 캘리브레이션 결과

실행일: **2026-10-06 23:33~23:38 KST**. 실제 Gazebo 영상·측정 관절·점군을 수집해 계산했다. 두 과정은 동일한 전용 partition `robocup_auto_20261006`에서 **순서대로** 실행했고, 수집용 서버·GUI는 종료했다.

**Wrist는 별도 자세 평가와 GT 비교에서 작은 오차를 보였다. Head–Mid360은 별도 관측 일관성 기준을 통과했지만 GT 대비 위치 오차 11.72 mm가 남았다.**

## 수집과 평가

| 항목 | Head D435–Mid360 | Wrist D435–Link6 |
|---|---:|---:|
| 학습 / 별도 평가 | 25 / 10 | 25 / 10 |
| 별도 평가 최대 거리 오차 | 4.247 mm (평면 offset) | 0.339 mm (고정 태그 위치) |
| 별도 평가 최대 방향 오차 | 0.814° (평면 normal) | 0.294° (고정 태그 자세) |
| 별도 평가 기준 | 10 mm / 1° | 5 mm / 1° |
| 판정 | 통과 | 통과 |
| Gazebo GT 대비 장착 위치 오차 | 11.720 mm | 0.598 mm |
| Gazebo GT 대비 장착 방향 오차 | 0.169° | 0.195° |

두 거리·방향 지표는 서로 다른 관측 관계를 평가한다. 평면 offset과 고정 태그 위치 오차를 같은 오차 정의로 비교하면 안 된다. GT 항목은 각각 추정한 센서 장착 변환과 실행 SDF의 장착 변환을 비교했다.

## 각도 중복과 학습·평가 분리

- Head 계획: 보드 roll/pitch `-24, -12, 0, 12, 24°` 조합 25개. 기존 사용자 관측 `train/001`을 보존하고 24개를 추가했다. 별도 평가 10개는 이 격자 사이의 다른 기울기를 사용했다.
- Head 계획에서 가장 가까운 학습–평가 방향 차이: **8.483°**. 사진 PnP로 측정한 실제 방향에서도 중복 0개, 가장 가까운 학습–평가 방향 차이 **7.742°**.
- Wrist 계획: 기존 `wrist_train.csv` 25개와 `wrist_holdout10.csv` 10개. 목표 관절 조합과 FK 플랜지 자세에 중복이 없었다.
- Wrist 실제 관절/FK에서도 중복 0개. 가장 가까운 학습–평가 관절 벡터 거리 **0.046904 rad**, 해당 플랜지 방향 차이 **2.707°**.
- Wrist 중복 기준: 관절 벡터 거리 0.002 rad 미만 또는 플랜지 위치 차이 0.05 mm 미만이면서 방향 차이 0.05° 미만. Head 방향 중복 기준: 0.1° 미만.
- 학습 관측만 보정 추정에 사용했고, 평가 관측에서는 보정값을 재추정하지 않았다. Head의 평가 보드 대응을 찾을 때는 학습 결과를 고정했다.

## Head 결과를 읽는 법

Head의 별도 평면 평가가 통과했다고 장착 위치가 GT와 4.25 mm 이내라는 뜻은 아니다. GT 위치 오차는 **11.72 mm**다. GT 변환으로 측정 평면 쌍을 비교하면 새 관측에 약 10 mm의 공통 offset이 나타났다. 영상에서 추정한 보드 거리의 편향 가능성이 있으며, 원인을 확정한 것은 아니다. 1 cm 미만의 장착 위치 정확도가 목표라면 Head는 개선 대상이다.

기존 첫 관측은 보존했다. 첫 사진에는 이전 모서리 검출 방식이 적용됐고 새 34개 관측에는 `CORNER_REFINE_SUBPIX`를 사용했다. 첫 관측의 GT 기준 평면 offset은 25.92 mm, 새 관측 평균은 약 10.02 mm였다. 이 첫 관측을 임의로 빼거나 통과 기준을 완화하지 않았다.

Head에서는 **카메라 optical ← Mid360**만 계산했다. 이 결과에 Base–Mid360 결과를 연결하면 Base–Head가 된다. 현재 보고서의 GT 오차도 Head–Mid360 변환에 대한 값이다.

## 저장 위치와 여는 방법

프로젝트 기준 폴더: `/Users/seoneum/Documents/Codex/2026-10-03/wh/outputs/robocup-tutorial-calibration`

| 산출물 | 프로젝트 안의 경로 | 내용 |
|---|---|---|
| Head 사진과 태그 pose | `calibration_data/sim/head_mid360_01/head_samples.json`, `images/` | 35개 사진의 시각·CameraInfo·PnP |
| Head 관측 쌍 | `calibration_data/sim/head_mid360_01/train/001…025`, `holdout/001…010` | 점군, XYZ, Head 보드 평면, 사진 번호 |
| Head 보정값 | `calibration_data/sim/head_mid360_01/results/automated_01/head_mid360.json` | Head←Mid360 행렬·위치·방향 |
| Head 평가 | `calibration_data/sim/head_mid360_01/results/automated_01/head_mid360_validation.json` | 10개 별도 관측의 평면 오차 |
| Head GT·중복 상세 | `calibration_data/sim/head_mid360_01/results/automated_01/evaluation_summary.json`, `actual_pose_overlap.json`, `ground_truth_plane_diagnostics.json` | GT 비교·각도 분리·공통 offset |
| Wrist 관측 | `calibration_data/sim/wrist_handeye_02/dataset.json`, `images/` | 35개 사진·실제 관절·FK |
| Wrist 보정값 | `calibration_data/sim/wrist_handeye_02/results/flange_wrist.json` | Link6←Wrist optical 행렬·위치·방향 |
| Wrist 계산 원본 | `calibration_data/sim/wrist_handeye_02/results/handeye_01.json` | OpenCV PARK 결과·개별 holdout 오차 |
| Wrist 평가 | `calibration_data/sim/wrist_handeye_02/results/handeye_validation_01.json` | 최대 위치·방향 오차와 판정 |
| Wrist GT·중복 상세 | `calibration_data/sim/wrist_handeye_02/results/evaluation_summary.json`, `actual_pose_overlap.json` | GT 비교·실제 관절 중복 |
| 전체 실행 기록 | `results/automated_calibration_20261006/commands.jsonl`, `angle_overlap_plan.json`, `summary.json` | 명령·목표각·최종 평가 |

```bash
export PROJECT="/Users/seoneum/Documents/Codex/2026-10-03/wh/outputs/robocup-tutorial-calibration"
open -a TextEdit "$PROJECT/calibration_data/sim/head_mid360_01/results/automated_01/evaluation_summary.json"
open -a TextEdit "$PROJECT/calibration_data/sim/wrist_handeye_02/results/evaluation_summary.json"
open "$PROJECT/calibration_data/sim/head_mid360_01/images"
open "$PROJECT/calibration_data/sim/wrist_handeye_02/images"
```

Head 각 새 관측의 `capture_record.json`은 명령한 보드 자세·촬영 id·사진 시각·점군 시각·partition을 남긴다. Wrist 각 관측의 `capture_record.json`은 목표 관절값·실제 관절값·사진 id·시각을 남긴다. GT와 CAD는 보드 초기 배치 및 마지막 GT 비교에만 사용했으며, 보정 추정에는 입력하지 않았다. 연속 영상·Gazebo 상태 로그 녹화는 사용하지 않았다.

이번 산출물은 센서 보정 JSON과 평가 기록이다. Git의 최종 URDF는 이번 실행에서 변경하지 않았다.
