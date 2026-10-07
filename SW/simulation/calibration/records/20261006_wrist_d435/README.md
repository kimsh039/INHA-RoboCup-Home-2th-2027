# Wrist D435–Link6 보정 기록 · 2026-10-06

> 2026-10-07 통합 상태: [현재 모델 기록](../integrated_calibration/README.md). Head는 새 Head–PiPER 결과를 채택했고 Wrist는 기존 결과를 모델에 적용했다. 아래 수치는 과거 측정 기록이다.

**학습 25개 + 별도 평가 10개. 별도 자세 기준을 통과했고 GT 장착 위치 오차는 0.598mm다.** Head/Wrist 카메라는 모두 D435다.

| 항목 | 저장된 결과 |
|---|---|
| 변환 방향 | `piper_link6 ← wrist_camera_optical_frame`: Wrist optical 점을 Link6 좌표로 변환 |
| 위치 xyz · m | `(-0.066254823, -0.002194749, 0.036703371)` |
| Quaternion xyzw | `(0.001152344, 0.000238673, -0.691202576, 0.722660096)` |
| 계산 방식 | Eye-in-hand · OpenCV `calibrateHandEye` PARK |
| 별도 평가 최대 고정 태그 위치 / 방향 오차 | **0.339mm / 0.294°** · 기준 5mm / 1° 통과 |
| Gazebo GT 장착 위치 / 방향 오차 | **0.598mm / 0.195°** |
| 실제 관절/FK 자세 중복 | 0개 · 가장 가까운 학습–평가 관절 벡터 거리 **0.046904rad**, 해당 방향 차이 **2.707°** |
| 최종 URDF 반영 | **미반영** · 이 JSON을 올렸다고 ROS TF가 자동으로 바뀌지는 않음 |

## 계산과 평가 과정

1. 고정 AprilTag를 바라보도록 팔을 학습 25개, 평가 10개의 관절 자세로 이동했다. 계획과 실제 관절/FK에서 중복이 없는지 기록했다.
2. 보드와 팔이 정지한 상태에서 D435 사진·CameraInfo·실제 관절·시각을 저장했다. AprilTag 36h11 ID 0, 검은 외곽 한 변 0.08m와 subpixel 모서리 검출을 사용했다.
3. 측정 관절의 FK와 사진 PnP를 학습 25개만 사용해 Link6←Wrist optical을 계산했다.
4. 이 변환을 고정한 채 별도 10개 자세에서 고정 태그의 위치·방향이 얼마나 일관적인지 평가했다. 평가 자료로 재추정하지 않았다.
5. 마지막에 실행 SDF의 Link6 기준 카메라 장착값과 비교했다. GT를 solver 입력으로 사용하지 않았다.

이 결과는 새 세션 `wrist_handeye_02`의 것이다. 동일 partition의 Gazebo 두 개가 섞였던 이전 실패 세션을 사용하지 않았다. 가장 가까운 학습–평가 자세의 플랜지 위치 차이는 3.056mm로 작으므로, 향후 더 넓은 작업 범위의 실험은 별도 세션으로 남긴다. 실물 장착 정확도나 배포 승인 판정은 아니다.

## 어디에 저장됐고 어떻게 보나

| 자료 | 파일·폴더 |
|---|---|
| 보정 행렬·위치·Quaternion | [flange_wrist.json](results/flange_wrist.json) |
| PARK 계산 원본·개별 holdout 오차 | [handeye_01.json](results/handeye_01.json) |
| 별도 평가 판정 | [handeye_validation_01.json](results/handeye_validation_01.json) |
| GT 비교와 실제 자세 중복 | [evaluation_summary.json](results/evaluation_summary.json), [actual_pose_overlap.json](results/actual_pose_overlap.json) |
| 35개 사진·실제 관절·FK·PnP·시각 | [dataset.json](dataset.json), [images/](images/) |
| 목표 관절·실제 관절·사진 id 연결 | [train/](train/), [holdout/](holdout/)의 `capture_record.json` |
| 목표 자세와 실제 측정 장면 | [config/](config/) |
| 원본 경로·SHA-256·보관 경로 | [manifest.json](manifest.json) |

Ubuntu에서 저장소 루트에 있는 터미널로 숫자를 읽는다. JSON은 에디터에서도 열 수 있고 PNG는 이미지 뷰어로 연다.

```bash
export REPO="$PWD"
export RECORD="$REPO/simulation/calibration/records/20261006_wrist_d435"
jq '{parent_frame, child_frame, translation_xyz_m, quaternion_xyzw}'   "$RECORD/results/flange_wrist.json"
jq '{held_out, ground_truth_error, pose_overlap}'   "$RECORD/results/evaluation_summary.json"
xdg-open "$RECORD/images/wrist_000_preview.png"
```

원본 JSON의 측정 당시 절대 경로와 SHA-256은 보존했다. 다른 PC에서는 `manifest.json`의 `archived_path`로 파일을 찾는다. `computed_validation_pending`은 계산 당시 상태이고 후속 판정은 위 평가 JSON에 있다. `config/wrist.world.sdf`는 측정 시점 원본이며 상대 메시·텍스처 경로를 바꾸지 않았으므로 보관 폴더에서 바로 실행하지 않는다. 로컬 원본은 유지했다. 이번 업로드에서는 재측정·재평가하지 않았다.

[함께 실행한 Head–Mid360 결과](../20261006_head_mid360/README.md) · [실행 기록·측정 모델·계산 코드 원본](../20261006_camera_run/README.md) · [전체 보정 요약](../../README.md)
