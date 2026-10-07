# Head D435–Mid360 보정 기록 · 2026-10-06

**학습 25개 + 별도 평가 10개. 평면 일관성 기준은 통과했지만 GT 위치 오차 11.72mm가 남았다.** Head/Wrist 카메라는 모두 D435다.

| 항목 | 저장된 결과 |
|---|---|
| 변환 방향 | `camera_optical_frame ← livox_frame`: Mid360 점을 Head optical 좌표로 변환 |
| 위치 xyz · m | `(0.001345422, 0.087028476, -0.043407516)` |
| Quaternion xyzw | `(0.499312560, 0.500751311, 0.500722611, -0.499211336)` |
| 별도 평가 최대 평면 거리 / 방향 오차 | **4.247mm / 0.814°** · 기준 10mm / 1° 통과 |
| Gazebo GT 장착 위치 / 방향 오차 | **11.720mm / 0.169°** |
| 실제 자세 중복 | 0개 · 학습–평가 최소 보드 방향 차이 **7.742°** |
| 최종 URDF 반영 | **미반영** · Base–Head 변환 합성도 아직 하지 않음 |

## 계산과 평가 과정

1. 보드를 서로 다른 roll/pitch로 이동하고, 정지한 보드의 D435 사진·CameraInfo·Mid360 점군을 저장했다. 기존 사용자 `train/001`을 보존하고 학습 24개와 평가 10개를 추가했다.
2. AprilTag 36h11 ID 0, 검은 외곽 한 변 0.08m를 사용했다. 사진의 PnP로 카메라 기준 보드 평면을 얻고 점군에서 대응 보드 평면을 추출했다.
3. 학습 25개 평면의 법선을 SVD로 정렬하고 offset을 최소제곱으로 풀어 Head←Mid360을 계산했다.
4. 계산값을 고정해 학습에 넣지 않은 10개 평면 쌍을 평가했다. 평가 대응 추출에도 고정된 학습 결과를 사용했고 재추정하지 않았다.
5. 마지막에 실행 SDF의 장착값과 비교했다. GT는 보드 초기 배치와 마지막 비교에만 사용했고 solver 입력으로 사용하지 않았다.

평면 일관성과 실제 장착 변환 오차는 다른 지표다. 새 관측은 GT 기준으로 약 10.02mm의 공통 평면 offset을 보였고, 기존 첫 관측은 25.92mm였다. 첫 사진의 기존 모서리 검출과 새 34개 사진의 subpixel 검출이 섞여 있다. 영상 거리 추정의 편향 가능성은 있으나 원인은 확정하지 않았다. **1cm 미만 장착 위치 정확도를 목표로 하면 개선 대상**이다.

## 어디에 저장됐고 어떻게 보나

| 자료 | 파일·폴더 |
|---|---|
| 보정 행렬·위치·Quaternion | [head_mid360.json](results/automated_01/head_mid360.json) |
| 별도 평가 판정과 10개 오차 | [head_mid360_validation.json](results/automated_01/head_mid360_validation.json) |
| GT 비교와 각도 중복 요약 | [evaluation_summary.json](results/automated_01/evaluation_summary.json) |
| GT 공통 offset 진단 | [ground_truth_plane_diagnostics.json](results/automated_01/ground_truth_plane_diagnostics.json) |
| 사진·시각·CameraInfo·PnP·실제 관절 | [head_samples.json](head_samples.json), [images/](images) |
| 학습 / 평가 점군과 촬영 연결 | [train/](train), [holdout/](holdout): `points_raw.json.gz`, `xyz.npz`, `head_plane.json`, `capture_id.txt` |
| 새 관측의 명령 자세·사진/점군 시각·partition | 각 관측의 `capture_record.json`; 기존 `train/001`에는 원래 이 파일이 없어 추가하지 않음 |
| 측정 장면·계획·태그와 평면 관계 | [config/](config) |
| 원본 경로·SHA-256·보관 경로 | [manifest.json](manifest.json) |

Ubuntu에서 저장소 루트에 있는 터미널로 숫자를 읽는다. JSON은 에디터에서도 열 수 있고 PNG는 이미지 뷰어로 연다.

```bash
export REPO="$PWD"
export RECORD="$REPO/simulation/calibration/records/20261006_head_mid360"
jq '{parent_frame, child_frame, translation_xyz_m, quaternion_xyzw}'   "$RECORD/results/automated_01/head_mid360.json"
jq '{held_out, ground_truth_error, pose_overlap}'   "$RECORD/results/automated_01/evaluation_summary.json"
xdg-open "$RECORD/images/head_000_preview.png"
```

원본 JSON/CSV의 측정 당시 절대 경로와 SHA-256은 보존했다. 다른 PC에서는 `manifest.json`의 `archived_path`로 파일을 찾는다. `computed_validation_pending`은 계산 당시 상태이고 후속 평가 판정은 위 평가 JSON에 있다. 측정 장면의 Mac 메시 경로는 기록용이며 새 PC에서 바로 실행하는 world가 아니다. 점군 HTML 뷰어·미완성 시도는 Git에서 제외했고 로컬 원본은 유지했다. 이번 업로드에서는 재측정·재평가하지 않았다.

[함께 실행한 Wrist 결과](../20261006_wrist_d435/README.md) · [실행 기록·측정 모델·계산 코드 원본](../20261006_camera_run/README.md) · [전체 보정 요약](../../README.md)
