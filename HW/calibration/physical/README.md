# 실물 로봇 실행에 선택한 캘리브레이션 기록

여러 브랜치의 실물 측정 기록을 현재 실행에서 사용할 수 있도록 모았습니다. 원본 JSON/CSV/코드 바이트를 유지하며 기존 main의 이전 기록을 덮어쓰지 않습니다. 실행 방법은 [실물 로봇 README](../../../SW/robot/README.md)에 있습니다.

| 기록 | 출처 | 적용 |
|---|---|---|
| [Base–G2](base_2dlidar/README.md) / [G2–Mid360](base_2dlidar/vs_mid360/README.md) | `calib/g2-mid360-physical`, b8be83d, 2026-10-12 | 6자유도 TF + 거리 배율 |
| [Base–Mid360](base_mid360/README.md) | 같은 커밋·세션 | 몸체 기준 TF + IMU 회전 |
| [Tracer odometry](tracer_odom/README.md) | 같은 커밋, 2026-10-11 실물 주행 | v4 속도 계수 + IMU bias/EKF |
| [Wrist hand-eye](wrist_handeye/README.md) | `feature/wrist-handeye-calibration`, 0bac1e5 | Link6 기준 실물 RGB optical 변환 |
| [Head–Mid360](../head_mid360/README.md) | 2026-10-12 koide3 4개 bag | 기존 보관 결과를 사용 |
| [Head/Wrist RGB intrinsics](../camera_intrinsics/README.md) | main의 실물 결과 | 각 RGB 보정 CameraInfo |

원본 경로는 `HW/calibration/base_2dlidar`, `HW/calibration/base_mid360`, `HW/calibration/tracer_odom`입니다. 손목 원본은 `SW/simulation/calibration/wrist_handeye`에 있지만 dataset의 측정은 `physical_eye_in_hand`입니다. 경로 이름 때문에 실물 결과를 제외하지 않았습니다.

손목 원본 YAML과 15자세 dataset은 그대로 보존했습니다. 원본 YAML의 validation 부분 들여쓰기가 잘못되어 있으므로 런타임은 변환 필드만 옮긴 `wrist_handeye/calibration.json`을 읽습니다. 수치나 원본을 고쳐 보정 정확도를 바꾸지는 않았습니다. 사용 파일과 해시는 `SW/robot/physical_application.json`에 있습니다.

여기의 원본 README에 나오는 과거 측정·검증·재현 명령은 당시 기록입니다. 이번 작업에서는 그 명령을 다시 실행하지 않았습니다. 대용량 bag·이미지·점군은 추가하지 않았습니다.
