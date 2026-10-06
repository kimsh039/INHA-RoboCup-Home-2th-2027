# 고정값으로 검증

상태: passed_provisional_limits

결과 방향: base_link ← laser_frame. 거리 m, 회전 quaternion xyzw.

| 자료 | 외벽 점 / 유효 거리 | 외벽 RMS (mm) | 절대오차 95백분위 (mm) | 판정 |
|---|---:|---:|---:|---|
| /Users/seoneum/Documents/Codex/2026-10-03/wh/outputs/robocup-tutorial-calibration/calibration_data/sim/base_2dlidar_02/validation/001 | 400/500 | 6.737003887304214 | 14.73569521724685 | 통과 |
| /Users/seoneum/Documents/Codex/2026-10-03/wh/outputs/robocup-tutorial-calibration/calibration_data/sim/base_2dlidar_02/validation/002 | 373/500 | 6.845772379938376 | 14.620908475031136 | 통과 |

z/roll/pitch는 archived SDF에서 고정한 입력이며 추정한 값이 아닙니다.
외벽 외 점(책상·로봇 자체·칸막이 등)은 gate로 제외합니다. 전체 점의 오차가 아닙니다.
validation에서는 calibration 변환을 재추정하지 않습니다.

세부 수치·입력 SHA-256·실제 pose·run·판정 사유는 같은 폴더의 JSON/YAML에 있습니다.

튜토리얼의 임시 검증 기준: {"max_wall_rms_mm": 15, "max_wall_p95_mm": 25, "min_wall_fraction": 0.6, "min_points_each_wall": 10, "wall_gate_m": 0.05}
passed는 아래 기록된 scan과 임시 기준에 한정한 판단입니다. 전체 센서 보정 완료를 뜻하지 않습니다.
