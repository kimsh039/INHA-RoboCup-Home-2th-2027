# train으로 계산한 Base–LiDAR

상태: computed_validation_pending

결과 방향: base_link ← laser_frame. 거리 m, 회전 quaternion xyzw.

추정 결과: **x=0.124 mm, y=0.521 mm, yaw=8.844783°**

현재 SDF 기준값과 비교: 평면 위치 오차 0.523 mm, yaw 오차 0.035328°.

다음 작업: 별도 validation 자세에서 scan과 실제 base pose를 저장하고 validate 명령을 실행합니다.

| 자료 | 외벽 점 / 유효 거리 | 외벽 RMS (mm) | 절대오차 95백분위 (mm) | 판정 |
|---|---:|---:|---:|---|
| /Users/seoneum/Documents/Codex/2026-10-03/wh/outputs/robocup-tutorial-calibration/calibration_data/sim/base_2dlidar_02/train/002 | 411/500 | 4.836250631785338 | 9.283471746320826 | train fitting |

z/roll/pitch는 archived SDF에서 고정한 입력이며 추정한 값이 아닙니다.
외벽 외 점(책상·로봇 자체·칸막이 등)은 gate로 제외합니다. 전체 점의 오차가 아닙니다.
validation에서는 calibration 변환을 재추정하지 않습니다.

세부 수치·입력 SHA-256·실제 pose·run·판정 사유는 같은 폴더의 JSON/YAML에 있습니다.
