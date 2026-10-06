# Mid360 별도 자세 A/B 평가 — 2026-10-06

2026-10-06 사용자가 저장한 별도 자세 A/B를 대상으로, 같은 날 요청받아 수행한 정확도 검사 결과를 보관합니다. **보정값을 재추정하지 않고 고정한 상태에서 임시 10mm/1° 평면 기준을 통과했습니다.** 이 자료는 2026-10-06 Git에 추가 업로드했습니다. 이번 게시 작업에서 새 검증을 수행하지 않았습니다.

- 적용된 고정값: [base_mid360.json](../results/auto_room_20261006_031114/base_mid360.json)
- 최종 적용 모델: [robocup.calibrated.urdf](../../../../robot_description/robocup.calibrated.urdf)
- 검사 요약: [accuracy_report.md](results/audit_20261006_202216/accuracy_report.md)
- 자세·시간·입력 해시·수치 근거: [accuracy_audit.json](results/audit_20261006_202216/accuracy_audit.json)
- 관측 A: [validation_report.json](results/validation_A/validation_report.json), [원자료·base pose](validation/A/)
- 관측 B: [validation_report.json](results/validation_B/validation_report.json), [원자료·base pose](validation/B/)
- 측정 당시 방: [world.sdf](config/world.sdf), [저장한 코드 커밋](config/repo_commit.txt)

| 평가 자세 | 실제 base x/y/yaw | 최대 평면 위치 오차 | 최대 평면 각도 오차 | 판정 |
|---|---|---|---|---|
| A | 0.9992 / 0.4995m / 29.997° | 3.594mm | 0.2100° | 임시 기준 통과 |
| B | -0.5007 / 1.0007m / -45.003° | 3.167mm | 0.1892° | 임시 기준 통과 |

Gazebo 장착 기준값과의 차이는 위치 **0.291mm**, 회전 **0.00404°**입니다. 최신 최종 모델에 적용된 변환과 보정 JSON 사이의 위치 차이는 0mm, 회전 차이는 수치 오차 수준이었습니다.

평면 위치 오차는 평면을 맞춘 뒤의 오차입니다. 50mm gate로 선택한 점의 방 평면 거리 RMS는 A/B **8.64/8.96mm**, P95는 **19.97/21.76mm**였으며 제외 점은 각각 5,086/5,703개입니다. 이 숫자를 전체 점 또는 실물 센서 정확도로 해석하지 않습니다.

원본 보정 JSON과 manifest의 `computed_validation_pending`은 최초 계산·업로드 당시 기록으로 유지합니다. 후속 평가는 이 폴더에 별도로 기록하며, 기계별 절대 경로가 남은 JSON/보고서는 원본 provenance입니다. Git에서 읽을 때는 위 상대 링크를 사용합니다. 큰 HTML 점군 미리보기는 게시하지 않고, 원시 압축 점군·XYZ·pose·평면 추출·평가 파일을 보관합니다.
