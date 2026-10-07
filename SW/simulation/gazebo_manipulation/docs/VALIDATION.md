# 사용자 실행 기록 · 2026-10-07

## 판정

사용자가 로컬 Gazebo에서 실행한 최종 report는 `PHYSICAL_PICK_PLACE_PASS_WITH_RELAXED_HOLD`, 마지막 stage는 DONE이다. 원래 strict hold 기준은 통과하지 못했다. 코드 이식 이후 에이전트가 새로운 simulation/build/test를 실행한 결과는 아니다.

원본 성공 JSON과 trace의 숫자를 유지하고 local filesystem prefix만 `<local-workspace>/`로 바꿨다. 시뮬레이션의 recorded physics state로 판정하며 영상 파일은 첨부하지 않았다. Gazebo에서 실제 접촉 파지와 물체 상승·배치·복귀를 기록했으나 실물 센서·정지 유지·파지의 검증은 아니다.

## 실제 수치

| 검사 | 결과 | 유효 기준 |
|---|---|---|
| 후보 | 741 | 물체 object-frame 기록 후보 재사용 |
| 양손가락 접촉 | link7 / link8 | 양쪽 접촉 |
| lift 최소 높이 | 0.193157m | 0.20±0.02m |
| lift 상대 위치 최대 변화 | 0.0068473m | 완화0.025m; 원래0.01m |
| lift 상대 회전 최대 변화 | 0.508388rad ≈29.1° | 완화π/3; 원래0.2rad |
| 배치 방향 | 오른쪽0.10m | 왼쪽 계획 불가 시 오른쪽 |
| 배치 위치 오차 | 0.0086786m | 0.02m |
| 최종 초기 관절각 복귀 | 최대0.0035812rad | 0.02rad |
| 원래 유지 기준 | false | 회전 기준 초과 |

## 실행 조건과 한계

- nominal 공용 URDF, TRACER/PiPER와 40mm cube, 접근 완료 위치 가정. calibrated 모델 결과와 구분한다.
- base를 고정하지 않는다. 실제 base pose/속도와 object-relative TCP 변화가 trace에 남는다.
- 손목 RGB-D320×240@5Hz, known-object ROI. Head/라이다/점군 필터는 껐다.
- MoveIt Cartesian TOTG timing 사용. 1ms physics step, wheel/caster 마찰·collision/inertia 유지. 실제 wheel 정지제어와 동등함을 주장하지 않는다.
- 손가락은 effort PID, 물체를 물리 weld하지 않는다. contact/topic에서 실제 양쪽 collision pair를 받는다.
- TCP3mm/0.04rad 정밀 기준은 유지해 보고한다. 최대20mm/0.1rad까지 실험 진행을 허용하며 hold를25mm/60°로 완화했다. 이는 실물 허용오차 추천이 아니다.
- 새 RGB-D→GPU GraspNet 추론/visual servo/실제 Nav2 접근/실물 CAN 실행은 성공 범위 밖이다.
- 비교 실행이 아닌 단일 사용자 성공 기록이며 재현 안정성·성공률 또는 성능 개선 배율을 확정하지 않는다.

## 게시 시 경로 이식

저장소 main의 HW/SW 구조에 맞춰 SW/simulation/gazebo_manipulation에 배치했다. 기본 source root는 SW이며 ROS message package를 같은 workspace에 포함했다. 성공 실행 당시 nominal URDF와 현재 URDF의 차이는 HW 상대 경로 prefix이다. raw hash와 canonical hash를 함께 기록하고 canonical model hash 검사로 물성/형상 변경을 계속 거절한다. 기존 metadata 검사에서 해당 URDF hash만 현재 raw hash로 적응시킨다. 후보·원본 MuJoCo/URDF 파일은 수정하지 않았다.

게시 과정에서 runtime 생성 world, build/install, 로컬 로그, 모델 가중치 및 인증정보는 포함하지 않는다. trace·결과 JSON·코드 checksum은 [provenance](provenance.json)에 있다. 게시된 경로에서의 실행은 사용자 요청에 따라 재검증하지 않았다.
