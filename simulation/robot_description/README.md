# 최종 로봇 URDF

[robocup.urdf](robocup.urdf)가 유일한 최종 URDF입니다. 예전 모델은 Git 이력에서 확인할 수 있습니다.
94개 링크, 93개 조인트이며 단일 루트는 `base_link`입니다.
일반 URDF 구조에 Gazebo 센서 확장을 포함해 RViz와 Gazebo에서 같은 파일을 사용합니다.
그리퍼 두 관절은 별도로 정의하며 제어 도구가 대칭 목표값을 보냅니다.

- Tracer → 랙: fixed, xyz `(0, 0, 0.01611)` m.
- 랙 → Piper 베이스: fixed, 랙 기준 XYZ `(-0.0195, 0, 0.790)` m, RPY `(0, 0, 0)`.
- `piper_gripper_base` → 손목 마운트: CAD 정합 변환으로 fixed 연결.
- 2D/3D 라이다, 헤드 RGB/depth, 손목 RGB/depth 센서 포함.
- 명목 총질량 약 50.96143 kg. Tracer 30 kg, 랙 15.64637 kg, 손목 카메라 75 g, 손목 마운트 약 15.07 g.
- 모든 가동 관절의 URDF 기준 자세는 0입니다. Gazebo 시작 위치는 X/Y/yaw=0, Z=0.145 m입니다.

팔의 영점 자세는 원본 Piper의 1번 관절을 1.6 rad(약 91.7°) 돌린 모습입니다.
`piper_joint1`의 `origin rpy="0 0 1.6"`으로 이 자세를 모델에 반영했으며 시작 후 회전할 필요가 없습니다.
각도는 `q1 = 원본 Piper q1 - 1.6`이며 한계는 `[-4.2179938, 1.0179938] rad`로 옮겨 기존 물리적 가동 범위를 유지합니다.
Gazebo·ROS의 관절값과 수동 제어 명령은 이 새 영점을 기준으로 하며 `home`은 사진 속 기본 자세로 복귀합니다.

메시는 `../../HW/URDF/` 아래를 상대 경로로 참조합니다. 메시 폴더는 삭제하지 마세요.
좌표·관성·센서 설정을 바꿀 때는 이 최종 파일을 수정한 뒤 Gazebo를 다시 실행합니다.
실물 체결 좌표·물성·광학 보정은 별도 검증이 필요합니다.

2026-10-06: Fusion `final_assembly`의 변경된 프로파일·브래킷 형상과 `Assembly`의 Piper 베이스 장착 위치를 반영했습니다. 기존 매니퓰레이터 판형 마운트는 제거됐습니다. 센서 마운트·센서 링크·Gazebo 센서 확장, Piper 내부 관절·영점·가동 범위, 손목 카메라 연결 및 제어 스크립트는 유지했습니다. 새 알루미늄 랙의 질량·무게중심·관성·경계 박스 충돌 형상을 재계산했으며, [변경 근거](../../HW/URDF/sensor_rack_description/rack_revision_20261006.json)를 보관합니다. 아래 Gazebo 실행 검증은 이번 형상 변경 전의 기록이며 새 모델의 실구동을 뜻하지 않습니다.

[조립 상세](../docs/ASSEMBLY.md) · [손목 장착 계산](../../HW/URDF/WRIST_CAMERA_INTEGRATION.md)

최종 확정 후 별도 Gazebo 서버에서 G2 10 Hz/500 rays, Mid-360S 10 Hz/20,000 rays·points,
헤드·손목 RGB/depth 약 30 Hz 메시지와 프레임을 확인했고 정면 검사 벽 거리 검사를 통과했습니다.
ROS 런치는 당시 Gazebo 개발 PC에 ROS가 없어 구문 확인까지만 했습니다. 별도 Jetson 실기 setup은 [Jetson 문서](../../setup/jetson/README.md)에 기록했으며 이 시뮬레이션 실행 검증과 구분합니다.

## 보정 모델 후보 — 측정·적용·정확도 평가

최신 명목 모델은 `robocup.urdf`이며 `robot_description/robocup.calibrated.urdf`는 보관한 Base–2D/Mid360 결과를 최신 구조에 적용한 별도 후보다. 센서 보정과 독립 관측 평가를 포함한 [Ubuntu 실행 가이드](../calibration/README.md)에서 로봇 이동, pose/점군 저장, 결과 적용, A/B 오차 판정과 보고서 경로를 따라 한다. Mid360 결과는 검증 대기이며 이 업데이트에서 ROS/Gazebo/정확도 평가를 실행하지 않았다.
