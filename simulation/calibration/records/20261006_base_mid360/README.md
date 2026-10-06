# Base–Mid360 측정 결과 — 2026-10-06

`results/auto_room_20261006_031114/base_mid360.json`은 실제 저장 점군으로 계산한 결과다. 위치는 약 **(-0.179962, 0.000156, 1.198867) m**, 방향은 roll 약 **180°**다. 계산 당시 상태 **computed_validation_pending**은 원본 JSON에 그대로 보존한다. **2026-10-06 후속 검사에서 같은 보정값을 고정한 A/B 자료가 임시 10mm/1° 기준을 통과**했다. [후속 평가 기록과 원자료](validation_20261006_201607/README.md)에 별도 판정을 보관한다.

- `train/001/`: 원시 압축 점군·XYZ와 측정 전후 실제 base pose.
- `config/`: 측정 당시의 방 SDF와 평면 기준표·입력 출처.
- `results/auto_room_20261006_031114/`: 고정 frame 방향·보정 JSON·평면 대응·지원 점 수.
- `results/runtime_recorded.urdf`: 측정 당시 구조의 보관본. 이전 랙/팔 장착과 당시 Mac 메시 URI가 들어 있으므로 새 Ubuntu 실행 입력으로 쓰지 않는다.
- `manifest.json`: 최신 모델에 적용한 후보와 측정 당시 자료의 관계.

**현재 최종 보정 모델은 `simulation/robot_description/robocup.calibrated.urdf`다.** 2026-10-06 [`b871b5b`](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/commit/b871b5b)에서 반영·업로드했으며 [적용 명령과 날짜별 이력](../../../robot_description/README.md)을 참고한다. 최신 main `1509ede`의 랙/PiPER 변경을 바탕으로 Base–2D와 Mid360 결과를 누적한 파일이다. 명목 모델 `robocup.urdf`와 Gazebo 측정 장착값은 자동 교체하지 않는다.

새 측정과 정확도 평가 순서는 [Mid360 Ubuntu 실행 가이드](../../BASE_MID360.md)에 있다. 새 모델로 Gazebo를 다시 만들었으면 새 세션에 그 world와 pose를 저장해 평가한다. 최초 URDF 업로드에서는 Gazebo/ROS 실행·정확도 평가를 하지 않았다. 이후 사용자가 저장한 A/B 자료에 대한 검사 결과를 이번 문서 업데이트에 함께 보관했으며, 문서 수정 중 추가 측정·검증은 하지 않았다.
