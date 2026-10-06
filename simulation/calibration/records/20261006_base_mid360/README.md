# Base–Mid360 측정 결과 — 2026-10-06

`results/auto_room_20261006_031114/base_mid360.json`은 실제 저장 점군으로 계산한 결과다. 위치는 약 **(-0.179962, 0.000156, 1.198867) m**, 방향은 roll 약 **180°**다. 상태는 **computed_validation_pending**이다. 별도 자세의 검증 통과를 의미하지 않는다.

- `train/001/`: 원시 압축 점군·XYZ와 측정 전후 실제 base pose.
- `config/`: 측정 당시의 방 SDF와 평면 기준표·입력 출처.
- `results/auto_room_20261006_031114/`: 고정 frame 방향·보정 JSON·평면 대응·지원 점 수.
- `results/runtime_recorded.urdf`: 측정 당시 구조의 보관본. 이전 랙/팔 장착과 당시 Mac 메시 URI가 들어 있으므로 새 Ubuntu 실행 입력으로 쓰지 않는다.
- `manifest.json`: 최신 모델에 적용한 후보와 측정 당시 자료의 관계.

실행 후보는 `simulation/robot_description/robocup.calibrated.urdf`다. 최신 main `1509ede`의 랙/PiPER 변경을 바탕으로 Base–2D와 Mid360 결과를 누적한 파일이다. 명목 모델 `robocup.urdf`와 Gazebo 측정 장착값은 자동 교체하지 않는다.

새 측정과 정확도 평가 순서는 [캘리브레이션 실행 가이드](../../README.md)에 있다. 새 모델로 Gazebo를 다시 만들었으면 새 세션에 그 world와 pose를 저장해 평가한다. 이 업데이트에서 Gazebo/ROS 실행·정확도 평가를 하지 않았다.
