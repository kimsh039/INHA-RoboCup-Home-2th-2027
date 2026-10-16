# 관제센터·매니퓰레이션 관절 입력 계약

현재 DetectionApproach는 `joint_contract.ModelJointFeedback`를 직접 사용한다. 완료된 일회성 source 패치 도구는 제거했으며 최신 코드에 다시 적용하지 않는다. [이전 패치 코드](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/a23d6e287a29ed34e65aaaab439b57405108636a/SW/manipulation/tools/patch_control_center_joints.py).

## raw와 model 관절을 구분한다

실물 raw `joint1..joint6`와 CAD/MoveIt의 `piper_joint1..piper_joint6`는 이름만 바꾸어 같은 값으로 취급하지 않는다. 물리 bridge는 모델 joint1의 +1.6rad origin에 맞춰 q1−1.6rad를 적용한다. 이미 보정된 model 입력에 이를 다시 적용하지 않는다.

`ModelJointFeedback`는 원본 시각과 6축 유한값·중복/누락·freshness를 확인한다. 기본 `/robot/joint_states`, 최대 age 0.6초다. 현재 config의 `detection.model_joint_topic`, `model_joint_max_age`로 지정한다. 입력이 없거나 오래되면 raw 피드백으로 우회하지 않고 계획을 막는다.

manipulation PiperBridge가 팔/모델 TF의 소유자인 구성은 model 토픽 `/joint_states`를 사용하므로 해당 토픽으로 지정하고 기존 물리 bridge를 중복 기동하지 않는다. 모델 이름을 raw 이름으로 바꾸지 않는다.

관절 topic만 확인했다고 그리퍼 effort/status·팔 실행 action까지 연결됐다고 볼 수 없다. [현재 인계 계약](MANIPULATION.md) · [원본 코드](joint_contract.py) · [관절 계약 테스트](tests/test_joint_contract.py).
