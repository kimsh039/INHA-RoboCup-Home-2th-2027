# 관제센터·매니퓰레이션 관절 입력 호환

로봇의 raw 팔 피드백 `joint1..joint6`와 CAD/MoveIt 모델 `piper_joint1..piper_joint6`를 구별한다. 관제 화면용 joints 채널의 raw 값을 계획에 직접 사용하지 않는다.

`joint_contract.ModelJointFeedback`는 기존 물리 로봇 bridge의 `/robot/joint_states`를 구독한다. bridge에서 이미 적용한 각도 보정을 보존하며 원본 시각, 여섯 관절의 유한값/중복/누락 및 최대0.6초 freshness를 검사한다. 오래된 피드백은 빈 입력을 반환하여 계획을 차단한다. raw 이름을 모델 이름으로만 바꾸거나 q1 offset을 다시 적용하지 않는다. 추가 모델·추론·점군 처리와 모터 명령은 없다. 수신 큐1과 단일 관절 snapshot만 유지한다.

## 최신 관제 소스에 적용

Jetson의 미커밋 detection_approach.py 전체를 오래된 사본으로 교체하지 않는다. 해당 checkout을 보존한 채 아래 도구로 관절 입력 부분만 변경한다. 도구는 원본 source backup을 만들고 예상한 1~2개 입력 위치 외 구조에서는 거절한다.

```bash
python3 SW/manipulation/tools/patch_control_center_joints.py \
  --control-center /실제/최신/checkout/SW/control_center --check
python3 SW/manipulation/tools/patch_control_center_joints.py \
  --control-center /실제/최신/checkout/SW/control_center
```

백업은 대상 디렉터리의 `.joint-input-backup-*`다. 원복은 그 안의 detection_approach.py를 원위치에 복사한다. joint_contract.py가 백업에 있으면 그것도 복구하고, 원래 없었다면 추가된 helper를 제거한다. 새 모듈은 다음 관제센터 시작 때 로드되며 도구는 프로세스를 시작/종료하지 않는다.

필요한 설정은 기존 config.json의 detection 항목에 `model_joint_topic` (기본 `/robot/joint_states`)과 `model_joint_max_age` (기본0.6초)로 지정할 수 있다. 모델 토픽이 정상 수신되지 않을 때 raw 토픽으로 우회하지 않는다.

## 매니퓰레이션과 연결

`robocup_real_manipulation/config/robot.yaml`은 `driver_joint_names=[joint1..joint6]`, `joint_names=[piper_joint1..piper_joint6]`로 이미 같은 계약이다. measured signs/offsets를 검증한 PiperBridge가 raw→모델 변환을 담당한다. 모델 이름을 raw 이름으로 바꾸지 않는다.

매니퓰 PiperBridge가 팔/TF의 단일 소유자가 되는 구성에서는 모델 출력이 `/joint_states`이므로 `detection.model_joint_topic=/joint_states`로 설정한다. 기존 물리 bridge를 중복 기동하지 않는다. 기존 read-only `/piper/joint_states_feedback`는 실제 그리퍼 effort/제어 status를 포함한 새 드라이버 전체 인터페이스를 대체하지 못한다.

현재 수정은 로컬에서 검증했다. 사용자 요청에 따라 재부팅 중인 Jetson에 업로드·재시작·구동하지 않았다.
