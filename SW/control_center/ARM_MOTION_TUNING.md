# 팔 이동 판정 완화 — 2026-10-16

사용자는 정지 중인 팔 때문에 주행이 차단되지 않도록 판정을 넉넉하게 완화하도록 요청했다.

- 현재 관제 충돌 보호의 `arm_motion_limit`: **0.08 → 0.5 rad/s**.
- `arm_settle_sec`: **0.5 → 0.4초**.
- 6초간 `/robot/joint_states` 906개를 읽기만 한 결과 관절 1–5는 고정, 관절 6의 전체 변동 폭은 약 **0.086°**, 인접 샘플로 계산한 최대 속도는 **0.230 rad/s**였다. 기존 0.08 기준은 3회 초과했다.
- 이 값은 현재 피드백에 대한 사용자 승인 튜닝이다. 성공 픽앤플레이스 task `7ece08d387f547cd942d794076f82bb0`에서 검증된 충돌 보호 임계값으로 표현하지 않는다. 당시 매니퓰레이터의 `joint_stable_s=0.4`, `stable_speed_rad_s=0.02`는 별도 기준이다. 매니퓰레이터 속도/위치 허용오차는 이번에 변경하지 않았다.
- 팔 피드백 필수 조건·시간 초과, 실제 URDF 충돌 검사, 점군 자기 마스킹의 5mm 이동 한도, 최종 베이스 잠금은 유지한다.

Jetson에서 현재 설정의 두 항목만 수정하고 원본/해시를 백업한다. 다른 작업자의 YAML 항목은 보존한다.

```bash
cd ~/INHA-RoboCup-Home-2th-2027
python3 SW/control_center/tune-arm-motion.py --speed 0.5 --settle 0.4
```

기존 노드는 설정을 시작 시 읽으므로 재시작이 필요하다. 실제 적용에서는 비작업·차체 정지를 확인하고, `physical.launch.py`의 기존 respawn을 이용해 `collision_guard_node`와 `collision_cloud_node` 두 프로세스만 재시작했다. 이 도구 자체는 노드를 종료하거나 하드웨어를 실행하지 않는다.

롤백 예시(실제 출력된 백업 경로 사용):

```bash
python3 SW/control_center/tune-arm-motion.py --restore \
  SW/robot/config/.arm-motion-backup-20261016-000639-546634
```

롤백 이후에도 노드 재시작이 필요하다. 튜닝 이후 다른 설정 수정이 있으면 해시 검사로 덮어쓰기를 거부한다. 초기 적용 후 점군 전처리의 거부 횟수 0·`live` 상태를 확인했으며 실제 팔/베이스 이동 시험은 수행하지 않았다.

최종 정지 검증에서는 ROS 원본 토픽을 30초간 읽었다. 관절 피드백 4,645개에서 관절6의 전체 변동 폭은 0.109°, 인접 샘플 최대 계산 속도는 0.283rad/s였다. 같은 데이터에서 기존 0.08 기준 초과는 10회, 새 0.5 기준 초과는 0회였다. 실제 guard 상태 144개와 cloud 상태 452개에서 `arm_is_moving`은 각각 0회였다. cloud의 별도 `processing_deadline_exceeded` 1회는 팔 오판정과 구분한다. 검증 종료 시 guard/actuator는 ready, Nav2는 available, 베이스는 UNLOCKED·stopped였다. 원본 백업/적용본 해시와 정확히 두 설정값만 변경된 diff도 확인했다.
