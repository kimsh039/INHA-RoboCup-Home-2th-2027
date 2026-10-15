# Trial and error 파라미터 관리

실측 보정·장비 정보는 `robot.measured.yaml`, 반복 실험 조정값은 `tuning.yaml`로 분리한다. `config/tuning.yaml`을 작업용 디렉터리에 복사해 수정한다. 소스 코드를 바꿀 필요 없다. 단위는 m/rad/s이며 폭100mm는0.100m다.

```yaml
observation_surface_clearance_m: 0.25  # 윗면↔렌즈, 초기 실험값
pregrasp_m: 0.10
lift_m: 0.12
retreat_m: 0.10
velocity_scale: 0.05
acceleration_scale: 0.05
classes:
  green_apple:
    open_m: 0.100
    close_m: 0.079
    # grip_height_m: 실측 테이블 기준 파지 높이로부터 조정
```

사용 예:

```bash
ros2 launch robocup_real_manipulation bringup.launch.py \
  hardware:=true repo:=/absolute/path/to/repo \
  config:=/absolute/path/robot.measured.yaml \
  tuning_file:=/absolute/path/tuning.yaml \
  start_driver:=false
```

`tuning_file`을 생략하면 config의 값만 쓴다. overlay에는 물리 보정·CAN·class ID 같은 키를 넣을 수 없다. 클래스별 폭/파지 높이/바나나 축 오프셋/사용 여부와 관측 거리·중간/복귀 자세·topdown 자세·속도·거리·품질·timeout·retry·effort 등 휴리스틱 값만 허용한다. class.enabled를 변경해도 미정 높이/폭/방향 조건은 hardware validation을 통과해야 한다.

값은 시작할 때 읽는다. `ros2 param set`으로 작업 도중 폭이나 높이를 바꾸는 기능은 제공하지 않는다. 한 trial을 끝내고 팔 정지·베이스 안전 상태를 확인한 뒤 coordinator/bridge/MoveIt launch를 종료하고 파일을 수정해 재시작한다. 외부 드라이버를 그대로 유지할 경우 `start_driver=false`를 사용한다. 실패로 recovery lock이 남았다면 먼저 물체·팔을 수동 복구하고 잠금 journal을 처리한다.

권장 실험 기록은 trial_id/task_id, 변경한 한두 항목, 위치 실측 오차(mm), 빈 닫힘/접촉 effort, 보유·place·복귀 결과, 실패 원인이다. 처음에는 한 번에 한 종류·한두 파라미터를 바꾼다. 관측 높이25cm가 실제 depth 최소 거리+coarse Z 오차+여유보다 낮으면 설정 검사가 거부한다. open≤100mm,close<open과 실제 driver 변환 범위도 검사한다.

각 실행에서 최종 merge 설정을 `config_<sha256>.json`으로 저장하고, 모든 JSONL stage/폭 피드백 기록에 같은 hash를 넣는다. 어떤 trial에 어떤 설정을 사용했는지 비교할 수 있다. 그리퍼 폭 명령값·실제 feedback 폭·effort도 함께 남긴다. 실제 위치 정확도는 외부 실측 기록으로 평가하며 coarse↔precise 거리를 정확도 정답으로 취급하지 않는다.

## 빠르게 검색할 주석

```bash
rg -n '\[TUNE:|\[CALIB:|\[PARAMETER_ENTRY\]' SW/manipulation/manipulation_ws/src/robocup_real_manipulation
```

- `[PARAMETER_ENTRY]`: 파일을 읽고 overlay를 합치는 코드
- `[TUNE:OBSERVATION]`: 관측 높이
- `[TUNE:GRIPPER]`: 클래스별 open/close
- `[TUNE:GRASP_HEIGHT]`: 실제 집는 높이
- `[TUNE:MOTION]`, `[TUNE:QUALITY]`, `[TUNE:TIMEOUT]`: 거리·속도·품질·시간
- `[CALIB:HAND_EYE]`, `[CALIB:TCP]`, `[CALIB:JOINT_MAPPING]`, `[CALIB:GRIPPER_UNITS]`: 실측 관계와 단위
