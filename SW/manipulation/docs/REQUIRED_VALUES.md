# 나중에 채워야 할 값

기존에 확인되지 않은 값만 `config/robot.yaml`의 `null`로 보존했다. **실물 hand–eye, CAD 장착/TCP, 카메라 모델·serial과 초록 사과 초기 높이는 이미 반영했다.** 원본을 복사해 실물 전용 robot.measured.yaml에서 미정 항목을 채우고 기존 값이 현재 장치에 해당하는지 확인한다. 예시 mock 값은 옮기지 않는다. 최초 단계별 실험에서 사용하는 클래스만 enabled=true로 둔다. [현재 반영 상태와 출처](CURRENT_STATUS.md)

| 채울 항목 | YAML 키 / 확인 방법 |
|---|---|
| Piper CAN 이름과 펌웨어 | `can_port`, `firmware`: 연결한 USB-CAN 이름과 장비 버전 |
| 손목 카메라 모델과 serial 확인 | 이미 D435IF /243222070076 반영. 실제 장착 장비와 대조하고 헤드·손목을 구분 |
| 탑다운 관측 거리 | `observation_surface_clearance_m`: 윗면 위 **0.25m를 조건부 초기 실험값**으로 입력. 실제 FOV·depth·팔 도달성으로 조정. 중심까지의 직접 거리 override `observation_camera_distance_m`는 필요 없으면 null 유지 |
| depth 최소 거리와 coarse 높이 오차 | `camera_min_depth_m`, `coarse_z_error_bound_m`: 실제 depth profile의 최소 거리와 측정한 coarse Z 최대 오차. 여유 `observation_depth_margin_m=0.02m` 포함해 관측 거리 검증 |
| 선택적 관측 중간 자세 | `observation_joints`: 필요할 경우 안전한 중간 이동의 6개 각도(rad). `null`로 두어도 된다. 최종 관측은 coarse 좌표 위에서 계산 |
| 작업 후 복귀 자세 | `return_joints`: 별도 6개 각도(rad). 관측 자세와 같다고 가정하지 않음 |
| 드라이버↔URDF 관절 대응 | `joint_signs`, `joint_offsets`: q_URDF=sign*q_driver+offset. CAD q1 회전 1.6rad를 실측 없이 적용하지 않음 |
| 베이스→팔 장착 기준 확인 | `base_arm_transform`: URDF와 동일 장착 조건으로 CAD 값 이미 반영 |
| 손목 hand–eye 적용 확인 | `handeye_transform`: 실물 PARK 결과 이미 반영. 카메라 optical→link6 방향이며 현재 장착과 대조 |
| link6→TCP 기준 확인 | `link6_tcp_transform`: CAD의 `[0,0,0.1425]m`, 축 평행 이미 반영. 필요시 [pivot 보완 도구](TCP_CALIBRATION.md) |
| TCP→선택한 파지 기준점 확인 | `tcp_to_grip_center_m`: jaw-tip 중심을 기준으로0 반영. 다른 접촉 위치를 선택할 때만 변경 |
| 탑다운 관측 방향 | `observation_camera_quaternion`: optical +Z를 아래로 향하게 하는 초기값 `[1,0,0,0]`. 실제 TCP 관측 자세는 보정 관계에서 자동 계산 |
| 수직 파지 자세 확인 | `topdown_quaternion`: 초기 `[1,0,0,0]` 반영. TCP +Z가 아래이고 원하는 닫힘 방향인지 확인 |
| 테이블 높이와 사용 영역 | `table_z_m`, `table_bounds_xy: [xmin,xmax,ymin,ymax]`: gravity frame 기준 |
| 주변 물체/장애물 | `obstacles`: `{id, center: [x,y,z], size: [x,y,z]}` 목록. 비어 있는 공간을 확인한 경우에만 `[]` |
| 실제 그리퍼 폭 대응 | `width_scale`, `width_bias_m`, `driver_width_max`: physical_width=scale*driver_position+bias. 최대 실제 폭 0.100m를 실측 |
| 실제 접촉 판정 기준 | `grasp_min_effort`: 빈 닫힘과 대상 접촉을 비교해 정한 feedback effort 임계값. `gripper_command_effort`의 명령값과 구분 |
| 클래스별 중심 높이 | `classes.<name>.center_height_m`: 테이블 위 물체의 실제 기하 중심 높이. 크기표만으로 용기 높이를 추정하지 않음 |
| 클래스별 집는 높이 | `classes.<name>.grip_height_m`: 테이블에서 양 손가락 중심까지 높이 |
| 바나나 길이와 방향 | `classes.banana.radius_m`: 지지/충돌용 보수적 반경, `closing_axis_yaw_offset_rad`: TCP 축과 닫힘 방향 관계. 인식 응답의 `axis_valid/axis_yaw` 필요 |
| 디텍션 출력 class names | 최신6클래스ID를 반영했다. Head는기존7클래스가중치 결과를변환한다. 손목도같은6클래스출력인지확인하고 model_names_verified를설정. [LIVE_DEMO.md](LIVE_DEMO.md) |
| 실제 ROS 이름 | `feedback_topic`, `gripper_feedback_topic`, `command_topic`, `arm_status_topic`, `enable_service`: 드라이버 실행 후 확인 |
| 베이스 잠금 연동 | `base_lease_topic`, `base_lock_topic`, `coarse_fixed_frame`: 내비게이션이 전체 이동 명령을 차단하고 작업 ID로 heartbeat를 응답해야 함 |

보정 확인 후 `calibration_verified`, `gravity_frame_verified`, `width_mapping_verified`, `model_names_verified`, `scene_verified`를 true로 변경한다. 드라이버·정지·작업 공간 검증까지 마친 뒤 `commissioned`, `execution_enabled`를 true로 한다. 이 flag는 실측값을 대신하지 않는다.

이미 입력된 초기값도 필요하면 조정한다: pregrasp/lift/retreat 높이, 속도·가속도, 위치 품질, timeout, retry, 충돌 접촉 link 목록. 클래스별 open/close는 Notion의 실험 초기값이며 성공을 보장하는 최적값이 아니다.

필수 항목을 묶어 확인:

```bash
python3 - <<'PY'
import yaml
from robocup_real_manipulation.core import validate
with open('/absolute/path/robot.measured.yaml') as f:
    validate(yaml.safe_load(f), hardware=True)
print('설정 형식·범위 검사 통과: 실물 성공 판정은 별도')
PY
```

미정 항목은 순서대로 `UNSET:...`, 미확인 사항은 `NOT_VERIFIED:...`로 표시된다. 최신6클래스 출력의 토마토ID2·컵ID4는 이미 반영했다. 기존7클래스 가중치를 쓰는 손목 adapter도 Head처럼 출력ID를 한 번 변환해야 한다. 실제 선 연결·측정·입력 명령은 [FIRST_CONNECTION.md](FIRST_CONNECTION.md)를 따른다.
