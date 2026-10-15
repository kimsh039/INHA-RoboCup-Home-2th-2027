# 실물 TF・보정·충돌 모델

URDF는 문제 있는 파일로 취급하지 않는다. CAD 형상/관절 이름/장착 구조와 실측 보정값은 근거가 다르다. 이번 구현은 **기존 robocup.urdf / robocup.calibrated.urdf를 수정하지 않는다**. 실물 전용 설정이 채워지면 별도 파일을 생성한다.

```text
odom (또는 실제 고정 frame)
└─ base_link                      외부 내비게이션 TF
   ├─ rack / body / sensors       기존 CAD 구조·collision mesh
   └─ piper_base_link             실측 base_arm_transform
      └─ piper_link1 ... link6    실제 관절 피드백, 실측 sign/offset
         ├─ tcp                  실측 link6_tcp_transform
         ├─ wrist_camera_optical_frame  실측 handeye_transform
         └─ wrist mount / gripper CAD   형상과 collision 유지
```

내비게이션은 coarse 취득시각과 접근 완료 시각 모두를 포함하는 충분한 TF 이력을 제공한다. coarse가 접근 전 base frame 좌표여도 `lookup_transform_full`로 source 취득시각→고정 frame→접근 완료 시각의 base로 변환한다. TF Buffer 기본 이력보다 오래된 coarse라면 TF 누락으로 실패하므로, 외부 모듈은 재측정하거나 보존한 고정 frame 좌표와 현재 유효 측정시각을 제공해야 한다. 원거리 좌표는 대상 연계만 수행하며 손목 측정 실패 시 grasp fallback에 쓰지 않는다.

손목 depth는 이미지 취득시각의 동적 관절 TF와 hand–eye로 변환한다. 최신 TF(Time=0)로 움직이는 카메라 영상을 변환하지 않는다. 베이스 이동이 잠금으로 차단된 이후에도 취득시각의 TF를 사용한다.

## 보정 항목

2026-10-12: 사용자 지시대로 URDF와 동일한 팔 장착을 CAD 기준값으로 반영하고, 확인한 TCP142.5mm와 `feature/wrist-handeye-calibration`의 **실물** best15 PARK 결과를 기본 설정에 반영했다. 현재 값과 출처/오차는 [CURRENT_STATUS.md](CURRENT_STATUS.md)를 따른다. 아래 보정 목록은 확인 항목이며 모든 값을 처음부터 다시 측정하라는 의미가 아니다.

EE→TCP의 정확한 A/B 기준점, 저장소 메시 그림, 카메라 없는 실물 pivot 수집/계산/적용은 [TCP_CALIBRATION.md](TCP_CALIBRATION.md)를 따른다. 현재 CAD jaw-tip 기준값은 link6의 `[0,0,142.5] mm`이며 실물 설정에 자동 복사하지 않는다.

1. 물리 베이스↔팔 장착 위치·방향: CAD reference와 실측 구분.
2. 실제 관절 이름·zero·sign·offset·범위: 제조사 firmware와 CAD 좌표 대응.
3. 손목 hand–eye: 실제 움직이는 link6↔광학 frame. 헤드–LiDAR 정적 보정과 독립.
4. TCP pivot 위치 + TCP 회전축 + TCP↔손가락 파지 중심: 위치 pivot만으로 도구 방향이 결정되지 않는다.
5. 카메라 intrinsics, rectified RGB↔aligned depth 일치와 depth scale.
6. 테이블 높이/지지 영역, 중력 방향 frame(+Z 위)과 오른쪽=-Y 기준.
7. 그리퍼 command/raw feedback↔실제 두 손가락 간격, 개방 상한, effort 접촉 threshold.

[기존 설명](../../simulation/robot_description/README.md)의 TCP는 CAD 기반 MuJoCo pivot이며 실제 물리 pivot·관절 오프셋·개방 폭을 별도로 측정해야 한다고 명시한다. 따라서 그 값을 자동으로 실물 보정값에 복사하지 않는다. 몸체/센서 collision mesh도 실제 부품·장착과 대조한다.

## coarse 기반 탑다운 재관측 위치

사용자 설명에 따라 접근 완료 좌표의 위에 팔을 탑다운으로 배치한다. `T_tcp_opt = inverse(T_link6_tcp) * T_link6_opt`를 실측 보정에서 계산한다. 관측에는 별도 `observation_camera_quaternion`으로 optical 광축을 아래로 정의한다. 실제 TCP 관측 자세는 `R_base_optical * inverse(R_link6_optical) * R_link6_tcp`로 계산하므로 카메라 장착 각도까지 반영한다. 파지 단계의 `topdown_quaternion`과 관측 TCP 자세를 같은 것으로 가정하지 않는다. `distance = observation_surface_clearance_m + class.center_height_m`(또는 명시적 center-distance override), `camera_position = coarse_center - optical_forward * distance`, `TCP_position = camera_position - R_TCP * tcp_to_optical_translation`이다. TCP가 coarse 위에 있어야 한다. 따라서 카메라 장착 오프셋 때문에 필요한 TCP의 XY 보정을 포함한다. 고정 joint pose는 선택적인 중간 경유점이며 final observation 목표를 대체하지 않는다.

## 중심과 표면점

초기 SURFACE 경로는 물체가 수평 테이블 위에 놓이고 카메라 optical +Z가 아래를 향하는 **10도 이내의 상부 관측**만 허용한다. mask 중앙 영역의 depth는 표면점이다. XY는 그 표면 patch를 사용하고 Z는 `table_z + 실측 center_height`로 기하 중심을 정의한다. top surface의 Z가 `table_z+2*center_height`에서 4cm 이상 벗어나면 거부한다. 이는 사과·용기 상부/몸통의 초기 실험 조건이며 임의 측면 관측의 구 중심을 정확히 복원하는 방식은 아니다.

다른 시야·형상의 물체는 외부 geometry adapter가 실제 CENTER를 계산해 보내야 한다. CENTER의 높이는 실측 table/center-height와 2.5cm 이내여야 한다. 컵·캔의 diameter로 높이를 유추하지 않는다. 바나나의 중앙 몸통점과 장축 방향은 별도 인식이 필요하다.

파지 손가락 중심은 `table_z + grip_height`이고 `TCP = grip_center - R_TCP * tcp_to_grip_center`이다. TCP +Z를 아래로 배치한 후 gravity +Z 기준으로 pregrasp/lift/retreat를 계산한다. 그리퍼 최대 폭은 100mm. 모든 ROS 기하와 width는 m, 관절/방향은 rad다.

## 물리 실행 모델 생성

`prepare_description` / hardware launch는 CAD 원본에서 Gazebo·ros2_control·transmission을 제거한다. uncommanded base wheel joint는 fixed로 처리해 manipulation state monitor와 분리한다. 몸체·센서·팔 collision을 유지하며 실측 base-arm/hand–eye/TCP를 적용한다. mesh 경로는 보존된 전체 저장소 내 파일의 절대 URI로 변환한다. 저장소 이동 후 모델을 다시 생성해야 한다.

SRDF에는 piper base→TCP arm chain과 바로 인접한 link 쌍의 collision 면제만 포함한다. 학습된 광범위 self-collision 면제표는 넣지 않는다. 실제 collision geometry가 겹치면 계획이 실패할 수 있다. RViz/MoveIt의 충돌 쌍을 확인해 CAD mesh를 실물에 맞추거나 필요한 면제를 별도로 검증한다.

MoveIt scene은 팔 이동 전에 테이블·주변 장애물을 추가하고, 정밀 파지 계산 후 대상의 보수적인 box를 추가한다. DESCEND에서 대상↔검증한 gripper touch links만 접촉 허용한다. VERIFY 후 대상 box를 TCP에 attach해 운반 충돌을 검사한다. 수직 liftoff와 최종 place 하강에서만 대상↔테이블 접촉을 허용한다. 운반 hover 경로에서는 이 허용을 제거하고 대상↔테이블 충돌도 검사한다. 팔↔테이블 충돌은 계속 검사한다. 개방 후에는 world에 별도 placed ID로 남겨 다음 작업에서도 장애물로 검사한다.

오른쪽=-Y 0.10m 목표의 지지 면적·IK/충돌 hover 계획·수직 내려놓기 경로를 먼저 검사한다. 계획 불가능일 때만 왼쪽=+Y를 검사한다. 실행 오류는 왼쪽 retry로 바꾸지 않고 정지한다. 양쪽 불가능하면 HELD 상태 실패다. MoveIt의 self/body/sensor/table/obstacle 충돌 검사와 Cartesian 완주 비율을 통과해야 실행한다.

초기 실물 실험은 정적인 주변 환경과 검증한 장애물 목록을 전제로 한다. 외부 사람이 물체를 이동시키거나 새 장애물이 들어오면 scene을 갱신하고 검증해야 한다. 이 패키지는 전역 LiDAR collision map이나 환경 변화 검출을 새로 구현하지 않는다.

Piper bridge는 MoveIt 경로의 각 관절 segment를 smoothstep으로 보간하고 속도·가속도·범위·현재 경로 오차를 검사한다. MoveIt 경로는 해당 segment 길이에 맞게 보수적으로 retime한다. 밀집 waypoint 때문에 timeout을 넘으면 명령을 전송하지 않고 `RETIME_EXCEEDS_STAGE_TIMEOUT`으로 실패한다. 이 제어 경로의 실제 추종·braking·CAN rate와 지터는 Jetson에서 검증해야 한다.

## 첫 관측 높이의 조건부 초기값

사용자 설명에 따라 선택한 물체만 재관측하고 coarse 오차가 수cm 수준이라는 조건에서, 손목 장비가 D435 계열·depth848×480일 때 물체 **윗면에서 렌즈까지25cm**를 초기값으로 제안한다. 사과 center_height=4cm이면 중심→렌즈는29cm다.25cm 역시 실물 검증 전의 조정 가능한 실험값이다.

[D435 공식 데이터시트](https://www.realsenseai.com/download/21345/?tmstv=1780880949)는 최소 depth 거리를1280×720에서280mm,848×480에서195mm로 제시한다.25cm는1280×720 depth에는 맞지 않는다. 실제 손목 모델/해상도의 `camera_min_depth_m`와 coarse 높이 오차의 실측 상한 `coarse_z_error_bound_m`를 채워야 hardware 설정 검사를 통과한다. 기본 표면거리에서 높이 오차를 빼도 최소 depth 거리+2cm 여유를 만족해야 한다. 예시25cm-2cm=23cm는19.5cm+2cm보다 크지만, 이2cm 오차를 실측값으로 취급하지 않는다.

센터까지의 직접 거리 override를 사용할 경우에는 물체의 center_height를 빼서 실제 표면거리를 따로 확인해야 한다. 실제 RGB/depth 공통 FOV, 최저 표면거리, 센서 구조물 간섭, 팔 도달성도 확인한다.25cm 목표가 작업 공간 밖이거나 설정한 관측 optical 방향이 수직이 아니면 자동 하강하지 않고 실패한다. 이 패키지는 기본3개 안정 프레임의 단일 높이 재관측을 구현한다.

## End Effector 기준점의 뜻

기존 `artifacts/calibration/wrist_handeye_setup.launch.py`의 `robot_effector_frame`은 제조사 모델의 `link6`다. 통합 모델의 이름은 `piper_link6`이며 `piper_flange_joint`의 xyz/rpy가0이므로 CAD에서 `piper_flange_link`와 같은 원점·축을 사용한다. End Effector라는 부품 덩어리의 임의 중앙점을 뜻하지 않는다. 이것은 저장된 파일의 설정이며 현재 실행 중인 장비의 TF를 확인한 결과는 아니다.

hand–eye는 이 고정 link6/flange frame↔카메라 optical frame 관계를 구한다. [easy_handeye2 공식 데모](https://github.com/marcoesposito1988/easy_handeye2_demo)도 robot_effector_frame을 명시적인 로봇 link로 설정한다. TCP는 양 손가락 접촉 부위 사이의 파지 중심으로 정의하는 편이 이 작업에 편리하다. link6↔TCP는 두 번째 hand–eye가 아니라 도구의 고정 tool offset(위치+축)이다. 위치는 CAD/실측/pivot으로 확인하고 회전은 접근축·닫힘축으로 따로 정의한다.

카메라 디텍션 점은 자동으로 EE 좌표가 되지 않는다. 취득시각의 `T_base_link6 * T_link6_camera * p_camera`로 물체 위치를 팔 기준으로 바꾼다. 그 위치와 grasp rule로 원하는 TCP pose를 생성한다. `T_tcp_camera = inverse(T_link6_tcp) * T_link6_camera`로 TCP에서 본 점을 표현할 수도 있지만, 움직임 계획의 목표는 gravity/base 기준 TCP pose다. TCP를 파지 중심에 정의하면 `tcp_to_grip_center_m=[0,0,0]`이고, 다른 기준으로 정의했다면 그 오프셋을 별도로 둔다.
