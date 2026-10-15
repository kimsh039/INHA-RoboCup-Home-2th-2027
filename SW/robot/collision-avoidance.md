# Tracer 외곽 기준 10 cm의 3D 장애물 회피

`config/collision_safety.yaml`의 `clearance: 0.10`은 URDF 충돌 형상의 바깥면부터 장애물까지의 최소 여유다. 센서나 `base_link` 원점부터 10 cm가 아니다. Tracer 차체 메시를 URDF origin으로 변환하면 약 0.685 × 0.570 m이며, 앞뒤·좌우로 각각 10 cm를 확보하는 직선 주행 영역은 약 0.885 × 0.770 m다. 사각형 확장은 모서리에서 원형 거리 확장보다 보수적이다.

SLAM Toolbox의 지도·위치추정·TF 소유권은 기존 구성을 사용한다. Nav2가 우회 경로를 만들고 3D 보호 노드가 최종 속도를 제한한다.

## 형상과 입력

- `collision_geometry.py`: 실물 URDF의 Tracer 및 팔, 전체 URDF의 바퀴·랙·고정 센서 collision을 읽는다. 메시에는 보수적인 oriented bounding box를 사용한다. 팔은 `/robot/joint_states`의 실측 관절값을 적용하며, 피드백이 없는 그리퍼 손가락은 전체 허용 스트로크를 포함한다. 랙은 추가로 부품 전체의 로컬 좌표 최솟값·최댓값에서 내부 제외용 외곽 상자를 만들며, 이 상자를 물리적 충돌 형상이나 Nav2 footprint에는 추가하지 않는다.
- `collision_processing/src/collision_cloud.cpp`: `/livox/lidar`의 점별 `timestamp`와 `/odom_ekf`를 이용해 운동 보상한다. 설치된 Livox 드라이버의 `timestamp`는 절대 나노초다. 개별 URDF 충돌 형상 안쪽, 랙 외곽 상자의 배선·빈 공간, MID-360 측정 원점에서 15 cm 이내인 점을 자기 반사로 제외한다. 제외된 점은 장애물 등록과 빈 공간 광선 갱신 양쪽에서 사용하지 않는다.
- `/mid360/points_filtered`: 공통 시각 `base_link` 점군. 바닥 반사를 포함하며 4 m 이내의 표시용 입력이다.
- `/mid360/points_obstacles`: 같은 점군에서 실측 바닥 평면 위 2 cm를 넘는 점만 분리한 Nav2 marking 입력이다. local/global costmap 모두 이 토픽으로 장애물을 등록한다.
- `/mid360/points_clearing`: 자기 반사·고립점 필터를 통과한 실제 측정 끝점이며 4 m 밖의 측정도 포함한다. Nav2는 이 입력을 clearing에만 사용하고 자기 raytrace 범위에서 자른다. Guard도 이 광선들을 지역 4 m 원통 안에서만 순회한다. 먼 벽에 도달한 광선이 가까운 셀을 통과했다면 그 셀의 빈 공간 증거로 사용한다.
- `/collision/points`: `odom` 좌표에서 유지하는 4 cm 점유 voxel 중심. 아래 고립점 필터를 통과한 관측만 등록하며, 통과한 점이 한 voxel에만 모였더라도 유지한다. 관측이 끊긴 장애물은 시간만으로 삭제하지 않는다. 새 광선이 해당 셀을 통과해 더 먼 지점에 도달하는 관측이 서로 다른 유효 프레임에서 세 번 누적되면 제거하며, 로봇의 지역 관측 범위 밖으로 나간 셀은 버린다. 광선의 끝점 근처 셀은 clearing에서 제외하며, 같은 프레임의 장애물 관측이 clearing보다 우선한다.

기본 자기 반사 여유는 5 mm, 거리 오차 여유는 5 cm다. 안전 판단에는 voxel 중심의 최대 위치 오차도 더한다. 따라서 10 cm는 유지하려는 최소값이고 감속·정지는 그보다 먼저 발생한다. 지원 바닥은 실물 보정의 `base_link.z = -0.1425 m`를 기준으로 하며, 경사면·단차 인식기는 포함하지 않는다.

## 경로와 속도

`prepare-navigation-profile.py`는 실행할 때 물리적 URDF 외곽의 footprint를 생성한다. Guard는 실제 팔 자세까지 반영해 local/global footprint를 갱신한다. 전 방향 footprint에 여유 거리를 덧붙여 뒤 장애물에서 멀어지는 출발까지 막지 않는다. 최소 여유와 거리·격자 오차는 최종 3D guard가 이동 방향·회전·제동 경로에 적용한다. DWB의 `BaseObstacle`과 `ObstacleFootprint` critic을 함께 사용하며 inflation 비용으로 장애물과 떨어진 경로를 선호한다. NavFn은 기존 전역 경로를 제공하고, 방향을 포함한 전체 형상 검사는 DWB와 최종 guard에서 수행한다. 매우 좁은 통로에서는 전역 후보 경로가 나와도 지역 제어가 거부할 수 있다.

footprint 내부의 장애물이 자동으로 지워지지 않도록 footprint clearing을 비활성화한다. `inflation_radius: 0.85`는 선호 경로를 위한 비용이고, 이동 방향별 최소 여유는 최종 3D gate가 적용한다.

```text
Nav2 controller → velocity_smoother ─┐
Nav2 회전·후진 / ROS teleop ─────────┤
                                   ↓
                                /cmd_vel
                                   ↓
                            collision_guard
                                   ↓  TwistStamped, 유효 시각 포함
                      /collision/cmd_vel_approved
                                   ↓
                         tracer_safety_watchdog
                                   ↓  Twist
                         /tracer/cmd_vel_safe
                                   ↓
                        remap된 tracer_base_node
```

Guard는 측정된 현재 속도의 반응 구간, 후보 속도의 반응 구간과 감속 구간에서 이동·회전하는 모든 collision box와 점유 voxel의 거리를 검사한다. 샘플 사이의 모서리 이동량도 거리 여유에 포함한다. 각 부품과 원래 측정점 사이의 부호 있는 거리가 줄어드는 이동에만 여유 거리 제한을 적용한다. 뒤쪽 장애물에서 멀어지는 직선 전진과 간격을 유지하는 평행 이동은 뒤쪽·옆쪽 근접점만으로 막지 않는다. 후진하거나 회전 중 뒤 모서리가 그 점 쪽으로 다가오면 검사하며, 충분한 여유가 없으면 감속하거나 정지한다. 현재 전 방향 최근접 거리는 표시용이며 명령과 무관한 일괄 정지 조건으로 사용하지 않는다.

MID-360, EKF, TF 또는 필요한 팔 피드백이 없거나 오래되면 정지한다. 팔이 움직이는 동안에도 차체를 정지시킨다. Guard 계산 시간이 한도를 넘으면 승인 속도를 0으로 만들고, 별도 watchdog은 승인이 0.15초 이상 끊기면 정지를 계속 출력한다. 두 보호 노드의 출력 소유권과 Tracer 구독 경로도 감시한다.

기본 속도 한도는 전진 0.25 m/s, 후진 0.15 m/s, 회전 0.6 rad/s다. 감속 모델은 선속도 0.5 m/s², 각속도 1.0 rad/s², 반응 시간 0.3초에 센서 관측 지연을 추가한다. 이는 설정된 가정이며 실제 적재·바닥 조건에서 측정한 제동 보장이 아니다. 충돌 노드·watchdog·드라이버 전체가 동시에 멈추는 경우는 별도의 하드웨어 정지 기능에 의존한다.

## 실행과 관제

실물 core의 `calibration:=true` 그룹이 3D 전처리, guard와 watchdog을 소유한다. 별도 CAN core의 `calibration:=false`는 같은 노드를 중복 실행하지 않고 Tracer 입력만 보호 토픽으로 remap한다. Nav2 실행기는 이전 시뮬레이션 XY self filter를 추가 실행하지 않는다.

코드 적용에는 관제센터와 여기서 실행한 실물 core의 재시작이 필요하다. **Nav2만 다시 시작하면 기존 Tracer의 입력 remap과 bridge의 보호 상태 구독이 바뀌지 않는다.** 기존 외부 Tracer가 `/cmd_vel`을 직접 구독하는 경우 보호 상태가 준비되지 않는다.

관제센터 NAV 카드에는 외곽 기준 거리, 감속·정지 사유와 watchdog 상태를 표시한다. 지도에는 Tracer 외곽과 정확한 10 cm 경계, 최근접 장애물을 표시한다. Nav2 서버는 보호 입력 대기 중에도 시작할 수 있다. 새 목표 전송과 실제 주행은 보호 입력·출력이 준비된 경우에만 허용하며, 취소는 항상 가능하다.

관제센터 기본 3D 채널은 `/mid360/points_filtered`를 우선한다. 따라서 기본 표시도 같은 자기 반사 필터와 4 m 처리 범위를 사용한다. `/livox/lidar` 원시 토픽은 진단을 위해 별도 선택할 수 있다.

PiPER 피드백은 `piper_feedback.py`가 수신 전용 SocketCAN으로 읽는다. 부팅 시 응답이 없어도 대기하며 인터페이스가 재연결되면 자동 재개한다. `0x2A5/0x2A6/0x2A7`의 세 관절 쌍을 모두 새로 수신해야 발행하고, 커널 수신 시각 중 가장 오래된 시각을 메시지에 보존한다. SDK의 전체 CAN FPS 또는 모터 상태의 시각으로 오래된 관절값을 갱신하지 않는다. 팔 응답이 없으면 차체 정지를 유지한다.

MID-360에서 보이지 않는 근접 사각지대, 투명·검은 물체의 미검출, 움직이는 장애물의 미래 위치와 적재물 외형은 이 모델이 보장하지 않는다. 감지된 장애물과 URDF 형상에 대한 ROS/CAN 주행 보호이며, 하드웨어 RC 직접 주행이나 팔 자체의 이동 명령을 제어하지 않는다.

## 근거

- Fox, Burgard, Thrun, [The Dynamic Window Approach to Collision Avoidance (1997)](https://publications.ri.cmu.edu/the-dynamic-window-approach-to-collision-avoidance): 동역학과 제동 가능성을 함께 고려하는 접근. 현재 gate는 이 원칙을 사용하며 논문 알고리즘 전체를 재현한 구현은 아니다.
- Macenski, Tsai, Feinberg, [Spatio-temporal voxel layer (2020)](https://journals.sagepub.com/doi/full/10.1177/1729881420910530): 지역 3D 관측과 시간·시야에 따른 장애물 관리. 이번 구현에는 STVL 의존성을 추가하지 않았으며 관측되지 않은 공간을 시간 만료로 비우지 않는다.
- Oleynikova et al., [Voxblox (IROS 2017)](https://arxiv.org/abs/1611.03631): 3D 거리장 기반 계획. 이번 구현은 OBB와 점유 voxel의 거리를 직접 계산하며 전체 ESDF를 구축하지 않는다.
- 설치 버전 Nav2 1.1.20의 [PointCloud 처리](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_collision_monitor/src/pointcloud.cpp), [stale source 처리](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_collision_monitor/src/source.cpp), [footprint critic](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp). Humble Collision Monitor의 높이별 XY 투영과 오래된 입력 무시를 3D 거리·watchdog 보장으로 간주하지 않았다.

초기 Python 구현 때는 사용자 지시에 따라 테스트·빌드·실행 검증을 수행하지 않았다. 이후 C++ 전환의 실행 파일 생성과 필수 안전 회귀 검사는 해당 변경에 포함된다.

## 실측 바닥 분리와 오래된 장애물 갱신

고정 `floor_z`는 바닥 추정의 사전 높이다. 전처리는 자기 반사와 고립점 제외 후, 로봇 주변 0.65~3 m의 낮은 점에서 방향을 제한한 RANSAC 평면을 추정한다. 표본은 최대 2,048개, 후보는 128개이며 기본 갱신 간격은 0.25초다. 기울기 5도 이내, 사전 높이와의 차이 5 cm 이내, 충분한 면적과 55% 이상의 지지 점을 요구한다. 원래 측정점의 평면 위 높이로 바닥을 분리하며 4 cm voxel 중심의 높이를 바닥 판정에 사용하지 않는다. 로봇 TF나 URDF 바닥 기준은 변경하지 않는다.

유효한 평면을 2초 넘게 갱신하지 못하면 `unreliable_floor_plane`으로 입력 승인을 중지한다. 유효한 평면으로 바닥에 해당하는 기존 저장 셀도 제거하며, 빈 공간 광선의 바닥 경계도 같은 기울기를 따른다. 바닥 위 2 cm 이내의 낮은 물체는 바닥 제외 범위에 포함된다. 이 구현은 단일 평면을 가정하며 계단·단차·복수 지면을 해석하는 지형 인식기는 아니다.

장애물 셀의 빈 공간 증거는 서로 다른 유효 프레임에서 세 번 누적되면 삭제한다. 중간에 해당 셀이 미관측·가림 상태이면 횟수를 유지하고, 실제 장애물이 다시 관측되면 0으로 초기화한다. 이전의 연속 세 프레임 조건은 비반복 스캔이 해당 셀을 건너뛸 때마다 삭제 진척을 지우므로 사용하지 않는다. 시간 경과만으로 장애물을 삭제하지 않는다.

2026-10-15의 재발 진단에서는 로봇 뒤 약 31 cm / 왼쪽 22 cm / 바닥 위 1.30 m의 약 330초 된 기록이 회전 명령을 막았다. 최신 원본 30프레임에서 그 4 cm 셀을 통과해 충분히 멀리 도달한 광선 2,980개가 있었지만, 끝점의 로봇 XY 거리가 모두 4.06~6.42 m라서 기존 4 m 범위 필터가 광선 전체를 버렸다. 이 중 1,434개는 5 cm 이웃점 조건도 충족했다. 즉 가려진 장애물을 보존하는 목적과 별개로, 실제로 다시 본 빈 공간을 삭제에 반영하지 않는 결함이 있었다. 수정 후에는 멀리 측정된 점도 원래 측정점에서 자기 반사·고립점 검사를 거쳐 clearing에 전달하고 지역 범위만 잘라 사용한다. 먼 끝점 자체를 가까운 가짜 장애물로 등록하지 않는다. `clearing_rays`, `long_range_clearing_rays`로 해당 입력 수를 보고한다.

`/collision/points`에는 `last_seen_age`, `height_above_floor`를 추가한다. Guard는 전 방향 최근접 점과 실제 후보 이동을 막은 점을 구분해 보고한다. 화면은 경로 장애물만 붉게 표시하고, 일반 최근접 점은 현재 관측 여부에 따라 청록색·회색으로 표시하며 바닥 위 높이와 마지막 관측 나이를 함께 표기한다. 새 프레임 발행 시각으로 오래된 셀의 나이를 갱신하지 않는다.

G2 `/scan`의 부품별 자기 반사 여유도 3D 입력과 맞춘다. 랙 내부 3 cm, 차체·바퀴 2 cm, 나머지 고정 부품 5 mm를 사용한다. 따라서 같은 랙 내부 배선이 3D에서 제외된 뒤 G2 경로로 Nav2에 재등록되는 설정 차이를 없앤다.

## MID-360 처리 지연 개선

2026-10-15 18:28의 읽기 전용 수집에서 원본 131프레임, 필터 출력 126프레임에 비해 retained scene 수신은 63프레임이었다. 같은 구간 전처리 상태 201개는 모두 `live`였으나 guard에는 `stale_mid360` 12개와 `missing_cloud_health` 3개가 기록됐다. 원본 센서의 수신 단절과 큰 scene의 전달 누락을 구분해야 한다. 당시 scene은 약 81,000점 × 32바이트, 프로세스의 Fast DDS 공유 메모리는 기본 약 512 KB였다. [Fast DDS 2.6 문서](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)는 메시지보다 작은 segment의 손실 위험을 설명한다.

두 실행기는 `config/collision_fastdds.xml`을 기본으로 사용한다. 기존 사용자 DDS 프로필이 있으면 보존한다. 공유 메모리는 참여자별 64 MiB이고 UDP 검색·원격 연결도 유지한다. `/collision/points` 양 끝은 reliable / keep-last 1로 연결하고 발행 수명은 `cloud_timeout`으로 제한한다. 원본 센서 입력과 다른 필터 출력의 QoS는 그대로다. 늦게 처리된 한 프레임은 버리되 직전 유효 scene의 원래 유효 기한을 줄이거나 늘리지 않는다. 다른 입력 오류와 세대 변경은 기존처럼 승인을 중지한다.

최신 전처리 상태와 scene에 짝지어진 상태를 별도로 보관한다. 전처리 관측은 최신인데 검사 scene이 만료된 경우 `stale_collision_scene`, 관측 자체가 만료된 경우 `stale_mid360`을 보고한다. `scene_source_age`, `scene_receive_age`, `cloud_health_receive_age`로 두 경우를 추적한다. 0.4초 만료와 150 ms 출력 watchdog은 유지한다. 신선한 관측이 없는데 계속 주행하면 새 장애물 진입을 판정할 수 없기 때문이다.

같은 수집의 전방 차단점은 odom `(1.875779, -0.479566, -0.128960)`이고, 로봇 기준 약 앞 54 cm / 오른쪽 7 cm였다. 현재 평면 위 높이는 2.93 cm, 마지막 관측은 한국 시각 18:21:00이며 수집 당시 나이는 약 453초였다. 저장 scene에 직진·직전 Nav2 요청을 대입한 계산에서 이 점이 `base_link/0`의 진행을 차단했고, 실제 이동 명령은 발행하지 않았다. 최신 원본 30프레임에는 이 점의 XY 주변 반경 12 cm에 측정점이 없었다. 가까운 바닥은 현재 관측되지 않으므로 빈 공간 광선 세 번으로 지우는 조건도 충족하지 못했다. 이 점은 물체 분류 결과가 아니라 바닥 평면 위 2 cm 조건을 통과해 남은 MID-360 점유 기록이다. 저장된 XYZ만으로 실물 종류나 바닥 반사 오차 여부를 확정할 수 없다. 노드 재시작은 이전 메모리를 비우지만 바닥 오검출의 재발 방지를 보장하지는 않는다.

Guard는 원래 측정점의 `nearest_source_point_base` / `blocking_source_point_base`와 `*_last_seen_stamp`도 보고한다. 관제센터는 앞뒤·좌우 거리, 현재 바닥 위 높이, 마지막 관측 시각과 경과 시간을 표시한다. 새 scene 시각으로 과거 반사점의 관측 시각을 갱신하지 않는다.

C++ `robocup_collision_processing` 노드는 ROS 수신 콜백과 별도 계산 작업자를 사용한다. 수신 콜백은 입력을 저장하고 반환하며 EKF와 관절 이력을 약 3초 유지한다. 계산 중인 점군 외에는 최신 대기 점군 하나만 보관한다. 대기 점군 교체는 기존 장애물 삭제와 무관하다.

광선 clearing은 32 cm 격자에서 장애물 셀이 없는 구간만 건너뛴 뒤, 장애물 셀이 있는 구간을 원래 4 cm 격자로 순회한다. 장애물 관측점이나 광선을 줄이지 않는다. 음수 좌표·경계와 다양한 방향의 광선에 대해 독립적인 ray/AABB 교차 계산과 비교한 253개 경우에서 같은 점유 셀이 선택됐다.

자기 반사 검사는 모든 로봇 형상의 AABB 밖에 있는 점을 먼저 제외하고, 각 형상의 AABB를 통과한 후보만 기존 OBB 포함 검사에 넣는다. AABB는 후보 계산을 줄이는 용도이며 마스크 여유 5 mm를 확대하지 않는다. 회전된 형상·음수 좌표·형상 표면과 그 안팎의 점 13,376개를 기존 OBB 판정과 비교해 마스크 경계가 바뀌지 않는지 검사했다.

점별 시각·EKF 보간·보정 TF를 보존하고, 관측 중간 시각을 공통 기준으로 삼는다. 팔 형상은 같은 시각의 관절값을 보간해 사용하고, 프레임 중 팔 형상 변화가 자기 반사 여유보다 크면 그 입력은 거부한다. 시각 역행과 오도메트리 초기화에는 새 세대를 부여해 이전 계산이 뒤늦게 발행되지 않게 한다. 계산 완료 때 관측 나이를 다시 검사한다.

최종 guard는 점군과 상태를 같은 관측 시각으로 연결하며, 계산 후에도 원래 입력의 유효성을 재확인한다. 각 승인 주기는 40 ms 계산 예산을 사용하며 독립 watchdog의 승인 만료 시간은 150 ms다. guard의 팔 속도 판정에는 콜백 도착 간격이 아니라 관절 메시지의 원래 시각 차이를 사용한다.

원래 관측점을 voxel 중심과 함께 보관하므로 `/collision/points`의 `source_x/source_y/source_z`와 `/collision/status`의 `nearest_source_point`로 랙 근처 거리 0 판정의 원인을 추적할 수 있다. 자기 반사 여유를 일괄 확대하지 않는다.

실행 파일 생성은 다음과 같다. 빌드 산출물은 기본적으로 `~/robot_setup/collision-runtime`에 두며 저장소에 추가하지 않는다.

```bash
bash ~/robocup/SW/robot/build-collision.sh
```

`run-collision-cloud.sh`와 `run-collision-guard.sh`는 기존 Python URDF 해석기로 같은 모델을 한 번 내보낸 뒤 각각 C++ 실행 파일을 시작한다. `collision_cloud.py`와 `collision_guard.py`의 직접 실행도 해당 실행기로 연결한다. 새 실행 파일이 없으면 기존 처리기로 자동 전환하지 않고 오류로 종료하며 차체 watchdog은 정지를 유지한다. 소스 변경 후 위 실행 파일 생성과 core 재시작이 필요하다.

빌드는 `collision-runtime/releases`에 새 산출물을 생성한 다음 전처리·guard 두 실행 파일이 모두 설치된 경우에만 `current` 심볼릭 링크를 원자적으로 전환한다. `flock`으로 여러 터미널의 빌드를 직렬화하고 매 빌드의 `CMAKE_INSTALL_PREFIX`를 명시해 같은 CMake 캐시의 설치 경로 충돌을 막는다. 실행 중인 파일을 덮어쓰지 않으며 새 산출물은 core 또는 해당 프로세스를 다시 시작할 때 적용된다. 전처리·guard 프로세스가 종료되면 launch가 2초 후 재시작하고, 그동안 승인 없는 차체는 watchdog에 의해 정지한다.

전처리 상태에는 `queue_wait_ms`, `processing_ms`, `processing_p95_ms`, 단계별 `timings_ms`, `odom_interval_max_ms`, `source_age`, 대기 입력 교체 횟수가 들어간다. 관제센터의 충돌 상태에는 최근 계산 시간·대기 시간·관측 나이를 표시한다. 이 수치는 실행 중 관측값이며 목표 성능을 달성했다는 보장은 아니다.

### 2026-10-15 실물 적용 관측

MID-360을 수신하며 정지한 상태에서 최종 실행 파일을 적용했다. 기존 `map_20261015_1316`의 SLAM Toolbox 자동 위치추정을 복원했으며 Nav2 서버는 적용 전과 같은 정지 상태로 두었다. 실제 이동 명령은 보내지 않았다.

재시작 직후 8초를 제외한 약 22초 동안 관제 상태로 수신한 서로 다른 처리 결과 109개의 관측값은 다음과 같다. rosbag을 고정한 비교 실험이나 주행 성능 보장은 아니다.

| 항목 | 중앙값 | 95백분위 | 최댓값 |
| --- | ---: | ---: | ---: |
| 전처리 시간 | 46.35 ms | 53.68 ms | 56.57 ms |
| 입력 대기 | 0.033 ms | 0.205 ms | 0.609 ms |
| 상태에 기록된 관측 나이 | 98.70 ms | 108.03 ms | 184.65 ms |
| 처리 스캔 구간의 최대 EKF 메시지 간격 | 20.22 ms | 23.72 ms | 24.16 ms |

최종 상태의 원시 입력은 20,064점, 유지 장애물은 29,537개였으며 대기 점군 교체 횟수는 0이었다. 전처리 95백분위 50 ms 목표에는 조금 못 미치지만 관측 나이 95백분위는 150 ms 목표 이내였다. 부하·장애물 분포·지도 크기에 따라 값이 달라진다.

같은 구간의 보호 상태 110개는 모두 입력 준비 상태였고 정지 사유는 `clearance_boundary`였다. 최근접 셀은 `rack_base_link/profiles_500mm_5_body1` 부근이며, 그 셀에 보관된 측정점은 URDF 표면에서 약 5.04 mm 밖에 있었다. 5 mm 자기 반사 여유를 벗어난 측정점이 4 cm voxel의 중심으로 표현되면서 거리 0으로 표시됐다. 이는 10 cm 경계 안의 측정점이라는 뜻이며 지연 오류는 아니었다. 이후 사용자가 랙 내부 배선·빈 공간 전체와 센서 원점 15 cm 이내의 제외를 요청해 아래 필터를 추가했다. 위 성능 수치는 이 추가 변경 전의 관측값이다.

### 랙 내부 및 센서 근거리 제외

`mask_rack_interior: true`는 URDF 랙 collision 전체를 감싸는 상자 안의 점을 제외한다. 외곽 상자는 랙 로컬 좌표에서 계산해 보정된 `base_link` 변환을 적용하므로 배선 위치나 임의의 지도 좌표를 하드코딩하지 않는다. Tracer·고정 센서·팔은 각 부품의 collision 내부도 제외하며, 팔 자세는 기존 실측 관절값을 사용한다. 팔과 랙 전체를 한 상자로 묶어 그 사이의 외부 공간까지 제거하지 않는다.

`min_sensor_range: 0.15`는 변환 전 MID-360 점의 `x²+y²+z² <= 0.15²`를 검사한다. `base_link` 기준 거리나 XY 원통이 아닌 센서 원점 중심의 3D 구이며, 경계를 포함한다. 두 제외 조건은 합집합으로 적용되고 Nav2 점군과 3D 보호용 장애물에 공통으로 반영된다. 원시 `/livox/lidar` 메시지는 보존한다. `clearance: 0.10`은 제외 영역 바깥에서 URDF 로봇 외곽에 적용하는 기존 거리 설정이다.

이 변경은 랙 내부에 실제로 들어온 외부 물체도 제외하므로 내부 공간은 장애물 감지 대상에서 빠진다. 이전 메모리에 남아 있는 점을 지우고 새 형상을 읽으려면 전처리와 guard를 포함한 관제센터를 새로 실행한다. 변경 당시 관제센터와 실물 core는 실행 중이 아니었다. 필수 회귀 검사는 기록된 배선 추정점·랙 빈 공간의 제외, 랙 외부 점의 보존, 제외용 상자와 실제 충돌 형상의 분리, 센서 거리의 15 cm 경계와 3D 거리 구분을 포함했다. 실물 재실행 후 감지 결과는 별도로 측정하지 않았다.

참고: [ROS 2 Humble 콜백 그룹](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Using-callback-groups.rst), [Casini 외 ECRTS 2019](https://retis.sssup.it/~d.casini/papers/2019/ecrts19.pdf), [LIO-SAM](https://arxiv.org/abs/2007.00258), [FAST-LIO2](https://arxiv.org/abs/2107.06829), [OctoMap](https://www.arminhornung.de/Research/pub/hornung13auro.pdf). 알고리즘 전체를 교체한 것은 아니며 실행 분리·시각 보존·증분 갱신과 빈 공간 관측 원칙을 참고했다.

### 랙 외곽의 자기 반사 여유

차체 뒤쪽 약 11 mm 바깥의 낮은 자기 반사점도 별도로 제외하도록 `base_self_padding: 0.02`를 적용한다. 차체 및 바퀴 collision에만 적용하며 팔·고정 센서의 기본 여유와 분리한다. 차체·바퀴 표면에서 이 여유 안에 있는 외부 물체도 제외된다. 변경 전 장애물 메모리와 표시 점군 이력은 적용 시 비운다.

랙 표면에서 약 2 cm 떨어진 측정점이 기존 5 mm 마스크를 통과해 정지를 유지하는 경우를 반영해 `rack_self_padding: 0.03`을 추가했다. `self_interior/rack_base_link` 제외 상자에만 3 cm 여유를 적용하며, 다른 부품의 `self_padding: 0.005`와 장애물까지의 `clearance: 0.10`은 별도 설정이다. 랙 내부와 이 여유 안의 점은 Nav2 입력·3D 장애물 등록·빈 공간 광선에서 제외한다. 이 범위에 들어온 외부 물체도 제외 대상에 포함된다. 이전에 저장된 점은 전처리 재시작으로 비운다.

### 고립된 단독 측정점 제외

`outlier_radius: 0.05`, `outlier_min_neighbors: 1`은 자기 자신을 제외한 다른 유효 측정점이 3D 거리 5 cm 안에 하나도 없을 때 그 점을 제거한다. 운동 보상한 원래 측정점에 적용하며, 화면 픽셀 수나 4 cm voxel의 개수로 물체 크기를 판정하지 않는다. 같은 voxel의 여러 측정점이나 5 cm 안에서 서로 지지하는 점은 유지한다. `outlier_min_neighbors: 0`은 이 단계만 비활성화한다.

URDF 자기 반사·센서 근거리 제외 후, 4 m 처리 범위 안의 점끼리만 이웃을 계산한다. 제외된 자기 반사가 바깥의 단독점을 지지하지 않게 한다. 제거된 점은 Nav2 입력과 3D 장애물 등록에서 모두 빠지고 빈 공간 갱신 광선으로도 사용하지 않는다. 실제 외부 전선이나 작은 물체도 단독점으로만 관측되면 제외되며, 이 조건으로 자기 반사 여부 자체를 판별하는 것은 아니다.

방식은 [PCL RadiusOutlierRemoval 공식 설명](https://pointclouds.org/documentation/tutorials/remove_outliers.html)의 반경·이웃 수 원칙을 사용한다. 추가 PCL 의존성 없이 공간 해시로 구현하며, 필수 회귀 검사는 전수 거리 비교 결과와의 일치, 단독점·경계·음수 좌표·마스크와 가까운 점들이 이어진 가는 구조의 보존을 포함한다. 상태에 `isolated_points_removed`, `outlier_radius`, `outlier_min_neighbors`와 `timings_ms.outlier_filter`를 제공한다.

추가 직전에는 정지 원인의 유지 셀 주위 10 cm 안에 다른 셀이 없었다. 최근 스캔에서 해당 영역의 단독 측정점이 한 번 보였고 다음 스캔들에서는 보이지 않아도 메모리 셀이 정지를 유지했다. 필터 적용 시 전처리의 이전 메모리와 Nav2 costmap을 함께 초기화하고 이동 목표는 재전송하지 않는다.

실물 적용에서는 Nav2를 멈춘 뒤 새 전처리 프로세스로 교체하고 관제 점군 이력을 비웠으며 Nav2 서버를 다시 실행했다. 적용 직후 보호 상태는 `ready: true`, `state: idle`, `reason: command_timeout`으로, 기존 고립점의 `clearance_boundary` 정지가 해제되고 이동 명령을 기다리는 상태였다. 이때 원시 19,872점 중 고립점 2,165개가 제외됐고, 해당 프레임의 이웃 필터 계산은 약 10 ms, 전체 전처리는 약 41 ms였다. 한 시점의 관측값이며 고정 부하 벤치마크는 아니다. 기존 목표는 취소 상태로 두었고 새 이동 명령은 보내지 않았다.

## G2 입력 누락 수정과 주행 보호 경량화

후속 조사에서는 G2 `/scan`에 `base_link` 기준 약 `(0.144, -0.166, 0.350) m`의 측정점이 남아 있었다. 이 점은 랙 내부 제외 영역에 속하며 로봇 중심에서 약 22 cm 떨어져 있었다. 당시 Nav2는 이 `/scan`도 장애물 입력으로 사용했지만 기존 URDF·고립점 필터는 MID-360 분기에만 적용돼 있었다. Nav2 로그에는 `Failed to make progress`와 `BaseObstacle/Trajectory Hits Obstacle`, 후보 궤적 419개 거부가 기록됐다.

### G2 수정

`calibrated_streams.py`가 발행하는 공통 `/scan`에 `scan_self_filter.py`를 적용했다. 같은 URDF의 고정 차체·바퀴·센서 형상과 랙 내부 상자, 센서 원점 15 cm 이내를 제외한다. 기존 각도 마스크도 유지한다. `/scan`을 받는 Nav2와 `/scan_slam` 생성기가 모두 이 결과를 사용한다. G2에는 MID-360용 5 cm 고립점 필터를 추가하지 않았다. 2D 각도 샘플은 먼 벽에서 점 사이 간격이 커지므로 같은 이웃 거리 기준을 그대로 적용하지 않는다.

센서 변환은 `sensor_tf.py`와 같은 실물 캘리브레이션의 `base_mid360.json`과 G2–MID360 `calibration.json`을 합성한다. 고정 장착이므로 시작 시 한 번 읽고, 빔 배열이 달라질 때만 각 빔과 URDF 상자의 교차 거리 구간을 계산한다. 매 스캔에서는 보정된 거리값이 이 구간 안인지 비교한다. 같은 방향이어도 형상 뒤의 먼 벽까지 통째로 지우지 않는다. 제거 결과는 `NaN`이며 원래 header·각도 순서·빔 수·시간 간격을 보존한다. 프레임이 예상된 `laser_frame`과 다르면 필터 없이 우회 발행하지 않는다.

G2 구독·발행 큐는 최신 스캔 하나만 유지하며, BLAS 스레드는 1개로 제한한다. 불변 보정 CameraInfo와 스캔 메타데이터는 얕은 복사를 사용해 반복 deep copy를 줄인다. `/scan/filter_status`에서 제거 원인별 점 수와 계산 시간을 확인할 수 있다. 팔의 현재 관절 자세는 기존 MID-360 자기 반사 필터와 3D 보호에서 처리하며 G2 고정 마스크에 팔의 전체 동작 범위를 추가하지 않는다.

### 거리 계산과 공간 인덱스

`collision_distance.py`는 관절값이 같은 형상의 중심·회전·반경·회전 시 최대 XY 반경을 캐시한다. KD-tree 조회는 8개 형상씩 묶고, 후보 점들의 OBB 거리를 NumPy로 계산한다. 내부 후보 계산은 4,096점 단위로 나눠 계산 예산을 검사한다. 거리 0이 나오면 더 작은 값이 없으므로 해당 조회를 종료하며, 이 경우에도 마감 시간 검사를 한다.

새 scene의 KD-tree는 `leafsize=32`, `compact_nodes=False`, `balanced_tree=False`로 생성한다. 10 Hz마다 재생성하는 작업에서 균형·노드 압축 비용을 줄이는 선택이며, 조회는 계속 `eps=0`, `workers=1`을 사용한다. 근사 검색·점 다운샘플링·3D 장애물 삭제는 추가하지 않는다. 입력은 바로 연속 float64 배열에 채워 중간 배열을 줄이고, 진단용 원래 측정 좌표는 입력의 float32 정밀도를 유지한다.

같은 실물 93,228점과 74개 URDF 형상으로 비교한 계산 시간은 다음과 같다. 실제 주행 속도나 전체 시스템 CPU가 같은 비율로 개선된다는 뜻은 아니다.

| 작업 | 기존 중앙값 | 변경 중앙값 | 비교 조건 |
| --- | ---: | ---: | --- |
| KD-tree 생성 | 61.84 ms | 19.99 ms | 같은 점군, 각 설정 3회 |
| 형상 거리 조회 | 4.80 ms | 1.80 ms | 같은 점군·기존 KD-tree·같은 자세 20개, 배치 계산만 비교 |

회귀 검사는 기록된 G2 내부 점의 제외·같은 방향의 먼 점 보존·회전된 형상·스캔 메타데이터 보존, 이전 개별 형상 계산과의 거리 및 경계 판정 일치, 계산 중 만료된 입력의 정지를 포함한다. 센서 데이터로 오프라인 계산만 수행했으며 이동 목표나 비영 속도를 보내지 않았다. 보호 상태에 `guard_processing_ms`, `guard_budget_ms`, `scene_parse_ms`, `scene_tree_build_ms`를 추가했다. 새 이동 명령이 3D 보호에서 차단되면 사유·요청 속도·검사 시간을 2초 간격으로 로그에 남긴다.

추가 경량화 후보는 다음 순서다.

1. **3D 보호 계산의 C++ 이식 또는 점유 셀 변경분 인덱스 갱신:** Python 콜백과 매 프레임 공간 인덱스 재생성 비용을 더 줄일 수 있다. 관측 세대·원래 시각·정지 watchdog의 기존 계약을 유지해야 한다.
2. **DWB 속도 샘플 수 조정:** 현재 20×20 설정을 줄이면 후보 계산이 감소하지만 좁은 공간의 경로 선택 해상도도 낮아진다. 이번 변경은 후보 수와 제어 주기를 유지했다.
3. **이미지 통신 경로 정리:** 보정 RGB 재발행을 소비 노드의 입력 remap 또는 같은 프로세스의 컴포넌트 통신으로 바꾸는 방안이다. 카메라 프레임률을 내리기 전에 복사·직렬화 경로부터 줄이는 순서가 적절하다.

근거: [ROS laser_filters BoxFilter](https://github.com/ros-perception/laser_filters/blob/ros2/include/laser_filters/box_filter.h)의 형상 내부 점을 NaN으로 바꾸는 방식, 설치된 SciPy 1.8.0의 [배치 반경 조회](https://docs.scipy.org/doc/scipy-1.8.0/reference/generated/scipy.spatial.cKDTree.query_ball_point.html)와 [KD-tree 생성 옵션](https://docs.scipy.org/doc/scipy-1.8.0/reference/generated/scipy.spatial.cKDTree.html). 생성 옵션은 데이터 분포에 따라 검색 성능이 달라질 수 있어 위 실물 점군의 생성·조회 비용을 함께 비교했다.

적용 후 약 6.8초 관측에서 `/scan` 51개에 남은 고정 URDF 내부 유효점은 모두 0개였다. 마지막 필터 상태는 입력 530빔 중 URDF 내부 56빔, 기존 각도 마스크에 해당하는 유효 98빔을 제외하고 290빔을 유지했으며, 두 제외 집합은 겹친다. 해당 G2 콜백 계산은 약 0.84 ms였다. `/scan_slam`을 쓰는 기존 저장 지도의 위치추정을 복원했다. Nav2는 자동으로 시작하지 않았으며 이후 수신된 속도 명령을 그대로 보호 처리했다.

같은 적용 후 관측에서 3D 보호 사유는 `clear_path` 26개, `stale_mid360` 4개, `computation_timeout` 1개였다. 마지막 scene은 46,155점, KD-tree 생성 약 9.24 ms, 보호 검사 약 24.75 ms였다. 프로세스 CPU는 한 코어 기준 약 83%였으나 적용 전은 다른 장면 크기와 정지 상태였으므로 CPU 수치를 직접 개선율로 비교하지 않는다. G2 입력 결함 수정과 오프라인 연산 비용 감소는 확인됐지만 실제 이동 중에는 센서 지연·계산 예산 초과가 추가로 정지를 만들 수 있다. 반복 보호 계산의 C++ 이식이 다음 경량화 우선순위다.

### 3D 속도 guard의 C++ 전환

반복 거리 검사·이동/제동 예측과 scene 파싱·KD-tree 생성을 `collision_processing/src/collision_guard.cpp` 및 `guard_core.hpp`로 옮겼다. 전처리와 guard는 같은 `robot_geometry.hpp`를 사용하며, 기존 실물 보정·URDF 내보내기에서 고정 몸체·팔·Tracer 형상을 받아 별도 형상 정의를 유지하지 않는다. Python `CollisionGuard` 클래스는 오프라인 비교 기준으로 남아 있고 실물 launch와 스크립트 직접 실행은 C++로 연결된다.

거리 계산은 nanoflann의 정확한 반경 검색(`eps=0`)으로 후보를 고른 뒤 각 OBB까지의 유클리드 거리를 계산한다. L2 반경을 제곱 값으로 전달하고, 라이브러리의 엄격한 경계 비교를 보정한다. 점군이나 형상, 속도 후보, 예측 시간 간격을 줄이지 않는다. URDF 외곽 여유 10 cm, 불확실성 5 cm, voxel/이동 샘플 여유, 현재 속도 반응 구간과 제동 예측은 그대로 유지한다.

수신 콜백과 제어 콜백은 다른 callback group을 사용한다. 최신 대기 scene 하나를 별도 작업자가 파싱하고 인덱싱하므로 새 EKF/팔 피드백이 scene 생성 뒤에 밀리지 않는다. 승인 계산은 불변 scene을 참조하고, 완료 직전 원래 관측의 나이·현재 센서 상태·세대·명령 변경·차체와 팔 이동을 다시 검사한다. 계산 중 명령이 바뀌면 이전 명령을 승인하지 않고 해당 주기에 0을 보낸다. 40 ms 예산 및 별도 Python watchdog의 150 ms 승인 만료는 유지한다.

실물 제어 로직 변경에 필요한 비교 범위로 84개 오프라인 조건을 검사했다. 빈 공간·거리별 벽·무작위 점군·저장된 실물 점군, 전후진·회전·측정 속도·관측 지연·명령 만료·잘못된 명령에 대해 Python과 C++의 정지 사유/상태/감속 비율이 같았고, 최근접 거리 허용 오차는 `1e-10`이었다. 비교 조건 전체의 계산 시간 중앙값은 Python 0.845 ms, C++ 0.030 ms, C++ 95백분위는 1.039 ms였다. 빈 공간과 즉시 정지 조건도 포함하므로 이 비율을 주행 CPU 개선율로 해석하지 않는다.

입력 실패 검사는 실제 로봇과 분리한 `ROS_DOMAIN_ID=179`, `/guard_cpp_test/*`에서 수행했으며 Tracer 드라이버나 실제 watchdog은 시작하지 않았다. 점군 지연·점군/상태 시각 불일치·세대 변경·오도메트리 지연·팔 피드백 지연·TF 불일치·팔 이동·잘못된 명령·명령 만료·전처리 오류 상태·NaN scene의 11가지에서 출력 0과 입력 복구 후 승인 재개를 확인했다. 초기 승인과 각 복구를 포함해 23개 단계이며 실제 로봇에 이동 명령을 보내지 않았다.

근거: 설치 버전 [nanoflann 1.4.2](https://github.com/jlblancoc/nanoflann/tree/v1.4.2)의 정확한 KD-tree 조회와 L2 제곱 거리 규약, [ROS 2 Humble callback groups](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html)의 서로 다른 그룹 간 병렬 실행 및 같은 그룹의 상호 배제 규칙.

#### C++ guard 실물 적용 관측

관제센터와 관리되는 센서 core를 재시작해 C++ guard를 적용했다. 적용 직전 사용자가 선택해 둔 `map_20261014_1927`의 SLAM Toolbox 자동 위치추정을 복원했으며, Nav2는 기존처럼 꺼 두고 주행 목표를 복원하지 않았다.

정지 상태 약 9.3초의 관측에서 보호 상태 46개 모두 `ready=true`, `implementation=cpp`였고 사유는 `command_timeout`이었다. 이는 이동 명령이 없는 정상 대기 상태다. 승인 출력 181개는 모두 0이었다. `/collision/cmd_vel_approved`의 발행자는 `collision_guard` 하나, `/tracer/cmd_vel_safe`의 발행자는 별도 `tracer_safety_watchdog` 하나였으며 watchdog도 준비 상태였다.

| 항목 | 중앙값 | 95백분위 | 최댓값 |
| --- | ---: | ---: | ---: |
| guard 계산 | 0.316 ms | 0.420 ms | 0.711 ms |
| scene 파싱 | 0.521 ms | 1.131 ms | 1.362 ms |
| scene KD-tree 생성 | 7.667 ms | 11.345 ms | 12.477 ms |

해당 구간 guard 프로세스 CPU는 한 코어 기준 약 22.4%였다. 마지막 scene은 29,493점이며 입력 대기 교체는 0이었다. 직전 Python 관측과는 scene 크기·주행 명령 유무가 다르므로 CPU 개선율을 직접 산출하지 않는다. 이 관측은 정지 상태의 실행 연결과 계산 비용에 대한 결과이며 실제 Nav2 주행 성공을 확인한 것은 아니다.
