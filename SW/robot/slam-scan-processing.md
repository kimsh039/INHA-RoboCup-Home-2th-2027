**실물 SLAM 스캔 처리**

G2의 현재 기본 경로는 다음과 같다.

```text
G2 driver -- /scan_raw ----------------> calibrated_streams.py -- /scan --> 관제 / Nav2
          -- /g2/points_raw (stamps) --+                             |
                                     +--> slam_scan.py <------------+
/odom_ekf ----------------------------+       |
측정된 base_link <- laser_frame TF -----------+--> /scan_slam --> SLAM Toolbox
```

`/scan`은 매 스캔마다 발행한다. 거리 배율을 적용하고, 0/비유한 값/센서 범위 밖/가림 구간은 NaN으로 표현한다. 장애물 인식용 가까운 유효 측정은 보존한다. NaN은 관측 불가능한 빔이며 자유 공간 제거를 위한 Inf로 바꾸지 않는다.

SLAM 입력에는 [config/slam_scan.yaml](config/slam_scan.yaml)의 별도 범위 0.35–8 m를 적용한다. `/scan_slam`은 가림 제거와 빔별 운동 보상을 거친 공통 시각의 스캔이다. 스캔 길이/센서 프레임은 유지하고, 이동 후 각도 bin이 중복되면 가장 가까운 반환을 사용한다. 비어 있는 bin은 NaN으로 남긴다. 원래 bin의 intensity는 새 bin과 대응하지 않으므로 SLAM 출력 intensity 배열은 비운다.

**취득 시각과 운동 보상**

G2 드라이버는 취득 순서의 `sensor_msgs/PointCloud`에 `stamps` 채널을 발행한다. 배열의 각도 정렬, reversion/inverted, 원본의 유효하지 않은 측정으로 인덱스와 취득 순서가 달라질 수 있으므로 `LaserScan` 배열 위치에서 빔 시간을 재구성하지 않는다. `/scan`과 점군은 동일한 header timestamp로 짝지으며, 점군 채널의 상대 시각을 사용한다. SDK가 고정 배열을 채우기 위해 덧붙인 마지막 점 중 한 바퀴 취득 기간 밖에 있는 점은 사용하지 않는다.

설치된 G2 SDK가 이 상대 시간을 스캔 주기와 원본 샘플 수로 추정한다는 한계는 남는다. 이는 하드웨어에서 측정한 독립적인 빔 timestamp와 같은 정확도를 보장하지 않는다. 임의의 100 ms 지연 보정이나 각도 인덱스 기반 시간 보정을 추가하지 않았다.

기준 시각은 G2의 경우 원본 스캔 취득 기간의 중앙, MID-360의 경우 첫/마지막 점의 중앙이다. 각 점에는 다음 변환을 적용한다.

```text
p_laser(ref) = inverse(T_odom_base(ref) * T_base_laser)
               * T_odom_base(t_i) * T_base_laser * p_laser(t_i)
```

`T_odom_base(t_i)`는 `/odom_ekf`에서 수신한 x/y/yaw를 시간 보간해 얻는다. yaw는 ±π 경계를 풀어서 보간한다. 센서 extrinsic은 메시지 시각의 TF에서 얻으며, 실제 3D 장착 회전을 사용한다. 몸체 운동은 기존 EKF의 평면 운동 모델을 따른다. 이 노드는 odometry나 TF를 발행하지 않으며 `map → odom`을 운동 보상에 사용하지 않는다.

필요한 EKF 구간이 아직 도착하지 않으면 최대 0.4초 기다린다. 80 ms 이상 odometry 공백, 시각/센서 프레임 불일치, 사용할 수 없는 빔 시각은 거절하고 원인을 status와 로그에 남긴다. 최신 TF/최신 속도로 대체하거나 시간 범위 밖으로 외삽하지 않는다. 원본 `/scan`과 Nav2 입력은 이 대기 조건의 영향을 받지 않는다.

보상한 스캔의 header는 기준 시각이며 `time_increment=0`이다. 모든 출력 빔이 그 시각으로 이동됐다는 의미다. 이후 발행 스캔의 `scan_time`은 실제 채택 스캔 간격을 기록한다.

**정지·회전·채택 조건**

시작 시에는 `initial_slam_scans: 3`개의 유효 스캔이 실제 SLAM에서 처리될 때까지 최소 0.2초 간격으로 초기 관측을 보낸다. `/pose`의 시각이 발행 스캔과 일치하는 응답만 세며 중복 응답은 세지 않는다. 이는 `min_pass_through: 2`의 셀 관측 횟수를 초기화하는 제한적인 관측이며, 계속 정지 스캔을 넣는 설정은 아니다. 설치된 2.6.10의 외부 채택 함수는 첫 스캔 이후 초기 몇 스캔을 건너뛰므로 발행 횟수와 실제 처리 횟수가 다를 수 있다. 초기화를 마친 뒤에는 마지막 발행 스캔 대비 최소 0.2초가 지나고, 4 cm 이동 **또는** 0.02 rad 회전한 스캔을 채택한다. 제자리 회전은 처리하고 정지 중 반복 제약은 억제한다. 강제 정지 갱신은 기본값 0으로 꺼져 있으며 `stationary_refresh_interval`을 양수로 지정하면 그 간격으로 정지 스캔을 허용할 수 있다. 유효 반환은 최소 40개를 요구한다. 이 값들은 운용 초기값이며 현장의 최적값으로 검증된 값은 아니다.

SLAM이 꺼져 있으면 발행 기준 자세를 소비하지 않는다. 새 SLAM 구독자가 연결되면 DDS 연결과 TF 이력을 위해 0.5초 기다린 뒤 초기 관측을 다시 시작한다. 따라서 센서 코어를 유지한 채 SLAM을 시작/초기화해도 정지 필터 때문에 지도가 비어 있지 않도록 처리한다. 이미 SLAM이 연결된 상태에서 진단 구독자만 추가하면 채택 기준을 초기화하지 않는다.

위치추정 중 `/initialpose`를 받으면 같은 제한된 초기 관측을 다시 시작한다. backend가 요청 위치를 적용하는 데 다음 스캔이 필요하므로, 로봇이 정지해도 초기 위치·방향 지정이 실제 정합으로 이어지게 한다. `/initialpose`와 이 노드의 출력 스캔 양쪽에 실제 `slam_toolbox` 구독자가 있을 때만 실행하므로 지도 작성 중에는 정지 제약을 추가하지 않는다. 초기 응답을 모두 받으면 다시 정지 필터로 돌아간다. `last_initial_pose_stamp_ns`로 마지막 처리 요청 시각을 확인할 수 있다.

SLAM Toolbox 자체 위치추정은 초기 자세 주변의 정합을 사용한다. 관제의 `slam-global-initializer.py`가 점유 지도 전체에서 현재 스캔의 위치·방향 후보를 비교해 이 초기 자세를 자동 계산한다. 후보 점수와 경쟁 후보 간 차이가 충분한 경우에만 `/initialpose`로 전달하며, 실제 SLAM Toolbox 응답을 확인한다. 수동 초기 위치 지정은 사용자 흐름에서 제거했다. backend의 원점/`map_start_pose`는 pose graph 로딩용 기본값이며 전역 탐색의 prior로 사용하지 않는다. AMCL을 실행하지 않는다. [자동 위치추정 안내](../control_center/automatic-localization.md).

전역 탐색 입력 `/scan_slam_continuous`와 `/scan_mid360_slam_continuous`는 매 프레임 운동 보상 스캔을 제공하며, 해당 구독자가 있을 때만 발행한다. `/scan_slam`의 이동·회전 조건과 별개이므로 전역 탐색기가 정지 상태의 신선한 관측을 읽어도 SLAM 정합 제약이 추가되지는 않는다.

Humble SLAM Toolbox의 외부 채택 함수에는 독립적인 회전 OR 조건이 없다. 따라서 backend의 `minimum_travel_distance`와 `minimum_time_interval`은 0으로 두고, 준비된 스캔의 채택은 전처리 노드 한 곳에서 담당한다. `minimum_travel_heading`을 바꾸는 것만으로 외부 translation gate 문제를 해결할 수는 없다.

설치된 SLAM Toolbox 2.6.10에는 `restamp_tf` 파라미터가 없으며 마지막 스캔 시각으로 TF를 반복 발행한다. `slam-runtime.launch.py`는 backend의 broadcaster에만 노드 범위 remap을 적용해 `/slam_toolbox/tf_raw`로 보낸다. backend의 별도 TF listener는 기존 `/tf`를 계속 수신한다. `slam-tf-clock.py`가 새로 수신한 `map → odom` 변환의 위치·회전을 그대로 유지하고 시각만 현재 시각 + `transform_timeout`으로 바꿔 공개 `/tf`에 전달한다. 공개 `map → odom` 발행자는 이 전달 노드 하나다. backend가 발행을 멈추면 전달도 멈추며, 캐시를 타이머로 무기한 재발행하지 않는다. 정지 중 새 정합 제약을 넣지 않아도 TF 유효 시각이 진행하게 하는 처리다. 센서 timestamp와 보상 기준 시각은 변경하지 않는다. 최신 Humble 브랜치의 지원 파라미터를 설치된 2.6.10에 그대로 적용하지 않는다.

**소유권과 실행 경로**

- `physical.launch.py`의 calibration 그룹이 G2 전처리 노드를 한 번 실행한다. 관제의 sensor_core도 같은 launch를 사용한다. G2의 mapping/localization 스크립트는 이 노드를 다시 시작하지 않는다.
- 통합 launch와 관제의 개별 G2 드라이버 모두 `point_cloud`를 `/g2/points_raw`로 remap한다.
- `map → odom`의 자세 계산은 SLAM/위치추정이, 공개 TF 전달은 `slam_tf_clock`이 단독으로 담당한다. `odom → base_link`는 EKF가 계속 담당한다. 휠 TF는 기존 `/wheel/tf` 경로를 유지한다.
- mapping과 localization은 같은 전처리 스캔을 사용한다. `prepare-slam-profile.py`가 기본/사용자 backend YAML의 scan_topic, 최소/최대 범위, 외부 채택 조건을 선택된 전처리 노드와 맞춘다. 사용자 solver/graph 설정은 보존한다. 입력 범위와 채택 조건의 기준 파일은 `config/slam_scan.yaml`이며 두 위치의 값을 서로 따로 조정하지 않는다.

**MID-360 선택 기능**

`slam_scan: mid360` 또는 `ROBOCUP_SLAM_SCAN=mid360`을 선택하면 SLAM 프로세스 그룹에 `mid360_slam_scan`이 시작된다. `/livox/lidar`의 개별 `timestamp`를 사용해 먼저 운동 보상하고, 기존 높이 대역(바닥 위 0.25–1.20 m)과 거리 0.55–15 m로 투영한 `/scan_mid360_slam`을 SLAM에 연결한다. 기존 보상 전 진단 토픽 `/scan_mid360`도 유지한다.

현재 설치된 Livox 드라이버의 FLOAT64 timestamp는 절대 nanoseconds이다. 소스의 `offset_time`이라는 이름과 달리 패킷 시각을 이미 더한 값이다. config의 `absolute_ns`는 이 구현에 맞춘 선택이다. 다른 드라이버로 교체할 때는 필드 이름과 absolute_ns/relative_ns/relative_sec를 명시적으로 지정해야 하며, 노드가 단위를 자동 추정하지 않는다.

MID-360을 사용해도 Nav2의 G2 및 `/mid360/points_filtered` 장애물 경로는 유지한다. MID-360의 높이가 다른 구조물을 평면에 투영한다는 한계는 남으며, 이것을 완전한 3D SLAM으로 설명하지 않는다. 기존 지도에서 사용한 센서와 다른 센서로 위치추정하지 않는다. 예를 들어 기존 `map_20261014_1614`는 MID-360 입력 지도이고 `1540`, `1645`는 G2 입력 지도다.

**IMU 초기화와 운전 조건**

IMU relay의 초기 bias는 신선한 휠 odometry를 동반한 400개의 연속 정지 샘플로 계산한다. 정지가 끊기거나 odometry가 오래되면 미완성 샘플을 비운다. 이후 bias의 정지 중 갱신도 같은 신선도 조건을 사용한다. 기존 IMU/휠 fallback 및 EKF 회전 입력의 소유권은 유지한다.

운동 보상이 바퀴 미끄러짐이나 잘못된 센서 시간을 교정하지는 않는다. 기존 Tracer 거리 보정의 좋은 성능은 주로 0.2 m/s 이하의 조건이다. 지도 작성 때 선속도 약 0.1–0.2 m/s와 완만한 가속을 우선 사용하고, 수동/RC 운전이 Nav2의 속도 제한을 자동으로 따르지 않는다는 점을 고려한다. 이 스캔 처리 노드는 `/cmd_vel`이나 RC 명령을 변경하지 않는다.

가림 구간은 실제 환경 정보가 없으므로 보간으로 복원하지 않는다. 약 120.4°의 가림과 평행 벽의 정합 퇴화는 장착 개선/추가 기하 관측으로 해결해야 할 부분이다. 이를 이유로 loop closing을 영구적으로 끄거나 알려지지 않은 장애물을 임의로 삭제하지 않았다.

**연구 근거와 적용 범위**

1. [LIO-SAM, IROS 2020](https://arxiv.org/abs/2007.00258): 스캔 중 센서 운동을 보상하고 초기 정합 자세에 활용하는 원리를 참고했다. [저자 구현](https://github.com/TixiaoShan/LIO-SAM#prepare-lidar-data)도 deskew에 각 점의 상대 시각이 필요함을 명시한다.
2. [Direct LiDAR-Inertial Odometry, ICRA 2023](https://arxiv.org/abs/2203.03749): 공통 시각으로 이동할 때 점마다 다른 자세가 필요하다는 근거다. 이번 구현은 기존 EKF의 평면 자세 보간이며 논문의 고차 운동 observer 전체를 구현한 것은 아니다. 급격한 운동/평면 밖 운동의 보상 정확도를 논문의 실험 결과와 동일하게 주장하지 않는다.
3. [Cartographer, ICRA 2016](https://research.google/pubs/real-time-loop-closure-in-2d-lidar-slam/)와 [motion filter 구현](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/motion_filter.cc): 중복 관측을 줄이는 시간/거리/각도 조건을 참고했다. 이 로봇의 기본값은 정지 중 강제 삽입을 하지 않는 별도 선택이다.
4. [2DLIW-SLAM, 2024](https://arxiv.org/abs/2404.07644): 단순한 실내 기하에서 LiDAR의 관측만으로 부족한 운동 방향이 있을 수 있다는 근거다. 기존 IMU+휠 EKF를 보존하며, 논문의 tightly coupled optimizer를 새로 도입하지 않았다.
5. [ROS LaserScan 정의](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg)와 [SLAM Toolbox 2.6.10 Karto 구현](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/lib/karto_sdk/include/karto_sdk/Karto.h): 첫 빔 시각, 빔 간 시간과 NaN 무효값 처리의 기준이다.
6. [설치 버전의 채택·TF 구현](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/src/slam_toolbox_common.cpp)과 [ROS 2 노드 범위 remap 정의](https://design.ros2.org/articles/static_remapping.html): 초기 스캔 처리 응답 및 broadcaster만의 remap 기준이다.

**운용과 이번 수정의 한계**

일반적인 physical/관제 시작 경로에 위 처리를 연결했다. 사용자의 실행 확인 요청에 따라 관리 중인 관제·센서 프로세스를 정상 종료 후 재기동하고 정지 상태의 새 매핑에 적용했다. 기존 저장 지도 파일은 유지했다. 새로운 colcon 빌드는 필요하지 않다.

`/scan_slam/status` 및 `/scan_mid360_slam/status`에는 발행/정지 필터/거절 횟수, 대기 입력 수와 이유가 JSON으로 발행된다. 기존 rosbag을 이후 분석에 사용할 때는 `/g2/points_raw`, `/odom_ekf`, `/tf_static`, `/scan_raw`, `/scan`과 SLAM pose/map을 함께 기록해야 빔별 보상과 지도 배치를 연결할 수 있다. 과거 `odom_wall_1708`에는 G2의 timed cloud가 없으므로 그 기록으로 완전한 deskew를 재현할 수 없다. 명시적으로 `deskew_enabled=false`를 설정하면 그 경우에도 NaN 정리와 이동/회전 선택은 적용할 수 있다.

운동 보상 기하, yaw 경계, odometry 공백, 초기 처리 응답과 TF 전달은 오프라인 테스트로 확인했다. 실시간 G2/MID-360 입력 및 공개 TF 발행 상태도 확인했다. 로봇에 이동 명령을 보내지 않았으므로 실제 회전·왕복 주행의 지도 중첩 개선률은 아직 측정하지 않았다. 이전에 저장된 지도/캘리브레이션 파일은 이번 수정으로 복구되거나 재계산되지 않는다.
