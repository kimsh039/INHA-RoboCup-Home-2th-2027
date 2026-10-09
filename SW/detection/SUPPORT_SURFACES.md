# Mid-360 작업면 추출 (테이블·선반 윗면)

Mid-360 점군에서 **수평 작업면의 높이·사각형 윤곽·가장자리**를 찾아 `map` 좌표로 발행합니다. [센서 역할](SENSOR_ROLES.md)의 구현 2순위 중 "작업면 기하 추출" 단계입니다. 접근 자세 생성과 closed approach는 이 출력을 사용하는 후속 작업입니다.

## 실행

Gazebo·SLAM이 실행 중일 때(`map` TF 필요), 패키지를 빌드한 뒤:

```bash
cd SW/detection/head/head_detection_ws
colcon build --packages-select robocup_detection_msgs robocup_head_detection --symlink-install
source install/setup.bash
ros2 run robocup_head_detection support_surface_node --ros-args -p use_sim_time:=true
```

RViz에 `MarkerArray` `/detection/support_surfaces/markers`를 추가합니다. 작업면마다 사각형과 `#번호 h=높이 크기` 라벨이 보입니다.

| 선 색 | 의미 |
|---|---|
| 초록 | 확인된 경계 (윗면 점이 변까지 오고, 변 바로 바깥을 윗면 높이에서 지나간 광선이 더 낮은 곳에 맞음) |
| 회색 | 미확인 (시야 끝·가림·윗면 자신의 그림자. 실제 가장자리가 더 바깥일 수 있음) |

## 동작

1. `/mid360/points_filtered`를 점군 시각의 TF로 `map`에 변환해 최근 2초를 누적합니다(이동 중에도 일관됨). 점군마다 센서(`livox_frame`) 위치도 함께 기록합니다.
2. 5 cm 칸 안에서 반사가 0.15 m 넘는 높이에 걸쳐 쌓인 곳(벽·기둥)은 작업면 후보에서 뺍니다. 테이블 끝 0.3 m 옆 벽에 찍힌 스캔 선이 테이블과 합쳐지던 문제(1.6 m 테이블 → 2.0 m)를 막습니다.
3. 높이 히스토그램 봉우리마다 같은 높이 점을 모으고, 15 cm 격자로 연결된 덩어리를 찾습니다. 짧은 변이 0.25 m보다 가는 덩어리는 버립니다.
4. 최소 면적 사각형을 맞추고 각 변의 `edge_observed`를 계산합니다. **윗면이 변까지 온 비율**과 **변 바로 바깥(0.02~0.4 m)을 윗면 높이에서 지나간 광선이 더 낮은 곳에 맞은 비율** 중 작은 값입니다. 광선 기준이라 가까이에서(변 바깥 바닥이 Mid-360 시야 밖일 때) 테이블 밑 바닥에 맞은 광선으로도 확인되고, 시야 끝·가림·그림자 쪽 변은 그런 광선이 없어 미확인으로 남습니다. 두 비율을 따로 보는 이유는 로봇 팔이 변 앞 바닥 일부를 가려도 윗면이 다 보이면 확인되게 하기 위해서입니다.
5. 주기마다 같은 높이·가까운 위치의 작업면에 같은 `id`를 유지합니다(5초간 못 봐도 유지).

코드: [`surface_geometry.py`](head/head_detection_ws/src/robocup_head_detection/robocup_head_detection/surface_geometry.py)(ROS 없는 계산), [`support_surface_node.py`](head/head_detection_ws/src/robocup_head_detection/robocup_head_detection/support_surface_node.py), 테스트 [`test_surface_geometry.py`](head/tests/test_surface_geometry.py).

## 출력

`/detection/support_surfaces` (`robocup_detection_msgs/SupportSurfaceArray`, 2 Hz)

| 필드 | 의미 |
|---|---|
| `id` | 작업면 번호 (시야에 있는 동안 유지) |
| `height`, `z` | 바닥 기준 높이 / `map`의 z |
| `centre` (`Pose2D`), `length`, `width` | 사각형 중심, 긴 변 방향(`theta`), 크기. 관측된 범위이므로 실제보다 작을 수 있음 |
| `corners[4]` | 반시계 방향. 변 i = `corners[i] → corners[i+1]`, 바깥 법선은 진행 방향의 오른쪽 |
| `edge_observed[4]` | 변 i가 확인된 경계인 정도 (0~1): 윗면 도달 비율과 광선 통과 비율 중 작은 값 |
| `inliers`, `residual` | 점 수, 높이 표준편차 |

## 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `input_topic` | `/mid360/points_filtered` | 자기 점이 제거된 점군 |
| `frame` | `map` | 출력 좌표계 (중력 방향 정렬 필요) |
| `floor_z` | `-0.1425` | `frame`에서 바닥의 z (아래 참고) |
| `sensor_frame` | `livox_frame` | 광선 원점(센서) 프레임 |
| `window_sec` | 2.0 | 누적 시간 |
| `min_height`, `max_height` | 0.3, 1.3 | 찾을 작업면 높이 범위 (바닥 기준, m) |
| `min_side` | 0.25 | 작업면 짧은 변 최솟값 |

## 주의

- **높이 보정 `floor_z`:** URDF에 바닥 기준 `base_footprint`가 없습니다. `base_link`는 바닥 위 0.1425 m(바퀴 축 -0.082 m + 반지름 0.0605 m)인데 `odom`/`map`에서는 z=0에 놓이므로, `map`의 모든 높이가 0.1425 m 낮습니다. 이 노드는 `floor_z`로 보정합니다. `base_footprint`가 추가되면 `floor_z: 0.0`으로 바꿉니다.
- **시뮬레이터 Mid-360:** 20줄 격자에서는 테이블 윗면에 선이 2~3개만 찍혀 추정이 어렵습니다. `sim/mid360-dense-scan`(60줄)과 함께 사용합니다.
- 수평면만 찾습니다(기울어진 면·계단 미지원). 작은 물체는 Mid-360으로 찾지 않습니다. 물체 위치는 헤드 D435 검출을 사용합니다.

## 확인 결과 (Gazebo 방, 2026-10-07, 당시 테이블 2개 world)

로봇 (-0.6, 0)에서 두 테이블(윗면 0.72 m, 1.6×0.8 m)을 관측:

| | 높이 | 통로 쪽 긴 변 y (실제) | 로봇 쪽 짧은 변 x (실제) | 먼 쪽 짧은 변 |
|---|---|---|---|---|
| table1 | 0.707 m | -0.59 (-0.60), 확인 0.50 | 0.99 (1.00), 확인 1.00 | 2.25 (2.60), 확인 0.25 → 미확인 |
| table2 | 0.706 m | 1.41 (1.40), 확인 0.47 | 1.00 (1.00), 확인 0.88 | 2.48 (2.60), 확인 0.12 → 미확인 |

남은 1.4 cm 높이 차이는 Gazebo에서 차체가 약 0.25° 기울어진 영향으로 보입니다(TF는 2D). 처리 시간은 2초 누적(약 12만 점) 기준 약 40 ms입니다.
