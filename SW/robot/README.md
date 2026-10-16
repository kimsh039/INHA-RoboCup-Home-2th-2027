> 현재 Wrist sensor_tf 입력은 [최신 55자세 ChArUco JSON](../../HW/calibration/physical/wrist_handeye/README.md)이다. 아래 과거 브랜치/15자세 출처는 이전 기록이며 현재 입력과 구분한다.

# 실물 캘리브레이션을 적용한 Jetson 로봇 실행

Tracer 주행은 [추가 이격 없이 URDF 몸체 형상을 사용하는 3D 충돌 보호](collision-avoidance.md)를 거칩니다. `body_only: true`, `clearance: 0.0`, `uncertainty_margin: 0.0`으로 원래 측정점과 로봇 형상의 겹침을 검사합니다. 평탄한 실내 바닥을 전제로 바닥 위 5 cm까지의 반사점은 제외하고, 그 위 의자·책상 다리 등의 장애물을 처리합니다. 실물 core는 점군 전처리·속도 guard·별도 watchdog을 실행하고, Tracer는 `/tracer/cmd_vel_safe`를 구독합니다. Nav2는 같은 장애물 scene을 경로 계획에 반영합니다.

2026-10-12 기준, 이 젯슨(`/home/sparo`)에서 **Git에 올라온 실물 측정값으로 Tracer·G2·Mid-360S·Head·Wrist와 팔 피드백을 실행**합니다. 실물 기록이 다른 브랜치나 `SW/simulation` 경로에 있어도 측정 내용을 기준으로 선택했습니다. Gazebo/MuJoCo 센서 보정과 시뮬레이션 TCP 보정은 실행 설정에 사용하지 않습니다.

## 바로 실행

MID-360 전처리와 3D 주행 보호 계산은 C++ 실행 파일을 사용합니다. 최초 실행 또는 해당 소스 변경 후 두 실행 파일과 Nav2의 공유 장애물 플러그인을 함께 생성합니다. 빌드에는 ROS 2 Humble 개발 환경, nav2_costmap_2d, pluginlib, Eigen3, yaml-cpp, nanoflann, nlohmann-json 헤더가 필요하며 이 Jetson에는 설치돼 있습니다.

```bash
bash ~/robocup/SW/robot/build-collision.sh
```

기존 개별 센서 실행기를 쓰고 있다면 먼저 종료합니다. 이 젯슨에서는 이전 실행기를 아래 이름으로 보존했습니다.

```bash
bash ~/livox_g2_calib/sensors.legacy-20261012.sh stop
```

이전에 따로 띄운 `imurelay`, `ekf`, `fusion`, `slam` 프로세스도 해당 터미널에서 Ctrl+C로 종료하세요. `/odom_ekf`, `/imu/base`, 같은 카메라·LiDAR 토픽과 TF를 발행하는 이전 프로세스가 남아 있으면 새 실행과 중복됩니다.

실물 보정값을 적용한 전체 실행:

```bash
cd ~/robocup
bash SW/robot/start_robot.sh
```

이 터미널을 유지하고 종료할 때 **Ctrl+C**를 누릅니다. 시작 후 처음 **5초 이상 로봇을 정지**시켜 Mid360 자이로 bias를 초기화하세요. 실행기는 센서·오도메트리·팔 관절 피드백과 차체 보호를 시작합니다. 목표나 팔 이동 명령을 자동 생성하지 않으며, 보호 출력에는 정지 명령을 지속적으로 보냅니다. PiPER는 수신 전용 `piper_feedback.py`를 사용하며 자동 모터 활성화를 하지 않습니다. 부팅 중 무응답으로 종료하지 않고, 모든 관절의 실제 응답이 돌아올 때 재개합니다.

백그라운드 tmux로 실행하려면:

```bash
bash ~/robocup/SW/robot/jetson_sensors.sh start
```

화면 접속과 정상 종료:

```bash
tmux attach -t robot_physical
# 화면에서 빠져나오면서 실행 유지: Ctrl+B, D
bash ~/robocup/SW/robot/jetson_sensors.sh stop
```

로컬 `~/livox_g2_calib/sensors.sh start|stop`도 이제 이 실행기로 연결됩니다. 이미 실행된 `robot_physical` 세션에 start를 다시 보내면 설정을 재적용하지 않습니다. 파일을 바꾼 뒤에는 stop → start로 재시작합니다.

CAN 포트 변경 또는 팔 피드백 없이 실행:

```bash
bash ~/robocup/SW/robot/start_robot.sh tracer_can:=can0 piper_can:=can_arm
bash ~/robocup/SW/robot/start_robot.sh arm:=false
```

`arm:=false`에서는 Tracer와 센서·EKF는 실행하고 팔 관절 피드백/팔 TF를 시작하지 않습니다. 따라서 손목 카메라 영상을 받아도 몸체까지 이어지는 손목 TF는 완성되지 않습니다.

## 어떤 실물 보정이 적용되는가

| 항목 | 선택한 실물 결과 | 실제 적용 위치 |
|---|---|---|
| Base–Mid360 | 2026-10-12, 헤드 D435 연결 후 구성. xyz ≈ (−156.473, 5.404, 1187.255) mm, RPY ≈ (179.767°, 1.130°, −1.926°) | `base_link → livox_frame` TF 및 IMU 회전 행렬 |
| G2–Mid360 | 같은 세션의 6자유도 정합. Base 기준 G2 xyz ≈ (5.029, 7.795, 344.269) mm, RPY ≈ (−1.279°, −0.953°, 0.881°) | `livox_frame → laser_frame` TF |
| G2 거리 배율 | 1.0070145819155318 | `/scan_raw`의 유효 거리·최소/최대 거리에 한 번 곱하여 `/scan`으로 발행 |
| Head–Mid360 | 2026-10-12 koide3 4개 bag 최적화, 사용자 수락 결과 | `livox_frame → camera_link`, 실제 RGB optical 프레임까지 드라이버 내부 TF로 연결 |
| Wrist hand-eye | 최신 실물 ChArUco 55자세. Link6←RGB optical xyz = (−64.927, 31.071, 19.479) mm | `piper_link6 → wrist_link`, 실제 RGB optical 프레임까지 드라이버 내부 TF로 연결 |
| Head RGB 내부 보정 | D435 `244622071116`, RGB 640×480, 저장된 K/D/R/P | `/head/calibrated/color/camera_info` |
| Wrist RGB 내부 보정 | D435IF `243222070076`, RGB 640×480, 저장된 K/D/R/P | `/wrist/calibrated/color/camera_info` |
| Tracer 오도메트리 | 2026-10-11 v4 속도 보정 + Mid360 gyro bias 제거 | 드라이버 `/odom`에 속도 보정, `/odom_ekf`와 `odom → base_link`에 IMU yaw 융합 |

최신 G2·Mid360·오도메트리의 원본은 [`calib/g2-mid360-physical`의 b8be83d](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/tree/b8be83d9108b076b398a13e515e0998b6db2c79d/HW/calibration), 손목 hand-eye는 [`feature/wrist-handeye-calibration`의 0bac1e5](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/blob/0bac1e558075600d21c6b7273f26a6858266ea0d/SW/simulation/calibration/wrist_handeye/wrist_handeye_calibration.yaml)입니다. 이 실행에서 사용하는 원본 사본은 [HW/calibration/physical](../../HW/calibration/physical/README.md)에 모았습니다. 이전 main의 2026-10-10 G2 결과와 이번 최신 결과를 구분하여 보관합니다.

## 좌표계 연결

```text
odom                                  ← EKF가 몸체 이동을 발행
└── base_link
    ├── livox_frame                   ← 실물 Base–Mid360
    │   ├── laser_frame               ← 실물 G2–Mid360
    │   └── camera_link               ← 실물 Koide + Head 드라이버 내부 TF
    │       └── camera_color_frame
    │           └── camera_color_optical_frame
    └── piper_base_link               ← CAD 장착 기준, 별도 실측 보정 없음
        └── piper_link1 … piper_link6  ← 실제 관절 피드백 + CAD 기구학
            └── wrist_link           ← 실물 손목 hand-eye + 드라이버 내부 TF
                └── wrist_color_frame
                    └── wrist_color_optical_frame
```

이번 실행에서 **몸체↔Mid360도 실물 보정값**입니다. 몸체↔G2는 몸체↔Mid360과 G2↔Mid360을 연결하여 얻고, 몸체↔Head도 몸체↔Mid360과 Koide 결과를 연결하여 얻습니다. 이 경로에 시뮬레이션 센서 장착 보정은 들어가지 않습니다.

카메라 보정은 RGB optical 원점에 대한 값입니다. RealSense 루트는 다른 원점이므로 노드는 드라이버가 발행하는 `RGB optical ← camera root`를 읽어 장착 TF를 계산합니다. 두 RealSense의 `publish_tf:=true`를 유지합니다. 내부 TF가 도착하기 전에는 해당 카메라의 장착 TF를 발행하지 않습니다.

팔 장착은 CAD xyz=(−0.0195, 0, 0.80611) m를 사용합니다. 이것은 실물 팔 장착 보정이 아닙니다. 관절 모델은 CAD 형상이며 q1의 모델 원점 회전 +1.6 rad에 맞춰 실제 관절 피드백에서 q1−1.6 rad를 전달합니다. 나머지 관절은 실제 피드백을 전달합니다. GUI나 시뮬레이터의 관절값으로 대체하지 않습니다. 그리퍼 손가락 프레임은 이 6관절 피드백 연결의 대상에 포함하지 않습니다.

## 로봇 소프트웨어에서 사용할 토픽

| 용도 | 입력 |
|---|---|
| 2D 스캔 / 관제 / Nav2 | `/scan` — 거리 배율·가림 마스크가 적용된 full-rate LaserScan, `laser_frame`; 무효 빔은 NaN |
| G2 SLAM 입력 | `/scan_slam` — 빔별 운동 보상 후 이동·회전 조건으로 채택한 LaserScan |
| 자동 위치 탐색 관측 | `/scan_slam_continuous` — 같은 운동 보상 스캔을 매 프레임 제공; 정합 제약 추가와 독립 |
| G2 취득 순서 점군 | `/g2/points_raw` — 원본 PointCloud, `stamps` 채널의 빔별 상대 시각 |
| 원본 2D 스캔 | `/scan_raw` — 드라이버 출력, 재보정·기록용 |
| 3D 점군 | `/livox/lidar` — PointCloud2, `livox_frame` |
| 몸체 오도메트리 | `/odom_ekf` — 보정 바퀴 선속도 + bias 제거한 IMU yaw rate |
| 바퀴 오도메트리 | `/odom` — v4 속도 보정은 적용되지만 방향은 바퀴 추정 |
| 몸체 좌표 IMU | `/imu/base` — 실물 Mid360 회전 + gyro bias 제거 |
| Head RGB 실측 내부 보정 사용 | `/head/camera/color/image_raw` + `/head/calibrated/color/camera_info` |
| Wrist RGB 실측 내부 보정 사용 | `/wrist/wrist/color/image_raw` + `/wrist/calibrated/color/camera_info` |
| Head 드라이버 원본 | `/head/camera/color/image_raw` + `/head/camera/color/camera_info` |
| Wrist 드라이버 원본 | `/wrist/wrist/color/image_raw` + `/wrist/wrist/color/camera_info` |
| Head aligned depth | `/head/camera/aligned_depth_to_color/image_raw` + 해당 드라이버 CameraInfo |
| Wrist aligned depth | `/wrist/wrist/aligned_depth_to_color/image_raw` + 해당 드라이버 CameraInfo |
| 실제 팔 관절 피드백 | `/piper/joint_states_feedback` |
| 모델에 맞춘 팔 관절값 | `/robot/joint_states` |

내부 보정 노드는 드라이버 CameraInfo의 timestamp/frame_id를 보존하여 **실측 CameraInfo만** 발행합니다. RGB는 RealSense 원본 토픽을 직접 구독하며, 보정 이미지 중계 토픽은 발행하지 않습니다. 원본 영상의 투영·PnP에는 K/D를 쓰고, 왜곡 보정한 영상에는 해당 보정 영상의 P를 사용합니다. 해상도가 640×480이 아니면 실측 CameraInfo를 발행하지 않습니다.

장치 펌웨어나 드라이버의 factory CameraInfo를 덮어쓰지는 않습니다. RealSense aligned depth는 factory 내부 파라미터로 정렬된 결과이므로 그 드라이버의 CameraInfo와 함께 사용하세요. `/calibrated/color`의 실측 K/P를 factory aligned depth에 그대로 끼워 넣으면 픽셀 좌표계가 달라집니다.

Koide Head 결과는 당시 `/camera/camera/color/camera_info`의 fx=608.926941, fy=607.607239, D=0을 사용하여 계산했습니다. 실측 Head RGB 내부 보정은 fx=605.53244, fy=608.06854, D≠0입니다. 두 결과를 각각 적용하고 있으나 **실측 K/D로 Koide를 다시 최적화한 결과는 아닙니다**. 기존 Koide 입력을 재현할 때는 원본 드라이버 영상/CameraInfo 경로를 유지합니다.

Nav2의 스캔 입력은 `/scan`, G2 SLAM의 입력은 `/scan_slam`입니다. 오도메트리 입력은 `/odom_ekf`, 몸체 프레임은 `base_link`, odom 프레임은 `odom`입니다. calibration 그룹은 `slam_scan.py`도 실행하며, 거리 범위·시간·이동·회전 조건은 [config/slam_scan.yaml](config/slam_scan.yaml)을 사용합니다. 빔별 시각과 소유권은 [SLAM 스캔 처리 안내](slam-scan-processing.md)에 정리했습니다. `map → odom`은 SLAM/위치추정 노드가 담당합니다. 이 실행기는 SLAM·지도·Nav2·인지 모델을 자동으로 시작하지 않습니다.

## 오도메트리 적용 방식

실측 계수는 [config/tracer.yaml](config/tracer.yaml)에 생성됩니다.

```text
선속도: v = 1.0362 × v_reported + 0.00632 × sign(v_reported), |v_reported| > 0.0035
각속도: w = 0.9924 × w_reported + 0.0343 × sign(w_reported), |w_reported| > 0.02
직진 deadband: |v_reported| > 0.05일 때 |w_reported| ≤ 0.0186은 0
```

이 계수는 `/cmd_vel`을 확대하는 계수가 아니라 **펌웨어 보고값으로 오도메트리를 계산할 때** 쓰는 값입니다. 현재 젯슨의 수정된 `tracer_base` 드라이버를 사용합니다. 새 설치에서는 [원본 드라이버 패치와 적용 안내](../../HW/calibration/physical/tracer_odom/README.md)가 필요합니다.

드라이버가 발행하는 바퀴 TF는 `/wheel/tf`로 분리하고, EKF만 전역 `/tf`에 `odom → base_link`를 발행합니다. 따라서 바퀴 추정과 IMU 융합 추정이 같은 child를 동시에 소유하지 않습니다. 같은 로봇에 이전 전체 URDF 실행기, 별도 센서 static TF 또는 다른 odom TF 발행기를 함께 켜지 마세요. 기존 `/slam/tf` 실험 설정은 이 전역 TF 구성을 대신하지 않습니다.

기존 실물 주행 기록은 급가속·고속에서 미끄럼이 늘어난다고 보고했습니다. 그 기록은 선속도 0.2 m/s 이하에서의 결과를 따로 제시하고, 내비게이션 가속도를 약 0.5 m/s²로 제한하도록 안내합니다. 이 실행기는 속도 명령을 제한하거나 Nav2 파라미터를 바꾸지는 않습니다.

## 이 젯슨의 실행 조건

- Ubuntu 22.04 / ROS 2 Humble. 기존 `tracer_ws`, `ws_livox`, `piper_ros`의 install 환경을 source합니다.
- Tracer: `can0` (젯슨 내장 CAN + SN65HVD230 트랜시버, 500 kbps). PiPER: USB-CAN 어댑터 `can_arm` (1 Mbps). 두 인터페이스는 `~/livox_g2_calib/can_boot_setup.sh`(udev 이름 고정 + systemd-networkd)로 부팅·재연결 시 자동으로 올라옵니다.
- G2: `~/tracer_ws/src/ydlidar_ros2_driver/params/G2.yaml`의 기존 USB 설정을 사용합니다.
- Mid-360S: 기존 `MID360s_config.json`의 센서 IP `192.168.1.184`, host IP `192.168.1.5`와 네트워크 설정을 사용합니다. 점군에 드라이버 extrinsic을 추가하지 않습니다.
- Head D435 `244622071116`: RGB/depth 640×480, 30fps (USB 3). Wrist D435IF `243222070076`: RGB/depth 640×480, 30fps.
- 이 작업은 현재 설치된 ROS 패키지, numpy, PyYAML을 사용합니다. 새 colcon 빌드를 수행하지 않았습니다.
- 모든 로봇 노드와 소비 프로그램은 같은 `ROS_DOMAIN_ID`를 사용합니다. 별도 지정이 없으면 기본 0입니다. tmux 실행기는 시작한 셸의 domain 값을 전달합니다.

## 보정 결과 변경 후 재적용

실물 기록을 변경했다면 먼저 실행기를 종료하고 생성 설정을 갱신합니다.

```bash
bash ~/robocup/SW/robot/jetson_sensors.sh stop
source /opt/ros/humble/setup.bash
python3 ~/robocup/SW/robot/apply_physical.py
bash ~/robocup/SW/robot/jetson_sensors.sh start
```

[apply_physical.py](apply_physical.py)는 Tracer·IMU·EKF 설정, CAD 팔 모델, [physical_application.json](physical_application.json)의 사용 기록과 입력 해시를 생성합니다. 센서 TF·거리 배율·RGB 내부 보정은 실행 시 해당 실물 기록을 직접 읽습니다. 센서 결과 적용을 위해 `integrate_calibration.py`나 시뮬레이터를 실행할 필요는 없습니다.

## 적용 범위와 기록

별도 실물 보정이 없는 **Base–PiPER 장착, TCP, 팔 관절 영점/링크 오차, 그리퍼 영점**을 보정 완료로 간주하지 않습니다. Base–Mid360 실측 높이는 당시 CAD의 바닥 높이를 기준으로 계산한 기록이므로 그 기준에 대한 의존성이 남습니다. 손목 hand-eye 원본은 별도 5자세에서 최대 위치 편차 12.71 mm / 회전 편차 1.25°를 보고합니다. 이 값은 기존 측정 기록이며 이번 작업에서 다시 측정한 결과가 아닙니다.

이 작업에서는 기록된 실물 결과의 선택·적용 코드·실행 문서를 작성했습니다. **로봇 실행, 추가 테스트, 빌드, 린트, 사후 검증은 수행하지 않았습니다.**

TF와 팔 피드백 동작의 기반은 [RealSense 공식 프레임 안내](https://github.com/realsenseai/realsense-ros#ros2-vs-optical-coordination-systems), [robot_state_publisher 공식 설명](https://github.com/ros/robot_state_publisher)입니다.
