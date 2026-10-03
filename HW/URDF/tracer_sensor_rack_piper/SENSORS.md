# 통합 로봇 센서 시뮬레이션

2026-10-03 기준. Gazebo Harmonic 8의 제조사 사양 기반 **기하학적 근사** 모델입니다. 기존 CAD 링크·축·질량을 보존하며 Gazebo용 URDF에 센서 확장을 추가했습니다. 일반 URDF는 형상/TF 용도로 유지합니다.

## 제조사 자료와 적용 값

### Livox Mid-360S

출처: [Livox 공식 사양](https://www.livoxtech.com/mid-360s/specs).

| 항목 | 제조사 값 | 적용 |
|---|---|---|
| 수평/수직 시야각 | 360°, −7°~52° | 동일 각도 범위 |
| 근접 사각 | 0.1 m | 최소 거리 0.1 m |
| 검출 거리 | 40 m @10% 반사율·100 klx, cutoff 100 m | 기하학적 cutoff 100 m |
| 점 생성률 | 첫 반사 200,000 points/s | 1,000×20=20,000 rays/frame, 10 Hz |
| 프레임 주기 | 대표값 10 Hz | 10 Hz |
| 거리 정밀도 | 10 m에서 1σ≤2 cm, 0.2 m에서 ≤4 cm | 해당 조건부 값을 고정 잡음으로 대체하지 않음 |

스캔 원점은 `livox_frame`의 기존 CAD 원점입니다. 랙 기준 `(-0.18,0,1.183)` m, roll=π인 뒤집힌 장착을 그대로 반영합니다. 수직 시야각은 센서 로컬 축 기준이므로 로봇 기준에서는 아래쪽을 향합니다.

**제한:** Gazebo `gpu_lidar`는 균일한 1,000×20 각도 격자입니다. 실제 Livox 비반복 스캔 궤적, 점별 시간차, 운동 왜곡, 반사율/조도에 따른 검출 확률, 강도, 다중반사, 패킷 및 Livox CustomMsg는 재현하지 않습니다. 20개 수직 샘플은 가상 격자이며 실제 채널 수가 아닙니다. 장치 내장 IMU ICM40609는 이번 설정에 포함하지 않습니다.

### YDLIDAR G2

출처: EAI 제조사 문서 **G2 Data Sheet V1.3(211230), DOC#01.13.002000**, 성능표/좌표 정의. [제조사 원본 주소](https://www.ydlidar.com/Public/upload/files/2022-06-21/YDLIDAR%20G2%20Data%20Sheet%20V1.3%28211230%29.pdf)는 404였으므로 [동일 제조사 PDF 보관본](https://robu-prod-media.s3.ap-south-1.amazonaws.com/uploads/2022/06/YDLIDAR-G2-Data-Sheet-V1.3211230.pdf)을 확인했습니다. [공식 SDK 표](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/Dataset.md)도 대조했습니다.

| 항목 | 제조사 문서 | 적용 |
|---|---|---|
| 시야각 | 360° | 360° 회전의 714개 각도 bins |
| 측정률 | 5,000 measurements/s | 714×7=4,998 rays/s (정수 샘플 수로 인한 반올림) |
| 회전 주기 | 5~12 Hz, 공장 기본 7 Hz | 7 Hz |
| 거리 | 0.12~16 m @80% 반사율 | 0.12~16 m |
| 레이저 경사 | 대표 1°, 범위 0.25~1.75° | 로컬 수평면 위 1° 단일 원뿔 스캔 |
| 각도 간격 | 7 Hz에서 0.504° | 약 0.5042° |

SDK에는 G2 최소 거리 0.28 m가 기재돼 있어 데이터시트와 다릅니다. 이번 프로필은 읽은 V1.3 데이터시트의 0.12 m를 선택했습니다. G2A/G2C 또는 다른 리비전으로 대체하지 않았습니다. 실물 리비전/드라이버 필터에 맞춰 최소 거리를 변경할 수 있습니다.

측정 기준은 `laser_frame`, 랙 기준 `(0.000175,0,0.3266)` m, yaw≈8.81°를 유지합니다. 제조사 원시 각도는 위에서 본 시계방향이고 케이블 출구가 영점입니다. Gazebo는 오른손 좌표계의 반시계방향 각도를 발행합니다. 원시 UART 프로토콜/회전 시간차/강도와 거리별 오차는 모사하지 않습니다. CAD yaw가 실제 케이블 영점에 일치하는지는 실측해야 합니다. 1° 경사 때문에 이상적인 수평 평면의 2D scan과도 약간 다릅니다.

### RealSense D435f

기존 카메라에도 depth/RGB 스트림을 추가했습니다. 출처: [RealSense D400 제조사 데이터시트](https://dev.realsenseai.com/download/42003/), [D435f 공식 사양](https://www.intel.com/content/www/us/en/products/sku/229673/intel-realsense-depth-camera-d435f/specifications.html).

깊이 1280×720 / 수평 FOV 87°, RGB 1920×1080 / 수평 FOV 69°로 분리하고 둘 다 선택한 운영 주기 30 Hz를 사용합니다. 깊이 clip은 0.2~3 m인 보수적인 시뮬레이션 운영 범위이며 제조사 최대 거리와 동일하다는 뜻이 아닙니다. RGB clip은 렌더링용 0.01~100 m입니다.

**제한:** 핀홀 투영이며 수직 FOV는 종횡비로 결정됩니다(깊이 약 56.2°, RGB 약 42.3°). 사양의 깊이 58°와 정확히 같지 않습니다. 실제 stereo baseline, IR 프로젝터, 필터, 왜곡, 공장 보정, 두 렌즈의 별도 광학 원점, 노출/노이즈, depth-color 정합은 재현하지 않습니다. 두 스트림은 기존 CAD 카메라 위치를 공유합니다. 실측 calibration 없이는 장치와 완전히 동일한 영상 모델이라고 할 수 없습니다.

Gazebo 렌더링 카메라는 +X 전방/+Z 상방을 사용하고 메시지의 `camera_optical_frame`은 기존 URDF의 +Z 전방/+Y 아래 방향 변환을 사용합니다.

## 토픽과 프레임

| Gazebo 토픽 | 메시지 | 프레임 |
|---|---|---|
| `/robocup/g2/scan` | `gz.msgs.LaserScan` | `laser_frame` |
| `/robocup/g2/scan/points` | `gz.msgs.PointCloudPacked` | `laser_frame` |
| `/robocup/mid360s/scan` | `gz.msgs.LaserScan` (다중 수직 행) | `livox_frame` |
| `/robocup/mid360s/scan/points` | `gz.msgs.PointCloudPacked` | `livox_frame` |
| `/robocup/camera/depth/image` | `gz.msgs.Image` (float depth, m) | `camera_optical_frame` |
| `/robocup/camera/depth/image/points` | `gz.msgs.PointCloudPacked` | `camera_optical_frame` |
| `/robocup/camera/depth/camera_info` | `gz.msgs.CameraInfo` | `camera_optical_frame` |
| `/robocup/camera/color/image` | `gz.msgs.Image` | `camera_optical_frame` |
| `/robocup/camera/color/camera_info` | `gz.msgs.CameraInfo` | `camera_optical_frame` |

3D 라이다는 `/points`를 사용하세요. 다중 행 LaserScan을 ROS 2의 단일 평면 LaserScan으로 브리지하면 3D 구조를 표현할 수 없습니다. ROS 2 사용 시 별도 `ros_gz_bridge`와 `robot_state_publisher`를 구성해야 합니다. 이번 작업은 Gazebo Transport의 데이터 생성과 검증까지이며 ROS 설치/브리지 실행은 포함하지 않습니다.

## 실행 및 검증

Gazebo Harmonic과 시스템 Python Gazebo 바인딩을 사용합니다.

```bash
cd HW/URDF/tracer_sensor_rack_piper
python3 build.py
./start_sim.sh
```

센서 렌더링에는 world의 `gz-sim-sensors-system`/`ogre2`가 필요합니다. URDF만 다른 world에 넣으면 해당 플러그인도 추가해야 합니다. 실행 중 Play 상태여야 데이터가 갱신됩니다.

검증용 벽은 생성 옵션으로만 추가되며 기본 world에는 포함되지 않습니다.

```bash
python3 make_sim.py --test-wall
GZ_PARTITION=robocup_sensor_test gz sim -s -r motion.world.sdf
# 다른 터미널
GZ_PARTITION=robocup_sensor_test python3 check_sensors.py --wall
```

`check_sensors.py`는 각 토픽의 실제 메시지 5개로 sim-time 발행 주기, frame_id, 스캔/포인트 수, 영상 크기/버퍼, 유효 거리와 정면 벽의 깊이를 검사합니다. 벽 앞면 X=1.9 m, 카메라 X=−0.12495 m이므로 중심 깊이 기대값은 약 2.025 m입니다.

`gz_frame_id`/`optical_frame_id`에 대해 SDFormat 확장 태그 경고가 나올 수 있습니다. 실제 지원/프레임은 메시지 검증으로 확인합니다. 측정률은 simulation time 기준이며 GPU 성능에 따라 wall-clock 발행률이 낮아질 수 있습니다.

## 확인 결과 (2026-10-03)

Harmonic 8.15.0에서 서버를 실행해 실제 메시지를 수신했습니다.

- G2: 714 rays/scan, 유효 거리 277개, `laser_frame`, 관측 sim-time 주기 약 7.07 Hz.
- Mid-360S: 20,000 rays 및 20,000 points/frame, `livox_frame`, 약 10.42 Hz.
- D435f depth/RGB: 각각 1280×720 / 1920×1080, `camera_optical_frame`, 약 31.75 Hz.
- 정면 벽 중심 depth: 2.024 m, 기대값 약 2.025 m.
- Depth와 RGB CameraInfo 토픽이 별도로 존재함을 확인.

설정 주기는 각각 7/10/30 Hz입니다. 샘플 5개의 timestamp로 추정한 관측값은 Gazebo 렌더링 업데이트 스케줄에 따라 소폭 달랐습니다. 스크립트는 설정값 대비 15% 이내를 검사합니다. 이것은 실제 센서 주기/노이즈/재질 응답의 인증 시험이 아닙니다.

거리 검사를 추가한 최종 재검증에서도 통과했습니다. G2 정면 ray는 기대 1.9001 m / 측정 1.9001 m, Mid-360S는 기대 2.0802 m / 측정 2.0794 m였습니다. 이때 관측 주기는 G2 7.04 Hz, Mid-360S 10.00 Hz, 카메라 30.08 Hz였습니다.
