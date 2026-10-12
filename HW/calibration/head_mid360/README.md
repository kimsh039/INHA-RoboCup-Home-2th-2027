# Head D435 ↔ Mid360: koide3 실측 보정

**현재 실물 로봇 실행은 [SW/robot/README.md](../../../SW/robot/README.md)를 따릅니다.** 최신 실물 Base–Mid360·G2·손목·내부 보정·오도메트리를 함께 사용하며, 아래의 모델 URDF 적용 기록과 실기 실행 설정을 구분합니다.

2026-10-12에 사용자가 수락한 4개 bag 최적화 결과입니다. 원본 [calib.json](calib.json)은 `/home/sparo/vlcal_results/mid360_camera_4bags/calib.json`에서 그대로 보존했습니다. 초기 추정값 대신 `results.T_lidar_camera`를 적용합니다.

변환은 **`livox_frame ← camera_color_optical_frame`**, 거리 m, quaternion xyzw입니다.

```text
translation: 0.03621342108120821 -0.026786900107094835 -0.08070957488472164
quaternion:  0.436295004181045 0.4425071265873687 0.5487692846319301 0.5591836768814089
```

이미지/점군 프레임 이름은 원본 bag의 CameraInfo/PointCloud2 헤더에서 가져왔습니다. [frames.json](frames.json)에 적용 조건이 있습니다. 기록에는 카메라 내부 TF가 없으므로 실기에서는 연결된 D435 드라이버의 내부 TF를 사용합니다. 토픽 namespace와 TF frame 이름은 별개입니다. 현재 센서 실행기의 `/head/camera/...` 토픽도 `camera_name:=camera`이면 같은 프레임 이름을 사용합니다.

## 이 젯슨에서 사용

로봇에 보정값을 적용한다는 것은 센서 사이의 위치·회전을 ROS TF로 발행하여 SLAM, 탐지, 점군 투영 등이 사용할 수 있게 하는 것입니다. Koide는 계속 실행할 필요가 없습니다.

이미 실행 중인 센서와 별도 터미널에서 다음 프로세스를 계속 켜두세요.

```bash
bash ~/robocup/SW/robot/start_sensor_tf.sh
```

젯슨의 `~/livox_g2_calib/sensors.sh start`는 이제 전체 실물 실행기 `robot_physical` tmux 세션을 시작합니다. 이 작업에서 센서를 시작하거나 재시작하지 않았습니다. 다음 start부터 TF 세션이 실행됩니다. 기존 센서를 유지하려면 위 명령만 실행하면 됩니다.

TF 연결은 다음과 같습니다.

```text
base_link → livox_frame → camera_link → camera_color_frame → camera_color_optical_frame
          → laser_frame
```

[실기 TF 노드](../../../SW/robot/sensor_tf.py)는 `HW/calibration/physical`에 보존한 2026-10-12 실물 Base–Mid360 및 G2–Mid360 변환을 발행합니다. RealSense 드라이버의 `publish_tf:=true`를 유지하세요. 노드는 드라이버 내부의 `camera_color_optical_frame ← camera_link`를 받아 `T_livox_camera_link = T_livox_color_optical × T_color_optical_camera_link`를 계산합니다. 드라이버가 카메라 내부 RGB/depth TF를 담당하고, 이 노드는 카메라 루트의 장착 TF만 담당합니다. 내부 TF가 도착하기 전에는 카메라 장착 TF를 발행하지 않습니다.

camera_name이나 tf_prefix를 바꾸면 실제 프레임 이름을 지정하세요.

```bash
bash ~/robocup/SW/robot/start_sensor_tf.sh \
  --camera-link head_camera_link --camera-optical head_camera_color_optical_frame
```

같은 프레임의 장착 TF는 이 노드 하나가 담당하도록 하세요. 같은 로봇에 전체 URDF robot_state_publisher나 별도 Base–LiDAR/카메라 static_transform_publisher를 함께 실행하면 부모·변환이 중복됩니다. 기존 SLAM 실험은 `/slam/tf`, `/slam/tf_static`로 분리되어 있으므로 그 실험의 TF 설정은 별도로 유지됩니다. `odom → base_link`는 베이스 드라이버 또는 EKF가 담당합니다. 이 노드는 팔·손목 관절 TF를 발행하지 않습니다.

## URDF 적용과 재생성

`robocup.calibrated.urdf`의 `camera_link_joint`에 기존 `rack_base_link ← livox_frame`과 이번 결과를 합성했습니다. 모델의 `camera_optical_frame`은 보정된 RGB optical 원점입니다. 이 모델의 `camera_link`는 그 원점을 공유하는 가상 body 프레임이며, 실제 D435의 좌측 IR 원점과 동일하다고 가정하지 않습니다. 실기에서는 위 TF 노드가 드라이버의 실제 내부 extrinsics를 반영합니다.

```bash
python3 ~/robocup/HW/calibration/head_mid360/tools/apply_koide.py
```

`integrate_calibration.py --replace`도 마지막 단계에서 이 도구를 호출하므로 기존 시뮬레이션 Head 결과로 되돌아가지 않습니다. 최종 결과는 integrated bundle의 `base_head_koide.json`, `head_mid360_koide.json` 및 summary/application sidecar에 기록됩니다. 기존 `base_head.json`과 arm/plane 결과는 이전 단계 기록이며 최종 Head 선택은 sidecar의 `head_selection`을 따릅니다. 새 Koide 관측에는 과거 15° CAD delta를 다시 적용하지 않습니다.

이 Koide 측정 자체는 Mid360–RGB의 상대 변환입니다. 현재 실기 실행에서는 별도 실물 Base–Mid360 결과와 연결합니다. 아래 모델 URDF 적용 기록은 이전 Mid360 기준으로 생성한 모델 기록입니다. 과거 hand-eye 잔차는 이번 Koide 결과의 정확도 수치가 아닙니다. 요청에 따라 추가 테스트·빌드·실기 실행·결과 검증은 생략했습니다. 대용량 원본 bag/PLY는 Git에 넣지 않았습니다.

좌표 변환 방향은 [Koide 구현](https://github.com/koide3/direct_visual_lidar_calibration/blob/02a0dc039f5509708f384be4ff3228e0ae09352d/src/calibrate.cpp), RealSense 내부 프레임과 `publish_tf`는 [공식 드라이버 문서](https://github.com/realsenseai/realsense-ros#ros2-vs-optical-coordination-systems)를 참고하세요.
