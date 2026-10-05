# 다음 날 실행할 직접 명령

> 이 문서는 2026-10-03 Jetson `/home/sparo/robot_setup/` 기록의 공유 사본입니다. 모델·venv·로그·설정 파일은 Jetson 로컬 경로를 사용합니다. [문서 모음](README.md) · [프로젝트 홈](../../README.md)

**ROS Humble Desktop과 Calibration/PiPER/Livox/YOLO 핵심 workspace가 빌드됐고, YOLO·SAM은 GPU 환경입니다.** RealSense/AprilTag/image_view/image_proc/camera_calibration 등 일부 apt는 아직 설치가 필요합니다. 해당 센서 명령은 설치 후 실제 serial/CAN/태그/TF 값을 입력해 사용합니다. 설치 절차는 README_SETUP에 있습니다.

각 노드는 별도 터미널에서 foreground로 실행하고 **그 터미널의 Ctrl+C**로 종료합니다. 기존 노드가 있으면 중복 실행하지 않습니다. 자신이 시작한 background 작업만 `$!`로 PID를 기록해 `kill -INT "$started_pid"`로 끝냅니다. 광범위한 pkill은 사용하지 않습니다.

## 1. 공통 source 순서와 최소 확인

터미널마다 ROS → 필요한 workspace overlay 순서입니다. venv를 activate하지 않고 모델 CLI는 절대 경로로 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/local_setup.bash
source ~/calibration_ws/install/local_setup.bash
source ~/ws_livox/install/local_setup.bash
source ~/detection_ws/install/local_setup.bash
export PYTHONPATH="$HOME/robot_setup/ros-python:$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$HOME/robot_setup/livox-sdk2/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

빌드하지 않은 workspace의 줄은 실행하지 않습니다. `local_setup.bash`는 이미 source한 Humble 및 앞선 overlay를 유지합니다. 실제 생성된 공식 setup 파일만 사용합니다.

이미 workspace/ROS import·서비스·공식 launch 인자를 확인했습니다. 아래 목록은 필요할 때만 확인하며 미설치 센서 패키지는 먼저 README의 남은 apt 설치를 완료합니다.

```bash
ros2 pkg executables realsense2_camera
ros2 pkg executables image_proc
ros2 pkg executables image_view
ros2 pkg executables apriltag_ros
ros2 pkg executables easy_handeye2
ros2 pkg executables piper
ros2 pkg executables yolo_ros
ros2 interface list | rg 'easy_handeye2_msgs|yolo_msgs'
ros2 bag record --help
ros2 bag play --help
ros2 launch realsense2_camera rs_launch.py --show-args
ros2 launch easy_handeye2 calibrate.launch.py --show-args
ros2 launch easy_handeye2 publish.launch.py --show-args
ros2 launch yolo_bringup yolo.launch.py --show-args
python3 -c 'import rclpy, cv_bridge, numpy, scipy, cv2, yaml; print(numpy.__version__, scipy.__version__, cv2.__version__)'
```

talker/listener는 별도 터미널에서 실행합니다. Publishing / I heard가 나오면 두 프로세스를 Ctrl+C로 종료합니다.

```bash
ros2 run demo_nodes_cpp talker
```

```bash
ros2 run demo_nodes_py listener
```

GUI 세션의 터미널에서 실행합니다. 창을 관찰할 수 없으면 실행 로그 확인과 화면 확인을 구분합니다.

```bash
glxinfo -B
rviz2
```

```bash
ros2 run rqt_image_view rqt_image_view
```

## 2. RealSense Head / Wrist

대응 SDK에 `rs-enumerate-devices`가 있으면 모델·serial·USB 속도·stream profile을 확인합니다. 없으면 apt wrapper 로그와 SDK 제공 CLI를 확인합니다. `lsusb -t`도 함께 봅니다.

```bash
rs-enumerate-devices
lsusb -t
ros2 launch realsense2_camera rs_launch.py --show-args
```

serial 변수에는 **실제 serial**을 입력합니다. 빈 값으로는 실행되지 않습니다. downloaded apt와 같은 wrapper 4.58.4의 공식 source에서 serial의 underscore prefix 제거 지원을 확인했습니다. 실제 serial과 장치 프로파일은 연결 후 대조합니다.

**2026-10-05 카메라 구성: Head D435 + Wrist D435.** 두 카메라는 서로 다른 실제 serial을 지정합니다. RGB 설정은 두 장치 모두 `rgb_camera.color_profile`, depth 설정은 `depth_module.depth_profile`을 사용합니다. [wrapper 4.58.4의 공식 인자](https://github.com/realsenseai/realsense-ros/blob/4.58.4/realsense2_camera/launch/rs_launch.py)를 기준으로 명령을 갱신했으며 두 장치의 실행·관측 품질을 확인한 결과는 아닙니다.

Head D435 터미널:

```bash
HEAD_SERIAL=''
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=sensors camera_name:=head \
  serial_no:="_${HEAD_SERIAL:?Head D435 실제 serial 필요}" \
  enable_color:=true enable_depth:=true \
  align_depth.enable:=true enable_sync:=true \
  pointcloud.enable:=true publish_tf:=true \
  rgb_camera.color_profile:=640,480,30 \
  depth_module.depth_profile:=640,480,30
```

Wrist D435 터미널:

```bash
WRIST_SERIAL=''
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=sensors camera_name:=wrist \
  serial_no:="_${WRIST_SERIAL:?Wrist D435 실제 serial 필요}" \
  enable_color:=true enable_depth:=true \
  align_depth.enable:=true enable_sync:=true \
  pointcloud.enable:=true publish_tf:=true \
  rgb_camera.color_profile:=640,480,30 \
  depth_module.depth_profile:=640,480,30
```

profile이 지원되지 않으면 실제 지원 목록에 맞춰 변경값을 기록합니다. `640×480@30`은 시작 설정이며 손목의 최소 관측 거리·품질을 보장하지 않습니다. `enable_sync`는 각 카메라의 stream 동기화이며 두 카메라 사이 hardware 동기화를 보장하지 않습니다. 두 D435의 intrinsic·외부 장착 보정값을 각각 관리합니다.

카메라별 예상 topic suffix:

| topic suffix | 타입 / 확인 내용 |
| --- | --- |
| color/image_raw | sensor_msgs/msg/Image, encoding, header stamp/frame |
| color/camera_info | sensor_msgs/msg/CameraInfo, intrinsic |
| aligned_depth_to_color/image_raw | 실제 encoding: 16UC1(mm) 또는 32FC1(m) |
| aligned_depth_to_color/camera_info | RGB에 align된 depth의 CameraInfo |
| depth/color/points | sensor_msgs/msg/PointCloud2 |

```bash
ros2 topic list -t
ros2 topic info -v /sensors/head/color/image_raw
ros2 topic info -v /sensors/head/aligned_depth_to_color/image_raw
ros2 topic echo --once --qos-reliability best_effort /sensors/head/color/camera_info
ros2 topic echo --once --qos-reliability best_effort /sensors/head/aligned_depth_to_color/image_raw --field encoding
ros2 topic info -v /sensors/wrist/color/image_raw
ros2 topic info -v /sensors/wrist/aligned_depth_to_color/image_raw
ros2 topic echo --once --qos-reliability best_effort /sensors/wrist/color/camera_info
ros2 run tf2_ros tf2_echo head_link head_color_optical_frame
```

Wrist도 `wrist_link wrist_color_optical_frame`으로 확인합니다. 실제 이름과 QoS를 config/명령에 반영합니다. **16UC1: depth[m]=raw/1000. 32FC1: 이미 m이므로 YOLO divisor=1.** 실측 없는 base_link→camera identity TF는 발행하지 않습니다.

## 3. PiPER feedback과 URDF

PiPER와 TRACER의 실제 CAN을 구분하고 SDK/노드/URDF 호환성을 확인합니다. 제조사 CAN 스크립트는 내용을 읽은 뒤 사용합니다. 이번 setup은 읽기 경로만 다룹니다.

[제조사 읽기 소스](https://raw.githubusercontent.com/agilexrobotics/piper_ros/humble/src/piper/piper/piper_read_slave_joint.py)는 상대 `joint_states`로 feedback을 발행하므로 전용 feedback topic으로 remap합니다. 시작 전에 실제 checkout과 SDK ConnectPort 내부도 확인합니다. 기존 feedback publisher가 있으면 재사용합니다.

```bash
ros2 node list
ros2 topic info -v /piper/joint_states_feedback
ip -details link show type can
```

실제 값이 확인된 후 읽기용 터미널:

```bash
PIPER_CAN=''
PIPER_GRIPPER=''
ros2 run piper piper_read_slave_joint --ros-args -r __ns:=/piper -r joint_states:=joint_states_feedback -p can_port:="${PIPER_CAN:?PiPER 실제 CAN 이름 필요}" -p gripper_exist:="${PIPER_GRIPPER:?true 또는 false 필요}"
```

robot_state_publisher 터미널: 실제 firmware에 맞는 제조사 URDF를 선택합니다. S-V1.6-3 경계의 old/new 기준은 제조사 README와 실물을 대조합니다.

```bash
PIPER_URDF=''
ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/piper -r joint_states:=/piper/joint_states_feedback -p frame_prefix:=piper/ -p robot_description:="$(cat "${PIPER_URDF:?firmware 대응 제조사 URDF 절대 경로 필요}")"
```

```bash
ros2 topic echo --once /piper/joint_states_feedback
ros2 run tf2_ros tf2_echo piper/base_link piper/link6
```

실제 joint name/stamp와 URDF link를 대조합니다. command `/joint_states`나 GUI 생성값을 feedback으로 사용하지 않습니다. follow launch가 command/GUI를 실행하는지 미확인이면 직접 노드를 사용합니다.

## 4. AprilTag → Hand–eye 보정

template YAML의 size 문자열을 실제 측정 float[m]로 바꿔 `head_apriltag.yaml`/`wrist_apriltag.yaml`로 저장합니다. Head 종료 후 Wrist를 순차 실행합니다.

Head rectification 터미널:

```bash
ros2 run image_proc rectify_node --ros-args -r __ns:=/sensors/head/color -r image:=/sensors/head/color/image_raw -r camera_info:=/sensors/head/color/camera_info -r image_rect:=/sensors/head/color/image_rect
```

Head tag 터미널:

```bash
ros2 run apriltag_ros apriltag_node --ros-args -r __ns:=/sensors/head -r image_rect:=/sensors/head/color/image_rect -r camera_info:=/sensors/head/color/camera_info --params-file ~/calibration_ws/config/head_apriltag.yaml
ros2 run tf2_ros tf2_echo head_link head_calib_tag
```

tf2_echo는 별도 터미널에서 실행합니다. Wrist는 namespace를 `/sensors/wrist`, marker를 wrist_calib_tag, YAML을 Wrist용으로 바꿉니다. raw Image + CameraInfo → rectify → apriltag → optical_frame→marker TF 순서이며, 내부 RealSense TF를 통해 head_link/wrist_link와 연결되는지 확인합니다.

확인한 기본 calibrate launch에는 dummy_publisher가 있습니다. 사용자 조건에 맞춰 **공식 노드를 직접 실행합니다.** 기준 commit의 dummy TF와 인자/서비스 선언을 로컬에서 확인했습니다. 공식 launch의 `--show-args`와 서비스 인터페이스도 확인했습니다.

```bash
ros2 run easy_handeye2 handeye_server --ros-args --params-file ~/calibration_ws/config/head_handeye.yaml
```

UI용 별도 터미널:

```bash
ros2 run easy_handeye2 rqt_calibrator.py --ros-args --params-file ~/calibration_ws/config/head_handeye.yaml
```

실제 feedback TF와 marker TF를 읽습니다. Wrist는 Head server/UI 종료 후 wrist_handeye.yaml로 실행합니다. 서비스가 절대 namespace인 구현에서는 name만 변경해도 충돌하므로 순차 실행합니다. 미선언 launch 인자 freehand_robot_movement를 추가하지 않습니다.

아래는 **후속 실제 수동 보정용**이며 이번 setup에서는 실행하지 않았습니다.

```bash
ros2 service list -t
ros2 interface show easy_handeye2_msgs/srv/TakeSample
ros2 interface show easy_handeye2_msgs/srv/ComputeCalibration
ros2 interface show easy_handeye2_msgs/srv/SaveSamples
ros2 interface show easy_handeye2_msgs/srv/SaveCalibration
ros2 service call /easy_handeye2/calibration/take_sample easy_handeye2_msgs/srv/TakeSample '{}'
ros2 service call /easy_handeye2/calibration/save_samples easy_handeye2_msgs/srv/SaveSamples '{}'
ros2 service call /easy_handeye2/calibration/compute_calibration easy_handeye2_msgs/srv/ComputeCalibration '{}'
ros2 service call /easy_handeye2/calibration/save_calibration easy_handeye2_msgs/srv/SaveCalibration '{}'
```

확인한 기준 commit 소스의 저장 위치는 `~/.ros2/easy_handeye2/calibrations/<name>.calib`와 `~/.ros2/easy_handeye2/samples/<name>.samples`입니다. 실제 save response의 filepath를 우선합니다. 계산 valid=true 및 실측 검증 후에만 공식 publish launch를 사용합니다.

```bash
test -s ~/.ros2/easy_handeye2/calibrations/head_piper.calib && ros2 launch easy_handeye2 publish.launch.py name:=head_piper
```

Wrist의 name은 wrist_piper입니다. RealSense 내부 TF를 유지하고 같은 child에 외부 publisher를 중복 실행하지 않습니다. `ros2 run camera_calibration cameracalibrator --help`로 intrinsic 도구를 확인합니다. 실제 pattern/크기를 입력한 후 intrinsic을 확인하며 factory intrinsic을 무조건 덮어쓰거나 COMMIT하지 않습니다.

## 5. MID360

실제 NIC/IP를 확인한 뒤 MID360_config.template.json을 채워 `MID360_config.json`으로 저장합니다. 192.168.1.184는 확인할 후보입니다. default route는 변경하지 않고 SDK extrinsic은 0을 유지합니다.

```bash
ip -br address
ip route
ip neigh
ros2 launch livox_ros_driver2 rviz_MID360_launch.py --show-args
```

[공식 RViz launch](https://raw.githubusercontent.com/Livox-SDK/livox_ros_driver2/master/launch_ROS2/rviz_MID360_launch.py)는 xfer_format=0과 frame_id=livox_frame을 코드 상수로 설정합니다. 실제 설정을 일반 YAML로 전달하는 공식 노드 CLI 경로를 사용합니다.

```bash
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args --params-file ~/calibration_ws/config/livox_driver.yaml
```

YAML의 user_config_path는 실제 JSON 절대 경로로 지정합니다. RViz는 별도 터미널에서 실행합니다.

```bash
rviz2 -d "$(ros2 pkg prefix --share livox_ros_driver2)/config/display_point_cloud_ROS2.rviz"
ros2 topic type /livox/lidar
ros2 topic info -v /livox/lidar
ros2 topic echo --once /livox/lidar --field fields
```

정상 조건은 sensor_msgs/msg/PointCloud2, intensity field, livox_frame 및 실제 점군 수신입니다. CustomMsg이면 실제 node parameter와 launch 소스를 확인합니다. 뒤집힌 장착 회전은 실측 ROS TF에 한 번만 적용합니다.

## 6. YOLO Head / Wrist

실제 --show-args와 topic/QoS 확인 후 실행합니다. 아래는 로컬 기준 commit의 공식 launch에 선언된 인자를 확인한 명령입니다. 실제 `--show-args` 실행도 완료했습니다. 카메라 입력의 QoS/토픽/TF는 연결 후 대조합니다.

Head 2D:

```bash
ros2 launch yolo_bringup yolo.launch.py namespace:=head_yolo model:=$HOME/detection_data/models/yolo11n.pt device:=cuda:0 input_image_topic:=/sensors/head/color/image_raw image_reliability:=2 use_tracking:=False use_3d:=False use_debug:=True
```

Head 초기 3D: Head 2D launch를 Ctrl+C로 종료한 뒤 교체합니다. 16UC1일 때만 divisor=1000입니다.

```bash
ros2 launch yolo_bringup yolo.launch.py namespace:=head_yolo model:=$HOME/detection_data/models/yolo11n.pt device:=cuda:0 input_image_topic:=/sensors/head/color/image_raw image_reliability:=2 input_depth_topic:=/sensors/head/aligned_depth_to_color/image_raw input_depth_info_topic:=/sensors/head/aligned_depth_to_color/camera_info depth_image_reliability:=2 depth_info_reliability:=2 depth_image_units_divisor:=1000 target_frame:=head_color_optical_frame use_tracking:=False use_3d:=True use_debug:=True
```

Wrist는 별도 namespace입니다.

```bash
ros2 launch yolo_bringup yolo.launch.py namespace:=wrist_yolo model:=$HOME/detection_data/models/yolo11n-seg.pt device:=cuda:0 input_image_topic:=/sensors/wrist/color/image_raw image_reliability:=2 use_tracking:=False use_3d:=False use_debug:=True
```

```bash
ros2 topic info -v /head_yolo/detections
ros2 topic echo --once /head_yolo/detections
ros2 topic info -v /head_yolo/debug_image
ros2 topic hz /head_yolo/detections
ros2 topic echo --once /head_yolo/detections_3d
ros2 interface show yolo_msgs/msg/Detection
ros2 interface show yolo_msgs/msg/Mask
```

실제 QoS와 정지 상태 추론률을 기록합니다. 보정 TF 확인 후 target_frame을 실제 base/piper frame으로 바꿉니다. use_3d는 aligned depth Image 입력입니다. 최신 TF lookup은 이동 중 이미지 시각의 TF 검증을 대신하지 않습니다.

YOLO ROS mask.data는 윤곽 polygon 좌표이며 H×W pixel mask가 아닙니다. SAM pixel mask와의 변환 및 ROS publisher는 별도 개발 범위입니다.

## 7. SAM 2.1

Jupyter는 token/password 인증을 유지하고 localhost에 bind합니다.

```bash
cd ~/detection_tools/sam2
~/venvs/sam21/bin/jupyter lab --ip=127.0.0.1 --no-browser --notebook-dir=/home/sparo/detection_data/samples
```

브라우저에서 token이 포함된 로컬 URL을 열고 `image_predictor_example_sam21_tiny.ipynb`를 선택합니다. SAM 2.1 (Jetson GPU Python 3.10) kernel과 tiny 설정이 지정되어 있습니다.

다음은 공식 예제 이미지에 대한 최소 point/box 추론입니다. 보조 runner 파일 없이 CUDA에서 직접 실행하고 실제 masks/scores를 저장합니다. 이미 GPU 추론을 확인했으므로 필요할 때만 재실행합니다.

```bash
cd ~/detection_tools/sam2
~/venvs/sam21/bin/python - <<'PY'
from pathlib import Path
import numpy as np
import torch
from PIL import Image
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
root = Path('/home/sparo')
image = np.array(Image.open(root/'detection_tools/sam2/notebooks/images/truck.jpg').convert('RGB'))
model = build_sam2('configs/sam2.1/sam2.1_hiera_t.yaml', str(root/'detection_data/models/sam2.1_hiera_tiny.pt'), device='cuda')
predictor = SAM2ImagePredictor(model)
with torch.inference_mode():
    predictor.set_image(image)
    masks, scores, _ = predictor.predict(point_coords=np.array([[500,375]]), point_labels=np.array([1]), multimask_output=True)
    box_masks, box_scores, _ = predictor.predict(box=np.array([425,600,700,875]), multimask_output=False)
out = root/'detection_data/results'
np.savez_compressed(out/'sam21_sample.npz', point_masks=masks, point_scores=scores, box_masks=box_masks, box_scores=box_scores)
best = masks[int(np.argmax(scores))].astype(bool)
overlay = image.copy()
overlay[best] = (0.5 * overlay[best] + 0.5 * np.array([0,255,0])).astype(np.uint8)
Image.fromarray(overlay).save(out/'sam21_sample_overlay.png')
print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), 'point', masks.shape, scores, 'box', box_masks.shape, box_scores)
PY
```

이는 공식 truck 이미지의 예제입니다. 실제 Wrist의 동일 RGB/depth/CameraInfo가 있을 때 bbox→SAM pixel mask→유효 aligned depth→대상 점군 순서로 처리합니다. depth=0과 무효값을 제외하고 단위/intrinsic을 맞춥니다. 실시간 ROS 구현은 준비되지 않았습니다.

## 8. 수동 저장 / rosbag

Humble [image_saver 소스](https://raw.githubusercontent.com/ros-perception/image_pipeline/humble/image_view/src/image_saver_node.cpp)의 save_all_image, encoding, filename_format과 save 서비스(std_srvs/srv/Empty)를 사용합니다. %s 확장자는 JPG를 선택하므로 16UC1 depth는 PNG 파일명으로 지정합니다. RGB의 bgr8과 depth의 실제 encoding을 확인합니다. 32FC1은 bag으로 보존합니다.

RGB saver 터미널:

```bash
ros2 run image_view image_saver --ros-args -r __ns:=/head_rgb_save -r image:=/sensors/head/color/image_raw -r camera_info:=/sensors/head/color/camera_info -p save_all_image:=false -p encoding:=bgr8 -p filename_format:=$HOME/detection_data/images/head_rgb_%04i.png
```

depth saver 터미널:

```bash
ros2 run image_view image_saver --ros-args -r __ns:=/head_depth_save -r image:=/sensors/head/aligned_depth_to_color/image_raw -r camera_info:=/sensors/head/aligned_depth_to_color/camera_info -p save_all_image:=false -p encoding:=16UC1 -p filename_format:=$HOME/detection_data/images/head_depth_%04i.png
```

저장 요청용 별도 터미널:

```bash
ros2 service list -t
ros2 service call /head_rgb_save/save std_srvs/srv/Empty '{}'
ros2 service call /head_depth_save/save std_srvs/srv/Empty '{}'
ros2 topic echo --once --qos-reliability best_effort /sensors/head/color/camera_info > ~/detection_data/images/head_camera_info.yaml
```

실제 서비스 이름은 service list를 우선합니다. 대응 CameraInfo가 수신되면 `.ini`도 저장됩니다. 파일 count나 수동 저장 호출을 엄밀한 RGB/depth 시간 동기화로 취급하지 않습니다. stamp 보존에는 bag을 사용합니다.

Koide용 기록: 매번 새 출력 경로를 사용합니다.

```bash
ros2 bag record -o "$HOME/calibration_data/head_mid360/bags/session_$(date +%Y%m%d_%H%M%S)" /sensors/head/color/image_raw /sensors/head/color/camera_info /livox/lidar /tf /tf_static
```

Detection용 RGB/depth/CameraInfo 기록:

```bash
ros2 bag record -o "$HOME/detection_data/bags/session_$(date +%Y%m%d_%H%M%S)" /sensors/head/color/image_raw /sensors/head/color/camera_info /sensors/head/aligned_depth_to_color/image_raw /sensors/head/aligned_depth_to_color/camera_info /sensors/wrist/color/image_raw /sensors/wrist/color/camera_info /sensors/wrist/aligned_depth_to_color/image_raw /sensors/wrist/aligned_depth_to_color/camera_info /tf /tf_static
```

Ctrl+C로 종료하여 metadata가 저장된 뒤 `BAG_PATH`에 실제 절대 경로를 입력합니다.

```bash
BAG_PATH=''
ros2 bag info "${BAG_PATH:?실제 bag 경로 필요}"
ros2 bag play "$BAG_PATH"
```

같은 topic에서 replay와 실기 publisher를 중복 실행하지 않습니다. bag 재생으로 로봇 동작 명령을 보내지 않습니다.

## 9. Koide 기록→전처리→초기화→최적화→viewer

이번에는 설정만 준비했습니다. 아래 Docker 명령은 기존 amd64 Koide PC 또는 호환 ARM64 실행 환경에서 사용합니다. 이 Jetson에서 humble amd64 이미지를 바로 실행하지 않습니다.

정지 상태의 실제 Head image/CameraInfo와 MID360 PointCloud2/intensity가 있을 때 기록합니다. 보정과 별도 검증 세션을 분리합니다.

```bash
ros2 bag record -o "$HOME/calibration_data/head_mid360/bags/session_$(date +%Y%m%d_%H%M%S)" /sensors/head/color/image_raw /sensors/head/color/camera_info /livox/lidar /tf /tf_static
# 별도 validation 촬영은 같은 토픽으로 validation 디렉터리에 기록합니다.
```

기준 소스에 선언된 topic/필터 인자를 사용합니다. 실제 container 프로그램 버전은 source commit과 같다고 가정하지 않습니다.

```bash
sudo docker run --rm -v "$HOME/calibration_data/head_mid360/bags:/tmp/input_bags:ro" -v "$HOME/calibration_data/head_mid360/processed:/tmp/preprocessed" koide3/direct_visual_lidar_calibration:humble ros2 run direct_visual_lidar_calibration preprocess /tmp/input_bags /tmp/preprocessed --image_topic /sensors/head/color/image_raw --camera_info_topic /sensors/head/color/camera_info --points_topic /livox/lidar --intensity_channel intensity --min_distance 1.0
```

`--min_distance`는 가까운 점을 제외합니다. 기준 checkout의 기본값 1.0m를 확인했습니다. 필요하면 옵션값을 변경하고 validation_processed에도 같은 필터를 사용합니다. 이 값은 장착 실측값이 아닙니다.

GUI는 실제 Xauthority와 로컬 DISPLAY를 전달합니다. XWayland도 유효한 cookie/socket이 필요하며 인증은 유지합니다.

```bash
KOIDE_XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
test -r "$KOIDE_XAUTH"
sudo docker run --rm -e "DISPLAY=${DISPLAY:?로컬 GUI DISPLAY 필요}" -e XAUTHORITY=/tmp/koide.xauth -e LIBGL_ALWAYS_SOFTWARE=1 -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "${KOIDE_XAUTH:?실제 Xauthority 경로}:/tmp/koide.xauth:ro" -v "$HOME/calibration_data/head_mid360/processed:/tmp/preprocessed" koide3/direct_visual_lidar_calibration:humble ros2 run direct_visual_lidar_calibration initial_guess_manual /tmp/preprocessed
```

최적화:

```bash
sudo docker run --rm -v "$HOME/calibration_data/head_mid360/processed:/tmp/preprocessed" koide3/direct_visual_lidar_calibration:humble ros2 run direct_visual_lidar_calibration calibrate /tmp/preprocessed
```

최적화에도 GUI가 필요하면 같은 Xauthority 설정을 추가합니다. viewer는 초기화 GUI 명령의 executable을 viewer로 바꿉니다. validation은 입력 mount를 validation, 출력 mount를 validation_processed로 변경합니다.

[기준 calibrate.cpp](https://raw.githubusercontent.com/koide3/direct_visual_lidar_calibration/02a0dc039f5509708f384be4ff3228e0ae09352d/src/calibrate.cpp)는 results.T_lidar_camera를 translation xyz + quaternion xyzw로 저장합니다. camera 좌표를 lidar 좌표로 옮기는 변환이므로 camera parent / livox child에는 역변환을 사용합니다.

실제 결과 7개 값이 있을 때만 별도 DDS domain의 계산용 frame으로 확인합니다. 빈 값으로는 실행되지 않습니다.

```bash
ROS_DOMAIN_ID=231 ros2 run tf2_ros static_transform_publisher --x "${RESULT_TX:?실제 결과}" --y "${RESULT_TY:?실제 결과}" --z "${RESULT_TZ:?실제 결과}" --qx "${RESULT_QX:?실제 결과}" --qy "${RESULT_QY:?실제 결과}" --qz "${RESULT_QZ:?실제 결과}" --qw "${RESULT_QW:?실제 결과}" --frame-id calc_lidar --child-frame-id calc_camera
```

별도 터미널:

```bash
ROS_DOMAIN_ID=231 ros2 run tf2_ros tf2_echo calc_camera calc_lidar
```

역변환과 실제 camera optical frame 좌표 규약을 확인한 후 최종 TF에 사용합니다. 이번에는 계산용/최종 TF를 모두 발행하지 않았습니다.

## 10. TCP / 장착 TF / original TRACER

Pivot은 [공식 CLI](https://raw.githubusercontent.com/SciKit-Surgery/scikit-surgerycalibration/master/sksurgerycalibration/ui/pivot_calibration_command_line.py)와 [입력 처리](https://raw.githubusercontent.com/SciKit-Surgery/scikit-surgerycalibration/master/sksurgerycalibration/ui/pivot_calibration_app.py)에 따릅니다. 각 파일에는 np.loadtxt로 읽을 수 있는 4×4 tracking matrix 수치가 필요합니다. 관절각 목록이나 header가 포함된 CSV를 사용하지 않습니다. 기준 frame과 translation 단위를 통일합니다.

```bash
~/venvs/pivot/bin/sksPivotCalibration --help
~/venvs/pivot/bin/sksPivotCalibration -i ~/calibration_data/tcp/matrices
```

matrices는 실데이터가 있을 때만 생성합니다. 결과는 위치 offset이며 TCP orientation/지그 offset은 별도로 정합니다.

base→arm, base→2D LiDAR, flange→TCP 각각에 실측값과 실제 parent/child를 지정합니다. 미입력 상태에서는 발행되지 않습니다.

```bash
ros2 run tf2_ros static_transform_publisher --x "${MOUNT_X:?실측 m}" --y "${MOUNT_Y:?실측 m}" --z "${MOUNT_Z:?실측 m}" --qx "${MOUNT_QX:?실측}" --qy "${MOUNT_QY:?실측}" --qz "${MOUNT_QZ:?실측}" --qw "${MOUNT_QW:?실측}" --frame-id "${MOUNT_PARENT:?실제 parent}" --child-frame-id "${MOUNT_CHILD:?실제 child}"
```

실측/보정 후 연결은 base_link→piper/base_link, piper/base_link→piper/link6(실제 feedback URDF), piper/base_link→head_link(Head 보정), piper/link6→wrist_link(Wrist 보정), camera optical→livox_frame(Koide 역변환), flange→TCP입니다. RealSense가 camera 내부 TF를 유지하며 각 child의 발행자는 하나로 정합니다.

original TRACER driver/odom이 필요하면 `ros2 topic info -v /odom`, TF 발행자와 적분 방식을 먼저 기록합니다. ugv_sdk의 standalone CMake와 tracer_ros2 Humble 호환성은 별도로 확인합니다. 2D LiDAR 모델 미확정으로 제품 driver는 보류합니다. FAST-LIO/FAST-LIO2는 설치하지 않습니다.

## 11. known_objects YOLO detection 학습

data.yaml의 names는 비어 있습니다. 실제 class/id를 입력하고 촬영 session 단위로 train/val/test를 분리한 뒤 학습합니다. [makesense.ai](https://www.makesense.ai/)의 YOLO bbox export를 사용합니다. 각 라벨 행은 `class_id cx cy w h`, 좌표는 0–1 정규화입니다. 같은 연속 영상의 근접 frame을 split 사이에 섞지 않습니다.

실제 데이터 확인 후 1 epoch 점검:

```bash
~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/yolo detect train model=$HOME/detection_data/models/yolo11n.pt data=$HOME/detection_data/datasets/known_objects/data.yaml epochs=1 device=0 project=$HOME/detection_data/results name=known_objects_check
```

후속 본학습은 별도 run 이름과 명시한 epochs로 진행합니다. val/test/predict/resume 직접 명령:

```bash
~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/yolo detect val model=$HOME/detection_data/results/known_objects_check/weights/best.pt data=$HOME/detection_data/datasets/known_objects/data.yaml split=val device=0
~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/yolo detect val model=$HOME/detection_data/results/known_objects_check/weights/best.pt data=$HOME/detection_data/datasets/known_objects/data.yaml split=test device=0
~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/yolo detect predict model=$HOME/detection_data/results/known_objects_check/weights/best.pt source=$HOME/detection_data/datasets/known_objects/images/test device=0 save=True save_txt=True
~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/yolo detect train model=$HOME/detection_data/results/known_objects_check/weights/last.pt resume=True device=0
```

ROS의 `model:=`에 검증한 best.pt 절대 경로를 전달합니다. bbox detection 학습, YOLO segmentation 학습, SAM fine-tuning은 별개입니다. 준비 범위는 detection 학습 경로와 사전학습 SAM이며 실제 본학습은 실행하지 않았습니다.
