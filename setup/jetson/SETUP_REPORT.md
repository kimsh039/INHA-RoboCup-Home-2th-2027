# Calibration + Detection 설정 결과

> 이 문서는 2026-10-03 Jetson `/home/sparo/robot_setup/` 기록의 공유 사본입니다. 모델·venv·로그·설정 파일은 Jetson 로컬 경로를 사용합니다. [문서 모음](README.md) · [프로젝트 홈](../../README.md)

최종 업데이트: 2026-10-03. 다운로드와 설치를 우선하고 검증은 설치 확인·필수 GPU 샘플 추론·짧은 ROS 통신 확인으로 제한했습니다.

**YOLO와 SAM을 모두 Jetson GPU 버전으로 전환했습니다. torch 2.8.0 / torchvision 0.23.0 / CUDA 12.6 / Orin sm_87이며, 두 환경에서 GPU 인식과 실제 추론을 확인했습니다.** 이전 보고서 이후 설치된 ROS Humble Desktop을 재사용하고 Calibration·PiPER·Livox·YOLO workspace의 핵심 패키지 빌드를 완료했습니다.

실제 보정·사용자 물체 학습·로봇 통합 완료를 의미하지 않습니다. 센서용 apt 일부와 실제 하드웨어 입력은 남아 있습니다. Koide는 사용자의 요청대로 설정만 준비했습니다.

## 1. 실제 PC / Python 환경

| 항목 | 현재 실제 값 |
| --- | --- |
| OS / CPU architecture | Ubuntu 22.04.5 Jammy / ARM64 aarch64 |
| kernel / L4T / JetPack | 5.15.199-tegra / 36.5.2 / 6.2.3+b81 |
| RAM / swap | 약 29 GiB / 14 GiB |
| NVIDIA GPU / driver | Orin / 540.5.0 |
| 기존 CUDA / cuDNN | CUDA 12.6 (nvcc 12.6.68) / cuDNN 9.3.0.75 |
| 시스템 Python | /usr/bin/python3, 3.10.12 |
| 시스템 numpy / scipy / cv2 / yaml | 1.21.5 / 1.8.0 / 4.5.4 / 5.4.1 |
| ROS | Humble Desktop, /opt/ros/humble |
| 기본 NIC | eno1=192.168.50.184/24; 문서의 MID360 IP 192.168.1.184는 미확인 |

이번 GPU 전환은 기존 NVIDIA 드라이버/CUDA/cuDNN을 그대로 사용했습니다. 시스템 NumPy/OpenCV를 GPU venv 설치로 변경하지 않았습니다. 이전 보고서의 cv2 4.8.0과 달리 사용자 시스템 설치 이후 현재 시스템 Python은 Ubuntu cv2 4.5.4를 import합니다. 최신 결과는 logs/system_python_update.log와 dpkg_inventory_update.tsv를 우선합니다.

## 2. 사용 목적별 환경

| 목적 | 실제 환경 | 주요 버전 / 사용법 |
| --- | --- | --- |
| ROS / 카메라 / LiDAR / 보정 / PiPER ROS / colcon | 시스템 Python + /opt/ros/humble | 모델 venv를 activate하지 않고 공식 setup을 source |
| YOLO 검출·분할·지정 물체 학습 | ~/detection_ws/src/yolo_ros/yolo_ros/.venv | torch 2.8.0 GPU, torchvision 0.23.0, numpy 1.26.4, ultralytics 8.4.6 |
| SAM 2.1 / Jupyter | ~/venvs/sam21 | 같은 GPU torch/torchvision, SAM 2.1 tiny |
| TCP Pivot | ~/venvs/pivot | scikit-surgerycalibration 0.2.6; torch 불필요 |
| PiPER SDK | 시스템 Python + ~/robot_setup/piper-python | piper-sdk 0.6.2 / python-can 4.6.1, PYTHONPATH로 사용 |
| 손눈 보정 Python 의존성 | 시스템 Python + ~/robot_setup/ros-python | transforms3d 0.4.2 |
| YOLO colcon build 도구 | 시스템 Python + ~/robot_setup/colcon-python | setuptools 65.5.1; build 때만 사용 |

일반 python3에는 torch를 전역 설치하지 않았습니다. YOLO/SAM만 GPU torch를 사용하며 ROS/Pivot/SDK 환경에 torch를 추가할 필요가 없습니다. 자세한 activate/deactivate/터미널 명령은 [VIRTUAL_ENVIRONMENTS.md](VIRTUAL_ENVIRONMENTS.md)에 있습니다.

## 3. 도구별 현재 상태

| 도구 | 상태 | 실제 완료 / 남은 조건 |
| --- | --- | --- |
| GPU torch / torchvision | VERIFIED_SOFTWARE, VERIFIED_HARDWARE | 두 venv에서 CUDA=True, Orin sm_87, CUDA tensor/GPU NMS/실제 모델 추론 확인. 로봇 센서 확인과 구분 |
| 공식 uv / 시스템 cmake | VERIFIED_SOFTWARE | uv 0.12.22 / cmake 3.22.1 사용 |
| ROS Humble Desktop / RViz / rqt plugins / TF / rosbag2 / control | VERIFIED_SOFTWARE | 설치 확인, rclpy/cv_bridge import, talker/listener 수신, bag CLI 확인. GUI 화면은 미검증 |
| 남은 센서/이미지/basic apt | BLOCKED | 아래 15개 요청 패키지 미설치; 다운로드 cache는 준비. 사용자 sudo 인증 필요 |
| easy_handeye2 | VERIFIED_SOFTWARE, CONFIGURE_REQUIRED | 기준 source, easy_handeye2/easy_handeye2_msgs build, import/서비스/공식 launch 인자 확인. 실제 tag/feedback TF/보정 필요 |
| PiPER SDK / ROS core / URDF | VERIFIED_SOFTWARE, CONFIGURE_REQUIRED | SDK import, piper/piper_msgs/piper_description build, 읽기 executable 확인. 실제 CAN/firmware/old-new URDF/feedback 필요 |
| Livox SDK2 / ROS driver2 | VERIFIED_SOFTWARE, CONFIGURE_REQUIRED | SDK build/사용자 경로 설치, driver build/실행 파일/ldd 확인. 실제 NIC/IP/PointCloud2 입력 필요 |
| YOLO CLI / ROS | VERIFIED_SOFTWARE, CONFIGURE_REQUIRED | GPU 검출·분할, 메시지/bringup/node build, 네 entrypoint import와 GPU venv shebang 확인. 실제 Head/Wrist 입력 필요 |
| SAM 2.1 image predictor / Jupyter | VERIFIED_SOFTWARE | GPU point/box 실제 분할, tiny checkpoint/config, GPU kernel/작업용 공식 notebook 준비 |
| SAM 영상 notebook extras | BLOCKED | eva-decord에 Linux ARM64 배포 없음. 이미지 분할 환경은 준비 완료 |
| Pivot / TCP | VERIFIED_SOFTWARE, CONFIGURE_REQUIRED | 설치/공식 CLI help 확인. 실제 4×4 pose/TCP/지그/orientation 입력 필요 |
| Koide | CONFIGURE_REQUIRED, BLOCKED | 요청대로 소스·경로·공식 명령·입출력만 준비. humble Docker는 amd64 전용이므로 이 Jetson native 실행 경로 미정 |
| 지정 물체 학습 | CONFIGURE_REQUIRED | 데이터 구조/data.yaml/CLI 준비. 실제 class/split/label 없어서 fine-tuning 실행 안 함 |
| base/arm/2D LiDAR 장착 TF / TRACER odom | CONFIGURE_REQUIRED | 실측값/2D LiDAR 모델/차체 CAN/odom 출처 미확정. 차체 이동이 필요하지 않아 관련 driver 추가 설치 보류 |

## 4. 실제 GPU 실행 결과

| 실행 | 입력 / 결과 | 실제 산출물 |
| --- | --- | --- |
| YOLO detection | 공식 bus.jpg, person 4 + bus 1, CUDA:0 Orin. 단일 이미지 추론 273.8ms | ~/detection_data/results/yolo_detect_gpu/{bus.jpg,labels/bus.txt} |
| YOLO segmentation | 공식 bus.jpg, person 4 + bus 1 + stop sign 1, CUDA:0 Orin. 추론 291.2ms | ~/detection_data/results/yolo_segment_gpu/{bus.jpg,labels/bus.txt} |
| SAM point prompt | 공식 truck.jpg, masks (3,1200,1800), 최고 score 약 0.9520 | ~/detection_data/results/sam21_gpu/sam21_gpu_sample.npz |
| SAM box prompt | 같은 이미지, mask (1,1200,1800), score 약 0.9691 | 같은 npz, sam21_gpu_overlay.png, inference_summary.json |

SAM 모델 parameter가 cuda:0인 것을 확인했습니다. 이미지 임베딩 + point/box 두 프롬프트 처리는 약 1.84초였습니다. 위 시간은 최소 샘플 실행 기록이며 실시간 카메라의 지속 FPS/추론률 검증은 아닙니다. 이전 CPU 결과는 기존 결과 디렉터리에 그대로 보존했습니다.

GPU 패키지는 NVIDIA가 안내하는 [Jetson AI Lab CUDA 12.6 index](https://pypi.jetson-ai-lab.io/jp6/cu126/+simple/)에서 다운로드했습니다. 두 wheel의 registry SHA256을 대조했습니다. 파일은 ~/robot_setup/downloads/gpu-wheels에 보관하고 YOLO 공식 uv가 local wheel을 계속 사용하도록 pyproject에 일반 설정을 추가했습니다. 원래 numpy<2 / ultralytics==8.4.6 조건을 유지했습니다.

## 5. 실제 빌드 / ROS 검증

- ~/calibration_ws: easy_handeye2_msgs + easy_handeye2, 약 1분. official calibrate/publish --show-args, 서비스 인터페이스, system import 확인.
- ~/piper_ros: piper + piper_description + piper_msgs. 읽기 노드와 제조사 URDF/메시지 준비. MoveIt/시뮬레이션 패키지는 빌드하지 않음.
- ~/ws_livox: driver2, 약 2분 7초. 공식 ROS2 package.xml/colcon, 사용자 SDK prefix. warning은 unused variable 1건, build 성공. libapr1-dev는 아직 apt 미설치지만 현 driver build/ldd는 통과.
- ~/detection_ws: yolo_msgs + yolo_ros + yolo_bringup. 기존 setuptools가 console entrypoint를 만들지 못하는 문제를 build 전용 setuptools 65.5.1과 공식 일반 colcon install로 해결.
- yolo_node/debug_node/tracking_node/detect_3d_node가 설치돼 있고 실제 entrypoint import를 GPU 환경에서 확인. yolo_node shebang: ~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/python.
- ROS talker/listener는 별도 domain 229/local-only에서 실제 I heard 수신을 확인. 테스트 프로세스는 자신이 시작한 PID만 종료. TF/bag 도구와 메시지 인터페이스 확인. 로봇 제어 노드는 시작하지 않음.

## 6. 센서 / GUI 확인 범위

D435 USB ID 8086:0b07이 한 차례 관측됐지만 이후 마지막 USB 목록에는 없었습니다. 실제 serial/지원 profile/영상/depth/CameraInfo/TF/QoS는 확인하지 못했습니다. D405도 영상 입력을 확인하지 못했습니다. RealSense apt wrapper/SDK 설치를 완료하고 카메라를 연결한 다음 실제 값을 대조해야 합니다.

can0/can1은 onboard mttcan입니다. PiPER/TRACER 버스로 임의 지정하지 않았습니다. 실제 feedback/팔 firmware/URDF 대응은 남아 있습니다. MID360의 문서 IP=192.168.1.184는 NIC eno1=192.168.50.184/24와 별도이며 실제 센서인지 미확인입니다. 네트워크 기본 route 변경은 하지 않았습니다.

로그인 GUI session/X socket은 존재하지만 도구 환경의 DISPLAY/XAUTHORITY와 실제 화면 확인은 하지 않았습니다. RViz/rqt 설치·CLI와 화면 확인을 구분하며, GUI/OpenGL·로봇 영상/점군/feedback은 미검증입니다. 로봇 센서나 구동계를 VERIFIED_HARDWARE로 보고하지 않습니다.

## 7. 다운로드 / 경로 / 설정

- 공식 source 7개와 기준 SHA: [VERSIONS.md](VERSIONS.md). 기존 수정사항을 reset하지 않음.
- 모델: ~/detection_data/models/yolo11n.pt, yolo11n-seg.pt, sam2.1_hiera_tiny.pt. checksum과 공식 URL 기록.
- tag36h11 ID0 PNG: ~/calibration_ws/config/tag36_11_00000.png. 인쇄 후 검은 외곽 테두리 한 변[m] 실측 필수.
- Calibration: ~/calibration_ws/{src,config,results,logs}, ~/calibration_data/{head,wrist,tcp,base_arm,base_laser,head_mid360}.
- Koide: head_mid360/{bags,processed,validation,validation_processed}; native SDK extrinsic=0, ROS 결과 TF로 적용. min_distance 기본 1.0m 확인.
- Detection: ~/detection_data/{models,samples,images,results,bags,datasets/known_objects}. 데이터셋 raw/images·labels train/val/test와 names 비어 있는 data.yaml 준비.
- SAM notebook: ~/detection_data/samples/image_predictor_example_sam21_tiny.ipynb. kernel은 SAM 2.1 (Jetson GPU Python 3.10), device=cuda, tiny config/checkpoint 지정.

Head eye_on_base / Wrist eye_in_hand 설정을 분리했습니다. raw+CameraInfo→image_proc→AprilTag→marker TF 흐름과 수동 서비스/저장 명령을 RUN_COMMANDS에 준비했습니다. 기본 easy_handeye calibrate launch의 dummy TF를 피하도록 공식 server/UI 직접 명령을 사용합니다. 실제 결과 없는 calibration publisher/외부 TF는 실행하지 않았습니다.

RealSense serial underscore 지원과 D405 color profile 인자를 다운로드한 wrapper와 같은 release source에서 확인했습니다. Livox 공식 포트/PointCloud2 xfer_format=0/livox_frame 설정을 준비했습니다. 16UC1 depth는 mm/1000, 32FC1은 m/divisor=1로 구분합니다. enable_sync를 두 카메라 하드웨어 동기화로 취급하지 않습니다.

Koide T_lidar_camera는 camera→lidar, xyz+xyzw이며 camera parent/livox child에는 역변환이 필요합니다. 실제 결과가 없으므로 최종 TF를 만들지 않았습니다. 뒤집힌 장착 회전을 driver와 TF에 이중 적용하지 않습니다.

## 8. 남은 문제와 재개 조건

**아직 설치되지 않은 요청 apt 15개:**

```text
python3-pip
python3-venv
nano
jq
mesa-utils
python3-transforms3d
ros-humble-rqt
ros-humble-image-view
ros-humble-image-proc
ros-humble-camera-calibration
ros-humble-realsense2-camera
ros-humble-realsense2-description
ros-humble-apriltag-ros
ros-humble-vision-msgs
libapr1-dev
```

의존성 .deb 약 427 MiB는 이미 다운로드했습니다. 현재 전체 요청 목록 기준 apt 시뮬레이션은 7개 upgrade / 40개 신규 / 제거 0개입니다. sudo 인증은 아직 password required이며 남은 apt 설치만 사용자 터미널 인증이 필요합니다. 구체적인 설치 명령은 README_SETUP에 있습니다. 비밀번호 저장/요청, sudoers 변경은 하지 않았습니다.

추가 실제 입력: D435/D405 serial·USB profile, tag 크기, PiPER/TRACER CAN, firmware/URDF/gripper, MID360 NIC/host/sensor IP, 장착/base↔arm/base↔2D LiDAR/TCP 값, 2D LiDAR 모델, 목표 class와 촬영 session별 split/label. [CONFIG_REQUIRED.md](CONFIG_REQUIRED.md)에 이유와 입력 경로를 기록했습니다.

SAM 전체 notebooks extras의 eva-decord는 Linux ARM64 배포가 없어서 영상 notebook 의존성만 BLOCKED입니다. image predictor/Jupyter/GPU 분할은 준비 완료. SAM CUDA extension은 기존 미빌드 상태를 유지하며 선택적 후처리 일부가 빠질 수 있으나 모델은 GPU에서 실행됩니다. SAM ROS publisher/training 환경은 추가하지 않았습니다.

Koide는 설정만 준비했으며 공식 humble Docker image는 amd64 전용입니다. registry digest는 VERSIONS에 기록했고 local image digest로 보고하지 않습니다. 기존 amd64 Koide 환경 또는 별도 ARM64 호환 실행 경로에서 사용합니다. Docker/Koide 실습/native heavy build/에뮬레이션은 하지 않았습니다.

Calibration/Detection Notion 페이지는 접근 404였으므로 하위 가이드는 읽지 못했습니다. 제공한 상세 요구사항과 공식 source/documentation으로 진행했습니다.

## 9. 다음에 가장 먼저 실행할 명령

지금 바로 GPU YOLO 샘플을 사용할 때:

```bash
source ~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/activate
yolo predict model=$HOME/detection_data/models/yolo11n.pt source=$HOME/detection_data/samples/bus.jpg device=0
```

GPU SAM notebook을 사용할 때:

```bash
~/venvs/sam21/bin/jupyter lab --ip=127.0.0.1 --no-browser --notebook-dir=/home/sparo/detection_data/samples
```

로봇 센서 입력을 진행하려면 먼저 사용자 터미널에서 남은 apt를 설치합니다:

```bash
sudo -v
mapfile -t remaining_packages < ~/robot_setup/apt-cache/remaining_packages.txt
sudo apt-get -c ~/robot_setup/apt-cache/apt-download.conf --no-download install "${remaining_packages[@]}"
```

이후 실제 입력을 채우고 [RUN_COMMANDS.md](RUN_COMMANDS.md)의 터미널별 source/공식 launch/CLI를 사용합니다. Jupyter 인증은 유지하고 로봇 노드의 중복 실행과 외부 TF 중복 발행을 피합니다.

## 10. 주요 로그 / 보존

| 로그 | 확인 내용 |
| --- | --- |
| logs/gpu_platform_selection.json / gpu_wheel_metadata.log / gpu_wheel_sha256.json | 실제 JetPack/CUDA 선택과 wheel checksum |
| logs/sam21_gpu_install.log / sam21_gpu_probe.log / sam21_gpu_inference.log | SAM GPU 설치/CUDA 실제 연산/분할 |
| logs/yolo_gpu_uv_sync.log / yolo_gpu_probe.log / yolo_gpu_detection.log / yolo_gpu_segmentation.log | YOLO GPU 전환과 실제 추론 |
| logs/yolo_gpu_packages.txt / sam21_gpu_packages.txt / pivot_packages.txt | 목적별 실제 package 목록 |
| logs/calibration_build.log / piper_build.log / piper_msgs_build.log / livox_driver_build.log | 핵심 workspace 실제 build |
| logs/yolo_build.log / yolo_entrypoint_build.log / yolo_ros_interpreter.log / yolo_ros_gpu_entrypoint_check.log | YOLO message/bringup/build·네 entrypoint·GPU interpreter |
| logs/ros_software_check_after_build.log / *_args.log / ros_listener_minimal.log | ROS import/서비스/launch 인자/실제 수신 |
| logs/current_requested_apt.tsv / dpkg_inventory_update.tsv / apt_remaining_install_simulation.log | 현재 설치·미설치 apt와 재개 계획 |
| logs/hardware_inventory_update.log / system_python_update.log / final_setup_state.json | 최종 host/USB/CAN/Python/상태 |
| logs/gpu_migration_backup/ | GPU 전환 전 일반 설정·문서·CPU lock/notebook 백업 |

로그 전체: /home/sparo/robot_setup/logs/. 초기 *_attempt*와 CPU inference 로그는 과거 작업 기록입니다. 최신 GPU/build/apt 상태 로그를 우선합니다.

실제 데이터·보정 결과·기존 정상 SDK/드라이버를 reset하지 않았습니다. 자체 .sh/env.sh/start_*.sh/runner/수집기/custom launch는 만들지 않았습니다. 일반 설정·문서·공식 notebook 작업용 복사본만 작성했습니다. 로봇 enable/관절 목표값/차체 이동/자동 자세 수집/사용자 물체 본학습을 실행하지 않았습니다. Nav2/MoveIt/FAST-LIO/GraspNet/MID360 영상 융합/자동 접근은 후속 범위입니다.

문서 바로가기:

- [설치 재현과 남은 apt](README_SETUP.md)
- [다음 날 실행 명령](RUN_COMMANDS.md)
- [사용별 가상환경](VIRTUAL_ENVIRONMENTS.md)
- [필요한 실제 입력](CONFIG_REQUIRED.md)
- [실제 버전 / SHA / checksum](VERSIONS.md)

## 11. GitHub 계정 연결

VERIFIED_SOFTWARE: GitHub 계정 [seoneum](https://github.com/seoneum)의 브라우저 인증을 완료했습니다. 인증된 GitHub API로 계정 ID 82085202를 확인했습니다. Git 2.34.1은 재사용하고 공식 GitHub CLI 2.102.0 ARM64를 ~/.local/bin/gh에 설치했습니다.

Git 전역 작성자는 KIM SEON UEM / 82085202+seoneum@users.noreply.github.com이며, HTTPS credential helper를 gh auth git-credential로 설정했습니다. 인증은 시스템 keyring에 저장됩니다.

상태 확인: `gh auth status`. Git/CLI 버전은 [VERSIONS.md](VERSIONS.md)에 기록했습니다. 로컬 상세 문서는 ~/robot_setup/GIT_SETUP.md, 확인 로그는 ~/robot_setup/logs/git_setup_verification.log입니다.
