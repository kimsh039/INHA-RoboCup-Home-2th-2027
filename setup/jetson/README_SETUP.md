# Calibration + Detection 설치 상태와 재현

> 이 문서는 2026-10-03 Jetson `/home/sparo/robot_setup/` 기록의 공유 사본입니다. 모델·venv·로그·설정 파일은 Jetson 로컬 경로를 사용합니다. [문서 모음](README.md) · [프로젝트 홈](../../README.md)

최종 업데이트: 2026-10-03. Ubuntu 22.04.5 / JetPack 6.2.3 / L4T 36.5.2 / ARM64 Orin / 시스템 Python 3.10.12 / CUDA 12.6.

**YOLO·SAM GPU 전환과 네 workspace의 핵심 패키지 build를 완료했습니다.** 이전 보고서 이후 설치된 ROS Humble Desktop을 재사용했습니다. ROS·torch의 전체 상태는 SETUP_REPORT.md, 버전/체크섬은 VERSIONS.md, 다음 날 직접 실행은 RUN_COMMANDS.md에 있습니다.

요청한 센서/이미지 도구 등 apt 일부는 미설치 상태입니다. GPU 전환 자체는 sudo 없이 완료했으며 남은 apt 설치에만 사용자 인증이 필요합니다. 실제 serial/CAN/태그 크기/장착/TCP/class는 CONFIG_REQUIRED를 먼저 채웁니다.

## 1. 남은 공통 apt 설치: 한 번

공식 Jammy ARM64 apt index와 요청 패키지/의존성 .deb 약 427 MiB를 다운로드해 두었습니다. 현재 설치된 항목은 logs/current_requested_apt.tsv, 아직 없는 항목은 apt-cache/remaining_packages.txt입니다. 이미 설치된 ROS Desktop을 다시 설치하거나 전체 PC upgrade를 하지 않습니다.

사용자 터미널에서 같은 세션으로 실행합니다. 비밀번호는 터미널에만 입력합니다.

```bash
sudo -v
mapfile -t remaining_packages < ~/robot_setup/apt-cache/remaining_packages.txt
sudo apt-get -c ~/robot_setup/apt-cache/apt-download.conf --no-download install "${remaining_packages[@]}"
```

현재 재개 계획은 `apt-cache/requested_packages.txt` 전체 기준 7개 upgrade, 40개 신규, 제거 0개입니다. remaining 목록만 설치하면 필요 의존성에 맞춰 apt가 계산합니다. 목록·cache가 오래되었으면 현재 정상 ROS 공식 저장소에서 `sudo apt update` 후 `sudo apt install "${remaining_packages[@]}"`로 진행합니다. ros2-apt-source 공식 .deb는 downloads/ros2-apt-source.deb에 보관했으며 이미 정상 ROS source가 있는 시스템에 중복 등록하지 않습니다.

UTF-8 locale/Universe는 정상입니다. 시스템 numpy 1.21.5 / scipy 1.8.0 / cv2 4.5.4 / YAML 5.4.1을 확인했습니다. 이전 기록의 NVIDIA cv2 4.8.0과 달리 현재 시스템 import는 사용자 시스템 설치 이후 Ubuntu cv2 4.5.4를 사용합니다. 이번 GPU 작업에서 시스템 OpenCV/NumPy/드라이버를 바꾸지 않았습니다. sudo pip와 전역 NumPy 2 업그레이드를 사용하지 않습니다.

[ROS 공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)의 systemd/udev 관련 조건은 확인했습니다. 초기 버전이 아닌 기존 구성을 재사용하며 전체 upgrade/재부팅/펌웨어 변경은 하지 않았습니다.

## 2. 공식 소스와 모델: 준비 완료

| 구성 | 실제 경로 |
| --- | --- |
| easy_handeye2 | ~/calibration_ws/src/easy_handeye2 |
| PiPER humble | ~/piper_ros |
| Livox SDK2 | ~/robot_setup/downloads/Livox-SDK2 |
| Livox ROS driver2 | ~/ws_livox/src/livox_ros_driver2 |
| Koide 기준 소스 | ~/robot_setup/downloads/direct_visual_lidar_calibration |
| YOLO ROS | ~/detection_ws/src/yolo_ros |
| SAM2 | ~/detection_tools/sam2 |

기준 SHA는 VERSIONS에 있습니다. reset하지 않았습니다. YOLO에는 GPU wheel을 고정하는 pyproject.toml의 일반 `[tool.uv]` 설정만 추가했으며 원래 numpy<2/ultralytics==8.4.6 조건을 유지했습니다. Git diff는 logs/yolo_gpu_source_changes.diff입니다.

모델: ~/detection_data/models/{yolo11n.pt,yolo11n-seg.pt,sam2.1_hiera_tiny.pt}. 이미 성공한 파일은 재사용합니다. 공식 URL/체크섬은 VERSIONS에 있습니다. tag36h11 ID0: ~/calibration_ws/config/tag36_11_00000.png. 원본 10×10 pixel은 인쇄 확대 시 nearest-neighbor 방식으로 사용하고 검은 외곽 테두리 한 변의 실제 크기[m]를 입력합니다.

## 3. Jetson GPU PyTorch: 실제 설치·추론 완료

GPU wheel은 [NVIDIA가 안내하는 Jetson AI Lab index](https://pypi.jetson-ai-lab.io/jp6/cu126/+simple/)의 Python 3.10 / ARM64 / CUDA 12.6 버전입니다.

- torch 2.8.0 + torchvision 0.23.0
- CUDA build 12.6 / Orin sm_87 / 기존 cuDNN 9.3.0
- YOLO·SAM venv 모두 CUDA=True, CUDA tensor 및 GPU NMS 확인
- YOLO 검출·분할과 SAM point·box 실제 GPU 추론 통과

wheel 위치: ~/robot_setup/downloads/gpu-wheels/. URL과 SHA256은 VERSIONS/다운로드 기록에 있습니다. YOLO pyproject의 local file URL은 이 파일을 참조하므로 보관해야 합니다. CPU index uv.toml은 백업 경로로 이동했고 현재 build가 CPU wheel로 되돌아가지 않도록 고정했습니다.

## 4. ROS dependency / Calibration build

ROS 환경과 apt rosdep source/cache가 준비되어 있습니다. rosdep 작업은 apt installer만 사용하며 이미 venv에 설치한 모델 Python 의존성을 전역 pip로 설치하지 않습니다.

```bash
source /opt/ros/humble/setup.bash
rosdep update --rosdistro humble
cd ~/calibration_ws
rosdep install --from-paths src --ignore-src --rosdistro humble --filter-for-installers apt --skip-keys 'python-transforms3d-pip python3-opencv' -r -y
```

현재 transforms3d는 시스템 Python이 사용할 ~/robot_setup/ros-python에 설치해 import했습니다. apt python3-transforms3d가 설치되면 apt 구성을 사용할 수도 있습니다. 재현:

```bash
uv pip install --python /usr/bin/python3 --target ~/robot_setup/ros-python --no-deps transforms3d
export PYTHONPATH="$HOME/robot_setup/ros-python${PYTHONPATH:+:$PYTHONPATH}"
cd ~/calibration_ws
set -o pipefail
colcon build --symlink-install --executor sequential 2>&1 | tee ~/robot_setup/logs/calibration_build.log
```

실제 easy_handeye2/easy_handeye2_msgs build, system import, 공식 launch --show-args와 서비스 인터페이스를 확인했습니다. 기본 calibrate launch의 dummy TF를 피하도록 RUN_COMMANDS에는 공식 server/UI 직접 실행을 준비했습니다. 외부 보정 결과가 없어 publisher는 실행하지 않았습니다.

## 5. PiPER: SDK import와 핵심 패키지 build 완료

piper-sdk 0.6.2 / python-can 4.6.1은 ~/robot_setup/piper-python에 설치했습니다. 시스템 site-packages를 덮어쓰지 않고 /usr/bin/python3에서 import했습니다. MoveIt/시뮬레이션 전체 requirements는 설치하지 않았습니다.

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
cd ~/piper_ros
rosdep install --from-paths src/piper src/piper_msgs src/piper_description --ignore-src --rosdistro humble --filter-for-installers apt -r -y
colcon build --symlink-install --packages-select piper piper_msgs piper_description --executor sequential
```

실제 piper/piper_msgs/piper_description build 완료. piper_read_slave_joint 및 SDK ConnectPort/PiperInit source를 확인했습니다. CAN/firmware는 미확정이라 연결/enable/목표값 전송은 하지 않았습니다. 읽기 출력만 /piper/joint_states_feedback으로 remap하고 제조사 firmware 대응 URDF를 선택합니다.

## 6. Livox SDK2 / ROS driver2: build 완료

SDK 설치 prefix: ~/robot_setup/livox-sdk2. SDK shared/static library/header 및 driver의 ldd 의존성을 확인했습니다. 시스템 SDK를 덮어쓰지 않았습니다.

```bash
cmake -S ~/robot_setup/downloads/Livox-SDK2 -B ~/robot_setup/downloads/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$HOME/robot_setup/livox-sdk2"
cmake --build ~/robot_setup/downloads/Livox-SDK2/build -j4
cmake --install ~/robot_setup/downloads/Livox-SDK2/build
source /opt/ros/humble/setup.bash
cd ~/ws_livox/src/livox_ros_driver2
test -e package.xml || cp package_ROS2.xml package.xml
cd ~/ws_livox
colcon build --symlink-install --executor sequential --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DLIVOX_LIDAR_SDK_LIBRARY="$HOME/robot_setup/livox-sdk2/lib/liblivox_lidar_sdk_shared.so" -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$HOME/robot_setup/livox-sdk2/include" -DCMAKE_BUILD_TYPE=Release
```

삭제하는 upstream build.sh를 읽고 일반 colcon 방식으로 build했습니다. 실행 시 LD_LIBRARY_PATH에 SDK lib 경로를 추가합니다. host/sensor IP는 아직 placeholder이며 기본 route는 변경하지 않았습니다.

## 7. YOLO: 공식 uv + ROS GPU 노드 준비 완료

uv 0.12.22: ~/.local/bin. 내부 venv: ~/detection_ws/src/yolo_ros/yolo_ros/.venv, 시스템 Python 3.10 / system-site-packages=true.

```bash
export PATH="$HOME/.local/bin:$PATH"
cd ~/detection_ws/src/yolo_ros/yolo_ros
uv sync --no-install-project --no-dev
```

Ubuntu 기존 setuptools에서는 pinned pyproject의 console_scripts가 생성되지 않았습니다. build 전용 사용자 경로에 setuptools 65.5.1을 설치하고 **YOLO는 일반 colcon install 방식**으로 build하여 official setup.py가 script interpreter를 내부 GPU venv로 설정하도록 했습니다. 시스템 setuptools를 덮어쓰지 않았고 자체 runner를 만들지 않았습니다.

```bash
uv pip install --python /usr/bin/python3 --target ~/robot_setup/colcon-python setuptools==65.5.1
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/robot_setup/colcon-python${PYTHONPATH:+:$PYTHONPATH}"
cd ~/detection_ws
rosdep install --from-paths src --ignore-src --rosdistro humble --filter-for-installers apt -r -y
colcon build --executor sequential
```

yolo_msgs/yolo_bringup는 최초 symlink build로 준비했고 yolo_ros는 위 일반 방식으로 재빌드했습니다. yolo_node/debug_node/tracking_node/detect_3d_node 네 실행 파일의 entrypoint import와 yolo_node의 GPU venv shebang을 확인했습니다. 다른 workspace를 build할 때 colcon-python을 필수로 사용할 필요는 없습니다.

## 8. SAM 2.1 / Jupyter / Pivot

SAM 환경: ~/venvs/sam21. 기존 editable SAM source와 image notebook 패키지는 유지하고 GPU wheel을 적용했습니다. GPU로 처음 설치를 재현하려면:

```bash
uv venv --python /usr/bin/python3 --seed ~/venvs/sam21
uv pip install --python ~/venvs/sam21/bin/python ~/robot_setup/downloads/gpu-wheels/torch-2.8.0-cp310-cp310-linux_aarch64.whl ~/robot_setup/downloads/gpu-wheels/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl 'numpy<2'
SAM2_BUILD_CUDA=0 uv pip install --python ~/venvs/sam21/bin/python --no-build-isolation -c ~/robot_setup/sam21.constraints.txt -e ~/detection_tools/sam2 'matplotlib>=3.9.1' jupyter jupyterlab ipykernel 'opencv-python>=4.7.0' PyYAML
~/venvs/sam21/bin/python -m ipykernel install --user --name sam21 --display-name 'SAM 2.1 (Jetson GPU Python 3.10)'
```

기존 venv는 재생성하지 않고 재사용합니다. 공식 image_predictor_example.ipynb의 작업용 복사본은 ~/detection_data/samples/image_predictor_example_sam21_tiny.ipynb. tiny checkpoint/config, CUDA device, SAM GPU kernel, 공식 이미지의 절대 경로를 지정했습니다.

SAM 전체 notebooks extras의 eva-decord는 Linux ARM64 배포가 없어 BLOCKED입니다. core/image notebook과 GPU 추론은 완료했습니다. CUDA extension은 이전 SAM2_BUILD_CUDA=0을 유지하여 일부 선택적 후처리가 생략되며 모델 본체는 CUDA로 실행합니다. training 의존성과 ROS publisher는 추가하지 않았습니다.

Pivot 환경은 ~/venvs/pivot.scikit-surgerycalibration 0.2.6, CLI help 확인 완료. 4×4 pose matrix 파일을 디렉터리에 넣고 sksPivotCalibration -i로 사용합니다. 실제 TCP 데이터가 없어 계산하지 않았습니다. Pivot/ROS/PiPER SDK에는 torch를 추가할 필요가 없습니다.

## 9. Koide: 요청대로 설정만

기준 소스와 head_mid360의 bags/processed/validation/validation_processed 경로를 준비했습니다. 공식 기록/전처리/초기화/최적화/viewer 명령, min_distance 기본 1.0m, T_lidar_camera의 camera→lidar 및 xyz+xyzw 순서를 확인했습니다.

공식 humble Docker image는 현재 amd64 전용입니다. ARM64 호환 실행 경로나 기존 amd64 PC에서 사용합니다. 이 Jetson에서는 Docker/image/Koide 실행·에뮬레이션·무거운 native build를 하지 않았습니다. digest는 VERSIONS에 registry 값으로 기록했습니다. [공식 Docker 안내](https://koide3.github.io/direct_visual_lidar_calibration/docker/) / [프로그램 안내](https://koide3.github.io/direct_visual_lidar_calibration/programs/).

## 10. 보존과 범위

실제 data/보정 결과/기존 정상 SDK/드라이버를 reset하지 않았습니다. 일반 설정·문서·공식 notebook 작업용 복사본만 작성했습니다. 자체 .sh/env.sh/start_*.sh/runner/수집기/custom ROS launch는 없습니다. tag·장착·TCP·class 실제값을 추정하지 않았고 최종 TF/보정/사용자 학습 결과는 만들지 않았습니다.

Nav2/MoveIt/FAST-LIO/GraspNet/SAM ROS publisher/MID360 영상 투영·융합/자동 접근은 후속 범위입니다. 로봇 enable/이동/자동 자세 수집은 하지 않았습니다.
