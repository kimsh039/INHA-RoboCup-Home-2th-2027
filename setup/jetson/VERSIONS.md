# 실제 설치 버전 / Git SHA / 체크섬

> 이 문서는 2026-10-03 Jetson `/home/sparo/robot_setup/` 기록의 공유 사본입니다. 모델·venv·로그·설정 파일은 Jetson 로컬 경로를 사용합니다. [문서 모음](README.md) · [프로젝트 홈](../../README.md)

2026-10-03 GPU 전환 후 상태입니다. 과거 CPU 결과는 로그에 남기고 현재 GPU 버전과 구분합니다.

| 항목 | 실제 값 |
| --- | --- |
| OS / architecture | Ubuntu 22.04.5 Jammy / aarch64, dpkg arm64 |
| kernel / L4T / JetPack | 5.15.199-tegra / 36.5.2 / 6.2.3+b81 |
| 시스템 Python | /usr/bin/python3, 3.10.12 |
| NVIDIA GPU / driver | Orin / 540.5.0; 변경 없음 |
| CUDA / nvcc | 12.6 / V12.6.68; 기존 toolkit 재사용 |
| 기존 cuDNN / torch 조회 | apt libcudnn9-cuda-12 9.3.0.75-1 / torch backend 90300 |
| ROS Humble Desktop | 0.10.0-1jammy.20260910.003916, /opt/ros/humble |
| RViz / cv_bridge | 11.2.29-1jammy.20260908.073726 / 3.2.1-1jammy.20260907.222938 |
| colcon / rosdep | python3-colcon-common-extensions 0.3.0-100 / python3-rosdep 0.27.0-1 |
| cmake | 시스템 3.22.1-1ubuntu1.22.04.2; SDK 초기 build는 사용자 경로에 추출한 binary 사용 |
| uv | 0.12.22, ~/.local/bin |
| 시스템 NumPy/SciPy/OpenCV/YAML | 1.21.5 / 1.8.0 / 4.5.4 / 5.4.1 |
| OpenCV 실제 경로 | /usr/lib/python3/dist-packages/cv2.cpython-310-aarch64-linux-gnu.so |
| YOLO build 전용 setuptools | 65.5.1, ~/robot_setup/colcon-python |
| Docker / Koide image | 미설치·미다운로드. Koide는 설정만 |

최신 apt 전체: logs/dpkg_inventory_update.tsv.요청 apt의 실제 상태: logs/current_requested_apt.tsv.이전 inventory는 과거 상태입니다. 이전 cv2=4.8.0과 달리 이후 사용자 시스템 설치를 거쳐 현재 import는 4.5.4입니다. 이번 GPU 작업에서 시스템 OpenCV/NumPy를 변경하지 않았습니다.

## Python 환경

| 환경 | 주요 실제 버전 / 결과 |
| --- | --- |
| YOLO .venv | Python 3.10.12, system-site-packages=true, torch 2.8.0 / torchvision 0.23.0 GPU, CUDA 12.6, arch sm_87, numpy 1.26.4, ultralytics 8.4.6, cv2 4.11.0 |
| ~/venvs/sam21 | torch 2.8.0 / torchvision 0.23.0 GPU, CUDA 12.6, numpy 1.26.4, sam-2 1.0 editable (pinned source) |
| ~/venvs/pivot | scikit-surgerycalibration 0.2.6, numpy 1.26.4; torch 없음 |
| ~/robot_setup/piper-python | piper-sdk 0.6.2, python-can 4.6.1; system Python에서 import |
| ~/robot_setup/ros-python | transforms3d 0.4.2; system calibration import 성공 |

두 GPU venv에서 torch.cuda.is_available()=True, CUDA tensor/GPU NMS/실제 모델 추론을 확인했습니다. YOLO ROS의 네 entrypoint import도 GPU 환경에서 통과했습니다. yolo_node의 shebang은 ~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/python입니다.

전체 package: logs/yolo_gpu_packages.txt / sam21_gpu_packages.txt / pivot_packages.txt.이전 yolo_packages.txt/sam21_packages.txt는 CPU 전환 전 기록입니다. SAM CUDA extension은 미빌드 (기존 SAM2_BUILD_CUDA=0 유지). 선택적 후처리 일부를 생략하며 모델 본체는 GPU로 실행합니다.

## 실제 checkout SHA

| repository | actual path | HEAD SHA |
| --- | --- | --- |
| easy_handeye2 | ~/calibration_ws/src/easy_handeye2 | 29bd50a7939e853d768d203d2abc61eb6e50facc |
| piper_ros humble | ~/piper_ros | 017ffefa64511bc6325bd77ddc4e16065c152051 |
| Livox-SDK2 | ~/robot_setup/downloads/Livox-SDK2 | c0796f04c143143899c87a773d9f6b7136453c0b |
| livox_ros_driver2 | ~/ws_livox/src/livox_ros_driver2 | 21445540f0d100dc86a7e6df312dd70bbdb4afdf |
| Koide | ~/robot_setup/downloads/direct_visual_lidar_calibration | 02a0dc039f5509708f384be4ff3228e0ae09352d |
| yolo_ros | ~/detection_ws/src/yolo_ros | c6d29ce65da555b125517db6ac53729320422b2b |
| sam2 | ~/detection_tools/sam2 | 2b90b9f5ceec907a1c18123530e92e794ad901a4 |

YOLO local config 차이: logs/yolo_gpu_source_changes.diff.기준 commit에 pyproject의 일반 Jetson GPU wheel 고정 설정을 추가했습니다. numpy<2 / ultralytics==8.4.6은 유지. CPU uv.toml은 logs/gpu_migration_backup으로 이동하고 uv.lock은 공식 uv가 재생성했습니다. Livox package.xml은 공식 package_ROS2.xml 복사본입니다.

## Model / tag / GPU wheel SHA256

| 실제 파일 | SHA256 |
| --- | --- |
| /home/sparo/detection_data/models/yolo11n.pt | 0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1 |
| /home/sparo/detection_data/models/yolo11n-seg.pt | 55ed65c56c91713d23e8402371c6c49a6fd84f257f7dce452e8d70e41dcbe152 |
| /home/sparo/detection_data/models/sam2.1_hiera_tiny.pt | 7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69 |
| /home/sparo/calibration_ws/config/tag36_11_00000.png | 48d811f770fc3595aaf5650a5fd8007843e7fece1f8fdcb94e0f113b4c9ed0c6 |
| /home/sparo/robot_setup/downloads/gpu-wheels/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl | 907c4c1933789645ebb20dd9181d40f8647978e6bd30086ae7b01febb937d2d1 |
| /home/sparo/robot_setup/downloads/gpu-wheels/torch-2.8.0-cp310-cp310-linux_aarch64.whl | 62a1beee9f2f147076a974d2942c90060c12771c94740830327cae705b2595fc |

모델 공식 URL:

- https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n.pt
- https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n-seg.pt
- https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt
- https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00000.png

GPU index: https://pypi.jetson-ai-lab.io/jp6/cu126/+simple/
GPU wheel URL/hash 원본: downloads/gpu-wheel-urls.txt, logs/gpu_wheel_sha256.json, logs/gpu_wheel_metadata.log. registry hash 대조를 통과했습니다. YOLO pyproject가 보관한 local wheel을 참조하므로 해당 파일을 유지합니다.

## Koide registry 기록

humble tag: linux/amd64만 제공. 크기1,191,336,178 bytes.
registry digest: `sha256:f7363cc3deadc2419f3527a7b9ec7bdabaa3031fd8e1b9c6cc46666ae1301bbb`.
원본downloads/koide_humble_tag.json.다운로드한 로컬 이미지 digest가 아닙니다. ARM64에서 pull/실행하지 않았으며 tag build와 기준 source SHA가 같다고 가정하지 않습니다. 요청대로 설정만 준비했습니다.

## Git / GitHub CLI

- Git 2.34.1 (기존 설치)
- GitHub CLI 2.102.0 linux_arm64: /home/sparo/.local/bin/gh
- 공식 release archive SHA256: 7862c86c72f43df3a2d93ddde6f473285b4e2af61b494849846827e513ef6484
- GitHub 인증 계정: seoneum (ID 82085202), HTTPS / keyring
- 확인 로그: logs/git_setup_verification.log
