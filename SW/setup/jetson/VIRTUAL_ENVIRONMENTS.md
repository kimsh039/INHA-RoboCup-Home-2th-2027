# 사용 목적별 Python 환경

> 이 문서는 2026-10-03 Jetson `/home/sparo/robot_setup/` 기록의 공유 사본입니다. 모델·venv·로그·설정 파일은 Jetson 로컬 경로를 사용합니다. [문서 모음](README.md) · [프로젝트 홈](../../README.md)

2026-10-03 GPU 전환 후 실제 구성입니다. 한 터미널에 여러 venv를 겹쳐 activate하지 않습니다. 다른 환경으로 바꿀 때 deactivate하거나 새 터미널을 사용합니다.

| 목적 | 실제 환경 | 상태 |
| --- | --- | --- |
| ROS/센서/보정/PiPER 노드/colcon | /usr/bin/python3 + /opt/ros/humble | ROS Desktop 및 핵심 workspace build 완료; 센서 apt 일부 미설치 |
| YOLO CLI 검출/분할/학습 | ~/detection_ws/src/yolo_ros/yolo_ros/.venv | torch 2.8.0 / torchvision 0.23.0 / CUDA 12.6 / Orin sm_87 / ultralytics 8.4.6 |
| SAM 2.1 image predictor / Jupyter | ~/venvs/sam21 | 같은 GPU torch/torchvision, tiny checkpoint, GPU point/box 추론 통과 |
| TCP Pivot | ~/venvs/pivot | scikit-surgerycalibration 0.2.6; torch 사용 없음 |
| PiPER SDK | 시스템 Python + ~/robot_setup/piper-python | 별도 venv 없음; PYTHONPATH로 piper-sdk 0.6.2 / python-can 4.6.1 |
| transforms3d | 시스템 Python + ~/robot_setup/ros-python | 손눈 보정 서버 의존성, 실제 import 통과 |
| YOLO ROS build 도구 | 시스템 Python + ~/robot_setup/colcon-python | setuptools 65.5.1; build 때만 PYTHONPATH 추가 |

## YOLO CLI

```bash
source ~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/activate
yolo predict model=$HOME/detection_data/models/yolo11n.pt source=$HOME/detection_data/samples/bus.jpg device=0
# 학습은 실제 class/split/label 준비 후 RUN_COMMANDS 참조
deactivate
```

YOLO ROS 실행은 venv를 activate하지 않고 ROS와 detection workspace를 source합니다. 공식 colcon 생성 노드의 shebang이 위 GPU venv를 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/detection_ws/install/local_setup.bash
# 카메라 apt 설치·실제 입력 준비 뒤 RUN_COMMANDS의 공식 launch 사용
```

## SAM Jupyter

```bash
source ~/venvs/sam21/bin/activate
jupyter lab --ip=127.0.0.1 --no-browser --notebook-dir=$HOME/detection_data/samples
```

브라우저에서 인증 token이 있는 로컬 URL을 열고 image_predictor_example_sam21_tiny.ipynb를 선택합니다. kernel은 **SAM 2.1 (Jetson GPU Python 3.10)**, device는 cuda입니다. Ctrl+C로 서버를 종료하고 deactivate합니다. image notebook은 준비됐고 영상용 eva-decord extras는 ARM64 미지원입니다.

## Pivot

```bash
source ~/venvs/pivot/bin/activate
sksPivotCalibration --help
# 실제 4×4 pose matrix 디렉터리가 준비되면 -i 경로 지정
deactivate
```

## 시스템 ROS / SDK

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/robot_setup/ros-python:$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$HOME/robot_setup/livox-sdk2/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

해당 작업에 필요한 공식 workspace setup만 추가 source합니다. 전체 순서는 RUN_COMMANDS에 있습니다. ROS/SDK/Pivot에는 GPU PyTorch가 필요하지 않아 torch를 전역으로 설치하지 않았습니다. YOLO/SAM GPU 환경을 ROS의 colcon build Python으로 선택하지 않습니다.

YOLO 재빌드에만 setuptools prefix를 추가합니다:

```bash
source /opt/ros/humble/setup.bash
export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$HOME/robot_setup/colcon-python${PYTHONPATH:+:$PYTHONPATH}"
cd ~/detection_ws
colcon build --executor sequential
```

## 현재 환경의 GPU 여부 확인

```bash
python -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available(), torch.cuda.get_device_name(0))'
```

위 명령은 활성화한 YOLO 또는 SAM venv에서 사용합니다. 시스템 Python에는 torch가 없습니다. 설치·추론 로그와 전체 패키지 목록은 logs/yolo_gpu_packages.txt, sam21_gpu_packages.txt에 있습니다.

## 자주 헷갈리는 경우

| 증상 | 먼저 확인할 것 | 조치 |
| --- | --- | --- |
| 시스템 python3에서 torch import 실패 | `which python3`가 /usr/bin/python3인지 | GPU torch는 YOLO/SAM venv에 설치돼 있습니다. 해당 환경을 activate |
| torch는 있는데 CUDA=False | `which python`, torch 버전/CUDA build | 맞는 venv인지 확인. JetPack 6 / CUDA 12.6 ARM64 wheel 고정 유지 |
| ROS에서 cv_bridge/rclpy import 실패 | 활성 venv와 source 순서 | 새 터미널에서 Humble과 필요한 overlay source. YOLO venv는 system-site-packages 사용 |
| ROS yolo_node의 interpreter가 다름 | 설치된 실행 파일의 첫 줄 | README_SETUP의 setuptools prefix + 일반 colcon install 절차 확인 |
| uv sync가 local wheel을 찾지 못함 | pyproject의 file URL과 downloads/gpu-wheels | /home/sparo 경로의 wheel 보존. 다른 사용자 경로로 이식하면 참조 수정 |
| Jupyter에서 SAM import가 안 됨 | notebook kernel | SAM 2.1 (Jetson GPU Python 3.10) 선택 |
| LiDAR shared library 오류 | LD_LIBRARY_PATH | ~/robot_setup/livox-sdk2/lib 추가 |

YOLO ROS interpreter는 source 후 다음 읽기 명령으로 확인할 수 있습니다:

```bash
head -n 1 "$(ros2 pkg prefix yolo_ros)/lib/yolo_ros/yolo_node"
```

예상 경로는 `~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/python`입니다. CUDA toolkit이나 NVIDIA 드라이버를 재설치하기 전에 환경과 wheel 조합부터 확인합니다.
