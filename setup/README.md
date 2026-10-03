# Setup · 개발 PC와 Jetson

[프로젝트 홈](../README.md) · [Jetson 문서 모음](jetson/README.md)

## 지금 사용하는 Jetson

Ubuntu 22.04.5 / JetPack 6.2.3 / ROS 2 Humble의 **실제 설치 기록과 GPU 환경**은 `jetson/`에 정리했습니다. 이미 준비된 Jetson을 사용할 때는 설치 스크립트를 반복 실행하지 않고 실행 문서부터 확인합니다.

| 작업 | 문서 |
| --- | --- |
| 다음 날 실행 / source 순서 / 센서 / 보정 / 모델 / bag | [RUN_COMMANDS.md](jetson/RUN_COMMANDS.md) |
| Python·venv 선택 / GPU 확인 | [VIRTUAL_ENVIRONMENTS.md](jetson/VIRTUAL_ENVIRONMENTS.md) |
| 설치 재현 / 남은 apt / workspace 빌드 | [README_SETUP.md](jetson/README_SETUP.md) |
| 실제 버전 / pinned source / checksum | [VERSIONS.md](jetson/VERSIONS.md) |
| serial / CAN / IP / 실측 / class 입력 | [CONFIG_REQUIRED.md](jetson/CONFIG_REQUIRED.md) |
| 완료 범위 / GPU 추론 / 미확인 하드웨어 | [SETUP_REPORT.md](jetson/SETUP_REPORT.md) |

## 기존 개발 PC 설치 스크립트

[install_ros2_gz.sh](install_ros2_gz.sh)는 Ubuntu **22.04 Jammy**용 기존 스크립트입니다. 저장소 루트에서 다음과 같이 사용하도록 작성돼 있습니다.

```bash
# 신규 Gazebo 개발 PC
bash setup/install_ros2_gz.sh

# 신규 실기 PC: 시뮬레이션 제외
bash setup/install_ros2_gz.sh --no-sim
```

기본 모드는 ROS 2 Humble과 Gazebo Harmonic, Python Gazebo transport 및 Humble–Harmonic bridge를 설치합니다. `--no-sim`은 시뮬레이션 패키지를 제외하며, Jetson용 GPU PyTorch·YOLO·SAM·센서 SDK까지 구성하지는 않습니다.

> [!IMPORTANT]
> 이 기존 스크립트에는 **apt 저장소 변경과 전체 `apt-get upgrade -y`**가 포함돼 있습니다. 현재 정상 Jetson에 그대로 재실행하는 용도로 권장하지 않습니다. 이 Jetson에서는 기존 드라이버·CUDA·ROS를 재사용하고 [남은 패키지 설치](jetson/README_SETUP.md#1-남은-공통-apt-설치-한-번)만 진행하세요. 스크립트 실행 시 일반 사용자 터미널에서 sudo 인증하며 비밀번호를 파일에 저장하지 않습니다.

Humble의 기본 Gazebo 조합과 Harmonic용 bridge를 섞지 않도록 패키지 구성을 먼저 확인합니다. 현재 Jetson은 실기 Calibration/Detection setup을 우선했으며 Gazebo 실행은 검증하지 않았습니다.
