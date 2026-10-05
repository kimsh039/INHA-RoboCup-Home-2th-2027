# Jetson AGX Orin · 운영 문서

[프로젝트 홈](../../README.md) · [Setup 안내](../README.md)

**기준일: 2026-10-03 · Ubuntu 22.04.5 · JetPack 6.2.3 · ROS 2 Humble · CUDA 12.6**

이 폴더는 `/home/sparo/robot_setup/`의 실제 setup 기록을 팀 저장소에 공유한 문서 사본입니다. 설치 로그·wheel·모델·venv·실측 데이터는 Jetson 로컬에 있으며 이 폴더에 포함하지 않습니다. 다른 PC에서 재현할 때는 CPU 아키텍처, JetPack/CUDA와 사용자 경로를 대조합니다.

**2026-10-05 운영 구성 갱신: Head D435 + Wrist D435.** [카메라 실행 명령](RUN_COMMANDS.md#2-realsense-head--wrist)과 [장치 입력 목록](CONFIG_REQUIRED.md)을 이 구성에 맞췄습니다. SETUP_REPORT의 2026-10-03 관측은 과거 기록이며, 이번 변경은 실기 설치·센서 실행 결과가 아닙니다. Mid-360의 장애물·작업면·접근 역할은 [센서 역할 문서](../../detection/SENSOR_ROLES.md)를 참고합니다.

## 읽는 순서

| 순서 | 문서 | 목적 |
| --- | --- | --- |
| 1 | [VIRTUAL_ENVIRONMENTS.md](VIRTUAL_ENVIRONMENTS.md) | 작업에 맞는 Python / venv 선택 |
| 2 | [RUN_COMMANDS.md](RUN_COMMANDS.md) | 이미 설치된 상태의 터미널별 직접 실행 |
| 3 | [CONFIG_REQUIRED.md](CONFIG_REQUIRED.md) | 실제 장치·실측·데이터 입력과 재개 조건 |
| 필요 시 | [README_SETUP.md](README_SETUP.md) | 설치 재현과 핵심 workspace build |
| 필요 시 | [VERSIONS.md](VERSIONS.md) | 버전·git SHA·모델/wheel checksum |
| 상태 확인 | [SETUP_REPORT.md](SETUP_REPORT.md) | 실제 확인 결과, 로그 경로, 미완료 범위 |

## 핵심 구분

- **ROS shell:** 시스템 Python + Humble + 필요한 workspace overlay.
- **YOLO CLI:** `~/detection_ws/src/yolo_ros/yolo_ros/.venv`.
- **YOLO ROS:** ROS shell에서 실행. 공식 노드의 shebang이 YOLO GPU venv를 사용.
- **SAM / Jupyter:** `~/venvs/sam21`, 전용 GPU kernel.
- **TCP Pivot:** `~/venvs/pivot`.

YOLO와 SAM은 torch 2.8.0 / torchvision 0.23.0으로 실제 GPU 추론을 확인했습니다. 시스템 Python과 모델 venv의 라이브러리를 전역 pip로 합치지 않습니다.

## 상태 읽기

| 상태 | 의미 |
| --- | --- |
| `VERIFIED_SOFTWARE` | 설치·빌드·import·CLI 또는 기록된 최소 추론을 확인 |
| `VERIFIED_HARDWARE` | 실제 해당 장치의 동작을 확인. GPU 연산과 로봇 센서 검증을 구분 |
| `CONFIGURE_REQUIRED` | serial·CAN·IP·태그·실측·class 등 입력 필요 |
| `BLOCKED` | apt 인증·아키텍처 미지원 등 재개 조건 필요 |

로봇 센서·팔 feedback 읽기, 실제 보정, 물체 fine-tuning과 실시간 통합은 아직 완료하지 않았습니다. 세부 상태는 SETUP_REPORT를 기준으로 확인합니다.
