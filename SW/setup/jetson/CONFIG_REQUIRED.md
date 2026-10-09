# 필요한 실제 입력과 재개 조건

> 이 문서는 2026-10-03 Jetson `/home/sparo/robot_setup/` 기록의 공유 사본입니다. 모델·venv·로그·설정 파일은 Jetson 로컬 경로를 사용합니다. [문서 모음](README.md) · [프로젝트 홈](../../README.md)

현재 입력 파일: `/home/sparo/robot_setup/setup_inputs.yaml`. 빈 값은 실측·장치 조회 후 채웁니다.

| 항목 | 상태 | 필요한 이유 / 확인 방법 |
| --- | --- | --- |
| 남은 apt 설치 / sudo 인증 | BLOCKED | ROS Desktop과 네 workspace의 핵심 build는 완료. RealSense/AprilTag/image tools 등 15개 요청 apt가 아직 없음. apt-cache/remaining_packages.txt와 README의 캐시 설치 명령으로 재개 |
| Head D435 / Wrist D435 serial·USB 연결 | CONFIGURE_REQUIRED | 한 차례 D435 ID 8086:0b07이 관측됐으나 마지막 USB 목록에는 없음. 두 장치의 모델·서로 다른 serial·USB 3 속도·지원 profile을 각각 기록. 두 카메라 영상은 미확인 |
| camera profile / TF / topic / QoS | CONFIGURE_REQUIRED | 640×480@30 지원과 namespace/frame 실제 이름 확인. --show-args 및 topic info -v |
| AprilTag 검은 외곽 테두리 한 변[m] | CONFIGURE_REQUIRED | pose의 길이 척도 결정. 인쇄 후 실측. 흰 여백 포함하지 않음 |
| PiPER CAN / TRACER CAN / USB bus ID | CONFIGURE_REQUIRED | 차체 버스와 팔 버스 분리. can0 이름만으로 판별하지 않음 |
| PiPER firmware / old-new URDF / gripper | CONFIGURE_REQUIRED | 제조사 README의 S-V1.6-3 경계 및 실제 장치와 대조. 펌웨어 변경 없음 |
| PiPER 실제 joint_states_feedback | CONFIGURE_REQUIRED | robot_state_publisher에 실측 관절 입력. 읽기 노드 출력 remap 후 timestamp/name 확인 |
| MID360 실제 IP / NIC / host IPv4 | CONFIGURE_REQUIRED | 현재 eno1=192.168.50.184/24. 문서의 192.168.1.184는 실제 센서인지 미확인. route/arp/장치와 확인 후 JSON에 입력 |
| MID360 뒤집힘 / extrinsic 적용 위치 | CONFIGURE_REQUIRED | SDK extrinsic 0 유지, 보정 결과 ROS TF에 회전 한 번만 적용 |
| Koide ARM64 실행 경로 | BLOCKED | manifest 확인 결과 humble은 amd64 전용. 이번에는 설정만 준비. 호환 ARM64 실행환경 또는 기존 amd64 PC 필요 |
| SAM video notebook extras | BLOCKED | eva-decord 0.6.1 Linux ARM64 배포 없음. 공식 image notebook/point·box 분할은 통과 |
| 실제 calibration/validation bag | CONFIGURE_REQUIRED | Head color/CameraInfo + PointCloud2 intensity, 동일 설정으로 독립 validation 촬영 |
| Head/Wrist 보정 결과 | CONFIGURE_REQUIRED | 실제 충분한 자세/영상 및 TF 기반 계산 필요. publisher는 결과가 있을 때만 |
| base↔arm / base↔2D LiDAR 실측 | CONFIGURE_REQUIRED | CAD/장착 좌표, 단위·축·quaternion 순서 확인 후 외부 TF 발행 |
| flange↔TCP / pivot matrices / 지그 offset | CONFIGURE_REQUIRED | pivot은 위치 offset만 보정. orientation/지그 offset 별도 입력 |
| 2D LiDAR (YDLIDAR G2) driver | CONFIGURE_REQUIRED | 모델은 G2로 확정(2026-10-09). 공식 `ydlidar_ros2_driver` + YDLidar-SDK 미설치. 시리얼 포트·udev 규칙, baudrate, frame_id `laser_frame`, 스캔 영점(CAD yaw 약 8.81도)을 연결 시 확인 |
| original TRACER driver 필요 여부 | CONFIGURE_REQUIRED | 해당 보정에 차체 odom이 필요한지 먼저 확인. standalone SDK와 Humble wrapper 조합 별도 검토 |
| /odom 출처/적분/frame/TF 발행자 | CONFIGURE_REQUIRED | 바퀴 적분인지 다른 출처인지 확인. 내장 IMU/융합 odometry 가정 금지 |
| 목표 물체 클래스/ID | CONFIGURE_REQUIRED | names 빈 템플릿 상태. 실제 class 목록 필요 |
| 촬영 세션별 train/val/test와 bbox 라벨 | CONFIGURE_REQUIRED | 같은 연속 프레임의 split 간 누출 방지. 라벨 검토 전 학습하지 않음 |

YOLO/SAM 모두 torch 2.8.0 + torchvision 0.23.0 CUDA 12.6/sm_87로 설치·실제 추론을 확인했습니다. 시스템 ROS apt, YOLO venv, SAM venv는 분리합니다. rosbag 외에 자체 수집기·TF 변환기·통합 노드는 작성하지 않습니다. Nav2/MoveIt/FAST-LIO/GraspNet/SAM ROS publisher/MID360 영상 투영·융합/자동 접근은 이번 setup 범위 밖입니다.
