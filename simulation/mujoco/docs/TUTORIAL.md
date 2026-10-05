# Manipulation Tutorial — 손목카메라 단일 관측에서 PiPER 픽앤플레이스까지

작성 기준: 2026-10-05에 `PICK_AND_PLACE_PASS`가 기록된 실행 코드·입력·GraspNet 결과. 이 문서는 Ubuntu 개인 PC에서 제공된 파일로 성공 장면을 재현하는 튜토리얼이다.

## 1. 무엇을 하는 프로젝트인가

Tracer 베이스, 랙, PiPER 팔, 손목 깊이카메라 CAD가 포함된 원본 조립 URDF를 MuJoCo에 올리고, 4cm 큐브를 잡아 들어 올린 뒤 옆에 놓는다. ROS 2 Humble은 상태·명령을 연결하고, MoveIt 2는 IK·충돌·경로를 검사한다. GraspNet-baseline은 CUDA가 있는 Colab에서 한 번만 추론한다. 로컬 PC에는 NVIDIA GPU나 GraspNet 설치가 필요 없다.

두 가지 실행 경로를 제공한다. **A: 제공된 점군과 포즈로 바로 재현**이 기본이다. **B: 제공된 코랩 노트북으로 동일 입력을 다시 추론**은 선택이다. A에서는 코랩을 다시 실행하지 않는다.

네비게이션과 물체 검출은 이 튜토리얼에서 실행하지 않는다. 명령 대상 큐브의 중심을 이미 얻었고, 물체 앞 40cm까지 접근해 베이스가 멈췄다고 가정한다. 실제 하드웨어의 센서 잡음·제어 성능을 검증한 결과는 아니다.

## 2. 제공 파일

저장소에 성공 입력·후보와 실행 코드를 포함한다. 가중치와 절대 경로를 포함한 생성 모델은 Git에 올리지 않는다. 각 PC에서 생성 모델을 다시 만든다.

| 저장소 경로 (`simulation/mujoco/` 기준) | 내용과 용도 |
| --- | --- |
| `run_*.py`, `manipulation/`, `launch/`, `config/` | 성공한 실행 코드와 설정 |
| `data/pointclouds/cube_wrist_camera/` | object/camera/network 점군과 metadata |
| `data/pointclouds/cube_wrist_camera/cube_wrist_camera_input.zip` | Colab 업로드용 완성 입력 |
| `colab/graspnet_wrist_camera.ipynb` | 성공 결과를 만든 Colab 원본 |
| `data/grasps/cube_wrist_camera/` | 성공 추론의 1,024개 후보·원시 decode·실제 샘플 입력 |
| `reports/wrist_camera_grasp_execution_success.json` | 원래 성공 실행의 접촉·lift·placement 측정 |
| `reports/repository_validation.json` | 팀 레포 경로로 옮긴 뒤의 CPU 검증 |
| `docs/provenance.json` | 파일 SHA256, 성공 환경과 외부 commit |
| `../robot_description/robocup.urdf`, `../../HW/URDF/` | 레포의 공용 로봇과 메시·라이선스 |

공식 GraspNet 소스는 노트북에서 가져오며 별도로 복제해 Git에 포함하지 않는다. 출력 ZIP은 새 Colab 추론 결과를 내려받고 import할 때 사용한다. 이미 보존한 후보로 실행하는 기본 경로에서는 출력 ZIP을 다시 받을 필요 없다.

### 입력 점군을 읽는 방법

`points.npy`는 **object frame**, 미터 단위, float32, 25,296×3 배열이다. 4cm 큐브를 손목카메라 한 시점에서 바라본 **보이는 윗면 표면점**이다. 완전한 3D 큐브를 모델에 넣은 것이 아니며, 숨은 옆면·바닥면을 보충하지 않는다. 테이블과 손가락에 먼저 닿은 광선은 큐브 입력에서 제외한다.

이 입력은 실제 RGB-D 촬영 파일이 아니라, 원본 URDF의 센서 장착 변환과 시뮬레이션 위치 관계를 반영해 `mujoco.mj_ray`로 만든 더미 깊이 점군이다. 192×192의 subpixel ROI 광선을 사용한다. 원본 센서 설정의 1280×720 해상도와 pinhole FOV를 반영하지만, 실제 센서의 점 밀도·노이즈를 재현하지는 않는다.

`points_camera.npy`와 `points_network.npy`는 카메라 optical frame의 XYZ이다. optical 축은 X 오른쪽, Y 아래쪽, Z 전방이다. GraspNet에는 이 3차원 XYZ에서 seed 42로 뽑은 20,000개 점을 입력한다. RGB 이미지나 2차원 픽셀 좌표만 넣는 방식이 아니다.

`metadata.json`에는 카메라/물체/베이스 변환, 관측 관절각, 센서 사양, 입력 해시가 있다. 입력과 포즈의 메타데이터가 다르면 실행 코드가 거절한다. 제공 파일을 임의로 바꾸지 않는다.

### 출력 포즈를 읽는 방법

`grasps.json`은 점수 순 후보 1,024개를 포함한다. 각 후보에는 `candidate_id`, `score`, `width`, `height`, `depth`, `rotation_matrix`, `translation`이 있다. 위치는 object frame·미터이고, R은 grasp frame에서 object frame으로 변환한다. R의 첫 열은 +X 접근 방향, 두 번째 열은 Y 개구 방향이다. 점수는 성공 확률이 아니다.

`grasps_network_raw.npy`는 카메라/network frame의 공식 N×17 decode 결과다. `grasps_raw.npy`는 object frame으로 강체 변환한 같은 결과다. 열은 `score, width, height, depth, R 9개, translation 3개, object_id` 순이다. `network_input_sensor.npy`는 실제 추론에 사용된 20,000×3 입력이다.

`pregrasps_object.json`에는 20cm 후퇴하는 기존 참고 pre-grasp가 남아 있다. **이번 성공 실행은 이 참고 자세로 이동하지 않고, 실제 손목 관측 자세에서 파지 접근을 시작한다.**

## 3. 시스템 방법론: 네비게이션부터 실행까지

1. 사용자가 “큐브 가져와”라고 명령하고, 네비게이션/인식 시스템이 큐브 중심을 찾는다. 이 단계는 상위 시스템의 역할이며 본 튜토리얼에서는 중심값이 알려졌다고 가정한다.
2. base_link 원점과 물체 중심의 **수평 거리 0.40m**까지 접근해 정지한다. 외장과 테이블 사이 간격을 40cm로 정한 것이 아니다. 베이스는 고정이며 Nav2나 주행 동역학을 실행하지 않는다.
3. 알려진 중심과 원본 로봇 FK로 손목카메라 관측 자세를 계산한다. 목표 optical origin은 물체 중심 위 0.30m, optical +Z는 수직 아래다. 손목카메라와 그리퍼의 서로 다른 좌표계는 원본 URDF의 장착 변환으로 연결한다. 실물 hand–eye 캘리브레이션은 별도 과제다.
4. MoveIt으로 관측 자세 경로를 계획·검증한 뒤 MuJoCo에서 관절 궤적으로 실행한다. 실제 관절 피드백으로 카메라 FK를 확인하고 `WRIST_OBSERVATION_READY`를 출력한다. 순간 이동으로 관측 자세를 만들지 않는다.
5. 그 관측 시점에서 큐브의 보이는 표면점을 생성한다. 제공된 점군을 사용하면 이 생성 작업은 다시 할 필요 없다.
6. Colab에서 GraspNet-baseline과 공식 RealSense `checkpoint-rs.tar`로 **한 번 추론**하고 모든 후보를 저장한다. 매 제어 스텝마다 추론하지 않는다. 탑다운 관측이 탑다운 파지 방향을 강제하는 것은 아니다.
7. RViz에서 점수 상위 50개 원본 후보를 먼저 확인한다. 이 표시는 로봇을 움직이지 않는다.
8. 성공 실행에서는 `--fixed-open-width`를 사용한다. 모델의 예측 개구 폭을 제한 검사에서 제외하고 **실제 최대 개구 약 7cm**로 연다. 원래 예측 폭은 결과에 보존한다. 큐브 전체가 실제 개구 안에 들어오는지 양쪽 1mm 여유와 파지 중심 편심을 검사한다.
9. 후보를 점수 순으로 검사한다. 관측 상태에서 접근하는 경로, grasp/lift IK, 관절 한계, 자기 충돌·테이블 충돌, 옮겨 놓기 경로가 통과한 첫 후보를 선택한다. 최고 점수만 무조건 실행하지 않는다.
10. 그리퍼를 열린 상태로 접근하고, 목표 TCP 오차를 확인한 뒤 닫기 목표 0을 보낸다. 실제 손가락 간격은 물체 접촉과 구동력에 따라 결정된다. 닫기 3초와 추가 대기는 **시뮬레이션 시간** 기준이다.
11. 양쪽 손가락–큐브 접촉을 확인한 뒤 world Z 방향으로 0.20m 들어 올린다. 접촉 유지, 손가락 대비 물체의 위치·회전 변화와 실제 상승량을 검사한다.
12. 베이스 기준 왼쪽 0.10m로 옮겨 놓는다. 왼쪽 경로가 불가능하면 오른쪽을 검사한다. 테이블 지지 접촉 확인 → 열기 → 후퇴 → 접힌 자세 복귀 → `DONE`으로 종료한다.

### 성공 장면과 좌표 변환

- 큐브: 한 변 0.04m, 질량 0.04kg, world 중심 `[0.5, -0.1, 0.7401]`m. 바닥면 지지 후 중심 높이는 약 0.73978m.
- 테이블 상판: world Z=0.72m.
- base_link: world `[0.107767729724, -0.021553545945, 0.142544676935]`m.
- 계획 카메라 optical origin: world `[0.5, -0.1, 1.0401]`m. 큐브 윗면까지 약 0.28m.
- 입력 축 변환: 카메라 optical→network는 항등 변환.
- 좌표 계산: `p_camera = T_camera_object · p_object`, `T_camera_object = inverse(T_world_camera) · T_world_object`.
- 모델 후보→world: `T_world_grasp = T_world_object · T_object_grasp`.
- GraspNet grasp origin과 실제 손끝 TCP는 동일하지 않다. `target_tcp()`가 depth 오프셋을 적용한다. 손끝 TCP 위치는 원본 손가락 메시에서 계산하며 축 매핑은 config에 기록되어 있다.

### 성공에 반영된 시뮬레이션 설정

팔은 기존 위치 서보에 모델의 중력·속도 편향 토크 보상을 적용한다. 모델의 실제 하드웨어 게인이 검증됐다는 의미는 아니다. URDF 구동력 제한은 유지한다.

손가락 메시의 평면 접촉이 한 점으로만 계산되는 문제를 줄이기 위해 **MultiCCD를 켜고 NoSlip 5회**를 적용한다. 질량, 마찰계수, 그리퍼 게인을 성공시키려고 크게 높이는 방식은 채택하지 않았다. 이 설정은 접촉 계산을 보정하며, 물체를 손에 weld하거나 위치를 강제로 따라오게 하지 않는다. MoveIt의 attached collision object는 경로 검사 표현일 뿐 MuJoCo 물체의 물리 고정이 아니다.

## 4. 개발환경 구축 — 로컬 NVIDIA GPU 불필요

검증된 환경은 **Ubuntu 22.04.5 LTS x86_64, Python 3.10.12, ROS 2 Humble, MoveIt 2.5.10, MuJoCo 3.3.7, NumPy 1.26.4, SciPy 1.15.3**이다. 다른 OS/ROS 배포판에서의 재현은 미검증이다. 데스크톱 OpenGL 창을 열 수 있어야 한다.

### 4-1. ROS 저장소 등록과 설치

ROS가 이미 설치되어 있으면 저장소 등록은 생략한다. Ubuntu 22.04에서 [ROS 2 Humble 공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)의 UTF-8 locale·Ubuntu Universe·ROS apt 저장소 등록 단계를 수행한다. 아래는 공식 저장소 등록 방식이다.

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
export TUTORIAL_UBUNTU_CODENAME=$(. /etc/os-release; echo "$UBUNTU_CODENAME")
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${TUTORIAL_UBUNTU_CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade
sudo apt install -y ros-humble-ros-base ros-humble-moveit ros-humble-rviz2 ros-humble-robot-state-publisher python3-venv python3-pip unzip libgl1 libglfw3
```

### 4-2. 저장소 전체 가져오기

공용 URDF가 `HW/URDF/`의 메시를 참조하므로 MuJoCo 폴더만 다운로드하지 않는다. 레포 루트의 Git 갱신 지침을 따른다.

```bash
git clone https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027.git
cd INHA-RoboCup-Home-2th-2027
git status
git pull --ff-only origin main
cd simulation/mujoco
```

이미 수정 중인 체크아웃은 변경을 보존하고 Git 이력을 확인한다. 강제 pull/reset으로 변경을 지우지 않는다.

### 4-3. ROS와 함께 사용할 Python 환경

```bash
cd "$HOME/INHA-RoboCup-Home-2th-2027/simulation/mujoco"
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages .ros_venv
source .ros_venv/bin/activate
python -m pip install -r requirements_success.txt
python tools/check_success_environment.py
python run_mujoco_moveit_bridge.py --prepare-only
```

`--system-site-packages`는 apt로 설치한 rclpy와 ROS 메시지 모듈을 가상환경에서 읽게 한다. Colab의 Torch/CUDA/pointnet2를 로컬에 설치하지 않는다. `--prepare-only`가 현재 PC 경로로 MuJoCo/MoveIt 모델을 생성하고 FK 일치를 검사한다. 이 모듈은 직접 Python/launch 파일을 실행하므로 튜토리얼 실행에 colcon 소스 빌드는 필요 없다.

위 apt 설치는 새 PC를 위한 안내이며 모든 새 OS 설치 조합을 시험한 것은 아니다. 성공 PC의 버전은 `docs/provenance.json`에 기록되어 있다.

## 5. 기본 재현 경로 A — 제공된 포즈 사용

모든 터미널의 공통 초기화는 아래와 같다. 폴더 위치를 바꿨으면 첫 줄만 수정한다.

```bash
cd "$HOME/INHA-RoboCup-Home-2th-2027/simulation/mujoco"
source /opt/ros/humble/setup.bash
source .ros_venv/bin/activate
export ROS_LOG_DIR="$PWD/reports/ros_logs"
```

### 터미널 1: MuJoCo

공통 초기화 후 실행한다. `MuJoCo viewer ready`가 나온 뒤 계속 켜둔다.

```bash
python run_mujoco_moveit_bridge.py
```

### 터미널 2: MoveIt

터미널 1이 준비된 뒤 공통 초기화하고 실행한다. 계속 켜둔다.

```bash
ros2 launch launch/moveit_mujoco.launch.py
```

### 터미널 3: 탑다운 관측 자세로 이동

공통 초기화 후 실행한다.

```bash
python run_wrist_observation.py
```

`WRIST_OBSERVATION_READY`가 나오고 명령이 종료될 때까지 기다린다. 실패하면 파지를 진행하지 않는다.

### 터미널 4: 상위 50개 후보 게시

공통 초기화 후 실행한다. 계속 켜둔다.

```bash
python scripts/publish_grasp_poses.py \
  --raw --top 50 \
  --grasps data/grasps/cube_wrist_camera/grasps.json \
  --metadata data/pointclouds/cube_wrist_camera/metadata.json \
  --cloud data/pointclouds/cube_wrist_camera/points.npy
```

### 터미널 5: RViz 확인

공통 초기화 후 실행한다.

```bash
rviz2 -d config/wrist_camera_grasps.rviz
```

주황색 점은 입력 표면, 파란 구체는 카메라 원점이다. RGB 축은 grasp XYZ이고 X는 접근, Y는 개구 방향이다. 자홍색 후보는 모델 예측 폭이 70mm를 넘는 경우이며, 이번 실행에서는 이것만으로 제외하지 않는다. RViz의 그리퍼 윤곽은 모델 예측 폭의 도식으로 실제 URDF 손가락 형상과 다르다. 청록색 20cm 참고선은 이번 실행에서 반드시 경유하는 자세가 아니다. `Stereo is NOT SUPPORTED`만으로 파지 실패를 의미하지 않는다.

### 터미널 3: 확인 후 픽앤플레이스 실행

RViz에서 확인한 뒤 관측 명령이 종료된 같은 터미널 3에 입력한다.

```bash
python run_grasp_moveit.py \
  --fixed-open-width \
  --grasps data/grasps/cube_wrist_camera/grasps.json \
  --metadata data/pointclouds/cube_wrist_camera/metadata.json \
  --result reports/wrist_camera_grasp_execution_latest.json
```

`CLOSE_GRIPPER → VERIFY_GRASP → LIFT_OBJECT → MOVE_TO_PLACE → LOWER_OBJECT → OPEN_GRIPPER → RETREAT → FOLD_ARM → DONE`을 확인한다. 결과 JSON의 `result`가 `PICK_AND_PLACE_PASS`여야 한다. 눈으로 잡힌 것처럼 보이는 것만으로 성공으로 판정하지 않는다.

재실행은 실행 중인 터미널 1/2를 Ctrl+C로 종료한 뒤 1→2→관측→파지 순으로 한다. 설정을 수정했는데 실행 중인 브리지를 그대로 두면 변경이 적용되지 않는다. UI Reset만으로 ROS 상태와 계획 장면이 모두 초기화된다고 가정하지 않는다.

## 6. 선택 경로 B — Colab에서 직접 한 번 추론

A를 재현한 뒤 필요할 때만 수행한다. 로컬 GPU 없이도 가능하다. 제공된 `graspnet_wrist_camera.ipynb`를 Colab의 파일→노트북 업로드로 연다. 런타임 유형을 CUDA GPU로 선택하고 위에서 아래로 실행한다.

| 셀 단계 | 하는 일 |
| --- | --- |
| 1 | GPU, Torch, CUDA compiler 확인 |
| 2 | 공식 GraspNet-baseline 확보, PointNet2 확장 빌드, inference 전용 import |
| 3 | `cube_wrist_camera_input.zip` 한 개 업로드, 해시·좌표계 확인 |
| 4 | 공식 `checkpoint-rs.tar` 재사용/다운로드 또는 업로드 |
| 5 | seed 42로 20,000점 샘플링, 한 번 추론, 원래 후보 보존·좌표 변환 |
| 6 | 입력과 상위 후보 프레임 시각화 |
| 7 | `cube_wrist_camera_graspnet_output.zip` 다운로드 |

체크포인트는 모델 가중치이며 입력 ZIP과 다르다. `.tar`를 일반 데이터 압축처럼 풀지 않고 torch checkpoint로 읽는다. [공식 baseline의 체크포인트 안내](https://github.com/graspnet/graspnet-baseline#training-and-testing)와 [공식 rs 다운로드](https://drive.google.com/file/d/1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk/view)를 사용한다. 이 저장소에는 가중치가 포함되지 않는다. 성공 가중치 SHA256은 `60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`이다.

노트북은 성공 결과에 기록된 코드 그대로 제공한다. 당시 Colab은 Python 3.13.15 / Torch 2.11.0+cu130 / CUDA 13.0이었다. 이것은 관측된 버전이며 미래 Colab에서도 같은 환경·수치가 나온다는 보장은 아니다. 설치/빌드가 실패하면 경로 A로 시뮬레이션을 재현할 수 있다. 실패한 micromamba 실험용 노트북은 이 튜토리얼에 사용하지 않는다.

호환 수정은 PointNet2 C++ API의 `.type().is_cuda()`→`.is_cuda()`, `.data<T>()`→`.data_ptr<T>()` 치환이다. CUDA 연산 알고리즘이나 체크포인트·학습 코드는 바꾸지 않는다. KNN training-label import는 inference-only guard로 처리하며 실제 호출되면 오류를 낸다. 변경 파일과 commit은 결과 provenance에 기록된다. 직렬화 helper 안의 stale torch 참조 복구도 노트북에 포함되어 있다.

결과를 내려받은 뒤 로컬에서 import한다. 새 결과를 import하면 기존 결과는 백업된다.

```bash
python scripts/import_graspnet_output.py \
  "$HOME/Downloads/cube_wrist_camera_graspnet_output.zip" \
  --metadata data/pointclouds/cube_wrist_camera/metadata.json \
  --output data/grasps/cube_wrist_camera
```

그 다음 RViz 게시 노드를 재시작해 새 상위 50개 후보를 확인하고 경로 A의 실행 명령을 사용한다. 새로운 GPU 추론은 후보/점수가 다를 수 있으므로 제공된 성공 후보와 동일하다고 가정하지 않는다.

### 장면이나 관측을 실제로 변경한 경우

제공된 점군과 포즈는 기존 장면 전용이다. 물체 위치·크기·카메라 관측 자세를 바꾸면 새 입력을 생성하고 Colab 결과도 새로 만든다. 실제 관측 자세에 도달한 후 다음 명령을 사용한다.

```bash
python scripts/generate_wrist_camera_cloud.py --live
```

생성된 ZIP을 Colab에 올리고 같은 metadata로 출력 ZIP을 import한다. 단, 접촉 계산 설정만 바뀐 경우는 광선 장면 변경과 구분하며 현재 해시 검사에서 허용한다.

## 7. 성공 기록과 판정 기준

2026-10-05 성공 로그에서 선택 후보는 **634**, 원래 모델 예측 개구 폭은 **77.503mm**다. 실행은 실제 최대 **70mm** 개구를 사용했다. 닫은 후 실제 간격은 **39.069mm**였다. 접촉력 합계를 모터 최대 힘과 혼동하지 않는다.

| 성공 로그 측정값 | 결과 |
| --- | --- |
| 최종 판정 | PICK_AND_PLACE_PASS |
| 마지막 단계 | DONE |
| 닫은 뒤 손가락 접촉력 | 1.806N / 1.756N |
| lift 검증 구간 실제 상승량 | 199.444~199.446mm |
| lift 구간 최대 상대 위치 변화 | 2.093mm |
| lift 구간 최대 상대 회전 변화 | 0.0904rad |
| 옮겨 놓기 | 베이스 기준 왼쪽 100mm |
| 최종 물체 위치와 기대 위치 차이 | 1.553mm |

파지 후 위치 변화 허용은 10mm, 회전 변화 허용은 0.2rad, 상승량 허용은 목표 200mm±20mm다. 이 성공 기록은 해당 큐브·모델·제어 설정에서의 한 실행이며 실물 PiPER의 보장이나 모든 물체의 성공률은 아니다.

## 8. 문제 해결

| 증상 | 확인과 조치 |
| --- | --- |
| `START_MUJOCO_BRIDGE_FIRST` | 터미널 1을 먼저 실행하고 같은 ROS 환경을 사용 |
| `START_MOVEIT_FIRST` | 터미널 2가 준비됐는지 확인 |
| `MID360_SCENE_CHANGED` | 메시지 명칭은 공통 검사에서 나온다. 장면 변경 여부·파일 세트 일치 확인. 새 버전에서는 접촉 설정만 변경하면 입력 재추론 불필요 |
| `MOVEIT_FAILED: -2` | 최종 경로가 유효하지 않음. 성공 코드에서 검사 간격을 촘촘히 하고 관측 계획 최대 3회 재시도. 터미널 2의 실제 충돌 쌍 확인 |
| `MOVEIT_FAILED: -6` | 성공 사례에서 실행 시간 초과로 확인됨. 일반적으로 번호만으로 원인 확정하지 말고 상세 로그 확인. 느린 CPU용 시간 여유와 제한은 코드에 포함 |
| `TCP_TARGET_NOT_REACHED` | 목표 TCP와 측정 FK 비교. 중력 보상 적용된 브리지 재시작 여부 확인. 무조건 허용 오차를 늘리지 않음 |
| `GRASP_VERIFICATION_FAILED` / 큐브 미끄러짐 | `Grasp contact:`의 접촉력·간격, lift 상대 위치·회전 로그 확인. MultiCCD와 NoSlip 5회가 적용된 브리지인지 확인 |
| `UNEXPECTED_COLLISION` | 침투가 감지된 실제 몸체/geom 확인. 충돌 검사를 끄지 않음 |
| RViz에 축만 보임 | 터미널 4 게시 노드, 입력/포즈 경로, RViz Fixed Frame 확인 |
| Colab CUDA 빌드 오류 | 설치 셀에서 중단. 로컬 시뮬레이션에 Colab CUDA 환경을 설치하지 않음. 제공된 포즈로 경로 A 가능 |

터미널 1에는 MuJoCo 물리/추종·시간 로그, 터미널 2에는 MoveIt 충돌·계획·실행 로그, 터미널 3에는 단계별 선택/실패 로그가 있다. `reports/wrist_camera_grasp_execution_latest.json`은 실행 종료 후 저장된다. 실행 중이면 이전 결과 파일이 남아 있을 수 있으므로 생성 시각도 확인한다.

## 9. 주요 코드와 출처

- `run_wrist_observation.py`: MoveIt 관측 이동, 실제 카메라 FK 검사.
- `manipulation/observation.py`, `wrist_camera_cloud.py`: 관측 자세 계산과 가림을 반영한 점군 생성.
- `colab/graspnet_wrist_camera.ipynb`: 실제 사용된 Colab 전체 코드.
- `scripts/import_graspnet_output.py`: 입력 일치·좌표 규약 검사 후 import.
- `scripts/publish_grasp_poses.py`: 원본 상위 50개를 RViz에 게시.
- `run_grasp_moveit.py`: 실제 개구 검사, IK·경로·배치 검증, 파지 상태 머신.
- `run_mujoco_moveit_bridge.py`: 실제 관절 궤적 실행, 편향 토크 보상, 접촉/힘·시간 피드백.
- `manipulation/inha_model.py`, `sim_model.py`: 원본 로봇으로 MuJoCo와 MoveIt 모델 생성, FK·TCP·충돌 형상 연결.
- [공식 GraspNet-baseline](https://github.com/graspnet/graspnet-baseline): 성공 provenance commit `280c215129f759ed8649cb4e89fc5dfee55f4f80`.
- [원본 조립 로봇](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027): 보존 소스 commit `0e0a28d7866832ecf1e0841bd41760ebde672a79`.
- [MuJoCo 접촉 설정 문서](https://mujoco.readthedocs.io/en/stable/XMLreference.html#option): MultiCCD와 NoSlip의 의미.

GraspNet 코드·모델은 공식 저장소의 비상업적 사용 조건을 따른다. 외부 로봇 자산은 포함된 라이선스와 원본 출처를 확인한다. 원본 URDF·메시는 수정하지 않고 변환·생성 파일과 제어 설정을 프로젝트 코드로 관리한다.

## 10. 팀 저장소와의 경계

- 로봇 조립의 기준은 `simulation/robot_description/robocup.urdf` 하나이며 메시·물성 출처는 `HW/URDF/`이다. 생성한 MoveIt/MuJoCo 파일은 로컬 `simulation/generated/`에 저장한다.
- 팀의 실제 센서 역할·D435 구성은 루트 README와 `detection/SENSOR_ROLES.md`를 따른다. 이 튜토리얼의 카메라는 공용 URDF의 optical frame과 카메라 사양을 사용하는 이상적 ray casting이다. 실제 D435 깊이를 녹화한 입력이 아니다.
- 네비게이션·헤드 디텍션의 출력 중심은 여기서 주어진 값으로 가정한다. 현재 `detection/head_detection_ws`와 Nav2를 연결해서 검증한 결과는 없다.
- 성공 기록은 원래 경로에서 실행한 보존 로그다. 팀 레포로 옮긴 뒤의 CPU/FK/입력 검증은 `reports/repository_validation.json`에 별도로 기록한다. 새 경로에서의 전체 ROS 물리 실행과 실물 실험을 같은 검증으로 간주하지 않는다.
- 모델 폭이 큰 원인은 통제 GPU 실험으로 확정하지 못했다. `--fixed-open-width`는 실제 개구와 충돌/접촉 검사를 이용한 실행 정책이며 모델 오차의 원인을 해결했다는 주장과 구분한다.
