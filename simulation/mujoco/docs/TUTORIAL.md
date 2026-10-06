# 변경 URDF · 상세 실행과 재개

## 1. 현재 수행 상태

2026-10-06 새 하드웨어 nominal URDF와 새 Colab 후보로 전체 픽앤플레이스를 수행했고, 초기 관절각 복귀까지 PICK_AND_PLACE_PASS / DONE을 확인했습니다. 근거는 reports/updated_urdf_pick_place.json입니다. 동일 입력으로 재현하려면 4절 공통 초기화 후 6~11절을 실행합니다. Colab 추론은 입력을 바꿀 때만 다시 수행합니다.

## 2. 새 PC 준비

Ubuntu 22.04 / ROS 2 Humble이 설치된 개발 PC 기준입니다. ROS가 없는 PC는 [ROS Humble 공식 설치 문서](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)로 저장소를 등록합니다. 다음 의존성이 필요합니다.

```bash
sudo apt install ros-humble-ros-base ros-humble-moveit ros-humble-rviz2 \
  ros-humble-robot-state-publisher python3-venv python3-pip libgl1 libglfw3
```

이 작업에서는 apt 설치를 새로 수행하지 않았습니다. 기존 성공 PC의 ROS/MoveIt을 사용했습니다. 로컬 CUDA나 micromamba를 설치하지 않습니다. 팀 저장소 전체를 clone하여 공용 URDF와 HW 메시의 상대 구조를 유지합니다. venv와 생성 모델은 새 PC에서 다시 만듭니다.

```bash
cd "$HOME/INHA-RoboCup-Home-2th-2027/simulation/mujoco"
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages .ros_venv
source .ros_venv/bin/activate
python -m pip install -r requirements_success.txt
python run_mujoco_moveit_bridge.py --prepare-only
python -m unittest discover -s tests -v
```

현재 PC의 venv는 이미 준비돼 있으므로 생성/설치 줄을 반복할 필요 없습니다. 원본 checkout commit을 바꾸면 모델 비교부터 다시 수행합니다.

## 3. Colab의 7개 코드 셀

[Colab](https://colab.research.google.com/)에서 파일 → 노트북 업로드로 다음 파일을 엽니다.

```text
$HOME/INHA-RoboCup-Home-2th-2027/simulation/mujoco/colab/graspnet_wrist_camera.ipynb
```

런타임 유형을 GPU로 선택하고 위에서 아래로 코드 셀을 실행합니다.

| 순서 / notebook cell index | 실행 내용 |
| --- | --- |
| 1 / 2 | GPU, PyTorch, CUDA compiler 확인 |
| 2 / 4 | 공식 baseline clone, PointNet2 빌드·기존 inference 호환 처리 |
| 3 / 6 | `data/pointclouds/cube_wrist_camera/cube_wrist_camera_input.zip` 업로드 |
| 4 / 8 | 공식 RealSense `checkpoint-rs.tar` 확보/기존 파일 사용 |
| 5 / 10 | seed 42, 20,000점 표본, GraspNet 1회 추론·원본 decode 보존 |
| 6 / 12 | 입력과 상위 10개 후보 시각화 |
| 7 / 14 | `cube_wrist_camera_graspnet_output.zip` 다운로드 |

입력은 `points_network.npy` optical XYZ이며 2D 픽셀 좌표가 아닙니다. 성공 노트북 SHA256은 `b3141fb5310b424f21f6f413bf616da55dc937fd806703050c837fd9f5b1164e`로 이번에 변경하지 않았습니다. 성공 checkpoint SHA256은 `60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`입니다. 실패한 셀 이후로 진행하지 말고 실제 오류 로그를 보관합니다.

## 4. 터미널 공통 초기화

아래를 **터미널 1~5 모두** 실행합니다. 검증에 사용한 ROS domain 47을 동일하게 유지하고 다른 Gazebo/관절 게시 노드와 섞이지 않게 합니다.

```bash
cd "$HOME/INHA-RoboCup-Home-2th-2027/simulation/mujoco"
source /opt/ros/humble/setup.bash
source .ros_venv/bin/activate
export ROS_LOG_DIR="$PWD/logs/ros"
export ROS_DOMAIN_ID=47
export ROS_LOCALHOST_ONLY=1
```

## 5. 터미널 3 · GPU 결과 import

한국어 다운로드 폴더를 예로 사용합니다. 실제 다운로드 위치만 바꿉니다.

```bash
python scripts/import_graspnet_output.py \
  '/home/doyeonlee/다운로드/cube_wrist_camera_graspnet_output.zip' \
  --metadata data/pointclouds/cube_wrist_camera/metadata.json \
  --output data/grasps/cube_wrist_camera
```

입력 metadata와 결과의 `input_metadata`가 완전히 일치해야 합니다. 기존 모델 해시를 새 해시로 교체해 우회하지 않습니다. 참고 pre-grasp는 생성되지만 이번 손목 경로에서 이동하지 않습니다.

## 6. 터미널 1 · MuJoCo

```bash
python -u run_mujoco_moveit_bridge.py 2>&1 | tee logs/resume_bridge.log
```

`MuJoCo viewer ready`를 기다립니다. GUI가 필요 없는 경우 `--headless`를 추가할 수 있습니다. 물리 step·실제 관절 상태·접촉 검사는 같은 브리지입니다. 초기 팔은 `[0,0,0,0,0,0]`, 베이스는 정지입니다.

## 7. 터미널 2 · MoveIt

```bash
ros2 launch launch/moveit_mujoco.launch.py 2>&1 | tee logs/resume_moveit.log
```

`You can start planning now!` 이후 터미널 3에서 관측을 수행합니다.

## 8. 터미널 3 · 관측 도달 확인

```bash
python -u run_wrist_observation.py 2>&1 | tee logs/resume_observation.log
```

`WRIST_OBSERVATION_READY`를 확인합니다. `reports/live_observation.json`에는 실제 피드백 카메라 변환과 목표 오차가 기록됩니다. 실패하면 파지로 진행하지 않습니다. **현재 업로드한 입력으로 재개할 때는 점군을 다시 생성하지 않습니다.**

처음부터 새 live 입력을 만들 경우에만 이 단계 이후 `python scripts/generate_wrist_camera_cloud.py --live`를 실행하고 해당 ZIP을 Colab에 올립니다. 브리지/MoveIt을 켜둔 채 결과 import → RViz → 파지를 이어갑니다.

## 9. 터미널 4 · 원본 상위 50개

```bash
python scripts/publish_grasp_poses.py --raw --top 50 \
  --grasps data/grasps/cube_wrist_camera/grasps.json \
  --metadata data/pointclouds/cube_wrist_camera/metadata.json \
  --cloud data/pointclouds/cube_wrist_camera/points.npy
```

원본 후보를 점수 순으로 표시하며 width/방향으로 미리 제거하지 않습니다. 입력 표면과 후보 중심·접근축·개구 방향을 확인합니다. 표시된 예측 폭을 실제 손가락 개구라고 해석하지 않습니다.

## 10. 터미널 5 · RViz

```bash
rviz2 -d config/wrist_camera_grasps.rviz
```

Fixed Frame `world`, 입력 점군·카메라 원점·RGB grasp 축을 확인합니다. 자홍색은 예측 폭이 70mm를 넘는 후보이며 이번 `--fixed-open-width` 정책에서는 폭만으로 제외하지 않습니다. 청록색 20cm 선은 참고 pre-grasp입니다. 후보 확인은 사람이 수행하는 단계입니다. JSON 성공 판정과 별도로 기록합니다.

## 11. 터미널 3 · 확인 후 파지·배치

```bash
python -u run_grasp_moveit.py --fixed-open-width \
  --grasps data/grasps/cube_wrist_camera/grasps.json \
  --metadata data/pointclouds/cube_wrist_camera/metadata.json \
  --result reports/updated_urdf_pick_place.json \
  2>&1 | tee logs/updated_urdf_pick_place.log
```

관측 자세 → 접근 → 닫기 → 양손가락 접촉 → world Z 20cm lift → 베이스 왼쪽 10cm 배치(불가 시 오른쪽) → 놓기 → 후퇴 → 시작 초기 자세 [0,0,0,0,0,0] 복귀(RETURN_TO_INITIAL) → DONE을 검사합니다. 실제 물체를 이동시키는 weld는 사용하지 않습니다. MoveIt의 attached object는 충돌 장면 표현입니다.

`reports/updated_urdf_pick_place.json`에 `PICK_AND_PLACE_PASS`, `DONE`, 양손가락 접촉과 물체 상승/유지/배치 수치가 함께 있어야 새 모델 성공으로 판정합니다. 후보가 없거나 IK·충돌·추종·접촉·lift·배치·접힘 실패가 있으면 실제 로그 원인대로 FAIL을 유지합니다.

## 12. 로그와 재검증

`logs/ 아래 실행 로그는 로컬에 생성됩니다. 보존한 성공·FK·관측 수치는 reports/에서 확인합니다.

실행 후 reports/updated_urdf_pick_place.json의 PICK_AND_PLACE_PASS, RETURN_TO_INITIAL, DONE 및 실제 관절 오차를 확인합니다. 이전 시간 역행 실패는 별도 파일에 보존하며 원인을 사용자 Reset으로 단정하지 않습니다.
