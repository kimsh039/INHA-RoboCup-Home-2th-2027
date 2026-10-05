# 손목카메라 GraspNet · Colab 실행

> **성공 실행의 노트북 원본을 보존합니다.** 로컬 CPU에서 추론하지 않습니다. 제공된 포즈로 MuJoCo를 재현하려면 Colab을 건너뛰고 [실행 안내](../README.md)를 사용합니다.

## 입력과 실행 순서

1. [graspnet_wrist_camera.ipynb](graspnet_wrist_camera.ipynb)를 다운로드하고 Colab의 파일 → 노트북 업로드로 엽니다.
2. 런타임 유형을 CUDA GPU로 선택합니다.
3. 위에서 아래로 셀을 실행합니다. 초기 셀은 Torch/CUDA 확인, 공식 baseline 확보, 확장 빌드를 수행합니다.
4. 업로드 셀에 [cube_wrist_camera_input.zip](../data/pointclouds/cube_wrist_camera/cube_wrist_camera_input.zip) **한 개**를 올립니다.
5. 체크포인트 셀에서 공식 RealSense `checkpoint-rs.tar`를 다운로드하거나 업로드합니다. 입력 ZIP과 체크포인트는 서로 다른 파일입니다.
6. 20,000점을 샘플링해 **한 번 추론**하고 전체 후보를 저장합니다.
7. 최종 셀에서 `cube_wrist_camera_graspnet_output.zip`을 다운로드합니다.

## 공식 소스와 호환 변경

[GraspNet-baseline](https://github.com/graspnet/graspnet-baseline)의 성공 provenance commit은 `280c215129f759ed8649cb4e89fc5dfee55f4f80`입니다. 모델 가중치는 [공식 체크포인트 안내](https://github.com/graspnet/graspnet-baseline#training-and-testing)에서 받습니다. 성공 가중치 SHA256은 `60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`입니다.

당시 GPU 환경은 Python 3.13.15 / Torch 2.11.0+cu130 / CUDA 13.0이었습니다. 이 값은 성공 결과의 기록이며 미래 Colab 환경을 고정한 설치 조합은 아닙니다. 원본 노트북은 외부 소스를 clone하므로 출력 provenance의 commit도 확인합니다.

원본 노트북에는 PointNet2 C++ API 호환 치환, inference-only KNN import guard, 직렬화 helper의 stale torch 참조 복구가 들어 있습니다. CUDA 알고리즘·학습 코드·체크포인트는 바꾸지 않습니다. 이전에 실패한 micromamba 실험 환경은 이 재현 경로에 포함하지 않습니다. upstream의 비상업적 사용 조건을 확인합니다.

## 로컬에 결과 반영

MuJoCo 프로젝트 디렉터리에서 ROS 가상환경을 활성화한 뒤:

```bash
python scripts/import_graspnet_output.py \
  "$HOME/Downloads/cube_wrist_camera_graspnet_output.zip" \
  --metadata data/pointclouds/cube_wrist_camera/metadata.json \
  --output data/grasps/cube_wrist_camera
```

입력 metadata가 다르면 import가 거절됩니다. 기존 결과는 백업됩니다. RViz 게시 노드를 재시작하여 새 상위 50개를 확인한 다음 실행합니다. 새 추론 결과가 보존된 성공 후보와 같다고 가정하지 않습니다.

## 관측을 변경했다면

브리지와 MoveIt을 켜고 손목 관측 자세에 실제 도달한 뒤:

```bash
python scripts/generate_wrist_camera_cloud.py --live
```

새 ZIP → Colab 1회 추론 → 동일 metadata로 import → RViz → 실행 순서로 진행합니다. 점군 생성은 CPU에서 가능합니다. [상세 튜토리얼](../docs/TUTORIAL.md)에 좌표계와 오류별 조치를 설명합니다.
