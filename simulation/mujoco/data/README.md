# 제공 입력·GraspNet 결과

> 4cm 큐브의 성공 실행에 사용한 작은 예제 데이터만 포함합니다. 체크포인트, bag, 실행 로그는 포함하지 않습니다.

## 파일과 좌표계

| 파일 | 형식·용도 |
| --- | --- |
| `pointclouds/cube_wrist_camera/points.npy` | float32 N×3, object frame, 미터, 23,070개 보이는 윗면 점 |
| `points_camera.npy`, `points_network.npy` | camera optical frame XYZ, 이 예제의 network 변환은 항등 |
| `metadata.json` | 센서 사양, 카메라·베이스·물체 변환, 관측 자세, SHA256 |
| `cube_wrist_camera_input.zip` | 위 네 파일의 Colab 업로드 패키지 |
| `grasps/cube_wrist_camera/grasps.json` | object frame의 점수 순 1,024개 후보, input metadata와 provenance |
| `grasps_network_raw.npy` | network frame N×17 공식 decode 결과 |
| `grasps_raw.npy` | 같은 후보를 object frame으로 강체 변환한 결과 |
| `network_input_sensor.npy` | 모델이 실제로 받은 20,000×3 샘플 |
| `candidate_frames.png` | 후보 시각화 참고 이미지 |
| `pregrasps_object.json` | 20cm 후퇴한 기존 참고 포즈; 성공 경로에서 별도 경유하지 않음 |

N×17 열 순서는 `score, width, height, depth, R(9), translation(3), object_id`입니다. 회전행렬 R은 grasp frame → object/network frame이며 +X가 접근, Y가 개구 방향입니다. score는 성공 확률이 아닙니다. 위치·width·height·depth는 미터입니다.

## 관측의 의미

입력은 실제 D435 촬영이나 완전한 3D 모델이 아닙니다. 공용 URDF의 손목 optical frame에서 192×192 subpixel ROI 광선을 쏘아 **먼저 맞은 큐브 표면만** 사용합니다. 테이블·로봇에 먼저 맞은 광선은 제외합니다. 센서 원본 1280×720과 pinhole FOV를 반영하지만 밀도·잡음은 이상적입니다.

카메라 optical 축은 X 오른쪽 / Y 아래 / Z 전방입니다. `T_camera_object`는 물체 좌표를 카메라 좌표로 옮깁니다. 모델 결과는 object frame으로 변환해 보존하고 실행 시 world frame과 실제 TCP의 depth 오프셋을 적용합니다.

## 재사용 범위

이 데이터는 제공된 장면과 관측 자세 전용입니다. 큐브 크기·물체 위치·카메라 자세가 바뀌면 새 관측과 추론을 수행합니다. 입력 metadata를 수정해 해시 검사만 우회하지 않습니다. 체크섬은 [provenance.json](../docs/provenance.json)에 있습니다.
