# 우분투 / Jetson 작업 인수인계 · 2026-10-13

## 현재 채택한 상태

- 모델: `SW/detection/head/models/head_yolo11n_20261013.pt`. 새 native 6클래스 YOLO11n이며 `best.pt`와 동일한 파일이다.
- 클래스 ID: 0 green_apple, 1 peach, 2 tomato, 3 fanta_can, 4 cup, 5 banana. **cup은 빨간 머그컵만**, 손잡이 포함.
- 추론: imgsz 640 / 다른 클래스 conf 0.25 / **cup conf 0.70** / NMS IoU 0.7.
- 32 epoch 정상 완료, best epoch 12. 검증 mAP50 0.97474 / mAP50–95 0.70237.
- KakaoTalk 테스트 961프레임 전체 처리. 컵 2개 검출 프레임 101 → 0, 모든 프레임에서 컵 박스 하나 유지. 사용자가 결과를 보고 채택.
- 모델·수정 데이터·실제 로그·테스트 원본과 결과는 Git에 보관한다. Jetson 실행/FPS는 아직 미측정.

## 1. Git과 Python 환경

```bash
cd ~/INHA-RoboCup-Home-2th-2027
git status --short
git pull --ff-only
source /opt/ros/humble/setup.bash
source ~/detection_ws/src/yolo_ros/yolo_ros/.venv/bin/activate
python -c "import torch, ultralytics; print(torch.__version__, ultralytics.__version__); print('CUDA:', torch.cuda.is_available())"
```

이전에 `~/robocup`으로 저장소를 옮겼다면 첫 줄만 실제 저장소 경로로 바꾼다. `git pull`이 로컬 수정 때문에 실패하면 우선 그 수정 내용을 확인한다. 임시 보관할 경우 `git stash push -u -m ubuntu-before-head-yolo11n-20261013` 후 다시 pull한다. 새 코드와 비교 후 필요한 수정만 복구한다.

기존 Jetson YOLO venv를 사용한다. Windows용 torch/CUDA wheel을 Jetson에 복사하지 않는다. 모델 로드 버전 문제가 있으면 현재 torch/JetPack을 유지하면서 Ultralytics 호환 버전을 확인한다. 학습은 Ultralytics 8.4.174에서 수행했다.

## 2. 카메라와 디텍션 화면

이미 Head 카메라가 실행 중이면 그대로 사용한다. 먼저 토픽을 확인한다.

```bash
ros2 topic list -t
ros2 topic hz /head/camera/color/image_raw
```

ROS 노드가 전혀 안 보이면 카메라 실행 여부와 카메라 프로세스와 같은 `ROS_DOMAIN_ID`인지 확인한다. 카메라만 새로 실행해야 할 때는 별도 ROS 터미널에서 `bash SW/detection/head/test/start_head_camera.sh`를 사용한다. 실제 보정 카메라가 관제센터에서 이미 실행 중이면 중복 실행하지 않는다.

원본 Head RGB를 사용하는 실행 명령:

```bash
python SW/detection/head/scripts/live_ros_viewer.py \
  --model SW/detection/head/models/head_yolo11n_20261013.pt \
  --topic /head/camera/color/image_raw \
  --conf 0.25 --cup-conf 0.70 --iou 0.7 --imgsz 640 \
  --host 0.0.0.0 --port 8787
```

물리 센서 실행기의 보정 토픽을 쓰려면 `--topic /head/camera/color/image_raw`로 바꾼다. 우분투 자체 브라우저는 **http://127.0.0.1:8787/**, 같은 LAN의 다른 PC는 **http://192.168.50.70:8787/**에서 화면을 확인한다. 로봇 IP가 바뀌면 주소를 바꾼다. 터미널에 매초 Camera FPS / Detection FPS / detect ms / callback-to-preview ms가 출력된다. 종료는 실행 터미널에서 Ctrl+C다. 이 화면 실행기는 `/cmd_vel`을 발행하지 않는다.

## 3. 관제센터 / ROS 추적 노드

`SW/control_center/start-head-detection.sh`의 기본 모델도 20261013으로 갱신했고 `head-web.yaml`에 conf 0.25 / cup_confidence 0.70 / nms_iou 0.7을 넣었다. 관제센터를 통해 디텍션을 사용하려면 먼저 변경된 ROS Python 패키지와 설정을 빌드한다.

```bash
cd SW/detection/head/head_detection_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robocup_detection_msgs robocup_head_detection --symlink-install
source install/setup.bash
cd ../../../..
bash SW/control_center/start-head-detection.sh
```

`ROBOCUP_MODEL`을 따로 설정한 셸에서는 그 값이 우선한다. 이전 모델을 가리키면 `unset ROBOCUP_MODEL` 후 실행하거나 새 모델의 절대 경로로 지정한다. 관제센터 디텍션은 단일 목표 YOLO/CSRT ROS 파이프라인이며 모든 객체를 보여주는 `live_ros_viewer.py`와 용도가 다르다. FPS 비교 때 두 디텍션 프로세스를 동시에 실행하지 않는다. 파라미터는 시작 시 고정이므로 설정 변경 후 프로세스를 재시작한다.

## 4. 영상 재현과 다음 작업

[영상 테스트 명령과 비교표](training/experiments/20261013/VIDEO_TEST.md) · [학습 결과](training/experiments/20261013/TRAINING_RESULTS.md) · [실제 학습 데이터](datasets/20261013/README.md).

다음 우분투 작업은 동일 조건의 카메라 단독 FPS와 디텍션 FPS를 기록하고 cup=0.70을 거리·조명·움직임이 바뀐 새 영상에서 확인하는 것이다. 그 다음 Jetson에서 TensorRT FP16과 저빈도 YOLO + OpenCV 추적을 비교한다. 현재 adapter는 `.pt`만 지원하며 `.engine`을 바로 연결할 수 없다. 학습 데이터의 미검토 라벨과 컵 뒤 물체 오라벨, 바나나 오검출도 후속 점검한다.

이 모델은 이전 두 보관 테스트 영상을 학습에 포함했다. 독립 평가에는 새 테스트 영상을 사용한다. 검증 라벨은 부분 수동 검토 상태이며 테스트 영상 정답 bbox·테이블 선택 정답은 아직 없다. 검증 mAP와 노트북 inference 시간만으로 Jetson 실시간 성능·테이블 선택 정확도를 확정하지 않는다.
