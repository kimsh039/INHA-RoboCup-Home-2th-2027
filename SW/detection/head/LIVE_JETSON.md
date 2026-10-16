# Jetson 디텍션 화면과 FPS

현재 설정·설치는 [TENSORRT_JETSON](TENSORRT_JETSON.md), 전체 관제/접근은 [control_center/DETECTION](../../control_center/DETECTION.md)을 따른다.

## 현재 관제 실행

```bash
cd ~/robocup   # 실제 clone 경로
git pull --ff-only
bash SW/control_center/jetson-center.sh start --detection
```

카메라·로봇 실행 조건과 네트워크/접속은 [관제센터 README](../../control_center/README.md)에서 확인한다. 이미 실행 중인 동일 센서를 중복 시작하지 않는다. 코드 pull만으로 실행 중 프로세스가 교체되지 않는다.

별도 디텍션만 실행하려면 카메라가 동작 중인 ROS 셸에서:

```bash
bash SW/control_center/start-head-detection.sh
```

기본 원본 RGB는 `/head/camera/color/image_raw`다. `/head/calibrated/color/image_raw` 중계는 현재 없다. `head-web.yaml` 기준 cup/banana 0.50, 나머지 0.25, imgsz 640, IoU 0.7, 클래스별 top-1이다.

## 별도 화면 비교

현재 engine 또는 .pt와 ROS를 사용할 수 있는 기존 Head 환경에서:

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/head_detection/bin/activate
python SW/detection/head/scripts/live_ros_viewer.py \
  --topic /head/camera/color/image_raw --host 0.0.0.0 --port 8787
```

Jetson 브라우저는 http://127.0.0.1:8787/ , 다른 PC는 Jetson IP의 8787로 접속한다. 실행 중 관제 결과 재사용/별도 YOLO 여부는 [모드 안내](TENSORRT_JETSON.md)를 확인한다. `--yolo-hz 3`의 KCF 비교를 할 때 관제 YOLO를 중복 실행하지 않는다.

Camera FPS는 수신 이미지 수, Detection FPS는 실제 YOLO 처리 수, Preview/Window FPS는 표시 프레임 수다. debug 기본 6Hz와 별도 미리보기 15FPS 제한을 실제 검출 FPS로 부르지 않는다. 해당 터미널 Ctrl+C로 종료한다.
