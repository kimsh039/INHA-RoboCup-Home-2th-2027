# Jetson Head 실시간 디텍션 화면·FPS

최신 배포 경로와 전체 화면 디텍션 / 선택형 3Hz 추적 설정은
[TensorRT Jetson 가이드](TENSORRT_JETSON.md)를 따른다. TensorRT 엔진이 있으면
최신 `.pt`보다 우선 선택한다. 관제 화면은 모든 객체 검출을 기본으로 한다.

현재 20261013 모델을 사용한다. 2026-10-13 실시간 관측 후 사용자 요청으로 cup=0.50 / banana=0.50 / 나머지=0.25로 조정했다. [우분투 인수인계](UBUNTU_HANDOFF_20261013.md)는 최초 게시 당시 cup=0.70 설정의 기록이다. 새 임계값의 정확도·성능은 아직 측정하지 않았다. 기존 Head ROS RGB 토픽을 구독해 모든 6클래스를 검출하고 브라우저 화면과 SSH 터미널 FPS를 제공한다. 카메라나 로봇 구동 노드를 새로 시작하지 않는다.

Jetson SSH 셸에서 저장소 상태를 먼저 확인하고 최신 main을 받는다. 로컬 수정이 있으면 그 내용을 보존한 상태로 처리한다.

```bash
cd ~/INHA-RoboCup-Home-2th-2027
git status --short
git pull --ff-only
source /opt/ros/humble/setup.bash
source ~/venvs/head_detection/bin/activate
python SW/detection/head/scripts/live_ros_viewer.py
```

물리 센서 실행기의 `/head/calibrated/color/image_raw`가 기본 입력이다. 센서 노드와 같은 `ROS_DOMAIN_ID`를 사용한다. 보정 토픽이 없고 원본 Head 카메라가 실행 중이라면 `--topic /head/camera/color/image_raw`를 추가한다. 입력이 없으면 터미널에 waiting for camera input이 출력된다.

YOLO GPU venv와 ROS source를 함께 사용한다. checkpoint는 저장소의 `models/head_yolo11n_20261013.pt`가 기본값이며 새 6클래스 재학습 모델이다. 출력 이름은 green_apple, peach, tomato, fanta_can, cup, banana다.

Windows에서 화면을 보려면 **별도 PowerShell**에 기존 SSH 로그인 방법으로 포트 전달을 연결한다. 인증은 사용자가 수행한다.

```powershell
ssh -N -L 8787:127.0.0.1:8787 sparo@192.168.50.70
```

Windows 브라우저의 **http://127.0.0.1:8787/**를 열면 현재 카메라 영상과 검출 박스가 표시된다. Jetson 자체 브라우저에서도 같은 주소로 볼 수 있다. 스트림은 Jetson loopback에만 바인딩하고 SSH 터널을 통해 전달한다. 설치·X11·Qt GUI 없이 기존 OpenCV와 Python HTTP 서버를 사용한다.

터미널은 매초 실제 측정한 다음 값을 출력한다.

```text
Camera FPS=... | Detection FPS=... | Tracking FPS=... | Preview FPS=... | detect=... ms | callback-to-preview=... ms
```

Camera FPS는 이 구독 콜백에 도착한 이미지 수, Detection FPS는 실제 YOLO 호출 수, Tracking FPS는 선택형 OpenCV 추적 프레임 수, Preview FPS는 JPEG 생성 수다. detect 시간은 YOLO adapter 호출 전체의 wall time이며 순수 GPU 커널 시간과 다르다. callback-to-preview는 해당 프레임 콜백부터 JPEG 준비까지의 시간으로, 카메라 캡처·네트워크·브라우저 렌더링 지연은 포함하지 않는다. 오래된 입력은 쌓지 않고 가장 최신 프레임으로 교체한다. 관제 디텍션 재사용 모드에서는 JSON 상태 메시지의 Camera/Detection FPS와 미리보기 JPEG FPS를 표시한다.

기본 imgsz 640, confidence 0.25, **cup 전용 confidence 0.50 / banana 전용 confidence 0.50**, IoU 0.7이다. `--cup-conf`와 `--banana-conf`로 각 클래스 기준을 변경한다. 작은 객체 검출 비교에는 `--imgsz 960`을 사용할 수 있다. 카메라의 실제 입력 FPS에 따라 디텍션 FPS도 제한된다. 이 실행기는 카메라 fps·전력 모드·센서 설정을 바꾸지 않는다.

디텍션 종료는 실행 SSH 터미널에서 **Ctrl+C**, 포트 전달 종료는 별도 PowerShell에서 **Ctrl+C**다. 사용 중인 다른 센서·로봇 프로세스에는 종료 명령을 보내지 않는다.
