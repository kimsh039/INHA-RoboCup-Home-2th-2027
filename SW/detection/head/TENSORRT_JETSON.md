# Jetson TensorRT와 전체 객체 검출 · 2026-10-13

현재 학습 가중치는 `models/head_yolo11n_20261013.pt`다. 이전 Head 학습 가중치
`head_yolo11n_20261007.pt`는 사용자의 요청으로 작업 복사본에서 삭제했다.
이전 학습·영상 기록은 당시 결과를 보존한다. COCO 초기 학습용 가중치와
Wrist/SAM 모델은 별도 용도로 유지한다.

Jetson에서 생성한 `head_yolo11n_20261013.engine`을 우선 사용한다. 엔진이 없는
PC는 최신 `.pt`를 사용한다. `.pt`는 재학습·엔진 재생성의 원본으로 유지한다.
엔진은 Git에 넣지 않으며 배포 대상 GPU/JetPack/TensorRT 환경에서 다시 생성한다.

## FP16 엔진 생성

Jetson TensorRT 10.3 / CUDA 12.6 / torch 2.8 / Ultralytics 8.4.6 환경을 사용한다.
기존 `~/venvs/head_detection`에 ONNX 1.17.0 / protobuf 4.25.8을 추가했다.
FP16 엔진 생성은 완료됐으며 최초 빌드 로그 기준 약 443초, 엔진 약 8.2MB다.
시스템의 TensorRT Python binding과 OpenCV contrib를 사용한다.

```bash
cd ~/INHA-RoboCup-Home-2th-2027
export ULTRALYTICS_AUTOINSTALL=False
~/venvs/head_detection/bin/python SW/detection/head/scripts/export_tensorrt.py
```

고정 입력 640×640, batch=1, FP16, opset=17, workspace=2 GiB다.
다른 입력 크기가 필요하면 최신 `.pt`로 실행하거나 엔진을 다시 생성한다.
최초 엔진 생성 로그는 `~/robot_setup/logs/head_tensorrt_export_20261013.log`에 남긴다.

## 현재 기본 동작

현재 실행 상태에서 사용자는 디텍션 FPS가 25 아래로 떨어지지 않는다고 보고했다.
이는 현장 관측이며 별도의 성능 벤치마크 결과는 아니다. 브라우저 미리보기의
기본 최대 15FPS는 YOLO 실행 FPS와 별개다.

관제센터는 전체 화면 YOLO로 모든 객체의 박스·클래스·신뢰도를 보여준다.
ROI 재검출·단일 목표 추적은 기본 화면에서 사용하지 않는다.
`/detection/head/objects`에는 전체 객체 JSON을, `/detection/head/debug_image`에는
주석 영상을 발행한다. 관제센터 실행 시 디텍션이 기본 활성화되며
`--no-detection`으로 비활성화할 수 있다.
카메라·LiDAR·모터 관련 기존 관제 수정 사항과는 독립적인 검출 실행기다.

신뢰도는 **cup=0.50 / banana=0.50 / 나머지=0.25**, NMS IoU=0.7이다.
클래스 매핑과 신뢰도 필터를 거친 후보 중 **클래스별 최고 신뢰도 박스 하나만**
남긴다. 동일 클래스 후보가 여러 개여도 위치·겹침 여부와 무관하게 하나만
표시·발행하며, 동일 신뢰도라면 먼저 나온 후보를 유지한다. 후보가 없는
클래스에는 박스를 만들지 않는다. 따라서 전체 출력은 최대 6개다.
이 규칙은 공통 YOLO adapter에서 적용되어 관제센터, ROS 노드, 순수 디텍션
미리보기와 선택형 KCF 미리보기에 동일하게 적용된다. 엔진 재생성은 필요 없다.
관제 실행기는 `~/venvs/head_detection/bin/python`을 기본으로 사용한다.
`ROBOCUP_MODEL`을 명시하면 그 경로가 우선하므로 삭제된 이전 모델을 지정하지 않는다.

```bash
# 이미 카메라가 실행 중인 SSH 셸에서 관제용 전체 객체 검출만 실행
bash ~/robocup/SW/control_center/start-head-detection.sh
```

ROS `HeadTarget`을 사용하는 별도 노드도 `detection_only: true`를 기본으로 한다.
이 메시지는 최고 신뢰도 목표 하나를 보내며 프레임마다 새로운 관측 ID를 사용한다.
시뮬레이션 추적 설정은 `detection_only: false`로 기존 동작을 명시한다.

## 선택 가능한 3Hz 검출 + OpenCV 화면 추적

관제 기본 모드는 전체 화면 디텍션이다. 아래 저빈도 모드는 별도 미리보기에서만 선택한다.
관제 디텍션이 이미 실행 중이면 기본 미리보기는 그 주석 영상과 FPS를 재사용한다.
YOLO를 두 번 실행하지 않는다. 3Hz 모드 비교는 관제 디텍션을 종료한 뒤 수행한다.

### PC / Jetson 실행기 설치

현재 사용 중인 실행기와 빈 인자 전달 오류 수정도 `scripts/`에 저장한다.
Linux PC에서 아래 명령을 실행하면 브라우저 자동 열기 실행기가 설치된다.

```bash
bash SW/detection/head/scripts/install_detection_helpers.sh --local
```

Linux Ubuntu PC의 OpenCV 창으로 보려면 `~/.local/bin/robocup-detection-window`를
실행한다. 시스템 Python에 GUI 지원 OpenCV와 NumPy가 필요하다. 브라우저를
열지 않으며 기존 Jetson MJPEG 영상만 받아 최신 프레임을 표시한다.
전송은 기존 SSH 터널을 사용하고 YOLO를 추가 실행하지 않는다.
종료는 창에서 Q / Esc 또는 터미널 Ctrl+C다. 터미널 Window FPS는
이 창에 새로 전달한 영상 수이며 디텍션 FPS와 별개다.
현재 사용 설정은 미리보기 최대 15FPS다. Camera/Detection FPS와 구분한다.
관제 디텍션이 실행 중이면 그 결과를 재사용하며, 별도 YOLO를 실행하지 않는다.
Jetson에서 미리보기를 시작하고 PC에서 팝업을 열려면 아래처럼 실행한다.

```bash
# Jetson SSH 터미널: Camera / Detection / Preview FPS 출력
~/robot_setup/bin/head-detection-test --preview-fps 15

# Ubuntu PC 로컬 터미널: OpenCV 영상 창 + Window FPS 출력
~/.local/bin/robocup-detection-window
```

원본 미리보기 제한은 그대로 적용되므로 더 높은 전송률을 비교하려면
Jetson의 기존 미리보기 실행을 Ctrl+C로 종료한 뒤
`~/robot_setup/bin/head-detection-test --preview-fps 30`으로 다시 실행한다.
30FPS 설정은 더 많은 JPEG 생성·전송 작업을 요구하며, 카메라·검출·화면의
실제 FPS가 30이 되는 것을 보장하지 않는다. 2026-10-13 현장 진단에서는
추론 시간 약 24~27ms인 상태에서도 독립 카메라 구독률이 약 8~25FPS로
흔들렸고 카메라 USB 통신 오류가 기록됐다. USB 오류가 해당 하락의 직접
원인인지는 확정하지 않았다.

Jetson에서는 아래 명령으로 카메라·디텍션 도우미를 설치한다.
현재 배포 경로인 `~/INHA-RoboCup-Home-2th-2027`과
`~/venvs/head_detection`을 사용한다. SSH 주소는 `sparo@192.168.50.70`,
공유 연결은 `/tmp/robocup-ssh-control`, 미리보기 포트는 8787이다.

```bash
bash ~/INHA-RoboCup-Home-2th-2027/SW/detection/head/scripts/install_detection_helpers.sh --jetson
```

```bash
# PC 로컬 셸: 순수 TensorRT 디텍션 + 브라우저 자동 열기
~/.local/bin/robocup-detection

# PC 로컬 셸: YOLO 약 3Hz + 나머지 프레임은 OpenCV KCF
~/.local/bin/robocup-detection --yolo-hz 3
```

매 YOLO 실행에서 전체 객체를 다시 검출하고 KCF 추적기를 새로 만든다.
추적 실패 시 해당 박스를 제거하며, YOLO 검증 나이가 0.5초를 넘거나
카메라 시각·해상도가 바뀌면 추적을 폐기한다. 추적 프레임의 confidence는
마지막 YOLO의 값이며 새로운 검출 신뢰도가 아니다. 추적 출력은 주행 명령으로 사용하지 않는다.

출력은 Camera FPS / Detection FPS(실제 YOLO 호출) / Tracking FPS /
Preview FPS / detect ms를 구분한다. 브라우저 JPEG 생성은 기본 최대 15FPS이며
`--preview-fps 30` 등으로 변경할 수 있다. 카메라 지연과 모델 정확도 개선을 보장하는 설정은 아니다.

엔진 생성과 실행 연결은 배포 작업이다. 이전 PyTorch 로그와 다른 조건의
FPS를 섞어 속도 향상을 계산하지 않는다.
