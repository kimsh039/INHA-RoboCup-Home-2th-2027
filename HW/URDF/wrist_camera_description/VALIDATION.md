# 손목 카메라 통합 검증 기록

> 아래는 통합 전후의 역사적 생성·Gazebo 검증 기록입니다. 당시 build.py와 선택형 URDF는 이후 정리됐습니다. 현재 최종 모델은 [robocup.urdf](../../../simulation/robot_description/robocup.urdf) 하나이며 2026-10-06 랙 변경 후 94 links / 93 joints입니다. 아래 질량·실행 검증은 해당 변경 전의 기록입니다. 현재 실기 선택은 Head·Wrist 모두 D435이며, 이 기록은 두 D435의 실기 검증 결과가 아닙니다.

2026-10-03, Windows에서 생성 파일을 읽어 확인했다.

- 기본 모델과 `--with-wrist-camera` 모델 모두 생성 성공. 기본 모델은 86/85 links·joints, 카메라 모델은 90/89이다. Gazebo 버전은 각각 85/84, 89/88이다.
- 단일 `base_link` 루트, 순환 없음, 이름 중복 없음, 메시 파일 존재 확인.
- 모든 메시 경로는 `/`를 사용하며 Ubuntu용 상대 경로로 저장했다.
- STL 두 개의 유한 좌표·폐곡면 및 미터 단위를 확인했다. 마운트 외형은 약 60×13×61.5 mm, 카메라 외형은 약 89.90×25.00×25.65 mm이다.
- URDF에서 계산한 `piper_gripper_base` → 카메라 CAD 변환이 저장된 정합 결과와 수치 오차 1e-10 이내로 일치했다.
- 6번 관절을 0 → 0.7 rad 변경했을 때 카메라 전역 자세가 바뀌고, 그리퍼 몸통 기준 상대 자세가 유지되는지 확인했다.
- 손가락 개폐 관절을 0 → 0.04 m 변경해도 카메라 자세가 유지되는지 확인했다.
- 정의한 카메라 body +X와 optical +Z 방향이 일치한다. 실제 렌즈 보정값을 검증한 것은 아니다.
- 관성 텐서의 양의 고유값과 삼각부등식을 확인했다. 양쪽 통합 모델 총 질량은 51.1691449781 kg이다.
- Gazebo 버전에 기존 센서 4개와 손목 센서 2개가 있고 센서 이름이 중복되지 않는다.
- Python 구문 및 ROS bridge YAML 구문·토픽 이름 중복을 검사했다.

Ubuntu의 ROS 노드·Gazebo 실행, fixed-joint lumping 후 생성 SDF, 이미지/CameraInfo/포인트클라우드 실제 발행, 렌즈 방향·간섭은 아직 확인하지 않았다. README의 실행·TF·토픽 확인 절차로 후속 검증한다.

## Ubuntu Gazebo 확인 (2026-10-03)

- `build.py --with-wrist-camera`로 표준 URDF 두 파일을 생성하고 `make_sim.py --world room`으로 변환했다.
- NVIDIA PRIME 환경에서 Gazebo GUI를 실행했다. fixed-joint lumping 후 손목 센서는 `piper_gripper_base`에 합쳐졌다.
- 헤드와 손목의 RGB 1920×1080/RGB_INT8 및 depth 1280×720/R_FLOAT32 메시지를 실제 수신했다.
- 손목 frame_id는 `wrist_camera_optical_frame`이며 헤드와 토픽·프레임이 구분됐다. 손목 depth에서 유효 거리 픽셀을 확인했다.
- 6번 관절 0.2 rad 명령 후 실제 관절 값 0.20000000000005 rad를 확인하고 0으로 복귀시켰다.
- `robot-camera --partition robocup_motion --camera wrist`로 RGB/depth 팝업을 실행했다.
- CAD 정합 장착값은 유지했으며 실물 체결·간섭·렌즈 외부 보정 검증은 포함하지 않았다.
