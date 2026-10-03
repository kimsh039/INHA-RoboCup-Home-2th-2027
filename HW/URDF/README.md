# 로봇 URDF

| 모델 | 경로 | 대표 URDF |
|---|---|---|
| Tracer 기본형 | [tracer/](tracer/) | `tracer/tracer_v1.urdf` |
| Piper 팔·그리퍼 | [piper/](piper/) | `piper/piper_with_gripper.urdf` |
| 센서 랙 | [sensor_rack_description/](sensor_rack_description/) | `sensor_rack_description/urdf/sensor_rack.urdf` |

Tracer와 Piper는 로컬에서 사용하던 URDF와 참조 메시를 함께 저장한 모델입니다. 메시 경로는 각 URDF 파일 위치 기준의 상대 경로입니다. Piper의 팔 단독 모델과 Gazebo용 사본은 [Piper 설명](piper/README.md)을 참고하세요. 세 모델은 아직 하나의 로봇으로 결합되어 있지 않습니다.

## 센서 랙 URDF

Autodesk Fusion의 `final_assembly`에서 내보낸 센서 랙 모델입니다. 프로파일 프레임과 마운트, YDLIDAR G2, Livox Mid-360S, RealSense D435f를 포함합니다. 이동 플랫폼 본체와 매니퓰레이터는 포함하지 않습니다.

## 파일 구성

```text
HW/URDF/
├── README.md
├── tracer/                      # Tracer 기본형 URDF + meshes
├── piper/                       # Piper 팔·그리퍼 URDF + meshes
└── sensor_rack_description/       # ROS 2 ament_cmake 패키지
    ├── README.md                 # 좌표·물성·보정 가정 상세 설명
    ├── CMakeLists.txt
    ├── package.xml
    ├── urdf/sensor_rack.urdf
    ├── meshes/                   # 미터 단위 STL 4개
    ├── launch/display.launch.py
    ├── rviz/display.rviz
    ├── export_report.json        # 원본 구성요소와 프레임 대응, 질량·관성
    └── validation_report.json    # 파일 검증 결과
```

## 좌표계

`base_link`는 하단 사각 프로파일 프레임 **바닥면 중앙**입니다. +X는 상단 카메라 전방, +Y는 왼쪽, +Z는 위쪽입니다. 원본 Fusion 좌표계에서 원점을 `(-152.711294, -135.000000, -47.772226) mm`만큼 이동했고 전역 축 방향은 유지했습니다.

| 링크 / 프레임 | base_link 기준 위치 (mm) | 설명 |
|---|---|---|
| base_link | 0, 0, 0 | 프로파일·브래킷·마운트 46개 본체 |
| g2_link / laser_frame | 0.175, 0, 326.600 | G2 TOP의 CAD 원점 위치 |
| mid360_link / livox_frame | -180.000, 0, 1183.000 | 기존 Mid-360S CAD 원점 위치 |
| camera_link / camera_optical_frame | -124.950, 0, 1270.000 | 카메라 CAD 전면 중심 위치 |

센서 본체 링크는 base_link와 같은 축 방향입니다. `laser_frame`, `livox_frame`, `camera_cad_frame`은 기존 센서 CAD 축을 보존합니다. 특히 `livox_frame`의 Z축은 현재 설치 방향에 따라 **아래쪽**을 향합니다. `camera_optical_frame`은 X 오른쪽, Y 아래쪽, Z 전방입니다.

원본 구성요소 54개의 축과 위치도 `cad_*` 고정 프레임으로 보존했습니다. 전체 모델은 62개 링크와 61개 fixed joint로 구성됩니다.

## 재료와 물성

프로파일·브래킷·마운트는 **알루미늄 6061**로 가정하고 CAD 체적에서 질량, 무게중심, 관성을 계산했습니다. 실제 부품의 합금과 마운트 재료가 다르면 물성 갱신이 필요합니다.

| 링크 | 질량 (kg) | 기준 |
|---|---:|---|
| base_link | 15.854080 | 알루미늄 CAD 물성 |
| g2_link | 0.185 | G2 V1.3 데이터시트 명목 질량 |
| mid360_link | 0.265 | Mid-360S 제조사 명목 질량 |
| camera_link | 0.075 | D435f 제조사 명목 질량 |
| 총합 | 16.379080 | 포함된 센서 랙 부품만 합산 |

센서 관성과 무게중심은 내부 재료 분포를 알 수 없어 CAD 솔리드를 균일 밀도로 보고 명목 질량에 맞춰 근사했습니다. 질량과 관성은 URDF의 `<inertial>`에 기록되어 있으며, 화면 표시용 재료 색상과 별개입니다. 제조사 출처와 G2 구형 모델의 214g 사양 차이는 [패키지 README](sensor_rack_description/README.md)에 정리했습니다.

## ROS 2에서 실행

`sensor_rack_description` 폴더를 ROS 2 워크스페이스의 `src` 아래에 복사합니다. 예를 들어 저장소를 워크스페이스 외부에 받았다면:

```bash
mkdir -p ~/ros2_ws/src
cp -r /path/to/INHA-RoboCup-Home-2th-2027/HW/URDF/sensor_rack_description ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select sensor_rack_description
source install/setup.bash
ros2 launch sensor_rack_description display.launch.py
```

RViz의 Fixed Frame은 `base_link`입니다. 모든 관절이 고정이므로 joint_state_publisher는 필요하지 않습니다. 실제 센서 드라이버의 frame_id를 확인하고, 동일한 TF를 중복 발행하지 않도록 연결하세요.

## 검증 범위와 보정할 항목

- XML 링크 연결, 메시 파일 존재·단위·경계·유효 삼각형, 관성의 양수성과 삼각부등식을 검사했습니다.
- 원본 CAD 구성요소 54개의 위치와 회전을 내보낸 프레임과 비교했습니다.
- ROS 2/RViz 실구동은 생성 당시 Windows 환경에서 수행하지 않았습니다.
- **센서 CAD 원점은 검증된 광학 원점과 동일하다고 보장하지 않습니다.** G2 스캔 평면 높이와 영점, Livox 제조사 좌표계와의 일치, RealSense 개별 이미지 센서 광학 원점은 도면·실측·드라이버 보정 TF로 확인해야 합니다.
- 충돌 형상은 각 솔리드의 경계 박스입니다. 프로파일 홈과 구멍은 충돌 형상에 표현하지 않습니다.
- 이동 로봇에 통합할 때는 실제 플랫폼 기준 좌표에 랙을 고정 연결해야 합니다. 현재 base_link는 랙 자체의 기준입니다.

자세한 센서 축 방향, 질량 출처, KDL 루트 관성 관련 설명은 [상세 README](sensor_rack_description/README.md)를 참고하세요.
