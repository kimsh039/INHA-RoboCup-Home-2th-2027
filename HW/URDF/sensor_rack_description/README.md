# 보관 자료 안내

최종 모델 확정으로 독립 랙 URDF·ROS 패키지 실행 파일은 삭제했습니다. 메시·CAD 검증 보고서는 유지합니다.
최종 파일은 [robocup.urdf](../../../simulation/robot_description/robocup.urdf)이며 실행은 [Gazebo 안내](../../../simulation/gazebo/README.md)를 사용하세요.
아래는 최초 랙 내보내기 당시 좌표·물성 기록입니다.

# 센서 프레임 URDF

Fusion의 `final_assembly`를 기준으로 내보낸 고정형 센서 랙입니다. 이동 플랫폼 본체와 매니퓰레이터는 포함되어 있지 않습니다.

## 기준 좌표

- `base_link`: 하단 사각 프로파일 프레임 바닥면의 중앙.
- +X: 상단 카메라가 바라보는 방향. +Y: 왼쪽. +Z: 위.
- Fusion 원래 전역 좌표에서 새 원점은 `(-152.711294, -135.000000, -47.772226) mm`입니다. 회전 없이 원점만 이동했습니다.
- 프로파일, 브래킷, 마운트 46개 본체를 `base_link`로 묶었습니다.
- 원본 54개 구성요소의 위치·축은 `cad_*` 고정 프레임으로 모두 보존했습니다. 원본 부품 이름과의 대응은 함께 제공한 `export_report.json`에서 확인할 수 있습니다.

| 프레임 | base_link 기준 위치 (mm) | 축 / 주의사항 |
|---|---|---|
| g2_link | 0.175, 0, 326.600 | 랙 기준 축과 평행 |
| laser_frame | 0.175, 0, 326.600 | G2 TOP의 기존 CAD 축, Z 위. yaw 약 8.81도. 실제 스캔 영점은 드라이버 보정값과 확인 필요 |
| mid360_link | -180.000, 0, 1183.000 | 랙 기준 축과 평행 |
| livox_frame | -180.000, 0, 1183.000 | 기존 Mid360 CAD 축. X 전방, Y 오른쪽, Z 아래로 향하는 현재 설치 방향 보존 |
| camera_link | -124.950, 0, 1270.000 | 카메라 CAD 전면 중심 위치, 랙 기준 축과 평행 |
| camera_cad_frame | -124.950, 0, 1270.000 | 기존 카메라 CAD 축 보존 |
| camera_optical_frame | -124.950, 0, 1270.000 | X 오른쪽, Y 아래, Z 전방. 개별 이미지 센서의 검증된 광학 원점은 아님 |

카메라 광학 원점, G2 스캔 평면 높이·영점, Livox 좌표의 제조사 기준 일치는 실측 또는 제조사 도면으로 최종 확인해야 합니다. 정확한 RealSense depth/color/infrared TF는 RealSense 드라이버의 보정된 변환을 사용하세요. 현재 출력은 사용자가 모델에 설정한 CAD 원점과 축을 보존한 결과입니다.

## 재료와 질량

프로파일, 브래킷, 마운트는 모두 알루미늄으로 가정했습니다. Fusion 라이브러리의 **알루미늄 6061**을 적용하고 CAD 체적에서 질량·무게중심·관성텐서를 계산했습니다. 실제 6063-T5 프로파일과의 합금 차이, 마운트가 다른 재료일 가능성은 따로 확인하세요. URDF의 재료 색상과 물리량은 별개이며, 물리량은 `<inertial>`에 기록됩니다.

| 링크 | 질량 (kg) | 계산 방법 |
|---|---:|---|
| base_link | 15.854080 | 알루미늄 CAD 물성 |
| g2_link | 0.185 | G2 V1.3 제조사 데이터시트 명목 질량 |
| mid360_link | 0.265 | Mid-360S 제조사 사양 |
| camera_link | 0.075 | D435f 명목 질량 |
| 합계 | 16.379080 | 현재 랙에 포함된 부품만 합산 |

센서 내부 재료 분포가 없으므로 센서의 무게중심과 관성은 CAD 솔리드를 균일한 밀도로 보아 명목 질량에 비례 조정했습니다. 표면 본체는 시각 형상에 포함하고 질량에는 포함하지 않았습니다. Fusion F3D의 센서 재료는 원래 CAD 재료이므로 센서 질량 보정은 URDF에 적용되어 있습니다. Fusion 속성창에서 센서 질량을 읽으면 URDF의 명목 질량과 다릅니다.

G2 구형 데이터시트에는 214g, V1.3에는 185g이 기재되어 있습니다. 실제 장치 무게가 214g이면 g2_link의 질량과 관성텐서 6개 값을 `214/185`배로 바꾸세요.

출처:

- [YDLIDAR G2 V1.3 제조사 데이터시트](https://www.ydlidar.com/Public/upload/files/2022-06-21/YDLIDAR%20G2%20Data%20Sheet%20V1.3%28211230%29.pdf): 현재 원본 링크는 접근이 불안정하며 검색된 제조사 문서에 185g이 명시되어 있습니다.
- [YDLIDAR 제조사 구형 데이터시트 사본](https://www.generationrobots.com/media/YDLIDAR_G2_Datasheet.pdf): 214g.
- [Livox Mid-360S 사양](https://www.livoxtech.com/mid-360s/specs): 265g.
- [RealSense D400 데이터시트](https://www.realsenseai.com/download/21345/?tmstv=1770190765): D435/D435f 계열 명목 75g, 실제 질량 편차 존재.

## ROS 2 실행

이 폴더를 Ubuntu ROS 2 워크스페이스의 `src` 아래에 복사한 뒤 실행합니다.

```bash
cd ~/ros2_ws
colcon build --packages-select sensor_rack_description
source install/setup.bash
ros2 launch sensor_rack_description display.launch.py
```

RViz의 Fixed Frame은 `base_link`입니다. 모든 관절이 fixed이므로 joint_state_publisher가 필요하지 않습니다. 실제 센서 드라이버의 frame_id가 `laser_frame` 또는 `livox_frame`과 맞는지 확인하고 동일한 TF를 다른 노드에서 중복 발행하지 마세요.

메시는 미터 단위이며 URDF scale은 1입니다. 충돌 형상은 각 솔리드의 경계 박스로 단순화해 프로파일 홈과 구멍을 표현하지 않습니다. 정밀 접촉 시뮬레이션에는 별도의 충돌 형상이 필요합니다.

base_link에 루트 관성이 있어 KDL은 루트 관성을 지원하지 않는다는 경고를 낼 수 있습니다. TF/RViz 표시는 가능합니다. 이동 로봇 URDF에 통합할 때 플랫폼 링크를 부모로 두고 랙 base_link를 고정 연결하거나, KDL 용도라면 질량 없는 부모 링크를 추가하세요.

검증: XML 연결 트리, 4개 STL의 단위·경계·유효 삼각형, 양의 관성과 관성 삼각부등식, 원본 54개 구성요소의 좌표 변환을 검사했습니다. ROS 2/RViz 실구동은 이 Windows 환경에서 수행하지 않았습니다.
