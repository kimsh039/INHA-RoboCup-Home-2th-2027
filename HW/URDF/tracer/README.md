# Tracer 기본형 URDF

로컬에서 사용하던 `tracer_v1.urdf`와 이 모델이 참조하는 메시 파일입니다.
메시 경로만 `meshes/` 상대 경로로 바꾸었으며 링크·조인트·형상·물성은 유지했습니다.
URDF와 `meshes/` 폴더를 함께 보관하세요. 로더가 URDF 파일 위치를 기준으로 상대 경로를 해석하도록 설정합니다.

- `tracer_v1.urdf`: 기본형 Tracer. 센서 랙·Piper는 결합되어 있지 않습니다.
- `meshes/`: 시각·충돌 형상에 필요한 DAE 파일.

이 URDF는 공식 Xacro에서 생성한 로컬 보기용 모델입니다. Gazebo Classic ROS 제어 플러그인과 transmission은 로컬 생성 과정에서 제거되어 주행 제어는 포함하지 않습니다.

원본: [AgileX tracer_ros / tracer_description](https://github.com/agilexrobotics/tracer_ros/tree/master/tracer_description)
