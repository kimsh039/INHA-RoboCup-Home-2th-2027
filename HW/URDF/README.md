# CAD·메시·URDF 사용 기준

최신 날짜의 파일 하나를 모든 실행에 공통 적용하지 않는다.

| 파일 | 용도 |
|---|---|
| [robocup.urdf](../../SW/simulation/robot_description/robocup.urdf) | nominal CAD 기준, Gazebo 생성 원본; 94 links / 93 joints |
| [robocup.calibrated.urdf](../../SW/simulation/robot_description/robocup.calibrated.urdf) | 보정 통합·시뮬레이션 기록; TCP 포함 95 / 94 |
| [robot.physical.urdf](../../SW/robot/robot.physical.urdf) | 실물 base+PiPER 부분 기구학 모델; 센서 TF 별도 |
| [실물 실행](../../SW/robot/README.md) | 실측 JSON + RealSense 내부 TF + 실제 팔 피드백 |

nominal/calibrated의 현재 질량 합은 51.0071956954071kg이며 과거 문서의 51.16914kg은 이전 형상 기록이다. 전체 모델에는 상자/원통 근사 관성과 충돌 형상 등 가정이 포함돼 있다.

## CAD에서 통합한 방식

Fusion 컴포넌트 변환·물성을 추출하고 링크 로컬 좌표의 미터 단위 STL을 사용한다. URDF xyz는 m, rpy는 rad다. visual·collision·inertial을 구분하고 고정 센서는 fixed joint로 연결했다. 카메라 CAD 원점과 실제 RGB/depth optical 원점은 같다고 가정하지 않는다.

- [Tracer 물성 산출](tracer/README.md), [PiPER 원본/라이선스](piper/README.md).
- [랙·15° 마운트](sensor_rack_description/README.md): 메시뿐 아니라 sensor pose·관성·충돌 상자를 함께 갱신.
- [손목 장착](wrist_camera_description/README.md): 그리퍼 몸통의 자식으로 연결; 손가락 개폐에는 상대 위치 유지.
- [손목 정합 계산](WRIST_CAMERA_INTEGRATION.md): CAD 형상 정합과 실측 hand-eye 구분.
- [모델/보정 이력](../../SW/simulation/robot_description/README.md).

기존 독립 부품 URDF를 다시 만들지 않는다. 실물 실행의 sensor_tf와 전체 URDF robot_state_publisher가 같은 센서 프레임을 중복 소유하지 않도록 [실물 실행 기준](../../SW/robot/README.md)을 따른다.
