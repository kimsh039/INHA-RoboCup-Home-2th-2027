# 로봇 메시·CAD 자료

독립 부품 URDF는 최종 모델 확정으로 삭제했습니다.
유일한 모델은 [robocup.urdf](../../simulation/robot_description/robocup.urdf)입니다.
이 폴더의 메시들은 최종 URDF가 참조하므로 유지합니다.

2026-10-06 프로파일 형상 및 Piper 베이스 장착 위치 변경을 반영했습니다. [최신 랙 변경 기록](sensor_rack_description/README.md#2026-10-06-프로파일-변경)을 참고하세요.

- [Tracer](tracer/README.md): 차체·바퀴 메시와 질량/관성 계산 기록.
- [Piper](piper/README.md): 팔·그리퍼 메시와 라이선스.
- [센서 랙](sensor_rack_description/README.md): 프로파일·라이다·헤드 카메라 메시와 CAD 보고서.
- [손목 카메라](wrist_camera_description/README.md): 마운트·카메라 메시, STEP, 장착 변환.

[Gazebo 실행](../../simulation/gazebo/README.md) · [제어·카메라](../../simulation/tools/README.md)
