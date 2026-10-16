# Hardware

CAD 형상·메시·질량/관성·실물 보정 자료를 보관한다. 실행 소프트웨어는 [SW](../SW/README.md)에서 관리한다.

| 자료 | 내용 |
|---|---|
| [통합 URDF 안내](URDF/README.md) | nominal / calibrated / physical arm 모델 구분 |
| [Tracer](URDF/tracer/README.md) | 차체·바퀴 메시와 30kg 명목 물성 가정 |
| [PiPER](URDF/piper/README.md) | 팔·그리퍼 링크/관절과 라이선스 |
| [센서 랙](URDF/sensor_rack_description/README.md) | Fusion 추출·15° Head 마운트·관성 |
| [손목 카메라](URDF/wrist_camera_description/README.md) | CAD 장착·STEP/STL·잠정 배치 |
| [실물 보정 입력](calibration/physical/README.md) | Base/LiDAR/Head/Wrist/오도메트리 원본 |
| [현재 Wrist hand-eye](calibration/physical/wrist_handeye/README.md) | 실물 sensor_tf가 읽는 최신 JSON |
| [RGB 내부 보정](calibration/camera_intrinsics/README.md) | 장치별 K/D와 결과 |

CAD 배치, 시뮬레이션 solver 잔차, 실제 실물 보정은 서로 다른 근거다. [발표 가이드](../SW/PRESENTATION_STUDY_GUIDE.md)에 좌표계와 검증 범위를 정리했다.
