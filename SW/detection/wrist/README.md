# Wrist Detection

Wrist D435로 접근 후 목표를 다시 관측하고, YOLO bbox를 SAM 2.1 Tiny의 prompt로 사용해 물체 마스크와 정합 depth 점군을 생성하는 경로입니다.

- [RealSense·SAM 실험 코드](realsense_sam2/README.md)
- [공통 센서 역할](../SENSOR_ROLES.md)
- [헤드→손목 전환과 파지 입력 설계](../PIPELINE.md)
- [Jetson 실행 명령](../../setup/jetson/RUN_COMMANDS.md)

현재 `realsense_sam2/`는 기존 카메라·분할 실험입니다. 두 D435의 실기 통합이나 GraspNet 연결 완료를 뜻하지 않습니다. 이번 7개 클래스 Head 학습 결과는 [head/training](../head/training/README.md)에서 관리하며, Wrist 시점 학습·평가는 별도 실험으로 기록합니다.
