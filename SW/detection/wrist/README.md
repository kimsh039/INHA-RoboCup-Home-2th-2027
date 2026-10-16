# Wrist 인지 · 현재 SAM 2.1 실행 위치

현재 손목 인지는 별도 USB Small 실험이 아니라 **[실물 manipulation 패키지](../../manipulation/README.md)** 안에서 실행한다.

| 항목 | 현재 기본값/경로 |
|---|---|
| frontend | [wrist_frontend.py](../../manipulation/manipulation_ws/src/robocup_real_manipulation/robocup_real_manipulation/wrist_frontend.py) |
| 설정 | [config/wrist.yaml](../../manipulation/manipulation_ws/src/robocup_real_manipulation/config/wrist.yaml) |
| 기본 분할 | `mask_source: sam2_auto` |
| 모델 | `sam2.1_hiera_tiny.pt` + `configs/sam2.1/sam2.1_hiera_t.yaml` |
| 자동 prompt | points_per_side 8 / points_per_batch 64 |
| RGB | `/wrist/wrist/color/image_raw` |
| depth/내부 행렬 | RGB-aligned depth + **factory RGB CameraInfo** |
| 요청/응답 | `/manipulation/wrist/request` / `/manipulation/wrist/result` |

SAM은 클래스 인식을 하지 않는다. 요청에 담긴 Head 클래스·coarse 위치와 mask depth/크기로 물체 후보를 대응시킨다. RGB/depth/CameraInfo 시각을 맞추고 촬영 시각 TF로 위치·영역을 연결한다. mask 내부 유효 depth를 역투영해 관측 기하를 얻고, 바나나는 PCA 주축과 실제 몸통 픽셀의 파지점 조건을 사용한다.

`sam2` 모드는 YOLO bbox + SAM prompt를 선택하는 별도 경로이고, `sam2_auto` 기본 경로는 자동 마스크다. 접시 요청은 원형성·크기·height·빈 평면 영역을 확인하며 모호한 후보를 성공 처리하지 않는다. 자세한 계약은 [INTERFACES](../../manipulation/docs/INTERFACES.md)와 [전체 Task](../../control_center/TASK.md).

50ms 예산과 p50/p95 측정 코드는 성능 달성을 뜻하지 않는다. checkpoint는 Jetson 로컬에 있어야 하며 Git pull만으로 모델/설치 runtime/실물 프로파일이 준비되지는 않는다. 물체별 commissioning과 접시 관측 자세가 남아 있다.

## 이전 Small 실험

`realsense_sam2/`의 SDK 직접 접근 Small 실험과 비어 있던 requirements/test 파일은 현재 ROS runtime에서 참조하지 않아 정리했다. [삭제 전 원본](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/tree/a23d6e287a29ed34e65aaaab439b57405108636a/SW/detection/wrist/realsense_sam2). 당시 설정을 현재 Tiny runtime 성능으로 인용하지 않는다.
