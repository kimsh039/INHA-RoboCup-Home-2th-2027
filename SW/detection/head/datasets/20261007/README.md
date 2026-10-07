# Head camera detection dataset · 2026-10-07

헤드 카메라 영상 7개 중 5개에서 **원본 프레임 0, 2, 4, …**를 추출한 YOLO detection 데이터셋입니다. 1920×1080 JPEG 이미지와 동일 파일명의 YOLO TXT 라벨을 포함합니다. 두 테스트 영상은 원본 영상으로 따로 보관하며 학습·검증에 포함하지 않습니다.

| 영상 | 분리 | 이미지 수 |
|---|---|---:|
| WIN_20261006_23_47_12_Pro.mp4 | train | 335 |
| WIN_20261006_23_48_36_Pro.mp4 | train | 370 |
| WIN_20261006_23_49_36_Pro.mp4 | test video | 0 |
| WIN_20261006_23_50_08_Pro.mp4 | train | 173 |
| WIN_20261006_23_50_46_Pro.mp4 | val | 175 |
| WIN_20261006_23_51_34_Pro.mp4 | train | 209 |
| WIN_20261006_23_52_09_Pro.mp4 | test video | 0 |

**총 1,262장: train 1,087장 / val 175장**, 라벨 초안 8,386개입니다. 영상 전체 단위로 분리했으며 두 테스트 영상은 `test_videos/`에 있습니다. 원본 학습 영상은 이 저장소에 포함하지 않습니다.

## 클래스와 라벨

| ID | 클래스 | 기준 |
|---:|---|---|
| 0 | apple | 빨간 사과 |
| 1 | banana | 송이 전체를 한 객체로 지정 |
| 2 | fanta_can | 캔 전체; 로고가 안 보이는 방향 포함 |
| 3 | green_apple | 초록 사과 |
| 4 | mug | 보이는 손잡이 포함 |
| 5 | peach | 복숭아 |
| 6 | plate | 음식이 아닌 접시 자체 |

식별 가능한 객체를 모두 라벨링합니다. 접시 위 과일은 접시와 별도로 라벨링하며 박스 겹침을 허용합니다. TXT 형식은 `class_id cx cy width height`이며 좌표는 이미지 크기로 정규화한 0–1 값입니다.

라벨은 기준 프레임에서 지정한 박스를 인접 프레임의 optical flow로 전달한 **검토용 초안**입니다. 사람이 모든 프레임을 확정한 정답은 아닙니다. 작은 과일·가림·머그 손잡이·화면 경계 구간은 확인과 수정이 필요합니다. 형식 검증과 이미지·라벨 대응 검사는 완료했으며, 이 검사가 박스의 시각적 정확성을 보장하지는 않습니다. 현재 검토 상태는 [review_state.json](review_state.json)에 있습니다.

## YOLO11n 사용

설치된 Ultralytics 환경에서 **저장소 루트**를 기준으로:

```bash
yolo detect train model=yolo11n.pt \
  data=SW/detection/head/datasets/20261007/data.yaml \
  epochs=100 imgsz=960 batch=8 device=0 seed=42
```

`data.yaml`의 경로는 YAML 파일이 있는 폴더를 기준으로 해석됩니다. Colab에서는 이 폴더 전체를 내려받거나 ZIP으로 묶어 업로드한 뒤 같은 YAML을 사용하면 됩니다. 학습 모델과 평가 결과는 이후 별도 커밋으로 등록합니다.

## 두 테이블 탐색용 메타데이터

- [frames.csv](frames.csv): 추출 이미지와 원본 프레임 번호·시간 대응.
- [object_table_assignments.csv](object_table_assignments.csv): 최초 라벨 초안의 객체–테이블 연결.
- [video_table_inventory.json](video_table_inventory.json): 기준 장면에서 확인한 테이블별 클래스 배치 초안.
- [audit_report.json](audit_report.json): 라벨 형식, split 중복, 테스트 원본 SHA256 검사.

`table_left` / `table_right`는 최초 방 시야에서 정의한 **물리 테이블 ID**이며 카메라가 돌 때 화면 좌우를 다시 붙인 이름이 아닙니다. 테이블은 YOLO의 검출 클래스에 포함하지 않습니다. bbox와 confidence를 작업면·지도·관측 이력에 연결하는 테이블 선택 단계는 별도로 평가해야 합니다.

두 테스트 영상에는 별도 bbox 정답이 없습니다. 따라서 현재 데이터만으로 테스트 precision·recall·mAP를 계산하지 않습니다. 같은 방과 소품을 사용했으므로 새 장소·새 개체의 일반화 평가도 별도 촬영이 필요합니다.
