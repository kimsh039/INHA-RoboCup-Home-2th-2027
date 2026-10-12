# Head camera · 현재 6클래스 데이터셋

2026-10-12 사용자 정정 반영: 기존 `apple` 객체는 **tomato**, `mug`는 **cup**, `plate`는 검출 대상에서 제외한다. 원본 [20261007 데이터셋](../20261007/README.md)은 기존 학습 기록을 재현하기 위해 보존했다.

| ID | 영어 클래스 | 대상 |
|---:|---|---|
| 0 | green_apple | 청사과 |
| 1 | peach | 복숭아 |
| 2 | tomato | 토마토 |
| 3 | fanta_can | 환타 캔 |
| 4 | cup | 컵, 손잡이 포함 |
| 5 | banana | 바나나 송이 |

train **1,087장**, val **175장**, 라벨 **7,202개**. 기존 접시 박스 1,184개를 제외하고 나머지 bbox 좌표를 그대로 유지했다. 이미지·원본 테스트 영상은 원본 데이터셋과 SHA256이 같다. 원본 영상의 0, 2, 4, … 프레임 추출과 영상 단위 분할을 유지한다. 접시 픽셀은 이미지에 남지만 검출 라벨은 없다.

바나나는 송이 단위, 환타 캔은 로고가 안 보이는 방향 포함, 컵은 손잡이 포함으로 라벨링한다. 식별 가능한 대상은 모두 라벨링하고 bbox 겹침을 허용한다. 라벨은 여전히 optical flow 기반 검토용 초안이며, 이름 수정이 박스 검토 완료를 뜻하지 않는다.

`object_table_assignments.csv`, `video_table_inventory.json`, `review_state.json`에도 같은 ID·이름 변경과 접시 제외를 적용했다. `table_left`와 `table_right`는 최초 시야로 명명한 물리 테이블이다. 검출 confidence를 테이블 존재 확률로 직접 해석하지 않는다. 테이블별 관측·추적·미관측 처리와 선택 정확도 평가는 별도 단계다.

학습용 설정은 [data.yaml](data.yaml), 패키지 SHA256은 [colab_package_manifest.json](colab_package_manifest.json), 파일 SHA256과 ID 변환은 [dataset_manifest.json](dataset_manifest.json)에 있다. 학습 ZIP에는 이미지·라벨과 메타데이터만 포함하며 `test_videos/`의 두 영상은 제외한다.

[Colab 노트북](../../training/head_camera_yolo11n_colab.ipynb)은 이 버전으로 새 6클래스 모델을 학습한다. 기존 [20261007 가중치](../../models/head_yolo11n_20261007.pt)는 7출력 모델이며, Head adapter가 이름·ID를 변환하고 접시 출력을 제외한다. **6클래스 재학습은 아직 실행하지 않았다.** 기존 검증 mAP는 원래 7클래스 실험 수치다.

다른 경로에 동일한 데이터셋과 업로드 ZIP을 만들려면 저장소 루트에서 실행한다. 대상 경로는 새 경로여야 한다.

```bash
python SW/detection/head/training/migrate_six_classes.py \
  --source SW/detection/head/datasets/20261007 \
  --target /absolute/path/head_dataset_6class \
  --archive /absolute/path/head_camera_yolo11_dataset_20261012.zip
```
