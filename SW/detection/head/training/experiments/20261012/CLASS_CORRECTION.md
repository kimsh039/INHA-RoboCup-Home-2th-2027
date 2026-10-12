# 2026-10-12 Head 클래스 정정

사용자 지정 순서의 6클래스는 **green_apple, peach, tomato, fanta_can, cup, banana**다. 기존 데이터의 apple 객체를 tomato로 정정하고 mug를 cup으로 통일하고 plate를 제외했다.

| 이전 ID / 이름 | 현재 ID / 이름 |
|---|---|
| 0 apple | 2 tomato |
| 1 banana | 5 banana |
| 2 fanta_can | 3 fanta_can |
| 3 green_apple | 0 green_apple |
| 4 mug | 4 cup |
| 5 peach | 1 peach |
| 6 plate | 제외 |

[새 데이터](../../../datasets/20261012/README.md)는 이미지 1,262장과 bbox 7,202개다. 접시 bbox 1,184개만 제외했으며 남은 bbox 좌표, JPEG, 영상 분할, 테스트 영상 바이트를 보존했다. 객체별 테이블 대응과 검토 상태도 변환했다. 기존의 20261007 데이터와 모델·로그·평가 기록은 재현을 위해 보존했다.

현재 Head runtime과 평가 스크립트는 위의 정확한 7클래스 스키마에만 변환을 적용한다. 새 6클래스 모델은 자신의 ID와 이름을 사용하고, COCO 등 다른 모델의 apple은 tomato로 바뀌지 않는다. **체크포인트는 원래의 7출력 YOLO11n이고 변환은 후처리다. 새 6클래스 모델의 학습·검증 완료를 뜻하지 않는다.** 직접 Ultralytics를 호출할 때는 이 변환을 별도로 적용하거나 재학습 가중치를 사용해야 한다.

[전체 검사 기록](MIGRATION_CHECKS.json): 모든 이미지·라벨 대응, bbox 좌표 보존, 데이터 SHA256, 테이블 대응 7,202행, 학습 ZIP의 테스트 영상 제외, 노트북 Python 문법을 확인했다. CPU 코드 테스트 51개 통과, ROS 환경이 필요한 테스트 1개는 건너뛰었다.

[두 영상 평가](holdout/TEST_RESULTS.md): RTX 5070 Laptop에서 기존 모델을 imgsz 960, conf 0.25, IoU 0.7로 실행해 308 + 431 = 739프레임의 현재 ID·이름을 확인했다. 대표 프레임을 육안으로 확인했으며 접시 박스는 제외되고 tomato 등 새 이름이 표시된다. 첫 영상 컵 검출은 159/308프레임, 두 번째 영상 컵 검출은 0/431프레임이고 중복 박스와 가림에 대한 한계가 남는다. 이 비율은 독립 정답에 대한 recall이 아니다. 테스트 bbox 정답과 테이블 선택 정답이 없어서 관련 정확도를 계산하지 않았다.

게시한 영상은 브라우저 재생을 위해 H.264/yuv420p/faststart로 변환했다. 보고서의 wall FPS는 추론·원래 주석 렌더링·mp4v 쓰기 측정값이며 이후 H.264 변환 시간은 포함하지 않는다. Jetson FPS는 측정하지 않았다.

[재학습 노트북](../../head_camera_yolo11n_colab.ipynb)과 학습 ZIP은 현재 6클래스 스키마로 준비했다. **재학습 시작 전 상태**이며 기존 7클래스 검증 mAP를 새 모델의 성능으로 보고하지 않는다.
