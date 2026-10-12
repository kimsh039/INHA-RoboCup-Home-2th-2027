# KakaoTalk 영상 테스트와 cup 신뢰도 조정

원본 `KakaoTalk_20261013_002408663.mp4`를 [source.mp4](video_test/source.mp4)로 보관했다. 640×480, 24 FPS, 961프레임, 약 40.04초다. 새 [6클래스 모델](../../../models/head_yolo11n_20261013.pt)로 프레임 생략 없이 처리했다. 추론 장치는 RTX 5070 Laptop이며 imgsz 640, NMS IoU 0.7이다.

[기준 결과 영상](video_test/baseline/annotated.mp4) · **[사용자가 채택한 cup=0.70 결과](video_test/cup070/annotated.mp4)**. 각 폴더의 `predictions.jsonl`, `summary.json`, 대표 이미지에 실제 검출 결과를 보관한다.

## cup=0.70 채택 근거

실제 붉은 머그컵 뒤의 붉은 원통형 물체도 cup으로 오인했다. 기준 결과에서 머그컵 박스의 최저 신뢰도는 0.757889, 추가 박스의 최고 신뢰도는 0.645664였다. 두 박스 IoU는 약 0.057–0.127로, 같은 위치의 겹친 박스를 제거하는 일반적인 NMS 조정만으로 해결하기 어렵다.

| cup 최소 신뢰도 | cup 박스 2개인 프레임 | cup 박스 없는 프레임 |
|---|---:|---:|
| 0.25 | 101 | 0 |
| 0.40 | 54 | 0 |
| 0.50 | 23 | 0 |
| 0.60 | 5 | 0 |
| **0.70** | **0** | **0** |
| 0.80 | 0 | 49 |

위 비교는 동일한 기준 예측 기록을 신뢰도별로 필터링해 집계했다. 별도로 cup=0.70을 적용해 전체 영상을 다시 실행했고, **961프레임 모두 cup 박스 정확히 1개**, 다른 클래스 예측은 기준 결과와 동일함을 검증했다. H.264 MP4로 변환하고 출력 961프레임 전체가 읽히는 것도 확인했다. 사용자가 결과를 확인한 뒤 이 설정을 채택했다.

이 영상에서 한 컵으로 유지됐다는 관측이며 객체마다 정답 라벨을 대조한 recall 측정은 아니다. 신뢰도 기준은 다른 거리·조명에서도 다시 확인해야 한다. 다음 데이터 수정 때 머그컵 뒤 물체가 cup으로 라벨링된 사례도 점검한다. 실제 컵이 여러 개 있으면 기준을 통과한 모든 컵을 출력한다; top-1 강제 선택은 하지 않는다.

## 우분투 재현

기존 YOLO 환경에 `imageio-ffmpeg`를 설치하고 저장소 루트에서:

```bash
python -m pip install imageio-ffmpeg==0.6.0
python SW/detection/head/scripts/evaluate_video.py \
  SW/detection/head/training/experiments/20261013/video_test/source.mp4 \
  --cup-conf 0.70 --device 0
```

새 결과는 `SW/detection/head/training/runs/video_tests/`에 만들어진다. 기존 출력 폴더가 있으면 덮어쓰지 않으며 `--output /tmp/head-video-test-2`처럼 다른 경로를 지정한다. 기준 결과를 재현하려면 `--cup-conf 0.25`를 사용한다. `--device cpu`는 GPU 환경이 없는 일반 우분투에서 기능만 확인할 때 사용한다.

정답 bbox가 없어 이 영상의 precision, recall, mAP, 두 테이블 선택 정확도는 **계산하지 않았다**. 출현 프레임 수를 정확도로 부르지 않는다.
