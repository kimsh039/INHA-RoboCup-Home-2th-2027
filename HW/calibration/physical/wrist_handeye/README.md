# 실물 Wrist hand-eye 적용

PiPER Link6 기준 RealSense D435IF `243222070076` RGB optical 변환입니다. `feature/wrist-handeye-calibration`의 0bac1e5 결과를 선택했습니다.

```text
parent: piper_link6
child:  wrist_camera_optical_frame (실행 시 wrist_color_optical_frame에 대응)
xyz[m]: -0.063877 0.028728 -0.012631
q[xyzw]: 0.018698 -0.005325 -0.705684 0.708260
method: PARK, physical_handeye_dataset_best15.json, 15 poses
```

원본 [wrist_handeye_calibration.yaml](wrist_handeye_calibration.yaml), [실물 15자세 dataset](physical_handeye_dataset_best15.json)을 보존했습니다. 런타임은 [calibration.json](calibration.json)의 정규화된 변환 필드를 사용하며, 실제 카메라 루트의 위치는 RealSense 내부 TF를 이용해 계산합니다.

원본 결과의 별도 5자세 기록: 평균/최대 위치 편차 9.37/12.71 mm, 회전 편차 0.99/1.25°. 시뮬레이션 손목 hand-eye 잔차와 구분합니다. 이번 작업에서는 새 계산이나 추가 실기 검증을 하지 않았습니다.

[전체 실물 로봇 실행 방법](../../../../SW/robot/README.md).
