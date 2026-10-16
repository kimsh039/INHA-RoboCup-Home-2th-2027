# 실물 Wrist hand-eye · 현재 입력과 과거 프로파일

현재 `SW/robot/sensor_tf.py`가 직접 읽는 입력은 [calibration.json](calibration.json)이다. 최신 main에는 2026-10-14의 **ChArUco 55자세** 결과가 들어 있다.

- 변환: `piper_link6 ← wrist_camera_optical_frame`; driver optical은 `wrist_color_optical_frame`.
- xyz(m): `[-0.064927, 0.031071, 0.019479]`.
- quaternion xyzw: `[0.005303, -0.002385, -0.697211, 0.716842]`.
- 두 보드 배치 28+27자세. factory K + 추정 k1/k2, Park 초기값 후 soft-L1 비선형 적합.
- 두 배치 별도 해 차이 3.9mm / 0.27°, 동일 데이터 RMS 10.5mm / 1.11°. 새 독립 실물 정확도 보증과 구분한다.
- RGB 내부 파라미터는 [현재 ost.yaml](../../camera_intrinsics/wrist/ost.yaml)을 확인한다.

## 과거 값은 어디에 쓰이는가

[기존 PARK YAML](wrist_handeye_calibration.yaml), [15자세 dataset](physical_handeye_dataset_best15.json)은 이전 측정 원본이다. 과거 기록의 5자세 9.37/12.71mm 및 0.99/1.25° 결과는 그 버전에 대한 값이다.

현재 manipulation 준비용 프로파일에는 이전 best15 결과가 별도로 포함돼 있다. **실물 sensor_tf의 최신 JSON과 팔 프로파일이 자동으로 같은 값으로 갱신되는 구조가 아니다.** [manipulation 보정 계약](../../../../SW/manipulation/docs/CALIBRATION.md)에 따라 일치와 적용 범위를 확인하고 commissioning한다. 이 정리 작업은 실측 값이나 execution/verified 플래그를 바꾸지 않는다.

카메라 루트의 외부 TF는 측정 parent←optical에 RealSense optical←root 내부 TF를 곱해 만든다. [sensor_tf 코드](../../../../SW/robot/sensor_tf.py) · [실물 실행](../../../../SW/robot/README.md).
