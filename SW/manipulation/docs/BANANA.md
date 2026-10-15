# 바나나: PCA 방향 + 실제 몸통 파지점

사용자 요청대로 전체 마스크의 PCA 방향을 유지하고, 그 방향의 축을 실제 몸통 위 중앙 파지점으로 평행이동한다. 마스크 전체 무게중심을 파지점으로 사용하지 않는다. skeleton/국소 PCA로 방향을 바꾸는 방식은 이번 구현에 포함하지 않았다.

1. 실제 YOLO로 바나나를 재분류하고 SAM2 마스크를 받는다.
2. 왜곡을 보정한 마스크 광선을 로봇 수평 XY 평면에 투영해 PCA 방향을 구한다. 카메라 화면의 각도를 그대로 로봇 yaw로 쓰지 않는다.
3. PCA 방향의 중앙 구간에서 충분히 두꺼운 마스크 내부점을 찾는다. 무게중심이 오목한 빈 공간이어도 파지점은 몸통 위에 놓인다.
4. 선택점 주변의 실제 depth로 표면 좌표를 계산한다. 그 점을 지나도록 PCA 방향의 포즈를 배치하고, 그리퍼가 PCA에 직각으로 닫히도록 TCP yaw를 만든다.
5. 관측3개의 위치·축 방향이 안정적으로 일치해야 한다. PCA의180도 부호 차이는 같은 축으로 취급한다.

복수 분리 마스크, 방향을 정하기 어려운 원형 마스크, 너무 얇은 몸통, 무효 depth, 설정한 충돌 반경보다 큰 실제 마스크는 거부한다. 전체 PCA를 쓰므로 매우 휜 바나나의 국소 몸통 방향과 차이가 남을 수 있다. 이 방식의 실물 파지 성공은 아직 검증하지 않았다.

## 크기와 초기 설정

- 사용자 제공: 길이0.160m, 폭0.030m.
- 높이는 길이가 아니라 **테이블→몸통 윗면의 수직 두께**다. 몸통을 눕힌 상태에서 수직 두께도0.030m라고 가정한 초기값을 넣었다. 실제 두께가 다르면 수정한다.
- 중심/파지 초기 높이: 테이블 위0.015m. 실제 깊이 표면은 손목에서 측정하며, 중앙 몸통 내부의 파지 높이는 설정값으로 정한다.
- 개방/닫힘 초기값:0.040/0.026m. 실제 접촉 결과로 trial 조정한다.
- 충돌/지지용 초기 반경:0.110m. **선택한 몸통 파지점에서 바나나 전체가 들어가는 범위**다. 두께의 반지름0.015m와 다르다. 손목 마스크의 전체 XY 범위가 이를 넘으면 BANANA_COLLISION_RADIUS_TOO_SMALL로 거부한다.
- CAD TCP는 link6와 축이 평행하고 손가락은 TCP±Y로 이동한다. 현재 포즈식의 closing_axis_yaw_offset_rad=-pi/2는 그 CAD 관계를 반영한 초기값이다. 실제 손가락 닫힘 방향과 대조한다.

## 실제 시험을 켜기

기존 실측 설정을 기본 파일로 덮어쓰지 않는다. robot.measured.yaml의 banana 항목만 새 기본값과 대조한다:

```yaml
banana:
  class_id: 5
  enabled: true  # 아래 치수/실제 닫힘 방향/scene을 확인한 뒤 활성화
  size_m: [0.160, 0.030]
  length_m: 0.160
  width_m: 0.030
  height_m: 0.030        # 수직 두께=폭이라는 초기 가정. 실제 값으로 확인
  center_height_m: 0.015
  grip_height_m: 0.015
  radius_m: 0.110
  open_m: 0.040
  close_m: 0.026
  axis_required: true
  closing_axis_yaw_offset_rad: -1.5707963267948966
```

기본 파일에는 enabled=false를 유지했다. 활성화 여부는 실물 설정에서 결정한다. 기존 wrist.yaml에 새 banana 설정 블록도 반영해야 한다. 다음 주석으로 빠르게 찾을 수 있다:

- robot.yaml: `[CALIB:BANANA]`
- wrist.yaml: `[TUNE:BANANA_PCA]` — 중앙 구간, 파지점 두께, PCA 품질
- phase1.tuning.yaml: `[TUNE:GRIPPER]`와`axis_stability_tolerance_rad` — 개방/닫힘·파지 높이·관측 방향 안정성

기존 실측 tuning 파일에 다른 banana.grip_height_m이 있으면 robot.measured의 값을 덮어쓰므로 함께 확인한다. 이후 [터미널별 실행 순서](PHASE1_TERMINALS.md)를 따라 마지막 명령의 green_apple을 banana로 바꾼다.

```bash
ros2 run robocup_real_manipulation phase1_pick banana \
  --config "$HOME/robot_config/robot.measured.yaml" \
  --already-docked --base-drive-isolated --input-durable
```

임의 XYZ나 합성 손목 결과를 입력하지 않는다. 초기 좌표는 실제 외부 디텍션, 새 정밀 좌표·축은 실제 손목 관측에서 얻는다. 로그의 Banana PCA/body에는 선택 픽셀, PCA 분산비, 전체 반경, 로봇 평면 yaw가 표시된다.
