# Gazebo 실행과 센서 확인

Gazebo Harmonic을 사용합니다. 아래 명령은 저장소 루트에서 실행합니다.
기존 Gazebo를 종료하고 하나의 시뮬레이션만 실행하세요.

## 실행

빈 월드와 가운데에 책상 1개·벽·칸막이가 있는 방:

```bash
./SW/simulation/gazebo/start_sim.sh
./SW/simulation/gazebo/start_sim.sh --world room
```

둘 중 하나만 실행합니다. NVIDIA 드라이버가 활성화돼 있으면 PRIME offload를 자동 설정합니다.
웹 제어도 `http://127.0.0.1:8081`에 함께 열립니다.
최종 URDF에는 손목 카메라가 항상 포함됩니다.

웹 서버 없이 NVIDIA GPU를 명시해 실행하려면:

```bash
python3 SW/simulation/gazebo/make_sim.py --world room
GZ_IP=127.0.0.1 GZ_PARTITION=robocup_motion \
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
gz sim -r SW/simulation/gazebo/build/motion.world.sdf
```

실행 중 GPU 확인: `nvidia-smi`.
생성된 `build/motion.world.sdf`는 절대 메시 경로를 포함하므로 Git에서 제외합니다.
추가 URDF는 임시 디렉터리에서 변환 후 삭제합니다.

## 실시간 센서 수신

각 새 터미널에서 먼저 설정합니다. Gazebo가 재생 상태여야 새 데이터가 들어옵니다.

```bash
export GZ_IP=127.0.0.1
export GZ_PARTITION=robocup_motion
```

아래 명령은 각각 별도 터미널에서 실행하고 Ctrl+C로 종료합니다.

```bash
# 2D 라이다 거리(m); inf는 해당 ray의 검출 없음
 gz topic -e -t /robocup/g2/scan | grep --line-buffered '^ranges:'
# 3D 라이다 거리(m)
 gz topic -e -t /robocup/mid360s/scan | grep --line-buffered '^ranges:'
# 2D 유효 거리만 출력
 gz topic -e -t /robocup/g2/scan | grep --line-buffered '^ranges:' | grep --line-buffered -vE 'inf|nan'
# 3D 점군 수신 주기
 gz topic -f -t /robocup/mid360s/scan/points
# 관절과 오도메트리
 gz topic -e -t /robocup/joint_states
 gz topic -e -t /robocup/odometry
# 전체 토픽
 gz topic -l
```

카메라 수신 주기(각각 실행):

```bash
gz topic -f -t /robocup/camera/color/image
gz topic -f -t /robocup/camera/depth/image
gz topic -f -t /robocup/wrist_camera/color/image
gz topic -f -t /robocup/wrist_camera/depth/image
```

영상 데이터는 바이너리이므로 [카메라 팝업](../tools/README.md#카메라-팝업)을 사용하세요.

## 센서 검사

기본 모델에 정면 검사 벽을 추가하고 실행합니다.

```bash
python3 SW/simulation/gazebo/make_sim.py --test-wall
GZ_IP=127.0.0.1 GZ_PARTITION=robocup_sensor_test gz sim -r SW/simulation/gazebo/build/motion.world.sdf
# 다른 터미널, 저장소 루트에서
GZ_PARTITION=robocup_sensor_test python3 SW/simulation/tools/check_sensors.py --wall
```

검사는 두 라이다·3D 점군·헤드/손목 영상 크기·프레임·sim-time 주기와 헤드 정면 벽 거리를 확인합니다.
손목 영상의 정면 거리는 팔 자세가 달라 헤드 검사 벽 기대값을 적용하지 않습니다.
현재 make_sim.py의 방은 과제(사람이 요청한 물체를 집어 테이블의 비어 있는 다른 자리에 놓기)용입니다.
내부 6×6 m(x -1~5 m, y -3~3 m), 벽 높이 1 m이고, 방 가운데 (2.0, 0)에 책상 1개가 있습니다.
책상의 긴 변은 x축 방향입니다(x 1.2~2.8 m, y -0.4~0.4 m).

책상은 팀이 쓰는 실물 [데스커 컴퓨터데스크 2.0 W1600×D800](https://www.desker.co.kr/product/detail/615)(DSDBB1608, 색상 MLWW)입니다. 제조사 도면 값:

| 부분 | 값 | 시뮬레이터 |
|---|---|---|
| 상판 | 1600×800 mm, 윗면 높이 720 mm, 두께 28 mm. E0 PB + 양면 LPM + 2 mm ABS 엣지, **메이플** | 메시(빌드할 때 생성), 메이플 색 |
| 앞쪽(북쪽, +y) 배선 홈 | 폭 470 mm(가장자리), 깊이 60 mm | 메시 그대로. 충돌은 홈 양옆 경사 부분을 뺀 박스 3개 |
| 다리·프레임·배선 트레이 | 스틸 분체도장, **화이트** | 흰색 박스 |
| 프레임 | 상판 아래 40 mm, 바닥에서 651 mm | 양쪽 짧은 변과 뒤쪽(남쪽) 긴 변 |
| 배선 트레이 | 520×122 mm, 바닥에서 581 mm | 홈 뒤에 매달린 뒤판·바닥·앞턱 |
| 지지 하중 | 50~70 kg(최대 분포 하중 100 kg) | — |

도면에 없어 제품 사진으로 추정한 값: 다리 30 mm 각관(네 꼭짓점 바로 아래), 프레임 위치(짧은 변과 뒤쪽), 트레이 앞뒤 위치(홈 바로 뒤). 로봇은 원점에서 책상을 보고(+x) 시작하므로 `map`·`odom` 축이 world 축과 같습니다.
책상 위 물체 배치는 [objects/README.md](objects/README.md)를 참고하세요.
G2는 책상 상판 아래에서 스캔하므로 다리만 검출합니다.

## 팔 픽앤플레이스

기존 주행/센서 실행과 별도로 [Gazebo Manipulation](../gazebo_manipulation/README.md)에서 손목 관측·물리 파지·배치·초기 복귀를 실행한다. 2026-10-07 사용자 성공은 베이스 밀림을 유지한 완화 hold 기준25mm/60° 실험이며 strict 유지/실기 검증과 구분한다.
