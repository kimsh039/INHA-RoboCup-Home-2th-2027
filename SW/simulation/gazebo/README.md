# Gazebo 실행과 센서 확인

Gazebo Harmonic을 사용합니다. 아래 명령은 저장소 루트에서 실행합니다.
기존 Gazebo를 종료하고 하나의 시뮬레이션만 실행하세요.

## 실행

빈 월드와 책상 2개·벽·칸막이가 있는 방:

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
현재 make_sim.py의 방은 내부 6×6 m, 벽 높이 1 m이며 같은 책상(1.6×0.8 m, 윗면 0.72 m) 2개가 있습니다.
책상 중심은 (1.8, -1.0)과 (1.8, 1.8)이고, 긴 변끼리 마주 보며 사이 간격은 2 m입니다.
G2는 책상 상판 아래에서 스캔하므로 다리만 검출합니다.
