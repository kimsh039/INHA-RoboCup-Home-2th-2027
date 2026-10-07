# Ubuntu에서 Piper 손목 카메라 통합하기

2026-10-03. STL·장착 좌표·CAD 원본을 보관한다. 손목 카메라는 최종 URDF에 통합했으며 독립 URDF와 병합 스크립트는 정리했다.

**2026-10-05 실기 선택은 Head D435 + Wrist D435입니다.** 아래 D435f 이름의 CAD·메시는 형상 원본으로 유지합니다. 손목 D435의 내부 보정·관측 거리·depth 품질과 PiPER hand–eye 보정은 실물 장치에서 별도로 측정합니다. [센서 역할](../../../SW/detection/SENSOR_ROLES.md) · [두 D435 실행 명령](../../../SW/setup/jetson/RUN_COMMANDS.md#2-realsense-head--wrist)

## 실행

최종 [robocup.urdf](../../../SW/simulation/robot_description/robocup.urdf)에 손목 카메라가 항상 포함됩니다.
독립 wrist_camera.urdf와 merge.py는 삭제했습니다. 저장소 루트에서:

```bash
./SW/simulation/gazebo/start_sim.sh --world room
# 다른 터미널
robot-camera --partition robocup_motion --camera wrist
```

[컨트롤러·영상 명령](../../../SW/simulation/tools/README.md)

## 포함된 자료

| 파일 | 용도 |
|---|---|
| `meshes/wrist_camera_mount_link.stl` | 마운트 컴포넌트 원점 기준, 미터 단위 |
| `meshes/wrist_camera_cad_link.stl` | 카메라 컴포넌트 원점 기준, 미터 단위 |
| `placement.json` | `piper_gripper_base` → 마운트 장착 변환 |
| `source/manipulator_mount_assembly.step` | 카메라와 마운트의 원본 CAD 형상 |
| `source/*json` | 두 Fusion 문서의 부품 변환과 정합 계산 결과 |
| `source/placement_check.png` | STEP과 기존 Piper 형상 비교 그림 |

STL은 배치 STEP에서 마운트 solid 60, 카메라 solids 61·62를 추출한 뒤, Fusion JSON의 해당 컴포넌트 전역 변환의 역변환을 적용해 각 컴포넌트 좌표로 되돌린 것이다. STEP 치수는 mm, STL은 m이다. 배치 전체 STEP(약 38 MB)은 포함하지 않았으며 우분투에서 병합·실행하는 데 필요하지 않다.

## 연결과 확인

```text
piper_gripper_base
└─ wrist_camera_mount_link
   └─ wrist_camera_cad_link
      └─ wrist_camera_link          # +X 전방, +Y 왼쪽, +Z 위
         └─ wrist_camera_optical_frame  # +Z 전방, +X 오른쪽, +Y 아래
```

모두 fixed 연결이다. Piper 6번 관절이 회전하면 카메라도 함께 회전하며 손가락 개폐에는 따라 움직이지 않는다. RViz에서 TF와 RobotModel을 표시하고 다음을 확인한다.

1. 그리퍼 몸통 위에 마운트·카메라가 배치되는지 확인한다.
2. 팔 6번 관절을 움직여 카메라가 몸통과 함께 회전하는지 확인한다.
3. 손가락을 열고 닫아 카메라의 몸통 기준 위치가 유지되는지 확인한다.
4. 광학 프레임의 +Z가 렌즈 전방인지 확인한다.

```bash
ros2 run tf2_ros tf2_echo piper_gripper_base wrist_camera_link
ros2 run tf2_ros tf2_echo base_link wrist_camera_optical_frame
ros2 topic list | grep wrist_camera
ros2 topic hz /wrist_camera/color/image_raw
```

새 ROS 토픽은 `/wrist_camera/color/image_raw`, `/wrist_camera/color/camera_info`, `/wrist_camera/depth/image_raw`, `/wrist_camera/depth/camera_info`, `/wrist_camera/depth/points`이다. Gazebo 원본 토픽 접두사는 `/robocup/wrist_camera/`이며 기존 랙 카메라와 구분된다. 최종 모델에서는 손목 카메라가 항상 포함된다.

## 가정과 검증 상태

- 장착 좌표는 [장착 계산 문서](../WRIST_CAMERA_INTEGRATION.md)의 **잠정 형상 정합값**이다. STEP과 기존 URDF의 그리퍼 리비전 차이가 있어 체결 기준면·볼트 구멍·실측 대조가 필요하다. `placement.json`에서 수정할 수 있다.
- CAD 카메라의 +Z 전방·+X 왼쪽·+Y 위를 가정해 body/optical 프레임을 구성했다. body/optical 프레임은 CAD 원점을 공유하며 실제 RGB/depth 센서의 별도 광학 원점·외부 보정은 반영하지 않았다.
- 마운트는 재료 미확인으로 **PLA 밀도 1240 kg/m³**, 체적 약 12.149 cm³, 질량 약 15.07 g을 가정했다. 실제 금속/다른 출력 재료이면 물성을 변경한다. 카메라는 기존 랙 D435f 모델과 같은 명목 75 g을 사용했다.
- 두 부품의 무게중심과 관성은 각각의 경계 상자를 이용한 균일 밀도 근사다. 충돌 형상도 경계 상자이므로 마운트의 그리퍼를 감싸는 홈은 표현하지 않는다. 간섭 검증에 쓰지 않는다.
- Gazebo depth/RGB는 기존 랙 카메라와 같은 핀홀 근사(30 Hz, depth 1280×720/FOV 87°, RGB 1920×1080/FOV 69°)다. 실제 D435f 보정·IR·정합·노이즈를 재현하지 않는다.
- Windows에서 XML 트리, 메시 존재/단위, 관성, 좌표 변환과 팔 운동 시 TF 관계를 수치 검증했다. Ubuntu Gazebo 이미지 수신은 [VALIDATION.md](VALIDATION.md)에 기록했다.

손목 카메라 포함 명목 총 질량은 약 51.16914 kg이다. 기본 모델의 51.07908 kg에 카메라 0.075 kg과 가정한 마운트 0.015065 kg을 더한 값이다.
