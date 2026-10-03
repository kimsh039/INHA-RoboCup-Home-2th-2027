# Piper URDF

로컬에서 사용하던 Piper 팔과 그리퍼 모델 및 참조 메시 파일입니다.
메시 경로만 `meshes/` 상대 경로로 바꾸었으며 각 파일의 링크·조인트·형상·물성은 유지했습니다.
URDF와 `meshes/` 폴더를 함께 보관하세요. 로더가 URDF 파일 위치를 기준으로 상대 경로를 해석하도록 설정합니다.

| 파일 | 구성 |
|---|---|
| `piper_description.urdf` | 원본 팔 단독 모델 |
| `piper_gazebo.urdf` | 로컬 Gazebo용 팔 단독 모델 |
| `piper_with_gripper.urdf` | 팔 + 플랜지 + 그리퍼. 원본 mimic 관계 포함 |
| `piper_with_gripper_gazebo.urdf` | 로컬 Gazebo용 팔 + 그리퍼 |

그리퍼 포함 모델은 `link6 → flange_link → gripper_base`를 fixed joint로 연결합니다.
Gazebo용 그리퍼 모델은 로컬에서 변환 문제를 일으킨 가상 `gripper_link`·`gripper` 조인트와 mimic 관계가 제거된 사본입니다. 양쪽 손가락 자동 동기화 제어는 포함하지 않습니다. 실제 동작·접촉 파지 검증은 수행하지 않았습니다.
Tracer·센서 랙과는 별개의 모델입니다.

원본: [AgileX agx_arm_urdf / piper](https://github.com/agilexrobotics/agx_arm_urdf/tree/main/piper)
원본 라이선스는 `LICENSE`에 포함되어 있습니다.
