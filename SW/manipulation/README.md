> 현재 통합·배포 기준: [2026-10-16 변경 및 원격 비교](../CHANGES_20261016.md), [관제센터 Manipulation](../control_center/MANIPULATION.md). 아래 정차 실험 기록은 현재 전체 도킹·팔 통합 성공을 뜻하지 않습니다.

# Jetson 실물 매니퓰레이션 — ROS 2 Humble

외부 목표·클로즈 어프로치 완료 → 베이스 잠금 확인 → 받은 coarse 좌표 위에 탑다운 관측 배치 → 손목 재관측 → 정밀 위치 → 클래스별 탑다운 픽앤플레이스 → 후퇴 → 팔 복귀를 수행하는 패키지다. GraspNet·Gazebo·MuJoCo는 실행 의존성이 아니다. 기존 실물 PARK hand–eye 결과와 CAD 장착/TCP를 기본 설정에 반영했다. 이번 작업에서 새 실물 보정이나 실제 구동은 수행하지 않았다.

- [나중에 채울 값](docs/REQUIRED_VALUES.md)
- [현재 구현된 기능·반영된 값·남은 항목](docs/CURRENT_STATUS.md)
- [실제 디텍션을 사용하는 라이브 데모 실행](docs/LIVE_DEMO.md)
- [설정·Jetson ROS 연결 점검 및 TCP 현장 확인](docs/DEMO_CHECK.md)
- [Jetson 설치·실행·실험](docs/DEPLOYMENT.md)
- [모듈 연결 계약·상태 전이](docs/INTERFACES.md)
- [검색 주석·trial 파라미터 관리](docs/TUNING.md)
- [TF·보정·충돌 모델](docs/CALIBRATION.md)
- [TCP 기준점 그림·실물 수집·보정·설정 반영](docs/TCP_CALIBRATION.md)
- [검증 결과·제약](docs/VALIDATION.md)

## 저장소에서 확인한 환경

| 항목 | 근거와 상태 |
|---|---|
| Jetson | `SW/setup/jetson/README_SETUP.md`, `VERSIONS.md`: 2026-10-03 기록의 AGX Orin ARM64, Ubuntu 22.04.5, JetPack 6.2.3, L4T 36.5.2, CUDA 12.6, Python 3.10.12, ROS 2 Humble. 이번 작업에서 Jetson에 접속해 재확인한 결과는 아니다. |
| 빌드 | 기존 head 디텍션의 `ament_python` + 메시지 `ament_cmake`, colcon. 동일 방식 사용. |
| Piper | 기록된 humble 커밋 `017ffefa64511bc6325bd77ddc4e16065c152051`이 노트북 `/home/doyeonlee/piper_ros`와 일치. JointState command/feedback, PiperStatusMsg, Enable service 소스를 확인했다. |
| 드라이버 의존성 | 기록상 piper-sdk 0.6.2 / python-can 4.6.1, Livox SDK2/driver2 빌드. 실제 CAN·장비·firmware 연결은 미확정. |
| RealSense | 실물 보정 브랜치에 손목 D435IF /serial243222070076이 기록되어 기본 설정에 반영했다. 현재 Jetson 설치·실물 장착 상태와 손목 detector ROS 연동은 별도 확인한다. |
| 클래스 | 최신20261012 출력은0 green_apple,1 peach,2 tomato,3 fanta_can,4 cup,5 banana. 기존7클래스가중치도최신Head adapter가이출력으로변환하고plate를제외한다. 토마토/컵규칙활성화, 바나나파지는방향·몸통기하설정대기. |
| Tracer | 형상과 시뮬레이션 주행 코드는 있으나 실물 접근 완료·이동 잠금 계약은 확인되지 않았다. 새 BaseLease 계약으로 외부 내비게이션에 연결한다. |
| URDF | CAD 형상/메시/관절/센서 구조를 재사용한다. `robocup.calibrated.urdf`의 시뮬레이션 보정 완료를 실물 보정 완료로 해석하지 않는다. 원본 파일은 변경하지 않았다. |

참고한 [사용자 Notion 파이프라인](https://app.notion.com/p/3f49814f16c380c78f40e5458c54bef0)의 값은 **초기 실물 실험값**이다. [Piper 공식 Humble 소스](https://github.com/agilexrobotics/piper_ros/tree/humble)의 인터페이스를 기준으로 하되 설치 기록의 고정 SHA를 사용한다.

## 코드 구성

```text
manipulation_ws/src/
  robocup_real_manipulation_msgs/    신규 topic/action 계약
  robocup_real_manipulation/
    config/robot.yaml               미정 값 null, 기본 구동 비활성화
    config/test_fixture.yaml        mock 전용 합성 값; hardware에서 거부
    launch/bringup.launch.py         기본 read-only, 명시적 hardware 실행
    launch/contract_test.launch.py   하드웨어 없는 통신 검사
    robocup_real_manipulation/
      core.py                      검증·이벤트 연결·depth·대상 연계·기하
      sequence.py                  명시적 상태 머신
      node.py                      외부 모듈 adapter와 orchestration
      motion.py                    MoveIt 계획·Cartesian·충돌 scene
      piper_bridge.py              arm action/폭 action → 실제 Piper ROS driver
      description.py               CAD + 실측 보정 → 별도 실행 URDF/SRDF
      test_sender.py, mock_node.py  동일 계약의 테스트 입력/합성 동작 결과
    test/test_core.py
```

이전 매니퓰레이션 사본의 `PickPlace/RunStage`는 GraspNet·시뮬레이션 관련 필드를 포함한다. 현재 실제 저장소의 `SW`에는 실물 manipulation 패키지가 없어, **이름이 구분되는 신규 패키지**로 추가했다. 기존 head detector는 변경하지 않는다. 외부 head/MID360 모듈은 `HeadTarget`의 class/ID/원본 측정시각과 자신이 계산한 3D를 `Target`으로 연결한다. bbox만 있는 `HeadTarget`을 3D로 간주하지 않는다.

## 빠른 시작 — 로봇 구동 없음

저장소 루트에서 실행한다. piper_msgs underlay가 필요한 빌드는 아래 설치 문서를 따른다.

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
cd SW/manipulation/manipulation_ws
colcon build --symlink-install --executor sequential
source install/setup.bash
ros2 launch robocup_real_manipulation bringup.launch.py
```

이 모드는 센서·Piper 드라이버를 시작하거나 모터를 enable하지 않으며, 모든 실물 작업 goal을 거부한다. 통신 계약 전체 검사:

```bash
export ROS_DOMAIN_ID=187
export ROS_LOCALHOST_ONLY=1
python3 ../test_ros_contract.py
```

실물 config를 채우기 전에도 빌드와 mock 검사는 가능하다. 실물 구동은 [DEPLOYMENT.md](docs/DEPLOYMENT.md)의 센서·잠금·보정·저속 단계별 절차를 따른다.

기존 디텍션·센서 퓨전 출력 수신부와 접근 완료 후 자동 인계 설정은 [EXTERNAL_INPUT.md](docs/EXTERNAL_INPUT.md)를 따른다. 디텍션 알고리즘이나 상위 물체 선택 실행기를 여기서 재구현하지 않는다.

2026-10-14 1차 실물 목표(접근 완료→실제 손목 관측→집기→옆에 놓기)는 [PHASE1_RUN.md](docs/PHASE1_RUN.md)를 따른다. 기존 패키지의 phase1.launch/phase1_pick을 사용하며 별도 ROS 패키지를 만들지 않는다.
