# 2026-10-15 검증 기록

## 관절 호환성 및 최신 관제 단계 수정

- 관제의 raw 화면 joints 입력을 접근 계획에 직접 쓰지 않도록 별도 모델 피드백 helper와 최소 변경 패치 도구를 추가했다. 매니퓰 robot.yaml의 이름과 자동 비교한다. 모델 각도에 raw 보정을 중복 적용하지 않는다.
- 최신 관제의 waiting_target/navigating_target/rerouting_table/planning_table을 수용한다. 좌표 없는 waiting_target에서는 인계를 보류하며 실측 좌표가 온 뒤에만 새 접근 ID를 만든다.
- 변경 후 로컬 Humble colcon 빌드 성공, Python 테스트104개 통과(매니퓰95+관절 입력9). 격리 ROS187에서 실제 관절 입력 helper와 최신 관제 단계의 인계 어댑터 토픽 교환도 각각 통과했다. 실제 로봇 동작 검증은 아니다.
- Jetson은 사용자 안내에 따라 재부팅 중이며, 이 수정본은 로컬 작업이다. 원격 소스 변경·서버 재시작·실물 구동은 하지 않았다. 아래 기존 ARM64/실물 기록은 이번 수정본의 배포 검증이 아니다.

## 관제센터 입력 연결 추가 검증

- 로컬 Humble overlay에서 변경된 `robocup_real_manipulation` colcon 빌드 성공.
- Python 테스트 **89개 통과**. 기존75개와 관제 입력14개를 포함한다.
- localhost/ROS_DOMAIN_ID=187에서 실제 어댑터를 별도 프로세스로 실행하고 합성 관제 JSON/odom으로 토픽 전달을 검증했다. 과거 succeeded 재생 거절, 이동·끊긴 odom 거절, odom 좌표와 원본 시각 보존, 동일 작업 ID, 단발 인계, 취소 거절을 통과했다. 팔 명령 및 BaseLease publisher가 없는 것도 확인했다.
- 기존 mock ROS 계약13경우도 통과했다. 로봇/웹 배포와 실제 팔 실행은 수행하지 않았다. 새 어댑터의 Jetson ARM64 빌드는 아직 미검증이다.
- 이후 확인한 최신 Jetson 관제 코드는 YOLO+MID-360 3D `point_odom`과 접근 상태를 JSON으로 발행한다. 아래 최초 조사 시점의 'XYZ 없음'은 당시 관측한 별도 head 출력에 대한 기록이다. 최신 연결은 [CONTROL_CENTER_CONNECTION.md](CONTROL_CENTER_CONNECTION.md)를 따른다.

## 수행 결과와 증거

| 범위 | 결과 | 증거 |
|---|---|---|
| 최신 GitHub 재조회 | local tracked HEAD와 origin HEAD 모두 a75350164913198315b3c674fd14e2e2a0be2b36 | git ls-remote, 파일 직접 조사. 기존 checkout 보존 |
| 로컬 Humble 최종 colcon 빌드 | detection_msgs, manipulation_msgs, head_detection, real_manipulation 4패키지 성공 | `validation/integration_build.log` |
| 최종 Python 테스트 | 75개 통과. 센서 입력은 각 테스트의 synthetic fixture이며 성공 로그/내참수 검사는 실제 기록을 사용 | `validation/integration_final_verification.log` |
| ROS action/topic 테스트 | 13경우 통과. **모션/피드백은 명시적 mock**, 실제 로봇 성공 아님 | 동일 로그, ROS_DOMAIN_ID=187 |
| Jetson ARM64 빌드 | 사용자 업로드 중지 지시 **전에 전송된 중간 소스**의 4패키지 빌드 성공. 이후 로컬 최종 수정본은 Jetson 재전송/재빌드하지 않음 | `validation/jetson_integration_build.log`, `/tmp/manipulation_integration_20261015.KVfT1J/overlay` |
| 셸/문법 | 설치·빌드·시작·검증 스크립트 bash -n, Python compileall 통과 | 로컬 검사 |
| 실제 젯슨 그래프·카메라·피드백·TF | 8초 read-only 구독, no action/service/motor command | `validation/jetson_passive_20261015.json` |
| 실제 성공 파지 로그 재검증 | 약80.61mm / effort=-0.358을 새 접촉 판정이 수락하고6mm slip을 거부 | `test/fixtures/physical_success.json`, `test_integration.py` |

중간 ARM64 빌드는 새로운 `/tmp` staging 소스/overlay에만 수행했다. 기존 `/home/sparo/robocup_work/manip_fix`, 공유 checkout의 소스/설정, 다른 사람의 수정 및 실행 프로세스는 변경하지 않았다. 사용자가 직접 업로드하겠다고 한 이후 추가 업로드·원격 소스 수정·구동은 하지 않았다. 중간 staging이 남아 있으며 삭제 명령도 임의로 실행하지 않았다. 최종 소스와 전송 명령은 로컬에만 제공한다.

## ROS 13경우

- Target/ApproachResult 두 도착 순서, 오른쪽 차단 시 왼쪽 계획.
- 접근 FAILED, 실제 base_stopped=false 입력의 팔 미시작 실패.
- 손목 timeout, 과거 stamp, 무효 depth, 누락 TF, 복수 후보 모호성.
- action cancel, 팔 복귀 실패(place=true/return=false), 실제 unlock ACK 누락 시 COMPLETE 단계 FAILED.
- mock은 isolated domain187만 허용하며 driver command publisher를 만들지 않는다.

샌드박스의 UDP socket/getifaddrs 제한 경고가 있었지만 테스트 action/topic 교환 및 결과 assertion은 모두 통과했다. 이 결과를 다중 컴퓨터 DDS 네트워크/실물 actuator 검증으로 해석하지 않는다.

## 실제 데이터로 확인한 내용

- 실제 관제 JSON 출력은 six-class ID 0..5와 bbox를 포함한다. XYZ가 없으며 HeadTarget/실제퓨전 Target로 오해하지 않는다.
- `/piper/joint_states_feedback`100개 표본(고정100개 최근 캐시) 원본→수신 age 약1.64~49.86ms. 현재 수동 SocketCAN 피드백 경로다. 과거 SDK driver의 약0.9초 지연 원인이 해결됐다는 증거가 아니다.
- 손목 RGB age 약77.53~168.78ms, aligned depth 약75.73~140.21ms, CameraInfo 약77.50~169.74ms.
- factory RGB640×480, fx606.4701/fy606.5556, cx317.7285/cy254.9584, D=0, frame=wrist_color_optical_frame. 실제 내참수로 전체 물체+trial coarse 오차를 수용하는 거리 계산을 검사했다.
- 카메라 callback 순서상 latest-only 조합에는 어긋나는 순간이 있다. probe의 recent100개 callback 조합 중54개가25ms 내였다. 이는 고유 프레임 성공률이나 프레임 손실률이 아니다. 새 frontend는6프레임 history에서 완전한 tuple을 선택한다.
- map←base_link, odom←base_link 및 base_link←wrist_color_optical_frame의 live TF를 조회했다. 그 순간 조회 가능성만 확인했으며 모든 작업 취득시각의 TF 연속성과 보정 정밀도를 검증한 것이 아니다.
- raw gripper/status와 manipulation Target/ApproachResult/BaseLease publisher가 없었다. 따라서 이 환경을 ready로 보고하거나 실제 action을 보내지 않았다.
- 성공 task는 COMPLETE/SUCCESS지만 initial_point는 임시 입력이다. 실제 coarse→주행→실제정지→정밀파지 통합 성공 근거로 사용하지 않는다. 00:39 wrist timeout 및00:44 stale feedback 후속 실패도 별개로 보존한다.

## 아직 실환경에서 검증하지 않은 항목

1. 현재 실제 detector/퓨전의 작업 UUID·취득시각 3D 입력과 접근 완료 연결.
2. 전체 velocity 소유자를 차단하는 실제 lease provider 및 정지/해제 handshake.
3. 이 최종 wrist YOLO+SAM 경로의 실제 장면 클래스 정확도, 전체 silhouette 확보, 제한 탐색 성공률, GPU 메모리·장시간 latency. 성공 사본의 class-agnostic SAM 자동분할과 동일하지 않다.
4. 새 measured voxel scene/전체 mask extent, fixed-frame 테이블/장애물과 배치 history의 실물 MoveIt 검증. 관측 표면 밖 가려진 장애물은 미관측이다.
5. 최종 bridge와 수정 Piper driver 사이의 실제 CAN·펌웨어·폭·힘·stamp·joint mapping 및 멈춤 성능. 중복 driver/TF 없이 연결해야 한다.
6. 청사과 외 다섯 클래스의 접촉조건·실제 충돌 범위·실제 픽앤플레이스. 바나나 높이/폭/닫힘축은 trial/CAD 가정.
7. 실제 초기 자세 경로. 성공 당시 비인접 self-contact 허용을 복사하지 않았으므로 처진 자세에서 엄격한 충돌검사가 실패할 수 있다.
8. 해제 뒤 다음 이동 및 다음 물체 작업의 반복 실험, 실제 실패·취소·물체 유지·운영자 복구.

패키지의 구현/빌드 완료와 실제 시스템 전체 성공을 구분한다. 위 외부 입력과 실물 검증이 남아 있다.
