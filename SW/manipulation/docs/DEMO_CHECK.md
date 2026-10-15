# 라이브 데모 연결 점검

최신GitHub6클래스 출력ID를 설정에 반영했다. 자동 픽앤플레이스 알고리즘은 변경하지 않는다. check_live_demo.py는 ROS 입력/장비/TF를 읽기만 하며 action goal, enable service, 관절 명령, 가짜 센서값을 보내지 않는다.

## 1. 설정에서 아직 비어 있는 값 확인

저장소 루트에서 실행한다. ROS 설치 전에도 NumPy/PyYAML로 이 단계는 가능하다.

```bash
python3 SW/manipulation/tools/check_live_demo.py \
  --config /absolute/path/robot.measured.yaml \
  --tuning-file SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/tuning.yaml \
  --output /tmp/manipulation_config_check.json
```

출력의 missing_values는 실제로 비어 있는 항목 전체다. unacknowledged_flags는 운영자의 확인이 필요한 flag이며 자동으로 활성화하지 않는다. hand–eye/TCP/base 변환은 적용된 값으로 별도 표시한다. 이 스크립트는 설정 사본에 내부적으로 flag를 대입해 숫자 검사를 수행할 수 있지만 그 사본을 저장·실행하지 않는다.

## 2. Jetson의 실제 장비/ROS 연결 확인

Piper, 센서, 디텍션/Nav adapter 및 준비된 manipulation 노드를 실제 운영 절차로 기동한 다음 같은 ROS_DOMAIN_ID의 터미널에서:

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
source SW/manipulation/manipulation_ws/install/setup.bash
python3 SW/manipulation/tools/check_live_demo.py \
  --config /absolute/path/robot.measured.yaml \
  --tuning-file SW/manipulation/manipulation_ws/src/robocup_real_manipulation/config/tuning.yaml \
  --live --seconds 5 --output /tmp/manipulation_live_check.json
```

관절/그리퍼 source timestamp 신선도, raw joint/폭/effort, 설정된 sign/offset을 적용한 관절 범위, 실제 publisher의 메시지 타입, 카메라 profile 정보, action server 존재, 최신 TF를 보고한다. 설정과 link6↔optical/TCP 및 base↔arm의 TF가1mm/0.001rad 이상 다르면 TF_CONFIG_MISMATCH로 표시한다. 이는 설정 적용 여부 검사이지 실물 절대 정확도의 합격 기준이 아니다. TF 최신 시각을 읽는 도구이며 이미지 취득시각 TF 검사는 실제 coordinator가 수행한다.

외부 디텍션이 다른 메시지 타입을 쓰면 TYPE_MISMATCH 또는 필요한 입력 publisher 없음이 표시된다. adapter가 필요한 지점을 이 기록으로 확인한다. ROS action server가 있다는 사실은 움직임/파지 성공을 뜻하지 않는다. 대기 중인 wrist result나 base lease는 실제 작업/관측 요청 전에는 샘플이 없을 수 있다. 카메라 토픽을 remap한 경우 해당 운영 토픽도 별도로 확인한다.

장비를 아직 연결하지 않았다면 NO_FEEDBACK/TF_MISSING은 예상되는 결과다. mock 데이터를 보내 이 경고를 없애지 않는다. 일단 살아 있는 드라이버의 실제 값부터 확인한다.

## EE↔TCP 완료 여부와 현장 확인

| 항목 | 현재 상태 |
|---|---|
| camera optical↔link6 hand–eye | 기존 실물 PARK 결과 반영 완료. 현재 장착과 같은 관계인지 확인 |
| link6↔TCP의 CAD 변환 | `[0,0,142.5]mm`, 축 평행 설정 완료 |
| 실제 프로브로 수행한 TCP pivot | 이번 작업에서 미수행 |
| 실제 파지 위치 오차 | 로봇을 이용한 시험 필요 |

현재 TCP는 손가락 끝 높이의 중심축 점이다. [기준점 그림](../validation/tcp_cad.png)에서 빨간A는link6원점, 파란B는TCP다. 그리퍼 연결판 중심과 혼동하지 않는다. 실제 손가락/장착 리비전이 CAD와 같다면 이 변환을 초기 기준으로 사용할 수 있다.

현장에서 할 일:

1. 실제 손가락 끝 높이와 좌우 중앙이 이 B 정의와 같은지 확인한다. 패드 중앙 등 다른 점을 집는 기준으로 택하면 TCP 또는 tcp_to_grip_center_m 중 한 곳에서 그 관계를 반영한다.
2. 테이블 위 물체 중심·지지면과 표시된 TCP 목표를 대조하고 첫 파지 위치를 저속 시험한다. 모션/충돌/피드백 기본 확인을 마친 후 trial로 파지 높이·폭을 조정한다.
3. 자세가 달라질 때 일관된 위치 오차가 남거나 필요한 정밀도를 만족하지 못하면 고정 프로브/소켓을 이용한 [TCP_CALIBRATION.md](TCP_CALIBRATION.md)의 실물 pivot 수집→계산→적용을 수행한다. 카메라는 필요 없다. 프로브 끝과 원하는 TCP의 실측 관계 및 도구축 정의가 필요하다.

CAD 값이 존재한다는 것과 실물 pivot 완료는 다르다. 이번 도구/검사 결과도 실제 로봇 구동 완료로 표시하지 않는다. 실행 순서는 [LIVE_DEMO.md](LIVE_DEMO.md)를 따른다.
