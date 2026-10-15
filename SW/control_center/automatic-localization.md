**SLAM Toolbox 기반 자동 위치추정**

관제에서 저장 지도를 고르고 자동 위치추정을 시작하면 로봇이 시작 위치와 방향을 계산한다. 수동 초기 위치 지정과 AMCL 프로세스는 사용하지 않는다. 지도 작성과 최종 위치 추적은 기존 SLAM Toolbox를 유지한다.

```text
G2 / MID-360 → 빔별 운동 보상 → *_continuous → 지도 전체 후보 비교
                                                ↓ 자동 초기 자세
저장 pose graph → SLAM Toolbox localization ← /initialpose
                      ↓ /pose 처리 응답 확인
                   자동 위치 확인 → Nav2 사용 가능
```

`global_scan_matcher.py`는 저장 YAML/점유 이미지의 알려진 자유 공간을 0.25 m 간격으로 탐색하고, 전체 방향을 5° 간격으로 비교한다. 후보 위치에서 스캔 끝점과 가까운 지도 점유 셀의 거리를 likelihood field 점수로 계산한다. 상위 후보를 각 위치·방향 주변에서 세밀하게 탐색한다. 검색은 별도 worker 하나가 담당하며 ROS 콜백을 막지 않는다. 중지 요청 시 검색도 취소한다.

기본 채택 조건은 점수 0.60 이상, 지도 점유 셀에서 20 cm 이내인 측정 65% 이상, 구별되는 다음 후보와 점수 차이 0.08 이상이다. 원점과 이전 추정 위치는 전역 후보의 prior로 사용하지 않는다. 이 임계값들은 운용 초기값이며 성공 확률이나 위치 오차를 직접 나타내지 않는다. 대칭 공간에서는 후보가 모호할 수 있고, 그때는 위치를 확정하지 않고 신선한 관측으로 다시 탐색한다.

검색 스캔의 시각과 센서 장착 TF를 사용해 점들을 `base_link`에 배치한다. 검색 중 이동한 경우 검색 시각/현재 시각의 `odom → base_link`로 자세 후보를 현재 시각에 전파한다. 최신 map TF를 초기 추정 근거로 사용하지 않는다. 후보가 채택되면 표준 `/initialpose`로 SLAM Toolbox에 자동 전달한다. 전처리 노드는 이 요청 후 정지 중에도 제한된 초기 관측을 재개한다. 요청 이후의 `/pose` 응답을 원래 관측 시각으로 변환해 지도 일치를 확인하고 서로 다른 시각의 응답 2개를 받으면 초기화를 완료한다. 조건을 만족하지 못하면 새 스캔으로 다시 탐색한다.

`/localization/status`에는 `waiting`, `searching`, `ambiguous`, `confirming`, `localized`, `error`와 후보·점수·응답 확인 상태를 전달한다. 관제의 Nav2 시작은 자동 초기화 완료 후 가능하다. 이 노드는 TF·속도·팔 명령을 발행하지 않는다. `map → odom` 자세는 SLAM Toolbox가 계산하고 기존 `slam_tf_clock`만 공개 TF로 전달하며, `odom → base_link`는 EKF가 단독 발행한다.

자동 초기화에는 `.yaml`과 점유 이미지가, SLAM Toolbox 추적에는 `.posegraph/.data`가 함께 필요하다. 선택한 G2/MID-360 전처리와 저장 지도의 센서가 같아야 한다. `global-localization.yaml`에서 탐색 간격·후보 수·채택 기준을 조정한다. 새로운 colcon 빌드는 필요하지 않으며 NumPy/SciPy/Pillow/PyYAML 의존성은 `setup-jetson.sh`에 포함했다.

[SLAM Toolbox 공식 설명](https://github.com/SteveMacenski/slam_toolbox#localization)은 자체 pose graph 최적화 기반 위치추정을 설명한다. [설치 버전의 초기 위치 처리](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/src/slam_toolbox_localization.cpp)는 요청한 자세 주변에서 다음 스캔을 처리하므로 자동 후보 제공 단계가 필요하다. [Cartographer, ICRA 2016](https://research.google/pubs/real-time-loop-closure-in-2d-lidar-slam/)의 넓은 위치·방향 범위에서 스캔 후보를 비교하는 원리를 참고했으며, 이번 구현은 coarse grid와 국소 탐색을 사용한다. 논문의 branch-and-bound 구현 전체를 도입한 것은 아니다.

자동으로 시작 위치를 찾는 기능이며, 추적 중 납치·미끄러짐의 모든 상황을 자동 복구하는 기능까지 추가한 것은 아니다. 정지 관측만으로 구별할 수 없는 대칭 구조에서는 추가 기하 관측이 필요하다. 자동 초기화는 로봇을 움직여 관측을 얻는 명령을 보내지 않는다.
