# Tracer 메시와 물성 기록

독립 URDF와 재계산 생성기는 삭제했고 메시·inertial_report.json을 유지합니다.
현재 최종 보정 모델은 [robocup.calibrated.urdf](../../../SW/simulation/robot_description/robocup.calibrated.urdf)입니다. [robocup.urdf](../../../SW/simulation/robot_description/robocup.urdf)는 CAD 원본으로 유지하며, 2026-10-06 반영·업로드한 2D/Mid360 보정과 랙 변경의 [날짜·커밋 이력](../../../SW/simulation/robot_description/README.md#모델-변경보정업로드-이력)을 기록합니다.

## 질량·관성 보정 (2026-10-03)

[Trossen 공식 Tracer 사양](https://docs.trossenrobotics.com/agilex_tracer_docs/specifications.html)은 로봇 무게 **30 kg**, 적재 허용량 **100 kg**을 명시합니다. 적재 허용량을 로봇 질량에 더하지 않습니다. [AgileX 제조사 카탈로그](https://trossenrobotics.americommerce.com/Shared/Agilex/product-catalog-1.pdf)도 Tracer 무게를 **28–30 kg**, 크기를 **685 × 570 × 155 mm**로 명시합니다. 이번 모델은 30 kg 명목값을 사용합니다.

기존 URDF의 `inertial_link`만 132.38985 kg이고 바퀴까지 합하면 152.38985 kg이었습니다. 원본 값의 산출 근거는 확인되지 않았으며 현재 모델에서는 제거했습니다. 랙·센서·Piper 물성을 유지한 통합 URDF의 총 질량은 **51.07908 kg**입니다. 이 값은 손목 카메라 추가 전 기준이며 최종 총질량은 약 51.16914 kg입니다.

부품별 실측 질량·무게중심·관성은 공개 자료에서 확인하지 못했습니다. 다음 분배는 **제조사 부품별 사양이 아니라 시뮬레이션 가정**입니다.

| 구성 | 개당 질량 | 개수 | 합계 |
|---|---:|---:|---:|
| 차체·배터리·내부 구동부 (`inertial_link`) | 26 kg | 1 | 26 kg |
| 구동 바퀴 | 1.5 kg | 2 | 3 kg |
| 캐스터 지지부 | 0.125 kg | 4 | 0.5 kg |
| 캐스터 바퀴 | 0.125 kg | 4 | 0.5 kg |
| Tracer 전체 | | | **30 kg** |

원본 바퀴의 과도한 관성값도 다시 계산했습니다. 무게중심은 각 메시 경계 상자 중심으로 근사합니다. 차체·캐스터 지지부는 균일 경계 상자, 구동 바퀴는 링크 Y축을 중심으로 하는 균일 원통, 캐스터 바퀴는 링크 Z축을 중심으로 하는 균일 원통으로 근사합니다. 관성은 해당 무게중심 기준 kg·m²이며 관성 좌표계의 RPY는 0입니다. 실제 배터리·모터·중공 구조에 의한 물성 분포는 재현하지 않습니다.

차체 메시의 경계는 약 685 × 570 × 130.29 mm이며 상단 레일을 포함한 차체만의 경계입니다. 제조사 카탈로그의 외형 폭·길이와 일치합니다. Trossen 웹 문서의 660 × 516 × 163.5 mm와는 차이가 있어, 웹 문서 치수로 형상을 재조정하지 않고 기존 메시 치수로 관성을 근사했습니다. 실물 리비전·배터리 옵션·무게중심 확인 후 물성을 갱신해야 합니다.

[inertial_report.json](inertial_report.json)에 부품별 질량·중심·메시 경계·관성 및 출처를 저장합니다. 스크립트는 총 질량 30 kg, 양의 주관성 및 관성 삼각부등식을 확인합니다. ROS/Gazebo의 실제 주행 동역학은 Ubuntu에서 추가 검증해야 합니다.

팀원이 기존 시뮬레이션을 실행 중이라면 저장소를 pull한 뒤 Gazebo를 종료하고 다시 시작해야 합니다. 생성된 SDF/world는 이전 물성을 계속 가질 수 있으므로 실행 스크립트로 재생성합니다.
