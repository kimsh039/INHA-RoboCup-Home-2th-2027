# 저장소 정리·검증 기준 · 2026-10-16

## 조사 범위

최신 main `a23d6e287a29ed34e65aaaab439b57405108636a`까지 pull한 뒤 Git 관리 파일 11,012개의 목록·용량·문서 링크를 조사하고, 현재 실행 진입점과 코드/설정 참조를 대조했다. 모든 이미지의 장면을 다시 수동 검토하거나 모든 실물 기능을 실행한 작업은 아니다.

현재 실행 기준은 [저장소 첫 페이지](../README.md), [SW 안내](README.md), [Head](detection/head/README.md), [Wrist](detection/wrist/README.md), [전체 Task](control_center/TASK.md)다.

## 무엇을 제거했는가

| 제거 대상 | 판단 근거 | 보존/대체 |
|---|---|---|
| datasets/20261007, 20261012의 이미지·라벨·중복 준비물 | 현재 20261013에 기존 영상/source_frame 1,262개가 전부 있고 이미지 SHA256도 동일 | 현재 2,155장·수정 txt·해시·실험 지표 유지, 과거 버전은 Git 커밋 |
| 구형 Colab notebooks / migrate_six_classes / training/data | 완료된 일회성 7→6클래스 마이그레이션과 구형 중복 실행 경로 | 현재 training/train_local.py / datasets/20261013 |
| head/test 직접 /cmd_vel 추종 프로토타입 | 현재 LiDAR·Nav2·closed approach/인계 경로에서 참조하지 않음 | 현재 control_center/DETECTION.md / 독립 제어·최종 guard |
| wrist/realsense_sam2 Small USB 실험 | 현재 ROS Wrist Tiny frontend와 별개, 빈 requirements/test 파일 포함 | manipulation/wrist_frontend.py / config/wrist.yaml |
| 구형 7클래스 last 가중치 | 현재 배포 모델과 관계없는 중간/최종 학습 산출물 | 20261013 .pt, 과거 모델/지표의 Git 이력 |
| 20261007/12의 반복 주석 MP4와 대표 frame 미디어 | 현재 영상과 별개인 재생성 가능한 중복 결과 | 원본 지표/CSV/JSON·그래프, 과거 미디어의 고정 커밋 링크 |
| patch_control_center_joints 도구와 전용 테스트 | 현재 DetectionApproach에 ModelJointFeedback가 이미 직접 통합돼 있어 구형 constructor 패치가 불필요 | joint_contract.py와 현재 관절 계약 테스트 유지 |

디텍션 작업 트리 파일 용량은 약 **891 → 454 MiB**로 줄었다. 제거 파일은 약 **437 MiB**다. 이는 현재 checkout의 감소이며 Git 이력/clone 전체 크기를 줄인다는 의미는 아니다. 이력 rewrite·강제 push·GC는 하지 않는다.

현재 데이터·학습 모델·보정 원본·실행 노드·ROS 메시지·실제 통합 테스트는 유지한다. 유효한 배경 이미지의 빈 label, ament resource marker, Python __init__ 파일은 “빈 파일”이라는 이유로 삭제하지 않는다. SIM_NAV_TEST의 demo/선택형 x86 Docker는 여전히 문서/실행 코드가 참조해 유지하되 Jetson 기본 경로와 구분했다.

## 문서와 메타데이터

루트/SW/HW/Head/Wrist/학습/실물 hand-eye README를 현재 구조에 맞췄다. 구형 설계에는 역사적 범위를 표시하고, 제거 자료의 상대 링크는 고정 커밋 링크로 바꿨다. manipulation의 과거 클래스 매핑 출처는 원래 JSON/YAML 계약값을 바꾸지 않고 Git 출처 URL로 유지한다.

실물 calibration, execution/commissioned/verified 플래그, 속도·힘·도킹·충돌 임계값은 이 정리에서 바꾸지 않는다. sensor_tf의 최신 ChArUco와 manipulation 프로파일의 이전 best15 차이는 문서에 명시했다.

## 읽기 전용 전체 검사

저장소 루트에서 Python 표준 라이브러리만으로:

```bash
python SW/tools/audit_repository.py --verify-dataset
```

현재 Git 관리/신규 파일의 Markdown 상대 링크, Python AST, JSON 구문, 주요 URDF 루트·메시 참조, 현재 데이터 이미지/라벨 4,310개 SHA256을 검사한다. 웹 URL의 실제 접근성, Markdown 앵커, ROS 런타임·C++ 빌드·모터·동역학·객체 정확도 검증까지 포함하는 검사는 아니다.

회귀 검사에서 전체 Task 탭이 사이에 들어간 UI 렌더 순서 가정과, 현재 odometry callback이 요구하는 pose가 없는 가짜 입력 두 곳을 현재 계약에 맞췄다. 실행 알고리즘/모터 명령 경로를 바꾼 수정이 아니다. Windows에서 가능한 검사와 ROS/Linux·symlink 권한이 필요한 검사를 구분해 최종 결과를 기록한다.

## 과거 자료 복구

삭제 직전 커밋은 [a23d6e2](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/tree/a23d6e287a29ed34e65aaaab439b57405108636a)다. 예를 들어 원본 코드만 읽으려면:

```bash
git show a23d6e2:SW/detection/wrist/realsense_sam2/src/sam_camera_depth.py
git archive -o /tmp/robocup-old-data.tar a23d6e2 SW/detection/head/datasets/20261012
```

복구는 별도 디렉터리에서 한다. 현재 데이터나 실물 프로파일을 과거 파일로 덮어쓰지 않는다. 과거 7클래스 배포 모델은 [0c6e820 당시 모델](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/tree/0c6e82043047c1a7b0181af7c333bb23d7e0992a/SW/detection/head/models)에 남아 있다.

[발표 공부 가이드](PRESENTATION_STUDY_GUIDE.md)

## 이번 정리의 최종 검사 결과

- 정리 후 관리/신규 파일 5,858개. Markdown 124개 링크 검사, Python 292개 AST, JSON 405개 구문, URDF 3개 루트/메시 확인: 오류 0.
- 현재 이미지+라벨 4,310개 SHA256 전부 일치. 현재 가중치·2155장·보정 JSON은 변경하지 않았다.
- Windows 실행 가능한 오프라인 회귀: **193 passed / 1 skipped / 3 deselected**, 추가 subtest 3개 통과. ROS/Linux/fcntl/권한이 필요한 11개 테스트 파일은 이 실행 대상에서 제외했다. deselected 3개는 테스트 메서드 내부에서 ROS 메시지를 요구한다. 이 숫자를 전체 ROS 통합 시험 통과로 부르지 않는다.
- 관제 app.js 구문 검사와 실제 DOM 상태를 사용하는 manipulation handoff Node 회귀 통과.
- 실물 모터·카메라·ROS 통합·C++ 빌드·전체 Task 물리 실행은 이번 정리에서 수행하지 않았다.
