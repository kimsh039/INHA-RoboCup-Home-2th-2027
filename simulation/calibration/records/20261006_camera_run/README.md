# Head / Wrist 공통 실행 기록 · 2026-10-06

**2026-10-06 23:33~23:38 KST**에 Gazebo에서 Head D435–Mid360과 Wrist D435–Link6를 순서대로 수집·계산·평가했다. 전용 partition은 `robocup_auto_20261006`이며 수집용 서버·GUI는 종료했다. 두 과정 모두 학습 25개, 별도 평가 10개다.

- [Head–Mid360 결과와 원자료](../20261006_head_mid360/README.md): 별도 평면 평가 4.247mm / 0.814°, GT 위치 오차 **11.720mm**.
- [Wrist 결과와 원자료](../20261006_wrist_d435/README.md): 별도 태그 평가 0.339mm / 0.294°, GT 위치 오차 **0.598mm**.
- [원래 실행 보고서](REPORT.md), [전체 평가 JSON](summary.json), [목표각 분리 기록](angle_overlap_plan.json), [실제 실행 명령과 시각](commands.jsonl).

## 무엇을 보관했나

`config/measurement_model.urdf`는 **측정 당시 FK·장면 배치에 사용한 모델의 원본**이다. 현재 로봇의 최종 보정 모델을 교체하는 파일이 아니다. Wrist 장면은 [Wrist config](../20261006_wrist_d435/config/wrist.world.sdf), Head 장면은 [Head config](../20261006_head_mid360/config/head_mid.world.sdf)에 보관했다. 목표 관절 CSV와 태그 PNG도 `config/`에 있다.

`source/`는 실행 시점 수집기·FK·평면 solver·hand-eye solver·자동화 드라이버·Python 패키지 버전의 **보관용 스냅샷**이다. 원래 프로젝트 폴더 구조와 Mac Gazebo backend를 그대로 보존한 코드이므로 이 폴더에서 바로 실행하는 Ubuntu 가이드로 해석하지 않는다. 실행 명령의 절대 경로도 당시 기록을 그대로 남겼다. Ubuntu에서 기존 저장 결과를 읽는 명령은 각 결과 README에 있다.

관측 dataset의 `repository_commit`은 수집 코드에 들어 있던 초기 프로젝트 커밋 문자열이다. 최신 Git 모델의 측정 증거로 사용하지 않는다. 실제 사용한 모델·장면·코드는 [manifest.json](manifest.json)의 SHA-256과 각 센서 manifest로 식별한다. 파일 내용과 내부 hash 참조를 유지하기 위해 원본 JSON/CSV를 재작성하지 않았다.

## 저장·계산·평가 범위

정지 자세마다 RGB 한 장과 필요한 점군 한 개를 저장했다. Gazebo 연속 상태 로그나 영상 녹화는 사용하지 않았다. Head 사진/점군은 보드가 정지한 상태에서 순서대로 수집했으며 동일 timestamp라고 가정하지 않는다. 이미지–관절 시각 차이는 최대 2ms였다. GT/CAD는 보드 배치와 최종 평가에만 사용했고 solver에는 측정 관측을 넣었다.

Git에는 RGB/미리보기 PNG, 원시 압축 점군, XYZ NPZ, 평면·관절·시각·자세 기록, 대응 CSV, 보정 JSON과 기존 평가 보고서를 올렸다. 재생성 가능한 `xyz.html`, 미완성 폴더, console 로그, venv는 제외했고 로컬 자료는 삭제하지 않았다. 이번 Git 업로드에서는 추가 측정·테스트·재평가를 실행하지 않았다.

두 카메라의 결과는 **보정 JSON과 평가 기록까지 등록**했다. Base–Head 합성 및 카메라 URDF 반영은 아직 하지 않았다. 기존 `robocup.calibrated.urdf`에는 Base–2D LiDAR와 Base–Mid360 보정이 들어 있다.
