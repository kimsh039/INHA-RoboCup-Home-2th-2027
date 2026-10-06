# Mac에서 현재 자료로 이어가기

현재 Mac의 전체 계산/평가 안내는 옵시디언의 `N - RoboCup Tutorial - Calibration 01 Base와 Mid360`이다. 이 Git의 README 8~12번과 같은 수집·이동·평가 명령을 사용하고 아래 환경 차이를 적용한다.

- REPO는 자신의 Git checkout 절대 경로, PROJECT는 기존 `robocup-tutorial-calibration` 폴더로 설정한다.
- Gazebo server: `gz sim -s -r --render-engine-api-backend metal "$REPO/simulation/gazebo/build/motion.world.sdf"`. GUI: `gz sim -g --render-engine-api-backend metal`.
- ROS 창에서는 `source "$HOME/miniforge3/etc/profile.d/conda.sh"` 후 `conda activate ros_jazzy`.
- 파일 열기는 `xdg-open` 대신 `open`을 사용한다.
- 기존 실제 보정값은 `$PROJECT/calibration_data/sim/base_mid360_01/results/auto_room_20261006_031114/base_mid360.json`.
- 최신 구조의 적용 후보는 `$REPO/simulation/robot_description/robocup.calibrated.urdf`. 예전 timestamp runtime은 이전 구조의 보관본으로 남긴다.

Notion 가이드는 Ubuntu 명령으로 별도 유지한다.
