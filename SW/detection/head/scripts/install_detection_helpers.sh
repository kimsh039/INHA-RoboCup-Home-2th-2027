#!/usr/bin/env bash
# Install the launchers used by the current Linux PC / Jetson deployment.
set -e
HEAD_HELPER_SCRIPTS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  --local)
    install -Dm755 "$HEAD_HELPER_SCRIPTS/robocup-detection" "$HOME/.local/bin/robocup-detection"
    install -Dm755 "$HEAD_HELPER_SCRIPTS/robocup-detection-window" "$HOME/.local/bin/robocup-detection-window"
    install -Dm644 "$HEAD_HELPER_SCRIPTS/native_detection_window.py" "$HOME/.local/share/robocup/native_detection_window.py"
    echo '설치 완료: ~/.local/bin/robocup-detection, robocup-detection-window'
    ;;
  --jetson)
    install -Dm755 "$HEAD_HELPER_SCRIPTS/head-detection-test" "$HOME/robot_setup/bin/head-detection-test"
    install -Dm755 "$HEAD_HELPER_SCRIPTS/head-camera-test" "$HOME/robot_setup/bin/head-camera-test"
    echo '설치 완료: ~/robot_setup/bin/head-detection-test, head-camera-test'
    ;;
  *)
    echo "Usage: bash $0 --local | --jetson" >&2
    exit 2
    ;;
esac
