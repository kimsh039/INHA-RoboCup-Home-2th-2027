#!/usr/bin/env bash
# Install the launchers used by the current Linux PC / Jetson deployment.
set -e
HEAD_HELPER_SCRIPTS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  --local)
    install -Dm755 "$HEAD_HELPER_SCRIPTS/robocup-detection" "$HOME/.local/bin/robocup-detection"
    echo '설치 완료: ~/.local/bin/robocup-detection'
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
