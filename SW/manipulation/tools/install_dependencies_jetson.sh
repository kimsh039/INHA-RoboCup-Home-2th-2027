#!/usr/bin/env bash
# Run by the operator on Jetson. Does not install/change camera drivers, models or team venvs.
set -euo pipefail
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep python3-yaml python3-numpy python3-opencv python3-pytest ros-humble-cv-bridge ros-humble-moveit ros-humble-control-msgs ros-humble-sensor-msgs ros-humble-tf2-ros ros-humble-robot-state-publisher
printf '%s\n' 'Reuse the installed Piper ROS workspace and existing Jetson PyTorch/YOLO/SAM venv.' 'See docs/SYSTEM_INTEGRATION.md for model paths and environment checks.'
