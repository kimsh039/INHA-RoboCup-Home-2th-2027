#!/usr/bin/env bash
# Optional one-time installation on Ubuntu 22.04 / ROS 2 Humble.
set -e
sudo apt-get update
sudo apt-get install -y tmux python3-opencv python3-numpy \
  ros-humble-rclpy ros-humble-cv-bridge ros-humble-tf2-ros \
  ros-humble-rosidl-runtime-py ros-humble-nav-msgs ros-humble-sensor-msgs \
  ros-humble-vision-msgs ros-humble-slam-toolbox
printf '\n웹 관제 의존성 설치 명령이 완료됐습니다. 카메라·LiDAR 드라이버와 YOLO 환경은 기존 Jetson 설치를 사용합니다.\n'
