# Portable x86_64 ROS 2 Humble image. Jetson needs a separate JetPack-compatible build.
FROM ros:humble-ros-base
ARG TORCH_INDEX=https://download.pytorch.org/whl/cpu
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions build-essential \
    ros-humble-sensor-msgs ros-humble-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*
COPY docker/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir --index-url "${TORCH_INDEX}" \
    torch==2.5.1 torchvision==0.20.1 \
    && python3 -m pip install --no-cache-dir --ignore-installed numpy==1.26.4 \
    && python3 -m pip install --no-cache-dir -r /tmp/requirements.txt \
    # Ultralytics installs regular OpenCV; keep exactly ONE cv2 distribution.
    && python3 -m pip uninstall -y opencv-python opencv-python-headless opencv-contrib-python \
    && python3 -m pip install --no-cache-dir --no-deps opencv-contrib-python-headless==4.10.0.84 \
    && python3 -c "import cv2; assert hasattr(cv2, 'TrackerCSRT_create'); assert hasattr(cv2, 'TrackerKCF_create')"
WORKDIR /ros_ws
COPY head_detection_ws/src /ros_ws/src
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "robocup_head_detection", "head_detection.launch.py"]
