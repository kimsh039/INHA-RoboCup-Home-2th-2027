# ROS 2 시뮬레이션

저장소 루트에서 ROS 2 Humble 환경을 활성화하고 실행합니다.
최종 `robot_description/robocup.urdf`를 읽으며 손목 카메라는 항상 포함됩니다.
`gazebo/start_sim.sh`와 동시에 실행하지 마세요.

```bash
source /opt/ros/humble/setup.bash
ros2 launch simulation/ros2/sim.launch.py world:=room
# GUI/RViz 없이 서버만 실행
ros2 launch simulation/ros2/sim.launch.py world:=room gui:=false rviz:=false
```

기본 world는 `empty`, gui/rviz는 `true`입니다. partition은 `robocup_motion`, 기본 GZ_IP는 127.0.0.1입니다.
Gazebo에 NVIDIA가 사용 가능하면 PRIME offload를 자동 지정합니다.
TF는 DiffDrive의 odom→base_link와 robot_state_publisher의 base_link 이하 트리로 연결됩니다.
RViz Fixed Frame은 `odom`, 모든 노드는 simulation time을 사용합니다.

다른 터미널에서 ROS 환경을 활성화한 뒤:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /joint_states
ros2 topic hz /wrist_camera/color/image_raw
ros2 topic hz /wrist_camera/depth/image_raw
```

현재 브리지는 /clock, /cmd_vel, /odom, /tf, /joint_states, /scan와 손목 color/depth 영상·CameraInfo·depth points입니다.
정확한 매핑은 [ros_bridge.yaml](ros_bridge.yaml)을 참고하세요.
헤드 영상과 Mid-360S는 Gazebo 토픽/팝업을 사용하며 현재 ROS 브리지에는 포함하지 않습니다.
SLAM/Nav2 설정은 slam_params.yaml과 nav2_params.yaml에 있지만 이 런치가 해당 노드들을 실행하지는 않습니다.

최종 URDF 정리 후 Gazebo 센서는 기존 개발 PC에서 재검증했습니다. 당시 개발 PC에는 ROS 2가 없어 ROS 런치는 구문 검사까지만 했습니다. 별도 Jetson에는 ROS Humble을 준비했지만 Gazebo·bridge를 설치하거나 이 시뮬레이션 launch를 실행하지 않았습니다.
