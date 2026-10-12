#!/bin/bash
# Robot sensors and accepted calibration TF in tmux: start | stop | status
#   tracer (CAN can2, odom v4) | g2 (/scan) | livox (Mid-360S PointCloud2 + IMU) | head (D435 244622071116, /head/camera/*)
H=/home/sparo; SRC="source /opt/ros/humble/setup.bash"
PAT=(tracer_base_node ydlidar_ros2_driver_node livox_ros_driver2_node realsense2_camera_node sensor_tf.py)
start() {
  tmux has-session -t tracer 2>/dev/null || tmux new-session -d -s tracer "$SRC; source $H/tracer_ws/install/setup.bash; ros2 launch tracer_base tracer_base.launch.py port_name:=can2 odom_frame:=odom base_frame:=base_link odom_topic_name:=odom is_tracer_mini:=false simulated_robot:=false 2>&1 | tee /tmp/tracer.log; exec bash"
  tmux has-session -t g2 2>/dev/null || tmux new-session -d -s g2 "$SRC; source $H/tracer_ws/install/setup.bash; ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args --params-file $H/tracer_ws/src/ydlidar_ros2_driver/params/G2.yaml 2>&1 | tee /tmp/g2.log; exec bash"
  tmux has-session -t livox 2>/dev/null || tmux new-session -d -s livox "$SRC; source $H/ws_livox/install/setup.bash; ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -r __node:=livox_lidar_publisher -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame -p user_config_path:=$H/ws_livox/src/livox_ros_driver2/config/MID360s_config.json -p cmdline_input_bd_code:=livox0000000001 2>&1 | tee /tmp/livox.log; exec bash"
  tmux has-session -t head 2>/dev/null || tmux new-session -d -s head "$SRC; ros2 launch realsense2_camera rs_launch.py camera_namespace:=head camera_name:=camera serial_no:=\"_244622071116\" rgb_camera.color_profile:=640,480,15 depth_module.depth_profile:=640,480,15 enable_gyro:=false enable_accel:=false pointcloud__neon_.enable:=true decimation_filter.enable:=true 2>&1 | tee /tmp/head.log; exec bash"
  tmux has-session -t calibration_tf 2>/dev/null || tmux new-session -d -s calibration_tf "bash $H/robocup/SW/robot/start_sensor_tf.sh 2>&1 | tee /tmp/robot_sensor_tf.log; exec bash"
}
stop() {   # Ctrl+C first and wait for a clean shutdown (the G2 driver stops its motor on exit)
  for s in calibration_tf head livox g2 tracer; do tmux send-keys -t $s C-c 2>/dev/null; done
  for i in $(seq 1 30); do alive=0; for p in "${PAT[@]}"; do pgrep -f "/$p( |$)" >/dev/null && alive=1; done; [ $alive = 0 ] && break; sleep 0.5; done
  for s in calibration_tf head livox g2 tracer; do tmux kill-session -t $s 2>/dev/null; done
  for p in "${PAT[@]}"; do pgrep -f "/$p( |$)" >/dev/null && echo "still running: $p"; done; echo stopped
}
status() {
  $SRC; source $H/tracer_ws/install/setup.bash
  for t in /odom /tracer_status /scan /livox/lidar /livox/imu /head/camera/color/image_raw /head/camera/depth/image_rect_raw; do
    r=$(timeout 8 ros2 topic hz $t 2>/dev/null | grep -m1 average); echo "$t ${r:-NO DATA}"; done
}
case "$1" in start) start; sleep 12; status ;; stop) stop ;; status) status ;; *) echo "usage: $0 start | stop | status" ;; esac
