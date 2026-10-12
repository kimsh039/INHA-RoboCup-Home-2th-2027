# Jetson 실기 센서 TF

```bash
bash ~/robocup/SW/robot/start_sensor_tf.sh
```

ROS 2 Humble의 rclpy, tf2_ros, geometry_msgs를 사용하며 별도 colcon 빌드 없이 실행합니다. 센서 드라이버는 별도로 실행합니다. [Head–Mid360 적용 안내](../../HW/calibration/head_mid360/README.md)에 프레임 구성, RealSense 내부 TF 사용, 중복 발행 방지와 재시작 방법이 있습니다.

이 프로세스를 계속 켜두면 기존 URDF의 Base–Mid360/G2와 2026-10-12 Koide Head 보정이 `/tf_static`으로 발행됩니다. 설정/URDF를 변경한 뒤에는 이 프로세스를 재시작하세요. 로봇 실행 시 사용하는 동일한 ROS_DOMAIN_ID 환경에서 센서와 TF 노드를 실행하세요.

센서와 TF를 tmux로 함께 실행하는 젯슨용 실행기도 포함했습니다. 현재 로컬 `~/livox_g2_calib/sensors.sh`와 같은 CAN/USB/센서 설정입니다.

```bash
bash ~/robocup/SW/robot/jetson_sensors.sh start
# 정상 종료
bash ~/robocup/SW/robot/jetson_sensors.sh stop
```
