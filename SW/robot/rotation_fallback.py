#!/usr/bin/env python3
"""Supply wheel angular velocity to EKF only while calibrated gyro is unavailable."""
import copy
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class RotationFallback(Node):
    def __init__(self):
        super().__init__('rotation_fallback')
        self.imu_received = None
        self.imu_stamp = None
        self.using_wheel = None
        self.pub = self.create_publisher(Odometry, '/odom/rotation_fallback', 10)
        self.create_subscription(Imu, '/imu/base', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)

    def on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if (msg.header.frame_id == 'base_link' and math.isfinite(msg.angular_velocity.z)
                and msg.angular_velocity_covariance[0] >= 0 and stamp > 0 and -0.5 <= age <= 0.5):
            self.imu_received, self.imu_stamp = time.monotonic(), stamp

    def on_odom(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link'
                or not math.isfinite(msg.twist.twist.angular.z) or stamp <= 0 or not -0.5 <= now - stamp <= 0.5):
            return
        gyro_live = (self.imu_received is not None and time.monotonic() - self.imu_received < 0.5
                     and -0.5 <= now - self.imu_stamp <= 0.5)
        using_wheel = not gyro_live
        if using_wheel != self.using_wheel:
            self.using_wheel = using_wheel
            self.get_logger().info('회전 입력: 바퀴 각속도 대체 (MID-360 IMU 대기)' if using_wheel
                                   else '회전 입력: 보정된 MID-360 IMU')
        if using_wheel:
            output = copy.deepcopy(msg)
            # Tracer publishes zero covariance. Give its fallback yaw rate a finite uncertainty.
            output.twist.covariance = [0.0] * 36
            output.twist.covariance[35] = 0.01
            self.pub.publish(output)


def main():
    rclpy.init()
    node = RotationFallback()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
