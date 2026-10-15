#!/usr/bin/env python3
"""Republish the Mid-360S IMU in base_link with the gyro bias removed, for robot_localization.

Usage (on the robot): python3 imu_base_relay.py      ->  /imu/base (sensor_msgs/Imu, frame base_link)

robot_localization does not estimate gyro bias, and the Livox gyro z bias is ~0.013 rad/s (~45 deg/min).
The bias uses 400 consecutive still samples with fresh wheel odometry, then is updated only
after the wheels have been still for > 1.5 s and |gz - bias| < 0.004 rad/s (20 s average).
Angular velocity and linear acceleration (g -> m/s^2) are rotated with the T_base_mid360 rotation, so no
TF to livox_frame is needed. Orientation is not provided (covariance[0] = -1).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from odom_imu_fusion import R_BASE_MID

G = 9.80665


class Relay(Node):
    def __init__(self):
        super().__init__('imu_base_relay')
        self.R = np.array(R_BASE_MID, float).reshape(3, 3)
        self.pub = self.create_publisher(Imu, '/imu/base', qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.bias = None; self.init = []; self.still_since = None; self.t_prev = None
        self.odom_stamp = None
        self.odom_timeout = self.declare_parameter('stationary_odom_timeout', 0.15).value

    @staticmethod
    def stamp(m): return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def on_odom(self, m):
        t = self.stamp(m)
        if not np.isfinite(t) or t <= 0:
            return
        if self.odom_stamp is not None and t < self.odom_stamp:
            self.still_since = None
            self.init.clear()
            self.t_prev = None
        self.odom_stamp = t
        if abs(m.twist.twist.linear.x) < 1e-3 and abs(m.twist.twist.angular.z) < 1e-3:
            if self.still_since is None: self.still_since = t
        else:
            self.still_since = None
            # Initial bias must not combine separate stops interrupted by motion.
            self.init.clear()

    def on_imu(self, m):
        t = self.stamp(m); w = self.R @ [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
        a = self.R @ [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
        if not np.isfinite(t) or t <= 0 or not np.all(np.isfinite(w)) or not np.all(np.isfinite(a)):
            return
        if self.t_prev is not None and t <= self.t_prev:
            self.init.clear()
            return
        fresh = self.odom_stamp is not None and abs(t - self.odom_stamp) <= self.odom_timeout
        still = fresh and self.still_since is not None and t - self.still_since > 1.5
        if self.bias is None:
            if fresh and self.still_since is not None and t - self.still_since > 0.5:
                self.init.append(w)
            else:
                self.init.clear()
            if len(self.init) >= 400:
                self.bias = np.mean(self.init, 0); self.get_logger().info(f'gyro bias {np.round(self.bias, 5).tolist()} rad/s, publishing /imu/base')
            self.t_prev = t; return
        if still and abs(w[2] - self.bias[2]) < 0.004:
            self.bias += (w - self.bias) * min(1.0, (t - self.t_prev) / 20.0)
        self.t_prev = t
        o = Imu(); o.header.stamp = m.header.stamp; o.header.frame_id = 'base_link'
        o.orientation_covariance[0] = -1.0
        g = w - self.bias; o.angular_velocity.x, o.angular_velocity.y, o.angular_velocity.z = map(float, g)
        o.angular_velocity_covariance[0] = o.angular_velocity_covariance[4] = o.angular_velocity_covariance[8] = 1e-6
        o.linear_acceleration.x, o.linear_acceleration.y, o.linear_acceleration.z = map(float, a * G)
        o.linear_acceleration_covariance[0] = o.linear_acceleration_covariance[4] = o.linear_acceleration_covariance[8] = 0.01
        self.pub.publish(o)


def main():
    rclpy.init(); n = Relay()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
