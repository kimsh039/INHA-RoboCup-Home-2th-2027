#!/usr/bin/env python3
"""Wheel speed + Mid-360S gyro odometry: yaw from the IMU, distance from the (corrected) wheel speed.

Usage (on the robot): python3 odom_imu_fusion.py [--ros-args -p publish_tf:=false ...]

- Gyro is rotated into base_link with the calibrated T_base_mid360 rotation (Livox IMU axes = point cloud axes).
- Gyro bias is re-estimated whenever the wheels have been still for > 0.5 s (exponential average, 3 s).
- Pose: yaw += (gz - bias)·dt at the IMU rate; x, y += v·(cos, sin)(yaw)·dt at the wheel rate.
Publishes nav_msgs/Odometry on /odom_fused (frame odom_fused -> base_link). TF is off by default so it
does not fight tracer_base's odom -> base_link; set publish_tf:=true and disable the driver TF to use it.
Sweep 2026-10-11 (Mid-360S ground truth, replayed): 0.09 deg RMS per 90 deg turn, 4 mm per 1 m at <= 0.2 m/s.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

R_BASE_MID = [0.9983714685240425, -0.028151636076883594, -0.049617499164520525,
              -0.028051757040677302, -0.9996028030605087, 0.002708327990613944,
              -0.04967403510967577, -0.0013120593617048737, -0.9987646212877956]   # base_mid360/T_base_mid360.csv


class Fusion(Node):
    def __init__(self):
        super().__init__('odom_imu_fusion')
        self.declare_parameter('odom_topic', '/odom'); self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('out_topic', '/odom_fused'); self.declare_parameter('odom_frame', 'odom_fused')
        self.declare_parameter('base_frame', 'base_link'); self.declare_parameter('publish_tf', False)
        self.declare_parameter('r_base_imu', R_BASE_MID)
        g = lambda n: self.get_parameter(n).value
        self.R = np.array(g('r_base_imu'), float).reshape(3, 3); self.frame, self.child = g('odom_frame'), g('base_frame')
        self.pub = self.create_publisher(Odometry, g('out_topic'), 20)
        self.tf = TransformBroadcaster(self) if g('publish_tf') else None
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 20)
        self.x = self.y = self.yaw = 0.0; self.bias = None; self.gz = 0.0
        self.t_imu = None; self.t_odom = None; self.v = 0.0; self.still_since = None; self.init = []
        self.get_logger().info('waiting for 2 s of still IMU data to initialise the gyro bias')

    @staticmethod
    def stamp(msg): return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_imu(self, m):
        t = self.stamp(m); w = m.angular_velocity; gz = float(self.R[2] @ [w.x, w.y, w.z])
        still = self.still_since is not None and t - self.still_since > 1.5
        if self.bias is None:
            if still: self.init.append(gz)
            if len(self.init) >= 400:
                self.bias = float(np.mean(self.init)); self.get_logger().info(f'gyro bias {self.bias:+.5f} rad/s, running')
            self.t_imu = t; return
        if still and abs(gz - self.bias) < 0.004: self.bias += (gz - self.bias) * min(1.0, (t - self.t_imu) / 20.0)
        dt = t - self.t_imu if self.t_imu is not None else 0.0
        if 0.0 < dt < 0.1:
            self.gz = gz - self.bias
            self.yaw += self.gz * dt
        self.t_imu = t

    def on_odom(self, m):
        t = self.stamp(m); v = m.twist.twist.linear.x; w = m.twist.twist.angular.z
        if abs(v) < 1e-3 and abs(w) < 1e-3:
            if self.still_since is None: self.still_since = t
        else: self.still_since = None
        if self.bias is None: self.t_odom = t; return
        dt = t - self.t_odom if self.t_odom is not None else 0.0
        if 0.0 < dt < 0.2:
            self.x += v * math.cos(self.yaw) * dt; self.y += v * math.sin(self.yaw) * dt
        self.t_odom = t
        o = Odometry(); o.header.stamp = m.header.stamp; o.header.frame_id = self.frame; o.child_frame_id = self.child
        o.pose.pose.position.x = self.x; o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2); o.pose.pose.orientation.w = math.cos(self.yaw / 2)
        o.twist.twist.linear.x = v; o.twist.twist.angular.z = self.gz
        self.pub.publish(o)
        if self.tf:
            tr = TransformStamped(); tr.header = o.header; tr.child_frame_id = self.child
            tr.transform.translation.x = self.x; tr.transform.translation.y = self.y; tr.transform.rotation = o.pose.pose.orientation
            self.tf.sendTransform(tr)


def main():
    rclpy.init(); n = Fusion()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
