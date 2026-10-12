#!/usr/bin/env python3
"""Publish physical sensor mounts; RealSense owns its internal optical/depth TF."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'HW/calibration/head_mid360/tools'))
from apply_koide import fixed_transform, from_quaternion, multiply, quaternion


class SensorTF(Node):
    def __init__(self, args):
        super().__init__('robot_sensor_calibration_tf')
        self.args = args
        root = ET.parse(args.urdf).getroot()
        self.mounts = [self.message('base_link', child, fixed_transform(root, 'base_link', child))
                       for child in ('livox_frame', 'laser_frame')]
        values = json.loads(args.calibration.read_text())['results']['T_lidar_camera']
        self.lidar_rgb = from_quaternion(values[:3], values[3:])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(self.mounts)
        self.get_logger().info('Published Base–Mid360 and Base–G2 mounts. Waiting for RealSense internal TF: '
                               + args.camera_link + ' -> ' + args.camera_optical)
        self.last_internal = None
        self.timer = self.create_timer(1., self.connect_camera)

    def message(self, parent, child, matrix):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = [matrix[i][3] for i in range(3)]
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = quaternion(matrix)
        return msg

    def connect_camera(self):
        try:
            # Optical <- driver root, using the driver's actual internal extrinsics.
            tf = self.buffer.lookup_transform(self.args.camera_optical, self.args.camera_link, Time())
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        signature = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        if signature == self.last_internal:
            return
        optical_root = from_quaternion(signature[:3], signature[3:])
        lidar_root = multiply(self.lidar_rgb, optical_root)
        self.broadcaster.sendTransform(self.mounts + [self.message('livox_frame', self.args.camera_link, lidar_root)])
        self.last_internal = signature
        self.get_logger().info('Applied Koide: livox_frame -> ' + self.args.camera_link
                               + ' -> ' + self.args.camera_optical)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, default=REPO / 'SW/simulation/robot_description/robocup.calibrated.urdf')
    parser.add_argument('--calibration', type=Path, default=REPO / 'HW/calibration/head_mid360/calib.json')
    parser.add_argument('--camera-link', default='camera_link')
    parser.add_argument('--camera-optical', default='camera_color_optical_frame')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = SensorTF(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
