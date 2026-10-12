#!/usr/bin/env python3
"""Publish physical sensor mounts; RealSense owns its internal optical/depth TF."""
import argparse
import json
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'HW/calibration/head_mid360/tools'))
from apply_koide import from_quaternion, multiply, quaternion


class SensorTF(Node):
    def __init__(self, args):
        super().__init__('robot_sensor_calibration_tf')
        self.args = args
        physical = REPO / 'HW/calibration/physical'
        mid = json.loads((physical / 'base_mid360/base_mid360.json').read_text())
        laser = json.loads((physical / 'base_2dlidar/vs_mid360/calibration.json').read_text())
        self.mounts = {
            'livox_frame': self.message('base_link', 'livox_frame', mid['matrix4x4']),
            'laser_frame': self.message('livox_frame', 'laser_frame', laser['matrix_4x4']),
        }
        values = json.loads(args.calibration.read_text())['results']['T_lidar_camera']
        self.cameras = [('livox_frame', args.camera_link, args.camera_optical,
                         from_quaternion(values[:3], values[3:]))]
        if not args.without_wrist:
            wrist = json.loads((physical / 'wrist_handeye/calibration.json').read_text())
            self.cameras.append((wrist['parent_frame'], args.wrist_link, args.wrist_optical,
                                 from_quaternion(wrist['translation_xyz_m'], wrist['quaternion_xyzw'])))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(list(self.mounts.values()))
        self.get_logger().info('Published physical Base–Mid360 and Mid360–G2 mounts. Waiting for RealSense internal TF.')
        self.last_internal = {}
        self.timer = self.create_timer(1., self.connect_camera)

    def message(self, parent, child, matrix):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = [matrix[i][3] for i in range(3)]
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = quaternion(matrix)
        return msg

    def connect_camera(self):
        for parent, camera_link, optical, measured in self.cameras:
            try:
                tf = self.buffer.lookup_transform(optical, camera_link, Time())
            except TransformException:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            signature = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
            if signature == self.last_internal.get(camera_link):
                continue
            optical_root = from_quaternion(signature[:3], signature[3:])
            mount = multiply(measured, optical_root)
            self.mounts[camera_link] = self.message(parent, camera_link, mount)
            self.broadcaster.sendTransform(list(self.mounts.values()))
            self.last_internal[camera_link] = signature
            self.get_logger().info('Applied physical calibration: ' + parent + ' -> ' + camera_link + ' -> ' + optical)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, default=REPO / 'HW/calibration/head_mid360/calib.json')
    parser.add_argument('--camera-link', default='camera_link')
    parser.add_argument('--camera-optical', default='camera_color_optical_frame')
    parser.add_argument('--wrist-link', default='wrist_link')
    parser.add_argument('--wrist-optical', default='wrist_color_optical_frame')
    parser.add_argument('--without-wrist', action='store_true')
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
