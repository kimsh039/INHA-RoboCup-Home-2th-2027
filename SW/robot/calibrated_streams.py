#!/usr/bin/env python3
"""Publish G2 distance-scaled scans and RGB images paired with measured CameraInfo."""
import copy
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
import yaml

REPO = Path(__file__).resolve().parents[2]


class Streams(Node):
    def __init__(self):
        super().__init__('physical_calibrated_streams')
        record = json.loads((REPO / 'HW/calibration/physical/base_2dlidar/vs_mid360/calibration.json').read_text())
        self.scale = record['g2_range_scale']
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_raw', self.scan, qos_profile_sensor_data)
        self.camera_publishers = {}
        self.camera_info = {}
        for role, driver in [('head', '/head/camera'), ('wrist', '/wrist/wrist')]:
            d = yaml.safe_load((REPO / f'HW/calibration/camera_intrinsics/{role}/ost.yaml').read_text())
            info = CameraInfo()
            info.width, info.height = d['image_width'], d['image_height']
            info.distortion_model = d['distortion_model']
            info.k = list(map(float, d['camera_matrix']['data']))
            info.d = list(map(float, d['distortion_coefficients']['data']))
            info.r = list(map(float, d['rectification_matrix']['data']))
            info.p = list(map(float, d['projection_matrix']['data']))
            self.camera_info[role] = info
            self.camera_publishers[role] = (
                self.create_publisher(Image, f'/{role}/calibrated/color/image_raw', qos_profile_sensor_data),
                self.create_publisher(CameraInfo, f'/{role}/calibrated/color/camera_info', qos_profile_sensor_data),
            )
            self.create_subscription(Image, driver + '/color/image_raw',
                                     lambda msg, role=role: self.image(role, msg), qos_profile_sensor_data)
        self.get_logger().info(f'G2 /scan_raw -> /scan: range scale {self.scale}; measured Head/Wrist RGB CameraInfo enabled')

    def scan(self, msg):
        out = copy.deepcopy(msg)
        out.ranges = [float(r * self.scale) if math.isfinite(r) else float(r) for r in msg.ranges]
        out.range_min, out.range_max = msg.range_min * self.scale, msg.range_max * self.scale
        self.scan_pub.publish(out)

    def image(self, role, msg):
        source = self.camera_info[role]
        if (msg.width, msg.height) != (source.width, source.height):
            self.get_logger().error(f'{role}: calibration requires {source.width}x{source.height}; received {msg.width}x{msg.height}', throttle_duration_sec=10.)
            return
        info = copy.deepcopy(source)
        info.header = msg.header
        image_pub, info_pub = self.camera_publishers[role]
        image_pub.publish(msg)
        info_pub.publish(info)


def main():
    rclpy.init()
    node = Streams()
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
