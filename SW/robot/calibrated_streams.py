#!/usr/bin/env python3
"""Publish G2 distance-scaled scans and RGB images paired with measured CameraInfo."""
import copy
import json
from pathlib import Path
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
import yaml

from collision_geometry import RobotGeometry
from scan_self_filter import ScanSelfMask

REPO = Path(__file__).resolve().parents[2]


class Streams(Node):
    def __init__(self):
        super().__init__('physical_calibrated_streams')
        record = json.loads((REPO / 'HW/calibration/physical/base_2dlidar/vs_mid360/calibration.json').read_text())
        self.scale = record['g2_range_scale']
        # Rack posts around the G2: beams in these sectors are blocked or jump at the post edges.
        mask = REPO / 'SW/robot/config/g2_self_mask.yaml'
        self.mask_sectors = yaml.safe_load(mask.read_text())['sectors_deg'] if mask.is_file() else []
        safety = yaml.safe_load((REPO / 'SW/robot/config/collision_safety.yaml').read_text())['/**']['ros__parameters']
        model = RobotGeometry()
        geometry = model.fixed+model.self_interiors if safety['mask_rack_interior'] else model.fixed
        mid = json.loads((REPO / 'HW/calibration/physical/base_mid360/base_mid360.json').read_text())
        # The exact same committed rigid transforms are published by sensor_tf.py.
        sensor_to_base = np.asarray(mid['matrix4x4']) @ np.asarray(record['matrix_4x4'])
        paddings = []
        for box in geometry:
            padding = safety['self_padding']
            if box.name == 'self_interior/rack_base_link':
                padding = max(padding, safety['rack_self_padding'])
            if box.name.startswith('base_link/') or '_wheel_link/' in box.name:
                padding = max(padding, safety['base_self_padding'])
            paddings.append(padding)
        self.scan_mask = ScanSelfMask(geometry, sensor_to_base, safety['self_padding'], safety['min_sensor_range'],
                                     paddings=paddings)
        self.scan_stats = dict(ready=False, reason='waiting_for_scan')
        self.scan_status_pub = self.create_publisher(String, '/scan/filter_status', 1)
        self.create_timer(1., lambda: self.scan_status_pub.publish(String(data=json.dumps(self.scan_stats))))
        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', scan_qos)
        self.create_subscription(LaserScan, '/scan_raw', self.scan, scan_qos)
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
        self.get_logger().info(f'G2 /scan_raw -> /scan: range scale {self.scale}, {len(self.mask_sectors)} self-occlusion '
                               f'sectors and URDF body/rack interior masked; measured Head/Wrist RGB CameraInfo enabled')

    def scan(self, msg):
        began = time.monotonic()
        try:
            out, stats = self.scan_mask.apply(msg, self.scale, self.mask_sectors)
            self.scan_stats = dict(stats, ready=True, reason='live', processing_ms=(time.monotonic()-began)*1000,
                                   source_stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec)
            self.scan_pub.publish(out)
        except ValueError as exc:
            self.scan_stats = dict(ready=False, reason=str(exc))
            self.get_logger().error('G2 self-filter rejected scan: '+str(exc), throttle_duration_sec=2.)

    def image(self, role, msg):
        source = self.camera_info[role]
        if (msg.width, msg.height) != (source.width, source.height):
            self.get_logger().error(f'{role}: calibration requires {source.width}x{source.height}; received {msg.width}x{msg.height}', throttle_duration_sec=10.)
            return
        info = copy.copy(source)
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
