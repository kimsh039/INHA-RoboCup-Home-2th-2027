#!/usr/bin/env python3
"""Publish only SLAM's map->odom correction with a current TF validity timestamp.

Humble 2.6.10 timestamps repeated transforms with the last scan time and does
not implement restamp_tf. The backend's broadcaster is node-remapped to a private
topic; this adapter is the sole public map->odom writer. Pose values are unchanged.
"""
import copy
import math

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def map_transforms(msg, map_frame, odom_frame, stamp):
    result = []
    for tr in msg.transforms:
        if tr.header.frame_id != map_frame or tr.child_frame_id != odom_frame:
            continue
        p, q = tr.transform.translation, tr.transform.rotation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            continue
        if q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12:
            continue
        current = copy.deepcopy(tr)
        current.header.stamp = stamp
        result.append(current)
    return result


class SlamTFClock(Node):
    def __init__(self):
        super().__init__('slam_tf_clock')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.timeout = Duration(seconds=self.declare_parameter('transform_timeout', 0.2).value)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(TFMessage, '/slam_toolbox/tf_raw', self.relay, qos_profile_sensor_data)
        self.get_logger().info('SLAM map->odom clock adapter; sole public writer, pose values preserved')

    def relay(self, msg):
        # Forward only newly received backend messages. Never extend TF validity
        # with a timer after the backend has stopped sending corrections.
        transforms = map_transforms(msg, self.map_frame, self.odom_frame,
                                    (self.get_clock().now() + self.timeout).to_msg())
        if transforms:
            self.broadcaster.sendTransform(transforms)


def main():
    rclpy.init()
    node = SlamTFClock()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
