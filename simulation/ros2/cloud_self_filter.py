#!/usr/bin/env python3
"""Drop point-cloud returns that hit the robot itself, for Nav2 costmaps.

The Mid-360S is mounted upside down at ~1.2 m and sees the rack and arm. Any point whose
base_link (x, y) lies inside the footprint box is removed; the rest is republished in
base_link as an xyz cloud. Moving the Piper arm outside the footprint still marks it.
Isolated returns (alone in their voxel) are dropped too: the Gazebo gpu_lidar emits a few
fixed stray points, and real sensors produce single-point speckle that Nav2 would keep as
lethal obstacles.

Params: input (/mid360/points), output (/mid360/points_filtered),
        box [x_min, x_max, y_min, y_max] in base_link (Tracer 0.70 x 0.58 m footprint),
        voxel (0.1 m) and min_points_per_voxel (2) for the isolated-point filter.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener


class CloudSelfFilter(Node):
    def __init__(self):
        super().__init__('cloud_self_filter')
        self.box = self.declare_parameter('box', [-0.36, 0.36, -0.30, 0.30]).value
        self.voxel = self.declare_parameter('voxel', 0.1).value
        self.min_points = self.declare_parameter('min_points_per_voxel', 2).value
        self.pub = self.create_publisher(
            PointCloud2, self.declare_parameter('output', '/mid360/points_filtered').value,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.declare_parameter('input', '/mid360/points').value,
            self.callback, qos_profile_sensor_data)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.transform = None  # sensor -> base_link is a fixed mount, look it up once

    def callback(self, msg):
        if self.transform is None:
            try:
                t = self.tf.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(f'waiting for base_link <- {msg.header.frame_id}: {e}',
                                       throttle_duration_sec=5.0)
                return
            q, p = t.transform.rotation, t.transform.translation
            self.transform = (Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                              np.array([p.x, p.y, p.z]))
        rot, trans = self.transform
        a = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.column_stack([a['x'], a['y'], a['z']]).astype(np.float32)
        pts = pts[np.isfinite(pts).all(axis=1)] @ rot.T.astype(np.float32) + trans.astype(np.float32)
        x0, x1, y0, y1 = self.box
        inside = (pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1)
        pts = pts[~inside]
        _, idx, counts = np.unique(np.floor(pts / self.voxel).astype(np.int32), axis=0,
                                   return_inverse=True, return_counts=True)
        pts = pts[counts[idx.ravel()] >= self.min_points]
        header = msg.header
        header.frame_id = 'base_link'
        self.pub.publish(pc2.create_cloud_xyz32(header, pts))


def main():
    rclpy.init()
    rclpy.spin(CloudSelfFilter())


if __name__ == '__main__':
    main()
