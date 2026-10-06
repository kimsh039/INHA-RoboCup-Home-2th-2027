"""Mid-360 -> horizontal support surfaces (table / shelf tops) in the map frame.

Clouds are transformed to the fixed frame at their own stamp and kept for a short
window, so surfaces stay consistent while the robot moves. Publishes the surfaces
and RViz markers at a fixed rate. Geometry lives in surface_geometry.py.
"""
from collections import deque
import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray
from robocup_detection_msgs.msg import SupportSurface, SupportSurfaceArray

from .surface_geometry import SurfaceConfig, find_surfaces


def matrix(transform):
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return rotation, np.array([t.x, t.y, t.z])


class SupportSurfaceNode(Node):
    def __init__(self):
        super().__init__('support_surface_node')
        get = lambda name, default: self.declare_parameter(name, default).value
        self.frame = get('frame', 'map')
        self.window = Duration(seconds=get('window_sec', 2.0))
        # map z of the floor. The URDF has no base_footprint: base_link is 0.1425 m above the
        # floor but sits at z=0 in odom/map, so every height there is 0.1425 m too low.
        self.cfg = SurfaceConfig(floor_z=get('floor_z', -0.1425), min_height=get('min_height', 0.3),
                                 max_height=get('max_height', 1.3), min_side=get('min_side', 0.25))
        self.match_distance = get('match_distance', 0.5)
        self.track_timeout = Duration(seconds=get('track_timeout', 5.0))
        self.clouds = deque()
        self.tracks, self.next_id = [], 1
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(SupportSurfaceArray, get('output_topic', '/detection/support_surfaces'), 10)
        self.markers = self.create_publisher(MarkerArray, '/detection/support_surfaces/markers', 10)
        self.create_subscription(PointCloud2, get('input_topic', '/mid360/points_filtered'), self.on_cloud,
                                 qos_profile_sensor_data)
        self.create_timer(1.0/get('rate', 2.0), self.publish)

    def on_cloud(self, msg):
        try:
            transform = self.tf.lookup_transform(self.frame, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                                                 timeout=Duration(seconds=0.05))
        except TransformException:
            return
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(float)
        rotation, translation = matrix(transform.transform)
        stamp = Time.from_msg(msg.header.stamp)
        self.clouds.append((stamp, xyz @ rotation.T + translation))
        while self.clouds and stamp - self.clouds[0][0] > self.window:
            self.clouds.popleft()

    def assign_ids(self, surfaces, now):
        """Keep an id while a surface at the same height stays within match_distance of where it
        was last seen, even if a cycle or two missed it (track_timeout)."""
        self.tracks = [t for t in self.tracks if now - t[4] < self.track_timeout]
        ids, used = [], set()
        for s in surfaces:
            best = min((t for t in self.tracks if t[0] not in used and abs(t[3]-s.height) < 0.05),
                       key=lambda t: math.hypot(t[1]-s.centre[0], t[2]-s.centre[1]), default=None)
            if best and math.hypot(best[1]-s.centre[0], best[2]-s.centre[1]) < self.match_distance:
                ids.append(best[0])
            else:
                ids.append(self.next_id)
                self.next_id += 1
            used.add(ids[-1])
        seen = {i: (i, s.centre[0], s.centre[1], s.height, now) for i, s in zip(ids, surfaces)}
        self.tracks = [seen.pop(t[0], t) for t in self.tracks] + list(seen.values())
        return ids

    def publish(self):
        if not self.clouds:
            return
        stamp = self.clouds[-1][0].to_msg()
        surfaces = find_surfaces(np.concatenate([c for _, c in self.clouds]), self.cfg)
        out = SupportSurfaceArray()
        out.header.stamp, out.header.frame_id = stamp, self.frame
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for sid, s in zip(self.assign_ids(surfaces, self.clouds[-1][0]), surfaces):
            m = SupportSurface(id=sid, z=s.z, height=s.height, length=s.length, width=s.width,
                               edge_observed=list(s.edge_observed), inliers=s.inliers, residual=s.residual)
            m.centre.x, m.centre.y, m.centre.theta = s.centre[0], s.centre[1], s.yaw
            m.corners = [Point(x=x, y=y, z=s.z) for x, y in s.corners]
            out.surfaces.append(m)
            markers.markers += self.surface_markers(sid, s, stamp)
        self.pub.publish(out)
        self.markers.publish(markers)

    def surface_markers(self, sid, s, stamp):
        result = []
        for i in range(4):                       # one line per side: green = observed, grey = not
            a, b = s.corners[i], s.corners[(i+1) % 4]
            seen = s.edge_observed[i]
            line = Marker(ns='support_surface_edges', id=sid*10+i, type=Marker.LINE_STRIP, action=Marker.ADD)
            line.header.frame_id, line.header.stamp = self.frame, stamp
            line.points = [Point(x=a[0], y=a[1], z=s.z), Point(x=b[0], y=b[1], z=s.z)]
            line.scale.x = 0.03
            line.color.r, line.color.g, line.color.b, line.color.a = (0.1, 0.9, 0.1, 1.0) if seen >= 0.5 \
                else (0.6, 0.6, 0.6, 0.8)
            line.pose.orientation.w = 1.0
            result.append(line)
        text = Marker(ns='support_surface_labels', id=sid, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        text.header.frame_id, text.header.stamp = self.frame, stamp
        text.pose.position.x, text.pose.position.y, text.pose.position.z = s.centre[0], s.centre[1], s.z+0.15
        text.pose.orientation.w = 1.0
        text.scale.z = 0.12
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f'#{sid} h={s.height:.3f} m {s.length:.2f}x{s.width:.2f}'
        result.append(text)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SupportSurfaceNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
