"""Target on a support surface -> staging pose (Nav2) and docking pose (closed approach).

Inputs: /detection/support_surfaces and the confirmed target point in the map frame.
Outputs: /detection/approach/{stage_pose,dock_pose,status} and RViz markers. Geometry and
the reach / clearance numbers are in approach_geometry.py.
"""
import math
from types import SimpleNamespace
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from robocup_detection_msgs.msg import SupportSurfaceArray

from .approach_geometry import ApproachConfig, approach_candidates, supporting_surface

FOOTPRINT = ((0.35, 0.29), (0.35, -0.29), (-0.35, -0.29), (-0.35, 0.29))   # Nav2 footprint, base_link


class ApproachNode(Node):
    def __init__(self):
        super().__init__('approach_node')
        get = lambda name, default: self.declare_parameter(name, default).value
        defaults = ApproachConfig()
        self.cfg = ApproachConfig(object_offset=tuple(get('object_offset', list(defaults.object_offset))),
                                  min_edge=get('min_edge', defaults.min_edge), d_stage=get('d_stage', defaults.d_stage),
                                  min_observed=get('min_observed', defaults.min_observed))
        # map z of the floor: base_link sits 0.1425 m above it at z=0 (no base_footprint in the URDF).
        self.floor_z = get('floor_z', -0.1425)
        self.surfaces = self.target = None
        self.plan = None                 # (target xy, best candidate, surface) kept for the same target
        self.last_status = ''
        self.stage_pub = self.create_publisher(PoseStamped, '/detection/approach/stage_pose', 10)
        self.dock_pub = self.create_publisher(PoseStamped, '/detection/approach/dock_pose', 10)
        self.edge_pub = self.create_publisher(Float32, '/detection/approach/dock_edge_distance', 10)
        self.status_pub = self.create_publisher(String, '/detection/approach/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detection/approach/markers', 10)
        self.create_subscription(SupportSurfaceArray, '/detection/support_surfaces', self.on_surfaces, 10)
        self.create_subscription(PointStamped, get('target_topic', '/detection/target_map_point'), self.on_target, 10)

    def report(self, label):
        self.status_pub.publish(String(data=label))
        if label != self.last_status:
            self.get_logger().info(label)
            self.last_status = label

    def on_target(self, msg):
        if msg.header.frame_id and msg.header.frame_id != 'map':
            return self.report('TARGET_NOT_IN_MAP_FRAME')
        if self.plan and math.hypot(msg.point.x-self.plan[0][0], msg.point.y-self.plan[0][1]) > 0.05:
            self.plan = None             # a different target: plan again
        self.target = msg
        self.update()

    def hold(self, reason):
        """Keep publishing the last plan for this target: close to the table the edge evidence
        changes (the dock node re-measures the edge itself), the plan should not flicker."""
        if self.plan is None:
            self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            return self.report(reason)
        _, best, surface = self.plan
        self.publish_plan(best, surface, f'APPROACH_HELD:{reason}')

    def on_surfaces(self, msg):
        self.surfaces = msg
        self.update()

    def update(self):
        if self.target is None:
            return self.report('WAITING_TARGET')
        if self.surfaces is None:
            return self.report('WAITING_SURFACES')
        p = self.target.point
        surfaces = [SimpleNamespace(id=s.id, height=s.height, edge_observed=tuple(s.edge_observed),
                                    corners=tuple((c.x, c.y) for c in s.corners)) for s in self.surfaces.surfaces]
        surface = supporting_surface(surfaces, (p.x, p.y), p.z-self.floor_z, self.cfg)
        if surface is None:
            return self.hold('NO_SUPPORT_SURFACE')
        candidates = approach_candidates(surface, surface.id, (p.x, p.y), self.cfg)
        if not candidates:
            return self.hold(f'NO_REACHABLE_APPROACH:surface_{surface.id}')
        best = candidates[0]
        self.plan = ((p.x, p.y), best, surface)
        self.publish_plan(best, surface, f'APPROACH_READY:surface_{surface.id}_side_{best.side}_depth_'
                                         f'{best.depth:.2f}_edge_{best.edge_distance:.2f}_offset_err_'
                                         f'{best.offset_error:.2f}_margin_{best.reach_margin:.2f}')

    def publish_plan(self, best, surface, label):
        stamp = self.surfaces.header.stamp
        self.stage_pub.publish(self.pose(best.stage, stamp))
        self.dock_pub.publish(self.pose(best.dock, stamp))
        self.edge_pub.publish(Float32(data=float(best.edge_distance)))
        self.marker_pub.publish(self.markers(best, surface, stamp))
        self.report(label)

    @staticmethod
    def pose(xyyaw, stamp):
        m = PoseStamped()
        m.header.frame_id, m.header.stamp = 'map', stamp
        m.pose.position.x, m.pose.position.y = xyyaw[0], xyyaw[1]
        m.pose.orientation.z, m.pose.orientation.w = math.sin(xyyaw[2]/2), math.cos(xyyaw[2]/2)
        return m

    def markers(self, best, surface, stamp):
        out = [Marker(action=Marker.DELETEALL)]

        def marker(mid, kind, rgb, scale):
            m = Marker(ns='approach', id=mid, type=kind, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
            return m

        z = surface.height + self.floor_z
        for mid, (x, y, yaw), rgb in ((0, best.stage, (0.2, 0.5, 1.0)), (1, best.dock, (0.1, 0.9, 0.1))):
            arrow = marker(mid, Marker.ARROW, rgb, (0.03, 0.06, 0.08))
            arrow.points = [Point(x=x, y=y, z=0.05), Point(x=x+0.35*math.cos(yaw), y=y+0.35*math.sin(yaw), z=0.05)]
            out.append(arrow)
        x, y, yaw = best.dock                          # robot outline at the docked pose
        outline = marker(2, Marker.LINE_STRIP, (0.1, 0.9, 0.1), (0.015, 0, 0))
        c, s = math.cos(yaw), math.sin(yaw)
        outline.points = [Point(x=x+c*u-s*v, y=y+s*u+c*v, z=0.02) for u, v in FOOTPRINT + FOOTPRINT[:1]]
        out.append(outline)
        reach = marker(3, Marker.LINE_STRIP, (1.0, 0.8, 0.0), (0.02, 0, 0))
        reach.points = [Point(x=best.edge_point[0], y=best.edge_point[1], z=z),
                        Point(x=self.target.point.x, y=self.target.point.y, z=z)]
        out.append(reach)
        text = marker(4, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0), (0, 0, 0.1))
        text.pose.position.x, text.pose.position.y, text.pose.position.z = x, y, 0.5
        text.text = (f'dock: target {best.target_in_base[0]:.2f} ahead / {-best.target_in_base[1]:.2f} right, '
                     f'edge {best.edge_distance:.2f} m')
        out.append(text)
        return MarkerArray(markers=out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ApproachNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
