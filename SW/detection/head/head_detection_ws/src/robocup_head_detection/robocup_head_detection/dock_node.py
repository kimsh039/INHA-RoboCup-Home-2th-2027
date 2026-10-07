"""Closed approach: Nav2 to the staging pose, then dock at the table edge with /cmd_vel.

States (/detection/dock/status): WAITING_APPROACH -> NAV_TO_STAGE -> MEASURING -> DOCKING ->
DOCKED, or ABORTED:<reason>. While the edge is visible to the Mid-360 every new cloud
re-measures it in base_link (dock_geometry.fit_edge) and moves the docking goal onto the
measured edge; inside the sensor's blind range the last goal is kept and the robot finishes
on odometry. G2 returns in the corridor ahead stop the robot.

Docking runs in the odom frame: the map pose is used once, to take over the approach plan.
Under the table the G2 scan changes so much that SLAM can jump; odometry does not.

When docked it publishes, in base_link and latched, the target (/detection/dock/target) and
the support surface outline at its top height (/detection/dock/surface) for the arm.
"""
import math
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, Float32, String
from tf2_ros import Buffer, TransformListener, TransformException
from robocup_detection_msgs.msg import SupportSurfaceArray

from .dock_geometry import DockConfig, EdgeConfig, blind_limit, dock_command, fit_edge
from .support_surface_node import matrix


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def xyyaw(pose_stamped):
    p = pose_stamped.pose
    return p.position.x, p.position.y, yaw_of(p.orientation)


class DockNode(Node):
    def __init__(self):
        super().__init__('dock_node')
        get = lambda name, default: self.declare_parameter(name, default).value
        self.d_dock_default = get('d_dock', 0.08)         # edge distance if the planner publishes none
        self.d_dock = self.planned_edge = None
        self.floor_z = get('floor_z', -0.1425)          # base_link sits 0.1425 m above the floor at z=0
        self.auto_start = get('auto_start', True)
        self.measure_time = get('measure_sec', 1.0)
        self.timeout = get('timeout_sec', 120.0)
        self.corridor = get('stop_corridor', [0.36, 0.50, 0.30])   # x from, x to, |y| in base_link
        self.edge_cfg, self.cfg = EdgeConfig(), DockConfig()
        self.state, self.reason = 'WAITING_APPROACH', ''
        self.stage = self.dock = self.surfaces = None
        self.goal = None                 # current docking goal (x, y, yaw) in map
        self.edges = []                  # (line point xy, normal yaw) measured in odom
        self.frozen, self.rejected = False, 0
        self.freeze_margin = get('freeze_margin', 0.10)   # freeze this far before the blind range, m
        self.started = None
        self.cloud = None
        self.blocked = False
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/detection/dock/status', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(PointStamped, '/detection/dock/target', latched)
        self.surface_pub = self.create_publisher(PolygonStamped, '/detection/dock/surface', latched)
        self.target_map = None
        self.create_subscription(PointStamped, '/detection/target_map_point', lambda m: setattr(self, 'target_map', m), 10)
        self.create_subscription(PoseStamped, '/detection/approach/stage_pose', lambda m: setattr(self, 'stage', m), 10)
        self.create_subscription(PoseStamped, '/detection/approach/dock_pose', lambda m: setattr(self, 'dock', m), 10)
        self.create_subscription(Float32, '/detection/approach/dock_edge_distance',
                                 lambda m: setattr(self, 'planned_edge', m.data), 10)
        self.create_subscription(SupportSurfaceArray, '/detection/support_surfaces',
                                 lambda m: setattr(self, 'surfaces', m), 10)
        self.create_subscription(PointCloud2, '/mid360/points_filtered', lambda m: setattr(self, 'cloud', m),
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Empty, '/detection/dock/start', lambda _: self.start(), 10)
        self.create_timer(0.1, self.step)

    # ----- helpers
    def report(self, state, reason=''):
        label = state + (f':{reason}' if reason else '')
        if label != (self.state + (f':{self.reason}' if self.reason else '')):
            self.get_logger().info(label)
        self.state, self.reason = state, reason
        self.status.publish(String(data=label))

    def stop(self):
        self.cmd.publish(Twist())

    def abort(self, reason):
        self.stop()
        self.report('ABORTED', reason)

    def robot_pose(self, stamp=None):
        t = self.tf.lookup_transform('odom', 'base_link', stamp or Time(), timeout=Duration(seconds=0.05))
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def to_odom(self, xyyaw):
        """Map pose -> odom pose with the current map->odom correction."""
        t = self.tf.lookup_transform('odom', 'map', Time(), timeout=Duration(seconds=0.2))
        R, tr = matrix(t.transform)
        x, y, _ = R @ np.array([xyyaw[0], xyyaw[1], 0.0]) + tr
        return float(x), float(y), xyyaw[2] + yaw_of(t.transform.rotation)

    def support_surface(self):
        """The surface whose outline is closest ahead of the docking pose (map)."""
        gx, gy, gyaw = xyyaw(self.dock)
        probe = np.array([gx + 0.3*math.cos(gyaw), gy + 0.3*math.sin(gyaw)])
        return min(self.surfaces.surfaces, default=None,
                   key=lambda s: np.linalg.norm(np.mean([[c.x, c.y] for c in s.corners], axis=0) - probe))

    def snapshot(self, surface):
        """Target and surface outline from map to odom, once, before the map can jump."""
        t = self.tf.lookup_transform('odom', 'map', Time(), timeout=Duration(seconds=0.2))
        R, tr = matrix(t.transform)
        self.surface_odom = [R @ np.array([c.x, c.y, surface.z]) + tr for c in surface.corners]
        self.target_odom = None
        if self.target_map is not None:
            p = self.target_map.point
            self.target_odom = R @ np.array([p.x, p.y, p.z]) + tr

    def publish_docked_context(self, pose):
        """Target and surface outline in base_link at the docked pose (odom -> base_link)."""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        to_base = lambda p: (c*(p[0]-x) + s*(p[1]-y), -s*(p[0]-x) + c*(p[1]-y), p[2])
        stamp = self.get_clock().now().to_msg()
        if self.target_odom is not None:
            m = PointStamped()
            m.header.frame_id, m.header.stamp = 'base_link', stamp
            m.point.x, m.point.y, m.point.z = map(float, to_base(self.target_odom))
            self.target_pub.publish(m)
        poly = PolygonStamped()
        poly.header.frame_id, poly.header.stamp = 'base_link', stamp
        poly.polygon.points = [Point32(x=float(a), y=float(b), z=float(z)) for a, b, z in map(to_base, self.surface_odom)]
        self.surface_pub.publish(poly)

    def on_scan(self, msg):
        try:
            T = self.tf.lookup_transform('base_link', msg.header.frame_id, Time())
        except TransformException:
            return
        R, t = matrix(T.transform)
        a = msg.angle_min + np.arange(len(msg.ranges))*msg.angle_increment
        r = np.asarray(msg.ranges)
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        pts = np.c_[r[ok]*np.cos(a[ok]), r[ok]*np.sin(a[ok]), np.zeros(ok.sum())] @ R.T + t
        x0, x1, half = self.corridor
        self.blocked = bool(np.any((pts[:, 0] > x0) & (pts[:, 0] < x1) & (np.abs(pts[:, 1]) < half)))

    # ----- state machine
    def start(self):
        if self.stage is None or self.dock is None:
            return self.report('WAITING_APPROACH')
        self.goal, self.edges, self.started = None, [], self.get_clock().now()
        self.frozen, self.rejected = False, 0
        self.report('NAV_TO_STAGE')
        request = NavigateToPose.Goal(pose=self.stage)
        if not self.nav.wait_for_server(timeout_sec=2.0):
            return self.abort('NAV2_UNAVAILABLE')
        self.nav.send_goal_async(request).add_done_callback(self.nav_accepted)

    def nav_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            return self.abort('STAGE_GOAL_REJECTED')
        handle.get_result_async().add_done_callback(self.nav_done)

    def nav_done(self, future):
        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            return self.abort(f'STAGE_NAV_FAILED_{future.result().status}')
        surface = self.support_surface()
        if surface is None:
            return self.abort('NO_SUPPORT_SURFACE')
        self.height = surface.height          # fixed for this docking; the map may jump under the table
        try:
            self.snapshot(surface)
        except TransformException:
            return self.abort('NO_MAP_TO_ODOM')
        # Planned base_link-to-edge distance (puts the target at the manipulation offset).
        self.d_dock = self.planned_edge if self.planned_edge is not None else self.d_dock_default
        try:
            self.dock_odom = self.to_odom(xyyaw(self.dock))
        except TransformException:
            return self.abort('NO_MAP_TO_ODOM')
        self.goal = self.dock_odom
        self.measure_until = self.get_clock().now() + Duration(seconds=self.measure_time)
        self.report('MEASURING')

    def measure_edge(self):
        """Fit the edge in the newest cloud and move the docking goal onto it.

        The edge expected from the current goal gates each fit: a fit far from it is some
        other boundary (e.g. the shadow of an object on the table). Once the expected edge
        nears the Mid-360's blind range the goal is frozen. False if not updated."""
        cloud, height = self.cloud, self.height
        if self.frozen or cloud is None or height is None:
            return False
        try:
            x, y, yaw = self.robot_pose(Time.from_msg(cloud.header.stamp))
        except TransformException:
            return False
        gx, gy, gyaw = self.goal
        expected = (gx + self.d_dock*math.cos(gyaw) - x)*math.cos(gyaw) + (gy + self.d_dock*math.sin(gyaw) - y)*math.sin(gyaw)
        if expected < blind_limit(0.0, height, self.edge_cfg) + self.freeze_margin:
            self.frozen = True
            return False
        pts = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True).astype(float)
        edge = fit_edge(pts, height, self.floor_z, self.edge_cfg)
        if edge is None:
            return False
        tol_d, tol_a = (0.08, math.radians(6)) if self.state == 'MEASURING' else (0.05, math.radians(5))
        angle_error = math.atan2(math.sin(yaw + edge.angle - gyaw), math.cos(yaw + edge.angle - gyaw))
        if abs(edge.distance - expected) > tol_d or abs(angle_error) > tol_a:
            self.rejected += 1
            return False
        normal = yaw + edge.angle
        foot = (x + edge.distance*math.cos(normal), y + edge.distance*math.sin(normal))
        self.edges.append((foot, normal))
        # Average the edge line over the measurements: normal (circular mean) and line offset.
        n = math.atan2(sum(math.sin(e[1]) for e in self.edges), sum(math.cos(e[1]) for e in self.edges))
        u = np.array([math.cos(n), math.sin(n)])
        offset = float(np.mean([np.dot(e[0], u) for e in self.edges]))
        # Keep the approach's lateral position (the target's line), move it onto the measured edge.
        gx, gy, _ = self.dock_odom
        along = np.array([gx, gy]) - np.dot([gx, gy], u)*u           # component along the edge
        goal = along + (offset - self.d_dock)*u
        self.goal = (float(goal[0]), float(goal[1]), n)
        return True

    def step(self):
        if self.state == 'WAITING_APPROACH':
            if self.auto_start and self.stage is not None and self.dock is not None and self.surfaces is not None:
                self.start()
            return
        if self.state in ('DOCKED', 'ABORTED', 'NAV_TO_STAGE'):
            return
        if (self.get_clock().now() - self.started).nanoseconds > self.timeout*1e9:
            return self.abort('TIMEOUT')
        if self.state == 'MEASURING':
            self.stop()
            self.measure_edge()
            if self.get_clock().now() < self.measure_until:
                return
            if len(self.edges) < 3:
                return self.abort('EDGE_NOT_MEASURED')
            self.edges = self.edges[-10:]
            self.report('DOCKING')
            return
        # DOCKING
        visible = self.measure_edge()
        try:
            pose = self.robot_pose()
        except TransformException:
            self.stop()
            return
        v, w, done = dock_command(pose, self.goal, self.cfg)
        if done:
            self.stop()
            self.publish_docked_context(pose)
            gx, gy, gyaw = self.goal
            u = (math.cos(gyaw), math.sin(gyaw))
            along = (gx-pose[0])*u[0] + (gy-pose[1])*u[1]
            lateral = -(pose[0]-gx)*u[1] + (pose[1]-gy)*u[0]
            return self.report('DOCKED', f'along_{along*1000:.0f}mm_lateral_{lateral*1000:.0f}mm_'
                                         f'heading_{math.degrees(pose[2]-gyaw):.1f}deg_edges_{len(self.edges)}'
                                         f'_rejected_{self.rejected}')
        if v > 0 and self.blocked:
            self.stop()
            return self.report('DOCKING', 'PATH_BLOCKED')
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd.publish(cmd)
        self.report('DOCKING', 'EDGE_TRACKED' if visible else ('GOAL_FROZEN' if self.frozen else 'EDGE_NOT_SEEN'))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DockNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
