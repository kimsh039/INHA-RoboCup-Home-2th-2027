"""One verified head target -> map position from Mid-360 points -> one fixed map goal -> Nav2.

The head camera gives the bbox (RGB); its range comes from the Mid-360: lidar clouds accumulated
in map over the last `accumulate_sec` are projected into the colour image and the points in the
bbox just above a support surface are taken (nav_geometry.lidar_target). The head depth stream
is not used. The raw cloud is used: the Nav2 self filter drops points alone in a 10 cm voxel,
which are exactly the few hits a small object gets at 2-3 m; the bbox never covers the robot.

No direct /cmd_vel control. Losing the RGB target after goal submission does not
cancel navigation: Nav2 owns the fixed map goal and obstacle-aware motion.
"""
from collections import deque
import math
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from robocup_detection_msgs.msg import HeadTarget, SupportSurfaceArray

from .nav_geometry import Intrinsics, lidar_target, standoff_goal, free_goal
from .support_surface_node import matrix
from .closed_approach import ClosedApproachRequest, request_closed_approach


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class DetectionNavGoalNode(Node):
    def __init__(self):
        super().__init__('detection_nav_goal_node')
        self.declare_parameter('auto_send', False)
        self.declare_parameter('standoff_distance', 1.0)
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('Gazebo demo requires use_sim_time=true')
        self.auto_send = self.get_parameter('auto_send').value
        self.standoff = self.get_parameter('standoff_distance').value
        if not math.isfinite(self.standoff) or self.standoff < .8:
            raise ValueError('standoff_distance must be >=0.8m')
        self.accumulate = self.declare_parameter('accumulate_sec', 1.0).value   # lidar points per estimate
        self.min_points = self.declare_parameter('min_lidar_points', 3).value
        self.color_info = self.grid = self.surfaces = None
        self.clouds, self.pending = deque(maxlen=50), deque(maxlen=10)
        # Confirmation: 3 measurements of the target class within confirm_sec that agree within
        # 0.15 m. Not tied to one track id: on small objects the detector's ROI re-verification
        # fails and it restarts the track about once a second.
        self.confirm_window = self.declare_parameter('confirm_sec', 5.0).value
        self.samples = deque(maxlen=3)            # (stamp s, x, y)
        self.sample_id = self.sample_class = ''
        self.frozen = False
        self.goal_handle = self.frozen_point = self.frozen_goal = None
        self.last_status = ''
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_pub = self.create_publisher(PoseStamped, '/detection/navigation_goal', 10)
        self.point_pub = self.create_publisher(PointStamped, '/detection/target_map_point', 10)
        self.status = self.create_publisher(String, '/detection/navigation_status', 10)
        self.handoff_pub = self.create_publisher(PointStamped, '/detection/closed_approach/target', 10)
        self.handoff_id_pub = self.create_publisher(String, '/detection/closed_approach/target_id', 10)
        self.create_subscription(HeadTarget, '/detection/head/target', self.on_target, 10)
        self.create_subscription(CameraInfo, '/head_camera/color/camera_info',
                                 lambda msg: setattr(self, 'color_info', msg), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.declare_parameter('cloud_topic', '/mid360/points').value,
                                 self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(SupportSurfaceArray, '/detection/support_surfaces',
                                 lambda msg: setattr(self, 'surfaces', msg), 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', lambda msg: setattr(self, 'grid', msg), map_qos)
        self.create_timer(.1, self.process)

    def report(self, label):
        self.status.publish(String(data=label))
        if label != self.last_status:
            self.get_logger().info(label)
            self.last_status = label

    def on_target(self, msg):
        if self.frozen:
            return
        if not msg.valid:
            self.pending.clear()
        elif msg.measured and not msg.reason:
            self.pending.append(msg)

    def on_cloud(self, msg):
        """Keep recent clouds in map, each transformed at its own stamp (the robot may move)."""
        try:
            transform = self.tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp),
                                                 timeout=Duration(seconds=0.05))
        except TransformException:
            return
        rotation, translation = matrix(transform.transform)
        raw = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)   # mixed field types
        xyz = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(float)
        self.clouds.append((seconds(msg.header.stamp), xyz @ rotation.T + translation))

    @staticmethod
    def intrinsic(info):
        return Intrinsics(info.width, info.height, info.k[0], info.k[4], info.k[2], info.k[5])

    def process(self):
        if self.frozen:
            return
        if not self.grid or self.grid.header.frame_id != 'map':
            return self.report('WAITING_SLAM_MAP')
        if not self.color_info or not self.clouds or not self.pending:
            return self.report('WAITING_VERIFIED_RGB_LIDAR')
        target = self.pending[0]
        now = self.get_clock().now().nanoseconds/1e9
        age = now-seconds(target.header.stamp)
        if age < 0 or age > 1.0:
            self.pending.popleft()
            return self.report('STALE_OBSERVATION')
        frame = target.header.frame_id
        if not frame or frame != self.color_info.header.frame_id:
            self.pending.popleft()
            return self.report('OPTICAL_FRAME_MISMATCH')
        if any(abs(v) > 1e-8 for v in self.color_info.d):
            self.pending.popleft()
            return self.report('DISTORTED_IMAGE_UNSUPPORTED')
        t = seconds(target.header.stamp)
        if self.clouds[-1][0] < t:
            return self.report('WAITING_LIDAR')        # use clouds up to the image, not older only
        try:
            stamp = Time.from_msg(target.header.stamp)
            cam = self.tf.lookup_transform(frame, 'map', stamp, timeout=Duration(seconds=0))
            robot = self.tf.lookup_transform('map', 'base_link', stamp, timeout=Duration(seconds=0))
        except TransformException:
            return self.report('WAITING_MAP_TF')
        self.pending.popleft()
        rotation, translation = matrix(cam.transform)
        cam_from_map = np.eye(4)
        cam_from_map[:3, :3], cam_from_map[:3, 3] = rotation, translation
        recent = [pts for s, pts in self.clouds if t-self.accumulate <= s <= t+.15]
        surfaces = [(np.array([(c.x, c.y) for c in s.corners]), s.z) for s in self.surfaces.surfaces] if self.surfaces else []
        found, count = lidar_target(np.concatenate(recent) if recent else np.empty((0, 3)),
                                    (target.x, target.y, target.width, target.height),
                                    self.intrinsic(self.color_info), cam_from_map, surfaces,
                                    minimum_points=self.min_points)
        if found is None:
            return self.report(f'TOO_FEW_LIDAR_POINTS:{count}')
        world_point = PointStamped()
        world_point.header.frame_id, world_point.header.stamp = 'map', target.header.stamp
        world_point.point.x, world_point.point.y, world_point.point.z = map(float, found)
        if self.sample_class != target.class_name:
            self.samples.clear()
            self.sample_class = target.class_name
        self.sample_id = target.target_id
        while self.samples and t-self.samples[0][0] > self.confirm_window:
            self.samples.popleft()
        self.samples.append((t, world_point.point.x, world_point.point.y))
        if len(self.samples) < 3:
            return self.report('CONFIRMING_MAP_POSITION')
        samples = np.asarray(self.samples)[:, 1:]
        median = np.median(samples, axis=0)
        if np.max(np.linalg.norm(samples-median, axis=1)) > .15:
            return self.report('UNSTABLE_MAP_POSITION')
        current_robot = robot.transform.translation
        goal_xy = standoff_goal(median, (current_robot.x, current_robot.y), self.standoff)
        world_point.point.x, world_point.point.y = float(median[0]), float(median[1])
        # Publish the confirmed target even if the standoff goal below is rejected: the approach
        # node plans from the support surface instead (a goal 1 m before a table object often
        # lands by a table leg).
        self.point_pub.publish(world_point)
        if goal_xy is None:
            self.frozen, self.frozen_point = True, world_point
            self.report('ALREADY_NEAR_TARGET')
            return self.closed_approach_handoff() if self.auto_send else None
        x, y, yaw = goal_xy
        if not free_goal(self.grid, x, y):
            return self.report('GOAL_OCCUPIED_OR_UNKNOWN:map_the_area_first')
        if self.auto_send and not self.client.server_is_ready():
            return self.report('WAITING_NAV2_ACTION_SERVER')
        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = 'map', self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = float(x), float(y)
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.goal_pub.publish(goal)
        self.frozen, self.frozen_point, self.frozen_goal = True, world_point, goal
        if not self.auto_send:
            return self.report('GOAL_PREVIEW_ONLY')
        request = NavigateToPose.Goal()
        request.pose = goal
        self.client.send_goal_async(request).add_done_callback(self.accepted)
        self.report('GOAL_SENT')

    def accepted(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                return self.report('NAV2_GOAL_REJECTED:restart_to_retry')
            self.goal_handle.get_result_async().add_done_callback(self.finished)
            self.report('NAVIGATING')
        except Exception as exc:
            self.report(f'NAV2_ERROR:{exc}')

    def finished(self, future):
        try:
            status = future.result().status
            self.goal_handle = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.report('ARRIVED')
                self.closed_approach_handoff()
            else:
                self.report(f'NAV2_FINISHED:{status}')
        except Exception as exc:
            self.report(f'NAV2_ERROR:{exc}')

    def closed_approach_handoff(self):
        # Reserved extension point. Frozen observations MUST be refreshed before
        # future closed approach / wrist manipulation; they are only coarse hints.
        request = ClosedApproachRequest(self.sample_id, self.frozen_point, self.frozen_goal)
        self.handoff_pub.publish(request.target_map_point)
        self.handoff_id_pub.publish(String(data=request.target_id))
        self.report(request_closed_approach(request))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DetectionNavGoalNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node.goal_handle is not None and rclpy.ok():
                future = node.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
