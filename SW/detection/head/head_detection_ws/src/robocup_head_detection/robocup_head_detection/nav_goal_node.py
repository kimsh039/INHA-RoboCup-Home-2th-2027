"""One verified RGB-D target -> one fixed map goal -> Nav2 -> reserved handoff.

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
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from robocup_detection_msgs.msg import HeadTarget

from .nav_geometry import Intrinsics, depth_array, target_range, standoff_goal, free_goal
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
        self.color_info = self.depth_info = self.grid = None
        self.depths, self.pending = deque(maxlen=30), deque(maxlen=10)
        self.samples = deque(maxlen=3)
        self.sample_id = ''
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
        self.create_subscription(Image, '/head_camera/depth/image_raw', self.depths.append, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/head_camera/color/camera_info',
                                 lambda msg: setattr(self, 'color_info', msg), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/head_camera/depth/camera_info',
                                 lambda msg: setattr(self, 'depth_info', msg), qos_profile_sensor_data)
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
            self.samples.clear()
            self.pending.clear()
        elif msg.measured and not msg.reason:
            self.pending.append(msg)

    @staticmethod
    def intrinsic(info):
        return Intrinsics(info.width, info.height, info.k[0], info.k[4], info.k[2], info.k[5])

    def process(self):
        if self.frozen:
            return
        if not self.grid or self.grid.header.frame_id != 'map':
            return self.report('WAITING_SLAM_MAP')
        if not self.color_info or not self.depth_info or not self.depths or not self.pending:
            return self.report('WAITING_VERIFIED_RGB_DEPTH')
        target = self.pending[0]
        now = self.get_clock().now().nanoseconds/1e9
        age = now-seconds(target.header.stamp)
        if age < 0 or age > 1.0:
            self.pending.popleft()
            self.samples.clear()
            return self.report('STALE_OBSERVATION')
        frames = [target.header.frame_id, self.color_info.header.frame_id, self.depth_info.header.frame_id]
        if not frames[0] or len(set(frames)) != 1:
            self.pending.popleft()
            return self.report('OPTICAL_FRAME_MISMATCH')
        if any(abs(v) > 1e-8 for info in (self.color_info, self.depth_info) for v in info.d):
            self.pending.popleft()
            return self.report('DISTORTED_IMAGE_UNSUPPORTED')
        depth = min(self.depths, key=lambda msg: abs(seconds(msg.header.stamp)-seconds(target.header.stamp)))
        if abs(seconds(depth.header.stamp)-seconds(target.header.stamp)) > .12 or depth.header.frame_id != frames[0]:
            return self.report('WAITING_SYNCHRONIZED_DEPTH')
        color = self.intrinsic(self.color_info)
        try:
            distance = target_range((target.x, target.y, target.width, target.height), depth_array(depth),
                                    color, self.intrinsic(self.depth_info))
        except ValueError:
            distance = None
        if distance is None:
            self.pending.popleft()
            return self.report('INVALID_TARGET_DEPTH')
        point = PointStamped(header=target.header)
        point.point.x = (target.x+target.width/2-color.cx)*distance/color.fx
        point.point.y = (target.y+target.height/2-color.cy)*distance/color.fy
        point.point.z = distance
        try:
            stamp = Time.from_msg(target.header.stamp)
            transform = self.tf.lookup_transform('map', frames[0], stamp, timeout=Duration(seconds=0))
            robot = self.tf.lookup_transform('map', 'base_link', stamp, timeout=Duration(seconds=0))
            world_point = do_transform_point(point, transform)
        except TransformException:
            return self.report('WAITING_MAP_TF')
        self.pending.popleft()
        if self.sample_id != target.target_id:
            self.samples.clear()
            self.sample_id = target.target_id
        self.samples.append((world_point.point.x, world_point.point.y))
        if len(self.samples) < 3:
            return self.report('CONFIRMING_MAP_POSITION')
        samples = np.asarray(self.samples)
        median = np.median(samples, axis=0)
        if np.max(np.linalg.norm(samples-median, axis=1)) > .15:
            return self.report('UNSTABLE_MAP_POSITION')
        current_robot = robot.transform.translation
        goal_xy = standoff_goal(median, (current_robot.x, current_robot.y), self.standoff)
        world_point.point.x, world_point.point.y = float(median[0]), float(median[1])
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
        self.point_pub.publish(world_point)
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
