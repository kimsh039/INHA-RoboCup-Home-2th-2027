"""Bounded, read-only handoff from the real control-center approach status.

No navigation commands, HTTP calls, inference, or BaseLease acknowledgements.
The web protocol has no task UUID, so only an observed new approach transition
may create one; a succeeded snapshot received at startup is never replayed.
"""
import copy
import math
import uuid

CLASSES = ('green_apple', 'peach', 'tomato', 'fanta_can', 'cup', 'banana')
ACTIVE_PHASES = {'waiting_nav', 'planning', 'navigating', 'settling',
                 'aligning', 'approaching', 'final_align', 'waiting_target',
                 'navigating_target', 'rerouting_table', 'planning_table',
                 'target_settling', 'point_aligning', 'point_approaching',
                 'recovering', 'observing_table'}
INACTIVE_PHASES = {'idle', 'searching', 'confirming', 'ready', 'canceled', 'error', 'at_target'}


def observed_target(value, now, max_age):
    label = value.get('label')
    if label not in CLASSES or value.get('classes') != list(CLASSES):
        raise ValueError('CONTROL_CENTER_CLASS_SCHEMA')
    target = value.get('target')
    if not isinstance(target, dict) or target.get('label') != label:
        raise ValueError('CONTROL_CENTER_TARGET_MISSING')
    point = target.get('point_odom')
    if (not isinstance(point, list) or len(point) != 3 or
            any(type(x) not in (int, float) or not math.isfinite(x) for x in point)):
        raise ValueError('CONTROL_CENTER_INVALID_POINT')
    stamp = target.get('capture_stamp')
    if (type(stamp) not in (int, float) or not math.isfinite(stamp) or
            stamp <= 0 or not -.1 <= now-stamp <= max_age):
        raise ValueError('CONTROL_CENTER_STALE_TARGET')
    lidar = target.get('lidar_stamp')
    oldest = target.get('lidar_oldest_stamp')
    if (type(lidar) not in (int, float) or type(oldest) not in (int, float) or
            not all(math.isfinite(x) for x in (lidar, oldest)) or
            not 0 < oldest <= lidar <= stamp+.1 or stamp-oldest > 2.0 or
            type(target.get('lidar_count')) is not int or target['lidar_count'] < 3 or
            target.get('source_topic') != '/mid360/points_obstacles' or
            target.get('point_method') != 'yolo_mid360_bbox_cluster'):
        raise ValueError('CONTROL_CENTER_UNVERIFIED_LIDAR')
    # point_odom is explicitly odom even when target.frame describes point_map.
    return dict(label=label, class_id=CLASSES.index(label), position=list(point),
                stamp=float(stamp), frame='odom')


class ApproachSession:
    def __init__(self, stream_timeout=1., target_age=120., stop_duration=.6):
        self.timeout, self.target_age, self.stop_duration = stream_timeout, target_age, stop_duration
        self.received = None
        self.armed = False
        self.task = None
        self.target = None
        self.succeeded_at = None
        self.sent = False
        self.reason = 'WAITING_FOR_NEW_APPROACH'

    def invalidate(self, reason):
        self.armed = False
        self.task = self.target = self.succeeded_at = None
        self.sent = False
        self.reason = reason

    def update(self, value, now):
        if self.received is not None and not 0 <= now-self.received <= self.timeout:
            self.invalidate('CONTROL_CENTER_STREAM_GAP')
        self.received = now
        if not isinstance(value, dict) or type(value.get('active')) is not bool:
            raise ValueError('CONTROL_CENTER_INVALID_STATUS')
        phase, active = value.get('phase'), value['active']
        if active:
            if phase not in ACTIVE_PHASES:
                raise ValueError('CONTROL_CENTER_INVALID_ACTIVE_PHASE')
            if phase == 'waiting_target' and value.get('target') is None:
                if self.task is not None:
                    self.invalidate('CONTROL_CENTER_TARGET_LOST')
                else:
                    self.reason = 'WAITING_FOR_MEASURED_TARGET'
                return
            target = observed_target(value, now, self.target_age)
            if self.task is None and self.armed:
                self.task = uuid.uuid4().hex
                self.target = target
                self.succeeded_at = None
                self.sent = False
                self.armed = False
            elif self.task is not None:
                if (self.succeeded_at is not None or target['label'] != self.target['label'] or
                        math.dist(target['position'], self.target['position']) > .25):
                    self.invalidate('CONTROL_CENTER_TARGET_CHANGED')
                    return
            self.reason = 'APPROACH_RUNNING' if self.task else 'WAITING_FOR_NEW_APPROACH'
        elif phase == 'succeeded':
            target = observed_target(value, now, self.target_age)
            if self.task and not self.sent:
                if (target['label'] != self.target['label'] or
                        math.dist(target['position'], self.target['position']) > .25):
                    self.invalidate('CONTROL_CENTER_TARGET_CHANGED')
                    return
                if self.succeeded_at is None:
                    # Preserve the measured point/time observed at approach start.
                    self.succeeded_at = now
                self.reason = 'WAITING_FOR_MEASURED_STOP'
        elif phase == 'at_target':
            # The web explicitly distinguishes object arrival from a confirmed
            # grasp staging pose. Neither obstacle-limited arrival nor Nav2
            # success is sufficient evidence to start the arm.
            self.invalidate('GRASP_STAGING_NOT_CONFIRMED')
            self.armed = True
        else:
            self.invalidate('WAITING_FOR_NEW_APPROACH')
            # A live nonterminal snapshot must precede an active transition.
            self.armed = phase in INACTIVE_PHASES

    def take(self, now, stopped_since):
        if self.received is None or not 0 <= now-self.received <= self.timeout:
            self.invalidate('CONTROL_CENTER_STREAM_GAP')
            return None
        if self.task is None or self.sent or self.succeeded_at is None:
            return None
        if not -.1 <= now-self.target['stamp'] <= self.target_age:
            self.invalidate('CONTROL_CENTER_STALE_TARGET')
            return None
        if stopped_since is None or now-max(stopped_since, self.succeeded_at) < self.stop_duration:
            return None
        self.sent = True
        self.reason = 'HANDED_OFF_EXTERNAL_BASE_LEASE_REQUIRED'
        return dict(task_id=self.task, target=copy.deepcopy(self.target), completed_at=self.succeeded_at)


def main():
    import json
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from robocup_real_manipulation_msgs.msg import Target, ApproachResult

    class Adapter(Node):
        def __init__(self):
            super().__init__('control_center_manipulation_input')
            from .integration import ProcessOwner
            self.owner = ProcessOwner(self.declare_parameter('owner_path',
                '~/.ros/robocup_real_manipulation/control_center_input.owner').value)
            self.session = ApproachSession()
            self.stop_since = self.odom_stamp = None
            status = self.declare_parameter('status_topic', '/detection/closed_approach/status').value
            odom = self.declare_parameter('odom_topic', '/odom_ekf').value
            self.create_subscription(String, status, self.input, 1)
            self.create_subscription(Odometry, odom, self.odometry, qos_profile_sensor_data)
            self.targets = self.create_publisher(Target, '/manipulation/target', 10)
            self.approaches = self.create_publisher(ApproachResult, '/manipulation/approach_result', 10)
            self.status = self.create_publisher(String, '/manipulation/control_center/status', 1)
            self.create_timer(.1, self.tick)
            self.create_timer(1., self.report)

        def now(self):
            return self.get_clock().now().nanoseconds/1e9

        def input(self, msg):
            try:
                if len(msg.data) > 65536:
                    raise ValueError('CONTROL_CENTER_STATUS_TOO_LARGE')
                self.session.update(json.loads(msg.data), self.now())
            except (ValueError, TypeError, KeyError) as exc:
                self.session.invalidate(str(exc))

        def odometry(self, msg):
            now = self.now()
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            previous = self.odom_stamp
            if (msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link' or
                    not 0 <= now-stamp <= .3 or (previous is not None and stamp <= previous)):
                self.stop_since = None
                return
            self.odom_stamp = stamp
            twist = msg.twist.twist
            values = [twist.linear.x, twist.linear.y, twist.linear.z,
                      twist.angular.x, twist.angular.y, twist.angular.z]
            if (not all(math.isfinite(x) for x in values) or
                    math.sqrt(sum(x*x for x in values[:3])) > .02 or
                    math.sqrt(sum(x*x for x in values[3:])) > .04):
                self.stop_since = None
            elif previous is None or stamp-previous > .3 or self.stop_since is None:
                self.stop_since = now

        def tick(self):
            now = self.now()
            if self.odom_stamp is None or not 0 <= now-self.odom_stamp <= .3:
                self.stop_since = None
            if not self.targets.get_subscription_count() or not self.approaches.get_subscription_count():
                return
            result = self.session.take(now, self.stop_since)
            if result is None:
                return
            item = result['target']
            target = Target(task_id=result['task_id'], instance_id=result['task_id'], class_id=item['class_id'])
            target.header.frame_id = item['frame']
            target.header.stamp = Time(nanoseconds=round(item['stamp']*1e9)).to_msg()
            target.coarse_position.x, target.coarse_position.y, target.coarse_position.z = map(float, item['position'])
            approach = ApproachResult(task_id=result['task_id'], outcome='SUCCESS', base_stopped=True,
                reason='CONTROL_CENTER_SUCCEEDED_AND_MEASURED_ODOMETRY_STOP;EXTERNAL_BASE_LEASE_REQUIRED')
            approach.header.stamp = Time(nanoseconds=round(result['completed_at']*1e9)).to_msg()
            self.targets.publish(target)
            self.approaches.publish(approach)

        def report(self):
            self.status.publish(String(data=json.dumps(dict(reason=self.session.reason,
                task_id=self.session.task, external_base_lease_required=True))))

    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
