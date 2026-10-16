"""Read model-space arm feedback; never rename uncalibrated driver angles.

Matches robocup_real_manipulation.config.robot.yaml joint_names. Physical
piper_joint_bridge owns driver->CAD conversion, including its q1 offset.
"""
import math

MODEL_JOINTS = tuple(f'piper_joint{i}' for i in range(1, 7))
DEFAULT_MODEL_TOPIC = '/robot/joint_states'


class JointSnapshot:
    def __init__(self, max_age=.6):
        if not math.isfinite(max_age) or not 0 < max_age <= 2.:
            raise ValueError('invalid_model_joint_max_age')
        self.max_age = max_age
        self.stamp = None
        self.values = {}

    def receive(self, names, positions, stamp, now):
        if (len(names) != len(positions) or len(set(names)) != len(names)
                or not set(MODEL_JOINTS).issubset(names)
                or not math.isfinite(stamp) or stamp <= 0
                or not -.1 <= now-stamp <= self.max_age):
            return False
        values = dict(zip(names, positions))
        if not all(math.isfinite(values[n]) for n in MODEL_JOINTS):
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False
        self.stamp = stamp
        self.values = {n: float(values[n]) for n in MODEL_JOINTS}
        return True

    def positions(self, now):
        if self.stamp is None or not -.1 <= now-self.stamp <= self.max_age:
            return {}
        return dict(self.values)


class ModelJointFeedback:
    def __init__(self, node, settings):
        from sensor_msgs.msg import JointState
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.node = node
        self.topic = settings.get('model_joint_topic', DEFAULT_MODEL_TOPIC)
        if not isinstance(self.topic, str) or not self.topic.startswith('/'):
            raise ValueError('invalid_model_joint_topic')
        self.cache = JointSnapshot(float(settings.get('model_joint_max_age', .6)))
        self.subscription = node.create_subscription(JointState, self.topic, self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def now(self):
        return self.node.get_clock().now().nanoseconds/1e9

    def receive(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        self.cache.receive(msg.name, msg.position, stamp, self.now())

    def positions(self):
        return self.cache.positions(self.now())
