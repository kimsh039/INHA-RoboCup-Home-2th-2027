"""Shared Gazebo Transport helpers for the RoboCup simulation tools.

Topic names, joint limits and the drive/arm/gripper commands used by control.py,
robotctl.py and joystick.py. Import this module before any gz.* module is used
elsewhere: `use_partition()` must set GZ_PARTITION / GZ_IP before the first Node.
"""
import math
import os
import time

DEFAULT_PARTITION = 'robocup_motion'

# Topics created by gazebo/make_sim.py and the URDF sensors.
DRIVE_TOPIC = '/robocup/cmd_vel'
CAMERA_PREFIX = {'head': '/robocup/camera', 'wrist': '/robocup/wrist_camera'}

ARM_JOINTS = [f'piper_joint{i}' for i in range(1, 7)]
GRIPPER_JOINTS = ('piper_gripper_joint1', 'piper_gripper_joint2')
# Piper joint limits in rad, as in robot_description/robocup.urdf.
ARM_LIMITS = [(-2.6179938, 2.6179938), (0, 3.1415926), (-2.9670597, 0),
              (-1.7453292, 1.7453292), (-1.2217304, 1.2217304), (-2.0943951, 2.0943951)]
GRIPPER_MAX = 0.05            # one-finger travel, m

# Speed limits for the manual tools (well below the robot's 1.6 m/s).
MAX_LINEAR = 0.2              # m/s
MAX_ANGULAR = 0.5             # rad/s


def joint_topic(name):
    return f'/model/robocup/joint/{name}/0/cmd_pos'


def default_partition():
    return os.environ.get('GZ_PARTITION', DEFAULT_PARTITION)


def use_partition(partition):
    """Select the Gazebo partition and keep discovery on this PC (other sims on the LAN)."""
    os.environ['GZ_PARTITION'] = partition
    os.environ.setdefault('GZ_IP', '127.0.0.1')


def check_range(value, low, high, label):
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{label}: expected {low}..{high}, got {value}')


def check_drive(linear, angular):
    check_range(linear, -MAX_LINEAR, MAX_LINEAR, 'linear')
    check_range(angular, -MAX_ANGULAR, MAX_ANGULAR, 'angular')


def check_joint(number, radians):
    if not 1 <= number <= len(ARM_JOINTS):
        raise ValueError(f'joint number: expected 1..{len(ARM_JOINTS)}, got {number}')
    check_range(radians, *ARM_LIMITS[number - 1], f'joint {number} position')


def check_grip(meters):
    check_range(meters, 0, GRIPPER_MAX, 'finger travel')


class RobotCommands:
    """Publishers for the Tracer drive, Piper joints and gripper.

    wait_for_gazebo: seconds to wait for a Gazebo subscriber when a topic is first used;
    raises RuntimeError if none appears (0 = publish without waiting).
    """

    def __init__(self, partition, wait_for_gazebo=0.0):
        use_partition(partition)
        from gz.transport13 import Node
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.twist_pb2 import Twist
        self._Double, self._Twist = Double, Twist
        self.partition = partition
        self.wait_for_gazebo = wait_for_gazebo
        self.node = Node()
        self._publishers = {}

    def publisher(self, topic, kind):
        if topic not in self._publishers:
            pub = self.node.advertise(topic, kind)
            self._publishers[topic] = pub
            until = time.monotonic() + self.wait_for_gazebo
            while not pub.has_connections() and time.monotonic() < until:
                time.sleep(0.05)
            if self.wait_for_gazebo and not pub.has_connections():
                raise RuntimeError(f'No Gazebo subscriber on {topic}; '
                                   f'check partition {self.partition} and world controllers.')
        return self._publishers[topic]

    def advertise_all(self):
        """Advertise drive and every joint topic now. A message published right after its topic
        is first advertised can be lost before Gazebo connects, so long-running tools do this
        at start-up."""
        self.open_drive()
        for name in (*ARM_JOINTS, *GRIPPER_JOINTS):
            self.publisher(joint_topic(name), self._Double)

    def open_drive(self):
        """Advertise the drive topic now (waits for Gazebo if wait_for_gazebo is set)."""
        return self.publisher(DRIVE_TOPIC, self._Twist)

    def drive_connected(self):
        return self.open_drive().has_connections()

    def joint_connected(self, name):
        return self.publisher(joint_topic(name), self._Double).has_connections()

    def drive(self, linear, angular):
        msg = self._Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.publisher(DRIVE_TOPIC, self._Twist).publish(msg)

    def stop(self):
        self.drive(0.0, 0.0)

    def joint(self, name, value):
        msg = self._Double()
        msg.data = value
        self.publisher(joint_topic(name), self._Double).publish(msg)

    def arm_joint(self, number, radians):
        check_joint(number, radians)
        self.joint(ARM_JOINTS[number - 1], radians)

    def grip(self, meters):
        """Open both fingers symmetrically by `meters` each."""
        check_grip(meters)
        self.joint(GRIPPER_JOINTS[0], meters)
        self.joint(GRIPPER_JOINTS[1], -meters)

    def home(self):
        """Arm and gripper to zero and stop the base (the base pose is not reset)."""
        for name in ARM_JOINTS:
            self.joint(name, 0.0)
        for name in GRIPPER_JOINTS:
            self.joint(name, 0.0)
        self.stop()
