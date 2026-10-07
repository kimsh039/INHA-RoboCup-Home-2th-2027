"""After docking, point the wrist camera at the target: a look-at pose from IK, driven along a
straight joint-space path checked against the table.

Inputs (latched, base_link): /detection/dock/target, /detection/dock/surface; /robot_description,
/joint_states. Output: /detection/wrist/observe_status (PLANNING -> MOVING -> OBSERVING, or
FAILED:<reason>) and the commanded pose. Joints are driven by SimArm (Gazebo joint position
controllers); a real-arm driver replaces it with the same interface.
"""
import math
import os
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PointStamped, PolygonStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from .arm_kinematics import Chain, ObservationConfig, Table, joint_path, observation_pose


class SimArm:
    """Gazebo joint position commands (/model/robocup/joint/<name>/0/cmd_pos)."""

    def __init__(self, names, partition):
        os.environ.setdefault('GZ_PARTITION', partition)
        from gz.transport13 import Node as GzNode
        from gz.msgs10.double_pb2 import Double
        self.Double, self.node = Double, GzNode()
        self.pubs = {n: self.node.advertise(f'/model/robocup/joint/{n}/0/cmd_pos', Double) for n in names}

    def command(self, names, q):
        for n, v in zip(names, q):
            msg = self.Double()
            msg.data = float(v)
            self.pubs[n].publish(msg)


class WristObserveNode(Node):
    def __init__(self):
        super().__init__('wrist_observe_node')
        get = lambda name, default: self.declare_parameter(name, default).value
        self.tolerance = get('joint_tolerance', 0.02)
        self.arm = SimArm([f'piper_joint{i}' for i in range(1, 7)], get('gz_partition', 'robocup_motion'))
        self.cfg = ObservationConfig()
        self.chain = self.target = self.surface = self.joints = None
        self.path, self.goal, self.state = None, None, 'WAITING'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/detection/wrist/observe_status', 10)
        self.goal_pub = self.create_publisher(Float64MultiArray, '/detection/wrist/observe_joints', latched)
        self.create_subscription(String, '/robot_description', self.on_description, latched)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(PointStamped, '/detection/dock/target', self.on_target, latched)
        self.create_subscription(PolygonStamped, '/detection/dock/surface', lambda m: setattr(self, 'surface', m), latched)
        self.create_timer(0.1, self.step)

    def report(self, state):
        if state != self.state:
            self.get_logger().info(state)
        self.state = state

    def on_description(self, msg):
        self.chain = Chain(msg.data, 'wrist_camera_optical_frame')

    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if self.chain and all(n in pos for n in self.chain.names):
            self.joints = np.array([pos[n] for n in self.chain.names])

    def on_target(self, msg):
        self.target = msg
        if self.state not in ('WAITING', 'OBSERVING') and not self.state.startswith('FAILED'):
            return
        self.state = 'PLAN_REQUESTED'

    def plan(self):
        p = self.target.point
        top = self.surface.polygon.points
        table = Table(tuple((c.x, c.y) for c in top), float(np.mean([c.z for c in top])))
        q, info = observation_pose(self.chain, (p.x, p.y, p.z), table, self.joints, self.cfg)
        if q is None:
            return self.report(f'FAILED:{info}')
        path = joint_path(self.chain, self.joints, q, table)
        if path is None:
            return self.report('FAILED:PATH_TOO_LOW_OVER_TABLE')
        self.goal, self.path = q, path
        self.goal_pub.publish(Float64MultiArray(data=[float(v) for v in q]))
        self.get_logger().info(f'observation: elevation {math.degrees(info["elevation"]):.0f} deg, '
                               f'distance {info["distance"]:.2f} m, clearance {info["clearance"]:.2f} m, '
                               f'q {np.round(q, 3).tolist()}')
        self.report('MOVING')

    def step(self):
        self.status.publish(String(data=self.state))      # every tick, so late subscribers see it
        if self.state == 'PLAN_REQUESTED':
            if self.chain is None or self.joints is None or self.surface is None:
                return self.report('PLAN_REQUESTED')       # waiting for inputs
            return self.plan()
        if self.state == 'MOVING':
            if self.path:
                self.arm.command(self.chain.names, self.path.pop(0))   # 0.05 rad per 0.1 s
                return
            self.arm.command(self.chain.names, self.goal)
            if np.max(np.abs(self.joints - self.goal)) < self.tolerance:
                self.report('OBSERVING')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WristObserveNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
