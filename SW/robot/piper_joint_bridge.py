#!/usr/bin/env python3
"""Map measured PiPER feedback into the integrated CAD arm joint convention."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Bridge(Node):
    def __init__(self):
        super().__init__('physical_piper_joint_bridge')
        self.pub = self.create_publisher(JointState, '/robot/joint_states', 20)
        self.create_subscription(JointState, '/piper/joint_states_feedback', self.receive, 20)

    def receive(self, msg):
        joints = dict(zip(msg.name, msg.position))
        if not all(f'joint{i}' in joints for i in range(1, 7)):
            return
        out = JointState()
        out.header = msg.header
        out.name = [f'piper_joint{i}' for i in range(1, 7)]
        out.position = [joints[f'joint{i}'] for i in range(1, 7)]
        out.position[0] -= 1.6
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
