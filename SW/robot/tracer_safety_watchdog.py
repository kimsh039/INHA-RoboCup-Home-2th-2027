#!/usr/bin/env python3
"""Independent deadman between 3D computation and the existing Tracer driver."""
import json
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String


class SafetyWatchdog(Node):
    def __init__(self):
        super().__init__('tracer_safety_watchdog')
        self.input = self.declare_parameter('approved_topic', '/collision/cmd_vel_approved').value
        self.output = self.declare_parameter('protected_topic', '/tracer/cmd_vel_safe').value
        self.base = self.declare_parameter('base_frame', 'base_link').value
        self.timeout = self.declare_parameter('approval_timeout', 0.15).value
        if self.input == self.output or not 0 < self.timeout <= 0.25:
            raise ValueError('Invalid actuator watchdog configuration')
        self.velocity = Twist()
        self.received = None
        self.stamp = None
        self.valid = False
        self.owner_valid = False
        self.pub = self.create_publisher(Twist, self.output, 1)
        self.status_pub = self.create_publisher(String, '/collision/actuator_status', 10)
        self.create_subscription(TwistStamped, self.input, self.receive, 1)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.025, self.forward, clock=self.clock)
        self.create_timer(0.2, self.status, clock=self.clock)
        from manipulation_base_lock import BaseLockAdapter
        self.manipulation_lock = BaseLockAdapter(self)
        from dock_motion_gate import DockMotionGate
        self.dock_gate = DockMotionGate(self)

    def receive(self, msg):
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if self.stamp is not None and stamp < self.stamp:
            self.stamp, self.received, self.valid = None, None, False
            return
        v = msg.twist
        valid = (msg.header.frame_id == self.base and stamp > 0 and
                 all(math.isfinite(x) for x in (v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z)) and
                 all(abs(x) <= 1e-6 for x in (v.linear.y, v.linear.z, v.angular.x, v.angular.y)) and
                 -0.15 <= v.linear.x <= 0.25 and abs(v.angular.z) <= 0.6 and
                 (self.stamp is None or stamp > self.stamp))
        self.valid = valid
        if valid:
            self.received, self.stamp, self.velocity = time.monotonic(), stamp, v

    def fresh(self):
        return (self.valid and self.owner_valid and self.received is not None and
                time.monotonic()-self.received <= self.timeout and
                -0.05 <= (self.get_clock().now().nanoseconds-self.stamp)/1e9 <= self.timeout)

    def forward(self):
        command = self.velocity if (self.fresh() and not self.manipulation_lock.task
            and self.dock_gate.allows(self.velocity, self.stamp)) else Twist()
        self.pub.publish(command)
        self.manipulation_lock.tick(abs(command.linear.x) <= 1e-6 and abs(command.angular.z) <= 1e-6)

    def status(self):
        try:
            sources = self.get_publishers_info_by_topic(self.input)
            outputs = self.get_publishers_info_by_topic(self.output)
            self.owner_valid = (len(sources) == 1 and sources[0].node_name == 'collision_guard' and
                                len(outputs) == 1 and outputs[0].node_name == self.get_name())
        except Exception:
            self.owner_valid = False
        self.status_pub.publish(String(data=json.dumps(dict(ready=self.fresh(),
                                    reason='live' if self.fresh() else 'missing_fresh_approval'))))
        self.dock_gate.publish()


def main():
    rclpy.init()
    node = SafetyWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
