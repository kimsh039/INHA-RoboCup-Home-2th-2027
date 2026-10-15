"""Safety regressions for mixed-frame health and delayed input callbacks.

No ROS node is initialized and no actuator topic is published.
"""
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collision_guard import CollisionGuard, ComputationExpired


class CollisionInputTiming(unittest.TestCase):
    def receiver(self):
        node = CollisionGuard.__new__(CollisionGuard)
        node.scene, node.scene_sources, node.scene_received, node.scene_stamp = None, None, None, None
        node.cloud_health, node.health_received = {}, None
        node.pending_scenes, node.pending_health = {}, {}
        node.scene_generation = ('boot', 1)
        return node

    def health(self, stamp, generation=1):
        return String(data=json.dumps(dict(ready=True, reference_stamp_ns=stamp,
                                           generation=generation, session_id='boot')))

    def test_health_from_another_scan_does_not_approve_scene(self):
        node = self.receiver()
        node.pending_scenes[100] = ('old-tree', None, time.monotonic())
        node.on_cloud_health(self.health(101))
        self.assertIsNone(node.scene_received)
        self.assertIsNone(node.health_received)

    def test_matching_observation_pair_is_accepted(self):
        node = self.receiver()
        node.pending_scenes[100] = ('tree', None, time.monotonic())
        node.on_cloud_health(self.health(100))
        self.assertEqual(node.scene, 'tree')
        self.assertEqual(node.scene_stamp, 100)
        self.assertTrue(node.cloud_health['ready'])

    def test_reset_cannot_reuse_old_scene_with_same_timestamp(self):
        node = self.receiver()
        node.pending_scenes[100] = ('old-generation', None, time.monotonic())
        node.on_cloud_health(self.health(100, generation=2))
        self.assertIsNone(node.scene_received)
        self.assertFalse(node.pending_scenes)

    def test_joint_velocity_uses_source_interval(self):
        node = self.receiver()
        node.model = SimpleNamespace(required_joints={'piper_joint1'})
        node.cfg = dict(arm_motion_limit=.08)
        node.joints = {'piper_joint1': 0.}
        node.joint_stamp, node.joint_received, node.arm_moved = 1_000_000_000, time.monotonic(), 0.
        message = JointState(name=['piper_joint1'], position=[.001])
        message.header.stamp = Time(sec=1, nanosec=100_000_000)
        node.on_joints(message)
        self.assertEqual(node.arm_moved, 0.)
        message.header.stamp = Time(sec=1, nanosec=50_000_000)
        message.position = [.5]
        node.on_joints(message)
        self.assertEqual(node.joints['piper_joint1'], .001)

    def controller(self):
        node = self.receiver()
        node.cfg = dict(require_arm_feedback=False, computation_budget=.04, clearance=.1,
                        uncertainty_margin=.05, safety_voxel=.04, command_timeout=.25, base_frame='base_link')
        node.model = SimpleNamespace(fixed=[])
        node.pose, node.joints = np.zeros(3), {}
        node.command, node.command_received, node.command_error = Twist(), time.monotonic(), ''
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123)))
        node.sent = []
        node.pub = SimpleNamespace(publish=node.sent.append)
        node.distance = lambda *args: (float('inf'), None, None)
        return node

    def test_scene_expiring_during_computation_stops_approval(self):
        node = self.controller()
        reasons = iter(('', 'stale_mid360'))
        node.healthy = lambda: next(reasons)
        node.control()
        self.assertFalse(node.result['ready'])
        self.assertEqual(node.result['reason'], 'stale_mid360')
        self.assertEqual(node.result['output'], [0., 0.])

    def test_computation_timeout_is_not_ready(self):
        node = self.controller()
        node.healthy = lambda: ''
        def overrun(*args):
            raise ComputationExpired()
        node.distance = overrun
        node.control()
        self.assertFalse(node.result['ready'])
        self.assertEqual(node.result['reason'], 'computation_timeout')
        self.assertEqual(node.result['output'], [0., 0.])

    def test_last_geometry_call_cannot_overrun_approval_budget(self):
        node = self.controller()
        clock = [100.]
        node.command_received = clock[0]
        node.healthy = lambda: ''
        def slow_distance(*args):
            clock[0] += .05
            return float('inf'), None, None
        node.distance = slow_distance
        with patch('collision_guard.time.monotonic', side_effect=lambda: clock[0]):
            node.control()
        self.assertFalse(node.result['ready'])
        self.assertEqual(node.result['reason'], 'computation_timeout')
        self.assertEqual(node.result['output'], [0., 0.])


if __name__ == '__main__':
    unittest.main()
