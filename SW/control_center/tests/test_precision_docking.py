"""Offline docking regressions; no ROS node or velocity publisher is created."""
from collections import deque
from concurrent.futures import Future
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from grasp_approach import GraspPlanner
from collision_geometry import Box
from detection_approach import DetectionApproach
from front_retreat import FrontRetreat


class PrecisionDockingTests(unittest.TestCase):
    def setUp(self):
        transform = np.eye(4)
        transform[:3, 3] = [0., 0., .67]
        self.box = Box('body', transform, [.2, .15, .06])
        model = SimpleNamespace(required_joints=set(), boxes=lambda _: [self.box])
        with patch('grasp_approach.RobotGeometry', return_value=model):
            self.planner = GraspPlanner({})
        self.target = [1.15, .2, .74]
        self.surface = dict(z=.7, corners=[[1., -1.], [2., -1.], [2., 1.], [1., 1.]],
                            edge_observed=[0., 0., 0., 1.], stamp=10.)
        self.plan = self.planner.plans(self.target, self.surface, [0., 0., 0.], {})[0]

    def test_normal_entry_places_object_at_arm_lateral_offset(self):
        dock = self.plan['dock']
        self.assertAlmostEqual(dock[2], 0.)
        self.assertAlmostEqual(self.target[1]-dock[1], -.078)
        self.assertLess(self.plan['stage'][0], dock[0])
        self.assertAlmostEqual(self.plan['stage'][1], dock[1])

    def test_ten_centimetres_or_lateral_error_cannot_succeed(self):
        x, y, yaw = self.plan['dock']
        for pose, gap in (((x-.10, y, yaw), .01), ((x, y+.04, yaw), .01),
                          ((x, y, yaw+.05), .01), ((x, y, yaw), .269),
                          ((x, y, yaw), -.01)):
            self.assertFalse(self.planner.docked(pose, self.plan, gap))
        self.assertTrue(self.planner.docked((x-.01, y+.01, yaw+.02), self.plan, .01))

    def test_lateral_steering_keeps_forward_motion(self):
        x, y, yaw = self.plan['dock']
        linear, angular, done = self.planner.command((x-.1, y+.04, yaw), self.plan)
        self.assertGreater(linear, 0.)
        self.assertLess(angular, 0.)
        self.assertFalse(done)

    def test_apron_band_includes_geometry_below_top(self):
        transform = np.eye(4)
        transform[:3, 3] = [.3, 0., .61]
        lower_body = Box('apron_intersection', transform, [.1, .1, .01])
        section = self.planner.section([self.box, lower_body], .7)
        self.assertAlmostEqual(section[:, 0].max(), .4)

    def test_fresh_edge_updates_selected_face_and_blind_zone_freezes(self):
        xs, ys = np.meshgrid(np.arange(1.02, 1.42, .01), np.arange(-.6, .61, .02))
        points = np.c_[xs.ravel(), ys.ravel(), np.full(xs.size, .7)]
        updated, state = self.planner.track_edge(self.plan, self.target, (.4, .278, 0.),
                                                0., points, 11., {})
        self.assertEqual(state, 'tracked')
        self.assertEqual(updated['side'], self.plan['side'])
        self.assertGreater(updated['dock'][0], self.plan['dock'][0])
        self.assertLess(updated['dock'][0]-self.plan['dock'][0], .02)
        updated, state = self.planner.track_edge(self.plan, self.target, (.9, .278, 0.),
                                                0., points, 12., {})
        self.assertIsNone(updated)
        self.assertEqual(state, 'blind')

    def test_recovery_commands_only_straight_reverse(self):
        approach = DetectionApproach.__new__(DetectionApproach)
        approach.command = Mock()
        approach.reset_edge_tracking = Mock()
        approach.report = Mock()
        approach.assess_reverse = Mock()
        approach.nav_busy = False
        approach.recovery_attempts = 0
        approach.begin_recovery(0., 'blocked')
        self.assertEqual(approach.recovery['reverse_distance'], .15)
        approach.advance_recovery(.5, (0., 0., 0.), dict(linear=0., angular=0.))
        approach.advance_recovery(1., (-.025, 0., 0.), dict(linear=-.05, angular=0.))
        for call in approach.command.call_args_list:
            if call.args:
                self.assertLess(call.args[0], 0.)
                self.assertEqual(call.args[1], 0.)

    def test_front_retreat_does_not_take_over_precision_docking(self):
        retreat = FrontRetreat.__new__(FrontRetreat)
        retreat.node = SimpleNamespace(nav_status={})
        retreat.approach = SimpleNamespace(active=True, phase='approaching')
        retreat.blocked_since = 10.
        retreat.consider(11., dict(ready=True, state='blocked'), True)
        self.assertIsNone(retreat.blocked_since)

    def test_staging_alignment_can_turn_before_front_edge_is_visible(self):
        approach = DetectionApproach.__new__(DetectionApproach)
        approach.active = True; approach.deadline = 100.; approach.phase = 'aligning'
        approach.plan = self.plan; approach.planner = self.planner
        approach.anchor = dict(point_odom=self.target)
        approach.np = np; approach.last_pose = None; approach.nav_busy = False
        approach.progress_at = 10.; approach.progress_best = math.inf
        approach.arm_feedback = SimpleNamespace(positions=lambda: {})
        approach.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
        approach.lidar = SimpleNamespace(odometry=lambda: (dict(linear=0., angular=0.), dict(received=10., stamp=10.)))
        approach.robot = lambda: ((.3, .278, .5), 0.)
        approach.relative = lambda _: np.array([.8, -.078, .74])
        approach.command = Mock(); approach.refresh_edge = Mock(return_value=False)
        approach.refresh_surface = Mock(); approach.require_protection_ready = Mock()
        approach.advance(10.)
        approach.refresh_edge.assert_not_called()
        self.assertEqual(approach.command.call_args.args[0], 0.)
        self.assertLess(approach.command.call_args.args[1], 0.)

    def test_cancel_before_nav_acceptance_cancels_late_handle(self):
        from navigation_controller import NavigationController
        controller = NavigationController.__new__(NavigationController)
        target = dict(x=1., y=0., yaw=0.)
        controller.nav_status = dict(state='sending', goal=target)
        controller.nav_goal = None
        controller.state = SimpleNamespace(log=Mock())
        controller.cancel_goal()
        handle = SimpleNamespace(accepted=True, cancel_goal_async=Mock())
        future = Future(); future.set_result(handle)
        controller.nav_accepted(future, target)
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(controller.nav_status['state'], 'idle')

    def test_slow_web_consumer_cannot_grow_status_backlog(self):
        from control_process import Outbox
        outgoing = Outbox()
        for i in range(1000):
            outgoing.put(('state', 'detection_control', dict(sequence=i)))
            outgoing.put(('log', str(i), 'info', 'detection'))
        messages = outgoing.drain()
        snapshots = [m for m in messages if m[0] == 'state']
        self.assertEqual(len(snapshots), 1)
        self.assertEqual(snapshots[0][2]['sequence'], 999)
        self.assertLessEqual(len(messages), 129)

    def test_cancel_superseded_handle_also_cancels_pending_replacement(self):
        from navigation_controller import NavigationController
        controller = NavigationController.__new__(NavigationController)
        target = dict(x=2., y=0., yaw=0.)
        old = SimpleNamespace(cancel_goal_async=Mock())
        controller.nav_status = dict(state='sending', goal=target)
        controller.nav_goal = old
        controller.state = SimpleNamespace(log=Mock())
        controller.cancel_goal()
        handle = SimpleNamespace(accepted=True, cancel_goal_async=Mock(), get_result_async=lambda: Future())
        future = Future(); future.set_result(handle)
        controller.nav_accepted(future, target)
        old.cancel_goal_async.assert_called_once()
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(controller.nav_status['state'], 'canceling')


if __name__ == '__main__':
    unittest.main()
