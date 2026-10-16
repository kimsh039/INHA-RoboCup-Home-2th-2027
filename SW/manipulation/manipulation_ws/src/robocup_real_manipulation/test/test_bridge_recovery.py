"""Feedback-only bridge regressions; no node, CAN adapter or actuator is opened."""
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from std_srvs.srv import Trigger
from robocup_real_manipulation.core import Fault, grasp_contact, require
from robocup_real_manipulation.piper_bridge import PiperBridge


class BridgeRecoveryTests(unittest.TestCase):
    def bridge(self):
        bridge = PiperBridge.__new__(PiperBridge)
        bridge.mutex = threading.RLock()
        bridge.busy = bridge.resetting = bridge.stopping = False
        bridge.tripped = True; bridge.trip_generation = 0
        bridge.q = np.zeros(6); bridge.v = np.zeros(6)
        bridge.width = .08; bridge.effort = .6; bridge.grip_speed = 0.
        bridge.joint_stamp = bridge.grip_stamp = bridge.status_received = 10.
        bridge.status = SimpleNamespace(err_code=0, **{
            key: False for i in range(1, 7)
            for key in (f'joint_{i}_angle_limit', f'communication_status_joint_{i}')})
        bridge.c = dict(feedback_age_s=.3, joint_stable_s=.04, stop_timeout_s=.2,
                        stable_speed_rad_s=.02, gripper_stable_speed_m_s=.002,
                        gripper_contact_min_travel_m=.002, gripper_tolerance_m=.001,
                        grasp_min_effort=.4, gripper_settle_s=.04, goal_tolerance_rad=.015)
        bridge.lease = None
        bridge.active_task = None
        bridge.command_pub = Mock()
        bridge.get_logger = Mock(return_value=Mock())
        return bridge

    def test_contact_allows_object_width_but_rejects_open_empty_and_low_force(self):
        cfg = self.bridge().c
        self.assertTrue(grasp_contact(.08, .6, .06, .1, cfg))
        for width, effort in ((.1, .6), (.0005, .6), (.08, .1), (.04, .6), (float('nan'), .6)):
            self.assertFalse(grasp_contact(width, effort, .06, .1, cfg))

    def test_reset_requires_settled_feedback_and_never_publishes(self):
        bridge = self.bridge(); clock = [10.]
        bridge.now = lambda: clock[0]
        def update(_):
            clock[0] += .01
            bridge.joint_stamp = bridge.grip_stamp = bridge.status_received = clock[0]
        with patch('robocup_real_manipulation.piper_bridge.time.monotonic', side_effect=lambda: clock[0]), \
                patch('robocup_real_manipulation.piper_bridge.time.sleep', side_effect=update):
            response = bridge.reset_service(None, Trigger.Response())
            with self.assertRaisesRegex(Fault, 'BASE_INTERLOCK'):
                bridge.valid()
        self.assertTrue(response.success)
        self.assertFalse(bridge.tripped)
        self.assertFalse(bridge.resetting)
        bridge.command_pub.publish.assert_not_called()

    def test_confirmed_idle_stop_does_not_latch_a_new_fault(self):
        bridge = self.bridge(); bridge.tripped = False; bridge.hold = Mock()
        clock = [10.]; bridge.now = lambda: clock[0]
        def update(_):
            clock[0] += .01
            bridge.joint_stamp = bridge.grip_stamp = clock[0]
        with patch('robocup_real_manipulation.piper_bridge.time.monotonic', side_effect=lambda: clock[0]), \
                patch('robocup_real_manipulation.piper_bridge.time.sleep', side_effect=update):
            response = bridge.stop_service(None, Trigger.Response())
        self.assertTrue(response.success)
        self.assertFalse(bridge.tripped)
        self.assertFalse(bridge.stopping)

    def test_reset_cannot_clear_a_fault_during_the_settling_interval(self):
        bridge = self.bridge(); clock = [10.]
        bridge.now = lambda: clock[0]
        def interrupt(_):
            clock[0] += .01
            bridge.trip('NEW_FAULT')
        with patch('robocup_real_manipulation.piper_bridge.time.monotonic', side_effect=lambda: clock[0]), \
                patch('robocup_real_manipulation.piper_bridge.time.sleep', side_effect=interrupt):
            response = bridge.reset_service(None, Trigger.Response())
        self.assertFalse(response.success)
        self.assertTrue(bridge.tripped)
        self.assertEqual(response.message, 'RESET_INTERRUPTED')

    def test_busy_or_stale_bridge_cannot_reset(self):
        for busy in (True, False):
            bridge = self.bridge(); bridge.busy = busy; bridge.now = lambda: 11.
            response = bridge.reset_service(None, Trigger.Response())
            self.assertFalse(response.success)
            self.assertTrue(bridge.tripped)

    def test_contact_close_succeeds_before_commanded_width(self):
        bridge = self.bridge(); bridge.tripped = False; bridge.busy = True
        bridge.width = .1; bridge.grip_stamp = 10.
        clock = [10.]
        bridge.publish = Mock()
        bridge.guard = lambda handle, deadline: require(clock[0] < deadline, 'MOTION_TIMEOUT')
        handle = SimpleNamespace(request=SimpleNamespace(width_m=.06, timeout_s=.5),
                                 is_cancel_requested=False, publish_feedback=Mock(),
                                 succeed=Mock(), abort=Mock())
        def feedback(_):
            clock[0] += .02
            bridge.grip_stamp = clock[0]
            bridge.width = .08; bridge.effort = .6; bridge.grip_speed = 0.
        with patch('robocup_real_manipulation.piper_bridge.time.monotonic', side_effect=lambda: clock[0]), \
                patch('robocup_real_manipulation.piper_bridge.time.sleep', side_effect=feedback):
            result = bridge.execute_gripper(handle)
        self.assertTrue(result.success)
        self.assertEqual(result.reason, 'CONTACT_GRASP')
        self.assertAlmostEqual(result.measured_width_m, .08)
        handle.succeed.assert_called_once()

    def test_repeated_feedback_or_obstructed_open_cannot_report_contact_success(self):
        for opening in (False, True):
            bridge = self.bridge(); bridge.busy = True; bridge.tripped = False
            bridge.width = .06 if opening else .08
            clock = [10.]
            bridge.publish = Mock(); bridge.confirm_stop = Mock(return_value=True)
            bridge.guard = lambda handle, deadline: require(clock[0] < deadline, 'MOTION_TIMEOUT')
            handle = SimpleNamespace(request=SimpleNamespace(width_m=.1 if opening else .06, timeout_s=.15),
                                     is_cancel_requested=False, publish_feedback=Mock(),
                                     succeed=Mock(), abort=Mock())
            def feedback(_):
                clock[0] += .02
                bridge.width = .08
                if opening: bridge.grip_stamp = clock[0]
            with patch('robocup_real_manipulation.piper_bridge.time.monotonic', side_effect=lambda: clock[0]), \
                    patch('robocup_real_manipulation.piper_bridge.time.sleep', side_effect=feedback):
                result = bridge.execute_gripper(handle)
            self.assertFalse(result.success)
            handle.succeed.assert_not_called()
            handle.abort.assert_called_once()


if __name__ == '__main__':
    unittest.main()
