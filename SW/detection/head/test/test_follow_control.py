"""Control behavior checks; never import ROS or send wheel commands."""
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'head_detection_ws/src/robocup_head_detection'))
from robocup_head_detection.core import Box
from follow_control import Controller, Decision, Settings, decode_depth, fresh_command


class FollowTests(unittest.TestCase):
    def setUp(self):
        self.controller = Controller()
        self.box = Box(280, 200, 80, 60, 5, 'banana', 0.9)
        self.depth = np.full((480, 640), 1.5, np.float32)

    def confirmed(self, controller=None, box=None, depth=None):
        controller = controller or self.controller
        for _ in range(3):
            result = controller.update([box or self.box], self.depth if depth is None else depth, 640, 480)
        return result

    def test_three_confirmations_before_forward_motion(self):
        for _ in range(2):
            result = self.controller.update([self.box], self.depth, 640, 480)
            self.assertEqual((result.linear, result.angular, result.reason), (0, 0, 'CONFIRMING'))
        result = self.controller.update([self.box], self.depth, 640, 480)
        self.assertAlmostEqual(result.linear, 0.08)
        self.assertEqual(result.angular, 0)

    def test_right_target_turns_right_and_turns_before_driving(self):
        result = self.confirmed(box=replace(self.box, x=470))
        self.assertLess(result.angular, 0)
        self.assertEqual(result.linear, 0)
        self.assertLessEqual(abs(result.angular), 0.25)

    def test_left_target_turns_left(self):
        result = self.confirmed(box=replace(self.box, x=90))
        self.assertGreater(result.angular, 0)
        self.assertEqual(result.linear, 0)

    def test_distance_deadband_and_reverse_opt_in(self):
        self.assertEqual(self.confirmed(depth=np.ones((480, 640), np.float32)).linear, 0)
        near = np.full((480, 640), 0.7, np.float32)
        self.assertEqual(self.confirmed(depth=near).linear, 0)
        controller = Controller(Settings(allow_reverse=True))
        result = self.confirmed(controller=controller, depth=near)
        self.assertLess(result.linear, 0)
        self.assertGreaterEqual(result.linear, -0.04)

    def test_lost_target_stops_and_requires_reconfirmation(self):
        self.confirmed()
        result = self.controller.update([], self.depth, 640, 480)
        self.assertEqual((result.linear, result.angular), (0, 0))
        result = self.controller.update([self.box], self.depth, 640, 480)
        self.assertEqual(result.reason, 'CONFIRMING')

    def test_wrong_class_and_low_confidence_cannot_drive(self):
        for box in [replace(self.box, class_name='tomato'), replace(self.box, confidence=0.1)]:
            result = self.confirmed(box=box)
            self.assertEqual((result.linear, result.angular), (0, 0))

    def test_neighbour_cannot_steal_confirmed_target_by_confidence(self):
        self.confirmed()
        neighbour = replace(self.box, x=470, confidence=0.99)
        result = self.controller.update([neighbour, self.box], self.depth, 640, 480)
        self.assertEqual(result.box.x, self.box.x)
        self.assertEqual(result.angular, 0)

    def test_new_location_requires_three_new_confirmations(self):
        self.confirmed()
        result = self.controller.update([replace(self.box, x=470)], self.depth, 640, 480)
        self.assertEqual(result.reason, 'CONFIRMING')
        self.assertEqual((result.linear, result.angular), (0, 0))

    def test_invalid_depth_and_resolution_stop(self):
        for depth in [None, np.zeros((480, 640)), np.full((480, 640), np.nan), np.ones((240, 320))]:
            result = self.controller.update([self.box], depth, 640, 480)
            self.assertEqual((result.linear, result.angular), (0, 0))
        mixed = self.depth.copy()
        mixed[200:230] = 3.0
        self.assertEqual(self.confirmed(depth=mixed).reason, 'INVALID_DEPTH')

    def test_too_near_and_too_far_stop_even_with_reverse_enabled(self):
        controller = Controller(Settings(allow_reverse=True))
        for distance in [0.25, 4.0]:
            result = self.confirmed(controller=controller, depth=np.full((480, 640), distance))
            self.assertEqual(result.reason, 'DISTANCE_OUT_OF_RANGE')
            self.assertEqual((result.linear, result.angular), (0, 0))

    def test_watchdog_stops_when_inference_or_camera_stalls(self):
        decision = Decision(0.08, -0.25, 'FOLLOW')
        self.assertEqual(fresh_command(decision, 1, 1.1), decision)
        for now in [0.9, 1.31, float('nan'), float('inf')]:
            stopped = fresh_command(decision, 1, now)
            self.assertEqual((stopped.linear, stopped.angular), (0, 0))
        self.assertEqual(fresh_command(decision, 1, 1.1, stopped=True).reason, 'STOPPED')
        self.assertEqual(fresh_command(Decision(float('nan'), 0.1), 1, 1.1).linear, 0)

    def test_depth_units_padding_and_endianness(self):
        msg = SimpleNamespace(encoding='16UC1', width=2, height=1, step=6, is_bigendian=0,
                              data=np.array([1000, 1500, 65535], dtype='<u2').tobytes())
        np.testing.assert_allclose(decode_depth(msg), [[1.0, 1.5]])
        msg = SimpleNamespace(encoding='32FC1', width=2, height=1, step=8, is_bigendian=1,
                              data=np.array([1.0, 2.0], dtype='>f4').tobytes())
        np.testing.assert_allclose(decode_depth(msg), [[1.0, 2.0]])
        msg.step = 1
        with self.assertRaises(ValueError):
            decode_depth(msg)

    def test_invalid_parameters_rejected(self):
        for values in [dict(distance=0.2), dict(linear_max=2), dict(angular_max=float('nan')),
                       dict(confidence=1.1), dict(confirm_frames=1.5)]:
            with self.assertRaises(ValueError):
                Settings(**values)


if __name__ == '__main__':
    unittest.main()
