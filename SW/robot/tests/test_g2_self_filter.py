"""Required regressions for G2 endpoint masking and scan timing preservation."""
import json
from pathlib import Path
import sys
import unittest

import numpy as np
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collision_geometry import Box, REPO, RobotGeometry, rpy_matrix
from scan_self_filter import ScanSelfMask


class G2SelfFilterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = RobotGeometry()
        cls.record = json.loads((REPO/'HW/calibration/physical/base_2dlidar/vs_mid360/calibration.json').read_text())
        mid = json.loads((REPO/'HW/calibration/physical/base_mid360/base_mid360.json').read_text())
        cls.transform = np.asarray(mid['matrix4x4']) @ np.asarray(cls.record['matrix_4x4'])

    def scan(self, ranges, angle=0.):
        message = LaserScan()
        message.header.frame_id = 'laser_frame'
        message.header.stamp.sec, message.header.stamp.nanosec = 123, 456
        message.angle_min, message.angle_increment = float(angle), -.01
        message.angle_max = float(angle-.01*(len(ranges)-1))
        message.time_increment, message.scan_time = -.0003, .1
        message.range_min, message.range_max = .01, 20.
        message.ranges, message.intensities = ranges, [2.]*len(ranges)
        return message

    def test_recorded_internal_g2_return_is_removed_but_farther_return_is_kept(self):
        point = np.array([.14445611084054, -.16555548183283, .35046226332007, 1.])
        sensor = np.linalg.inv(self.transform) @ point
        angle = float(np.arctan2(sensor[1], sensor[0]))
        scale = self.record['g2_range_scale']
        mask = ScanSelfMask(self.model.fixed+self.model.self_interiors, self.transform)
        near = self.scan([float(np.linalg.norm(sensor[:2])/scale)], angle)
        out, stats = mask.apply(near, scale, [])
        self.assertTrue(np.isnan(out.ranges[0]))
        self.assertEqual(stats['urdf_removed'], 1)
        out, _ = mask.apply(self.scan([float(2./scale)], angle), scale, [])
        self.assertAlmostEqual(out.ranges[0], 2., places=6)

    def test_mask_preserves_header_timing_order_and_original_message(self):
        mask = ScanSelfMask([], np.eye(4))
        original = self.scan([.1, 1., float('inf'), float('nan')])
        out, _ = mask.apply(original, 1., [])
        for field in ('header', 'angle_min', 'angle_max', 'angle_increment', 'time_increment', 'scan_time', 'intensities'):
            self.assertEqual(getattr(out, field), getattr(original, field))
        self.assertAlmostEqual(original.ranges[0], .1, places=6)
        self.assertTrue(np.isnan(out.ranges[0]))
        self.assertEqual(out.ranges[1], 1.)
        self.assertTrue(np.isnan(out.ranges[2]))

    def test_cached_intervals_match_direct_rotated_box_membership(self):
        transform = np.eye(4)
        transform[:3, :3] = rpy_matrix([.1, -.2, .3])
        transform[:3, 3] = [-.12, .04, .2]
        box_pose = np.eye(4)
        box_pose[:3, :3] = rpy_matrix([.3, -.1, -.2])
        box_pose[:3, 3] = [.35, -.2, .25]
        box = Box('rotated', box_pose, [.3, .2, .15])
        mask = ScanSelfMask([box], transform, minimum_range=0.)
        random = np.random.default_rng(20261015)
        for _ in range(12):
            message = self.scan(random.uniform(.03, 2., 181).tolist(), 1.)
            out, _ = mask.apply(message, 1., [])
            angles = message.angle_min+np.arange(len(message.ranges))*message.angle_increment
            ranges = np.asarray(message.ranges)
            sensor = np.column_stack([ranges*np.cos(angles), ranges*np.sin(angles), np.zeros(len(ranges))])
            points = sensor @ transform[:3, :3].T+transform[:3, 3]
            np.testing.assert_array_equal(np.isnan(out.ranges), box.contains(points, .005))

    def test_box_at_other_height_does_not_hide_external_returns(self):
        pose = np.eye(4)
        pose[:3, 3] = [1., 0., 1.]
        mask = ScanSelfMask([Box('above_scan', pose, [.1, .1, .1])], np.eye(4))
        out, _ = mask.apply(self.scan([1.]), 1., [])
        self.assertEqual(out.ranges[0], 1.)

    def test_unknown_frame_is_rejected(self):
        message = self.scan([1.])
        message.header.frame_id = 'unexpected_sensor'
        with self.assertRaises(ValueError):
            ScanSelfMask([], np.eye(4)).apply(message, 1., [])


if __name__ == '__main__':
    unittest.main()
