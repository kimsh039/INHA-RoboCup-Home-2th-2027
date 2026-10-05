import unittest
from types import SimpleNamespace as NS
import numpy as np
from robocup_head_detection.nav_geometry import Intrinsics, target_range, standoff_goal, free_goal, depth_array
from robocup_head_detection.closed_approach import ClosedApproachRequest, request_closed_approach


class NavigationGeometryTests(unittest.TestCase):
    def test_rgb_depth_different_fov_mapping(self):
        color = Intrinsics(640, 480, 400, 400, 320, 240)
        sensor = Intrinsics(320, 240, 150, 150, 160, 120)
        depth = np.full((240, 320), 4, np.float32)
        depth[110:132, 175:195] = 2
        self.assertAlmostEqual(target_range((350, 200, 80, 80), depth, color, sensor), 2)

    def test_invalid_depth_and_stride(self):
        info = Intrinsics(20, 20, 10, 10, 10, 10)
        self.assertIsNone(target_range((2, 2, 16, 16), np.full((20, 20), np.nan), info, info))
        msg = NS(encoding='32FC1', is_bigendian=False, width=2, height=1, step=12,
                 data=np.array([1, 2, 999], dtype='<f4').tobytes())
        self.assertEqual(depth_array(msg).tolist(), [[1, 2]])

    def test_standoff_goal_not_inside_object(self):
        self.assertEqual(standoff_goal((3, 0), (0, 0), 1), (2, 0, 0))
        x, y, yaw = standoff_goal((0, 3), (0, 0), 1)
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 2)
        self.assertIsNone(standoff_goal((.5, 0), (0, 0), 1))

    def test_goal_requires_known_free_footprint(self):
        grid = NS(info=NS(width=40, height=40, resolution=.1,
                         origin=NS(position=NS(x=0, y=0), orientation=NS(x=0, y=0, z=0, w=1))),
                  data=[0]*1600)
        self.assertTrue(free_goal(grid, 2, 2))
        grid.data[20*40+20] = -1
        self.assertFalse(free_goal(grid, 2, 2))
        grid.data[20*40+20] = 100
        self.assertFalse(free_goal(grid, 2, 2))
        self.assertFalse(free_goal(grid, -1, 2))

    def test_closed_approach_is_reserved_not_executed(self):
        request = ClosedApproachRequest('target:1', object(), object())
        self.assertEqual(request_closed_approach(request), 'CLOSED_APPROACH_PENDING')


if __name__ == '__main__':
    unittest.main()
