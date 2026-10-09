import unittest
from types import SimpleNamespace as NS
import numpy as np
from robocup_head_detection.nav_geometry import Intrinsics, lidar_target, standoff_goal, free_goal, depth_array
from robocup_head_detection.closed_approach import ClosedApproachRequest, request_closed_approach


class NavigationGeometryTests(unittest.TestCase):
    # Camera 1.4 m above the floor at x=0 looking along +x (optical z = +x, x = -y, y = -z).
    CAM = np.array([[0, -1, 0, 0], [0, 0, -1, 1.4], [1, 0, 0, 0], [0, 0, 0, 1]], float)
    COLOR = Intrinsics(640, 480, 400, 400, 320, 240)
    TABLE = [(np.array([(1.6, -.4), (2.4, -.4), (2.4, .4), (1.6, .4)]), .72)]

    def scene(self):
        """Table top 0.72 m (x 1.6..2.4), a wall at x 3, an object side face at x 1.95, z .72-.80."""
        g = np.mgrid[-.4:.4:.02, 1.6:2.4:.02].reshape(2, -1).T
        top = np.c_[g[:, 1], g[:, 0], np.full(len(g), .72)]
        wall = np.c_[np.full(400, 3.0), np.repeat(np.linspace(-1, 1, 20), 20), np.tile(np.linspace(0, 2, 20), 20)]
        obj = np.c_[np.full(25, 1.95), np.repeat(np.linspace(-.03, .03, 5), 5), np.tile(np.linspace(.73, .79, 5), 5)]
        return np.r_[top, wall, obj]

    def box(self, x, y, z0, z1, half):
        """Image bbox of an object at range x, lateral y (m), heights z0..z1 (m)."""
        u0, u1 = sorted(400*(-(y+s*half))/x + 320 for s in (-1, 1))
        v0, v1 = sorted(400*(1.4-z)/x + 240 for z in (z0, z1))
        return (u0, v0, u1-u0, v1-v0)

    def test_lidar_target_takes_the_object_not_table_or_wall(self):
        found, count = lidar_target(self.scene(), self.box(1.95, 0, .72, .80, .04), self.COLOR, self.CAM, self.TABLE)
        self.assertIsNotNone(found)
        self.assertAlmostEqual(found[0], 1.95, places=2)
        self.assertAlmostEqual(found[1], 0, places=2)
        self.assertGreater(found[2], .72)

    def test_lidar_target_needs_points(self):
        found, count = lidar_target(self.scene(), self.box(1.95, .3, .72, .80, .04), self.COLOR, self.CAM, self.TABLE)
        self.assertIsNone(found)                     # nothing above the table there
        self.assertEqual(count, 0)
        self.assertIsNone(lidar_target(np.empty((0, 3)), (2, 2, 16, 16), self.COLOR, self.CAM)[0])

    def test_without_surfaces_the_nearest_cluster_wins(self):
        found, _ = lidar_target(self.scene(), self.box(1.95, 0, .74, .80, .04), self.COLOR, self.CAM)
        self.assertLess(found[0], 2.1)               # not the wall at 3 m

    def test_depth_array_stride(self):
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
