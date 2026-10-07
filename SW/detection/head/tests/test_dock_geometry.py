import math
import unittest
import numpy as np
from robocup_head_detection.dock_geometry import DockConfig, EdgeConfig, blind_limit, dock_command, fit_edge


def table_top(distance, normal, height=0.72, depth=0.8, step=0.02, cut=None):
    """Top points in base_link of a table whose near edge is `distance` away along `normal`."""
    n = np.array([math.cos(normal), math.sin(normal)])
    t = np.array([-n[1], n[0]])
    pts = [distance*n + a*t + b*n for a in np.arange(-1.0, 1.0, step) for b in np.arange(0.0, depth, step)]
    pts = np.array([(x, y, height) for x, y in pts])
    if cut is not None:                       # sensor blind zone: nothing closer than cut(y)
        pts = pts[[x >= cut(y) for x, y, _ in pts]]
    return pts


class EdgeFitTests(unittest.TestCase):
    def test_distance_and_angle(self):
        for d, a in ((0.7, 0.0), (0.6, math.radians(-14.6)), (0.5, math.radians(10))):
            e = fit_edge(table_top(d, a), 0.72)
            self.assertAlmostEqual(e.distance, d, delta=0.02)
            self.assertAlmostEqual(e.angle, a, delta=math.radians(1.5))

    def test_occluded_bins_rejected(self):
        pts = table_top(0.6, 0.0)
        pts = pts[~((pts[:, 1] > 0.3) & (pts[:, 0] < 0.9))]     # an object hides the edge at y > 0.3
        e = fit_edge(pts, 0.72)
        self.assertAlmostEqual(e.distance, 0.6, delta=0.02)
        self.assertAlmostEqual(e.angle, 0.0, delta=math.radians(1.5))

    def test_hidden_edge_in_blind_zone(self):
        cfg = EdgeConfig()
        limit = lambda y: blind_limit(y, 0.72, cfg)
        self.assertAlmostEqual(limit(0.0), -0.18 + (1.34-0.72)/math.tan(math.radians(52)), places=6)
        # Edge at 0.15 m: the sensor only sees the top from its blind limit on, so no edge.
        self.assertIsNone(fit_edge(table_top(0.15, 0.0, cut=limit), 0.72, cfg=cfg))
        # Edge at 0.5 m is outside the blind zone and measured.
        self.assertAlmostEqual(fit_edge(table_top(0.5, 0.0, cut=limit), 0.72, cfg=cfg).distance, 0.5, delta=0.02)

    def test_floor_offset(self):
        e = fit_edge(table_top(0.6, 0.0, height=0.72-0.1425), 0.72, floor_z=-0.1425)
        self.assertAlmostEqual(e.distance, 0.6, delta=0.02)


class DockCommandTests(unittest.TestCase):
    def run_dock(self, start, goal, dt=0.1, steps=2000):
        x, y, yaw = start
        cfg = DockConfig()
        for _ in range(steps):
            v, w, done = dock_command((x, y, yaw), goal, cfg)
            if done:
                return (x, y, yaw), True
            self.assertGreaterEqual(v, 0.0)                     # never reverses toward the robot's back
            x += v*math.cos(yaw)*dt
            y += v*math.sin(yaw)*dt
            yaw += w*dt
        return (x, y, yaw), False

    def test_converges_from_lateral_and_heading_error(self):
        goal = (1.88, -0.52, -math.pi/2)
        for start in ((1.64, 0.10, math.radians(-75.5)), (2.05, 0.0, math.radians(-100)), (1.88, 0.05, -math.pi/2)):
            (x, y, yaw), done = self.run_dock(start, goal)
            self.assertTrue(done, start)
            u = (math.cos(goal[2]), math.sin(goal[2]))
            along = (goal[0]-x)*u[0] + (goal[1]-y)*u[1]
            lateral = -(x-goal[0])*u[1] + (y-goal[1])*u[0]
            self.assertLess(abs(along), 0.006)
            self.assertLess(abs(lateral), 0.01)
            self.assertLess(abs(math.atan2(math.sin(yaw-goal[2]), math.cos(yaw-goal[2]))), math.radians(1.0)+1e-9)

    def test_turns_in_place_when_far_off_heading(self):
        v, w, done = dock_command((0, 0, 0), (0, -0.6, -math.pi/2))
        self.assertEqual(v, 0.0)
        self.assertLess(w, 0.0)
        self.assertFalse(done)


if __name__ == '__main__':
    unittest.main()
