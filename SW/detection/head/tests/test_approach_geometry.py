import math
import unittest
from types import SimpleNamespace as NS
from robocup_head_detection.approach_geometry import (OBJECT_OFFSET, ApproachConfig, approach_candidates, contains,
                                                      reach_at, supporting_surface)


def table(cx, cy, length, width, height, observed=(1, 1, 1, 1)):
    """Axis-aligned table outline with counter-clockwise corners (side 0 runs along -y edge)."""
    x0, x1, y0, y1 = cx-length/2, cx+length/2, cy-width/2, cy+width/2
    return NS(corners=((x0, y0), (x1, y0), (x1, y1), (x0, y1)), height=height, edge_observed=observed)


class ApproachGeometryTests(unittest.TestCase):
    def test_contains(self):
        t = table(1.8, -1.0, 1.6, 0.8, 0.72)
        self.assertTrue(contains(t.corners, (1.9, -0.8)))
        self.assertFalse(contains(t.corners, (1.9, -0.5)))
        self.assertTrue(contains(t.corners, (1.9, -0.57), margin=0.05))

    def test_supporting_surface_by_outline_and_height(self):
        t1, t2 = table(1.8, -1.0, 1.6, 0.8, 0.72), table(1.8, 1.8, 1.6, 0.8, 0.72)
        self.assertIs(supporting_surface([t1, t2], (1.9, -0.8), 0.76), t1)
        self.assertIsNone(supporting_surface([t1, t2], (1.9, -0.8), 1.5))   # floating far above
        self.assertIsNone(supporting_surface([t1, t2], (0.0, 0.0), 0.76))

    def test_reach_table(self):
        self.assertEqual(reach_at(0.0), 0.50)
        self.assertAlmostEqual(reach_at(-0.2), 0.45)
        self.assertIsNone(reach_at(0.3))

    def test_target_lands_at_manipulation_offset(self):
        # Cup 0.2 m inside the y = -0.6 edge: robot faces -y, target 0.392 ahead, 0.078 right.
        t = table(1.8, -1.0, 1.6, 0.8, 0.72)
        best = approach_candidates(t, 1, (1.9, -0.8))[0]
        self.assertEqual(best.side, 2)
        x, y, yaw = best.dock
        self.assertAlmostEqual(yaw, -math.pi/2)
        self.assertAlmostEqual(y, -0.8+OBJECT_OFFSET[0])          # 0.392 behind the cup
        self.assertAlmostEqual(x, 1.9-OBJECT_OFFSET[1])           # robot's left is +x: cup 0.078 to its right
        self.assertAlmostEqual(best.edge_distance, OBJECT_OFFSET[0]-0.2)
        self.assertAlmostEqual(best.offset_error, 0.0)
        self.assertAlmostEqual(best.stage[1], -0.6+0.60)

    def test_deep_target_stops_at_min_edge(self):
        t = table(1.8, -1.0, 1.6, 0.8, 0.72)
        best = approach_candidates(t, 1, (1.9, -0.95))[0]         # 0.35 deep
        self.assertAlmostEqual(best.edge_distance, 0.08)
        self.assertAlmostEqual(best.target_in_base[0], 0.43)
        self.assertGreater(best.reach_margin, 0)
        # 0.45 deep is beyond the arm from the y = -0.6 side; the far side then wins.
        far = approach_candidates(t, 1, (1.9, -1.05))[0]
        self.assertEqual(far.side, 0)

    def test_unconfirmed_sides_skipped(self):
        t = table(1.8, -1.0, 1.6, 0.8, 0.72, observed=(0.9, 0.1, 0.1, 0.1))
        self.assertEqual(approach_candidates(t, 1, (1.9, -0.8)), [])            # 0.6 deep from side 0
        (only,) = approach_candidates(t, 1, (1.9, -0.8), ApproachConfig(reach_scale=1.6))
        self.assertEqual(only.side, 0)
        self.assertAlmostEqual(only.dock[1], -1.4-0.08)

    def test_robot_kept_off_the_corners(self):
        # Only the y = -0.6 side is confirmed; the target sits near the x = 1.0 corner.
        t = table(1.8, -1.0, 1.6, 0.8, 0.72, observed=(0, 0, 1, 0))
        cfg = ApproachConfig()
        (c,) = approach_candidates(t, 1, (1.2, -0.75), cfg)
        self.assertAlmostEqual(c.dock[0], 1.0+cfg.half_width+cfg.corner_clearance)   # body clears the leg
        self.assertAlmostEqual(c.target_in_base[1], 1.2-c.dock[0])                  # target 0.18 m right
        self.assertAlmostEqual(c.offset_error, abs(c.target_in_base[1]-OBJECT_OFFSET[1]))
        # 0.1 m from the corner the target would end up 0.28 m to the side: beyond the arm.
        self.assertEqual(approach_candidates(t, 1, (1.1, -0.75), cfg), [])


if __name__ == '__main__':
    unittest.main()
