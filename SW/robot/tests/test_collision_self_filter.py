"""Required regression cases for the requested rack interior exclusion."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collision_geometry import RobotGeometry


class RackInteriorFilterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = RobotGeometry()

    def test_recorded_wire_point_inside_rack_is_excluded(self):
        point = np.array([[-0.16331843, -0.11995699, 1.05478942]])
        self.assertTrue(any(b.contains(point, .005)[0] for b in self.model.self_interiors))

    def test_open_space_inside_rack_is_excluded(self):
        point = np.array([[0., 0., 1.]])
        self.assertTrue(any(b.contains(point)[0] for b in self.model.self_interiors))

    def test_outside_rack_is_preserved_at_same_height(self):
        point = np.array([[-.31, -.145, 1.1]])
        self.assertFalse(any(b.contains(point, .005)[0] for b in self.model.self_interiors))

    def test_self_interior_does_not_become_physical_collision_geometry(self):
        boxes = self.model.boxes({name: 0. for name in self.model.required_joints})
        self.assertFalse(any(b.name.startswith('self_interior/') for b in boxes))


if __name__ == '__main__':
    unittest.main()
