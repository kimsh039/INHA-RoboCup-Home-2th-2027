import importlib.util
from pathlib import Path
import unittest

import numpy as np

path = Path(__file__).resolve().parents[1]/"scripts/derive_pregrasps.py"
spec = importlib.util.spec_from_file_location("pregrasp", path)
pregrasp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pregrasp)


class PregraspTest(unittest.TestCase):
    def test_rotated_approach_and_preserved_orientation(self):
        R = np.array([[0,-1,0],[1,0,0],[0,0,1]], dtype=float)
        p = np.array([0.3,0.2,0.1])
        result = pregrasp.derive(R, p, 0.2)
        T = np.array(result["T_object_pregrasp"])
        np.testing.assert_allclose(T[:3,3], [0.3,0,0.1], atol=1e-12)
        np.testing.assert_array_equal(T[:3,:3], R)
        self.assertAlmostEqual(np.linalg.norm(p-T[:3,3]), 0.2)

    def test_invalid_transform_and_distance(self):
        for rotation in (np.zeros((3,3)), np.diag([1,1,-1])):
            with self.assertRaises(ValueError):
                pregrasp.derive(rotation, [0,0,0], 0.2)
        with self.assertRaises(ValueError):
            pregrasp.derive(np.eye(3), [0,0,0], -0.2)


if __name__ == "__main__":
    unittest.main()
