"""Geometry and reproducibility checks independent of MuJoCo and ROS."""
import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("generator", ROOT/"scripts/generate_dummy_pointclouds.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class CloudsTest(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT/"config/pointclouds.json").read_text())

    def test_surface_and_reproducibility(self):
        for kind in ("cube", "pringles", "can"):
            with self.subTest(kind=kind):
                cloud, metadata = generator.generate(kind, self.cfg)
                duplicate, _ = generator.generate(kind, self.cfg)
                np.testing.assert_array_equal(cloud, duplicate)
                self.assertEqual(cloud.shape, (20000, 3))
                self.assertTrue(np.isfinite(cloud).all())
                self.assertEqual(metadata["source_frame"], "object")
                dimensions = self.cfg["objects"][kind]
                if kind == "cube":
                    half = np.array(dimensions["dimensions_m"])/2
                    self.assertTrue(np.all(np.abs(cloud) <= half+1e-7))
                    self.assertTrue(np.all(np.any(np.isclose(np.abs(cloud), half), axis=1)))
                else:
                    radius, halfheight = dimensions["radius_m"], dimensions["height_m"]/2
                    radial = np.linalg.norm(cloud[:, :2], axis=1)
                    self.assertTrue(np.all(radial <= radius+1e-7))
                    self.assertTrue(np.all(np.abs(cloud[:, 2]) <= halfheight+1e-7))
                    self.assertTrue(np.all(np.isclose(radial, radius) | np.isclose(np.abs(cloud[:, 2]), halfheight)))

    def test_invalid_dimensions_and_noise(self):
        self.cfg["objects"]["can"]["radius_m"] = -1
        with self.assertRaises(ValueError):
            generator.generate("can", self.cfg)
        self.cfg["noise_std_m"] = -0.1
        with self.assertRaises(ValueError):
            generator.generate("cube", self.cfg)


if __name__ == "__main__":
    unittest.main()
