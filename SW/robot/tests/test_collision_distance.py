"""Compare batched distances with the previous per-box implementation."""
from pathlib import Path
import sys
import time
import unittest

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collision_geometry import Box, rpy_matrix
from collision_distance import ComputationExpired, PreparedBoxes, build_scene


def reference(boxes, scene, pose, radius):
    transform = np.eye(4)
    transform[:3, :3] = rpy_matrix([0., 0., pose[2]])
    transform[:2, 3] = pose[:2]
    best = float('inf')
    for box in boxes:
        moved = box.moved(transform)
        indices = scene.query_ball_point(moved.transform[:3, 3], np.linalg.norm(moved.half)+radius)
        if indices:
            best = min(best, float(moved.distances(scene.data[indices]).min()))
    return best


class CollisionDistanceTests(unittest.TestCase):
    def test_distances_and_clearance_decisions_match_per_box_search(self):
        random = np.random.default_rng(20261015)
        boxes = []
        for i in range(21):
            transform = np.eye(4)
            transform[:3, :3] = rpy_matrix(random.uniform(-1., 1., 3))
            transform[:3, 3] = random.uniform(-.7, .7, 3)
            boxes.append(Box(str(i), transform, random.uniform(.01, .2, 3)))
        prepared = PreparedBoxes(boxes)
        scene = cKDTree(random.uniform(-2., 2., (6000, 3)))
        optimized_scene = build_scene(scene.data.copy())
        for _ in range(24):
            pose = random.uniform(-.4, .4, 3)
            for radius in (.185, .285):
                expected = reference(boxes, scene, pose, radius)
                measured, index, owner = prepared.nearest(optimized_scene, pose, radius, time.monotonic()+2.)
                self.assertAlmostEqual(measured, expected, places=12)
                self.assertEqual(measured < radius, expected < radius)
                if index is not None:
                    self.assertTrue(0 <= index < len(scene.data))
                    self.assertTrue(0 <= owner < len(boxes))

    def test_empty_scene(self):
        prepared = PreparedBoxes([Box('test', np.eye(4), [.2, .2, .2])])
        measured, index, owner = prepared.nearest(cKDTree(np.empty((0, 3))), np.zeros(3), .2, time.monotonic()+1.)
        self.assertEqual(measured, float('inf'))
        self.assertIsNone(index)
        self.assertIsNone(owner)

    def test_expired_budget_is_rejected(self):
        prepared = PreparedBoxes([Box('test', np.eye(4), [.2, .2, .2])])
        with self.assertRaises(ComputationExpired):
            prepared.nearest(cKDTree(np.zeros((1, 3))), np.zeros(3), .2, time.monotonic()-1.)


if __name__ == '__main__':
    unittest.main()
