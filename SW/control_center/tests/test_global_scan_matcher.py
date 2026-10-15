"""Independent ray-cast scenes verify recovery and rejection of ambiguous poses."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('global_scan_matcher', ROOT/'global_scan_matcher.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
SETTINGS = yaml.safe_load((ROOT/'global-localization.yaml').read_text())['slam_global_initializer']['ros__parameters']


def scene(asymmetric):
    res = .05
    cells = np.zeros((100, 120), dtype=np.int16)
    cells[[0, -1], :] = 100; cells[:, [0, -1]] = 100
    rectangles = [(-3., 3., -2.5, 2.5)]
    if asymmetric:
        rectangles += [(-.75, -.35, .4, 1.4), (1.6, 3., 1.3, 2.5)]
        for x0, x1, y0, y1 in rectangles[1:]:
            cells[round((y0+2.5)/res):round((y1+2.5)/res),
                  round((x0+3)/res):round((x1+3)/res)] = 100
    pose = np.array([1., -.7, .9])
    points = []
    for local_angle in np.linspace(-np.pi, np.pi, 180, endpoint=False):
        direction = np.array([np.cos(local_angle+pose[2]), np.sin(local_angle+pose[2])])
        hits = []
        for x0, x1, y0, y1 in rectangles:
            for x in (x0, x1):
                if abs(direction[0]) > 1e-9:
                    t = (x-pose[0])/direction[0]
                    y = pose[1]+t*direction[1]
                    if t > .01 and y0 <= y <= y1: hits.append(t)
            for y in (y0, y1):
                if abs(direction[1]) > 1e-9:
                    t = (y-pose[1])/direction[1]
                    x = pose[0]+t*direction[0]
                    if t > .01 and x0 <= x <= x1: hits.append(t)
        distance = min(hits)
        points.append([distance*np.cos(local_angle), distance*np.sin(local_angle)])
    return module.MapMatcher(cells, res, [-3., -2.5, 0.], SETTINGS), np.asarray(points), pose


class GlobalMatchingChecks(unittest.TestCase):
    def test_recovers_position_and_heading_without_a_pose_prior(self):
        matcher, points, truth = scene(True)
        result = matcher.search(points)
        self.assertTrue(result['accepted'], result)
        pose = np.asarray(result['best']['pose'])
        self.assertLess(np.linalg.norm(pose[:2]-truth[:2]), .15)
        self.assertLess(module.angular_distance(pose[2], truth[2]), .06)

    def test_symmetric_room_keeps_competing_modes_instead_of_accepting_one(self):
        matcher, points, _ = scene(False)
        result = matcher.search(points)
        self.assertFalse(result['accepted'], result)
        self.assertLess(result['margin'], SETTINGS['minimum_score_margin'])
        self.assertGreaterEqual(len(result['candidates']), 2)

    def test_yaml_image_preserves_unknown_and_rotated_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            Image.fromarray(np.array([[0, 205], [254, 254]], dtype=np.uint8)).save(path/'map.pgm')
            (path/'map.yaml').write_text(yaml.safe_dump(dict(
                image='map.pgm', resolution=.1, origin=[2., 3., np.pi/2],
                negate=0, occupied_thresh=.65, free_thresh=.196)))
            matcher = module.MapMatcher.load(path/'map.yaml', SETTINGS)
            self.assertEqual(matcher.cells[1, 0], 100)
            self.assertEqual(matcher.cells[1, 1], -1)
            ix, iy = matcher.world_to_grid(1.85, 3.05)
            self.assertEqual((int(ix), int(iy)), (0, 1))

    def test_search_can_be_cancelled_when_its_owned_process_is_stopped(self):
        matcher, points, _ = scene(True)
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            matcher.search(points, lambda: True)


if __name__ == '__main__':
    unittest.main()
