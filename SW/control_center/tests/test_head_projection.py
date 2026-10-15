"""Regression: an off-screen LiDAR return must not become a cup coordinate."""
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from head_projection import camera_ray_mask


K = (605.53244, 0., 331.95871, 0., 608.06854, 243.32325, 0., 0., 1.)
D = (.082175, -.163474, -.001177, .005889, 0.)


class CameraRayVisibility(unittest.TestCase):
    def test_recorded_false_cup_point_is_rejected(self):
        point = np.array([[1.977014004672268, .512434422919768, 1.2495946883698412]])
        pixel, _ = cv2.projectPoints(point, np.zeros(3), np.zeros(3),
                                    np.array(K).reshape(3, 3), np.array(D))
        x, y = pixel.reshape(2)
        self.assertTrue(395.9 < x < 419.7 and 243.8 < y < 268.3)
        self.assertFalse(camera_ray_mask(point, K, D, 640, 480)[0])

    def test_real_sensor_rays_remain_visible(self):
        pixels = np.array([(x, y) for x in np.linspace(0., 639., 17)
                           for y in np.linspace(0., 479., 17)])
        rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2),
                                  np.array(K).reshape(3, 3), np.array(D)).reshape(-1, 2)
        points = np.column_stack((rays, np.ones(len(rays))))
        self.assertTrue(camera_ray_mask(points, K, D, 640, 480).all())

    def test_nonfinite_behind_and_pinhole_offscreen_are_rejected(self):
        points = np.array([[0., 0., 1.], [0., 0., -1.], [np.nan, 0., 1.],
                           [2., 0., 1.], [0., 0., 0.]])
        self.assertEqual(camera_ray_mask(points, K, (0.,) * 5, 640, 480).tolist(),
                         [True, False, False, False, False])


if __name__ == '__main__':
    unittest.main()
