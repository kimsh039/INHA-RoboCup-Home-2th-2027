import unittest
import numpy as np
from scripts.generate_single_view import generate


class SingleViewTest(unittest.TestCase):
    def test_front_view_has_only_front_face(self):
        points, camera, meta = generate()
        np.testing.assert_allclose(points[:, 0], -.02)
        np.testing.assert_allclose(camera[:, 2], .48)
        np.testing.assert_array_equal(points, generate()[0])
        self.assertEqual(meta['dimensions']['dimensions_m'], [.04]*3)

    def test_oblique_view_has_three_visible_faces_and_no_hidden_face(self):
        points, camera, meta = generate(camera_position=[.30, -.30, .35])
        seen = []
        for axis, sign in [(0, 1), (1, -1), (2, 1)]:
            seen.append(np.isclose(points[:, axis], sign*.02, atol=1e-8, rtol=0))
        self.assertTrue(np.stack(seen).any(axis=0).all())
        self.assertTrue(all(mask.sum() > 2000 for mask in seen))
        self.assertTrue((camera[:, 2] > 0).all())
        T = np.array(meta['T_camera_object'])
        recovered = camera@T[:3, :3]-T[:3, 3]@T[:3, :3]
        np.testing.assert_allclose(recovered, points, atol=5e-8)

    def test_camera_inside_cube_rejected(self):
        with self.assertRaises(ValueError):
            generate(camera_position=[0, 0, 0])
