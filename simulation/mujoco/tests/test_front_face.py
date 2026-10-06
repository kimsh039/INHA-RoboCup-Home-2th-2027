"""Check single-view geometry and frame conversion without GraspNet/ROS execution."""
import unittest
import numpy as np
from scripts.generate_front_face import generate
from manipulation.synthetic_camera import grasps_camera_to_object
from scripts.derive_pregrasps import derive


class FrontFaceTest(unittest.TestCase):
    def test_one_face_depth_and_reproducibility(self):
        points, camera, plane, meta = generate()
        np.testing.assert_array_equal(points, generate()[0])
        np.testing.assert_allclose(points[:, 0], -0.02)
        np.testing.assert_allclose(camera[:, 2], 0.48)
        np.testing.assert_allclose(camera[:, :2], plane)
        self.assertTrue((np.abs(points[:, 1:]) <= 0.02).all())
        self.assertEqual(np.linalg.matrix_rank(points-points.mean(axis=0), tol=1e-3), 2)
        T = np.array(meta['T_camera_object'])
        np.testing.assert_allclose(T[:3, :3].T@T[:3, :3], np.eye(3))
        self.assertAlmostEqual(np.linalg.det(T[:3, :3]), 1)

    def test_pose_inverse_preserves_predictions_and_pregrasp(self):
        _, _, _, meta = generate()
        T = np.array(meta['T_camera_object'])
        raw = np.zeros((1, 17))
        raw[0, :4] = [.9, .06, .02, .03]
        raw[0, 4:13] = np.eye(3).reshape(-1)
        raw[0, 13:16] = [.01, -.01, .48]
        result = grasps_camera_to_object(raw, T)
        np.testing.assert_array_equal(result[:, :4], raw[:, :4])
        R = result[0, 4:13].reshape(3, 3)
        p = result[0, 13:16]
        np.testing.assert_allclose(T[:3, :3]@R, np.eye(3))
        np.testing.assert_allclose(T[:3, :3]@p+T[:3, 3], raw[0, 13:16])
        pre = derive(R, p, .20)
        np.testing.assert_allclose(np.array(pre['T_object_pregrasp'])[:3, 3]-p, -.20*R[:, 0])

    def test_invalid_depth(self):
        with self.assertRaises(ValueError):
            generate(distance_m=.01)
