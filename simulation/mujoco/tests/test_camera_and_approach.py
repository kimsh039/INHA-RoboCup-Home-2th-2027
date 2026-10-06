import unittest
import numpy as np
from manipulation.synthetic_camera import camera_transform,grasps_camera_to_object
from manipulation.approach_checks import segment_intersects_box
class CameraApproachTest(unittest.TestCase):
    def test_grasp_frame_inverse_and_camera_forward(self):
        T=camera_transform([.3,-.3,.35]);np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(T[:3,:3]),1)
        self.assertGreater(T[2,3],0)
        raw=np.zeros((1,17));raw[0,:4]=[.5,.04,.02,.04]
        raw[0,4:13]=T[:3,:3].reshape(-1);p=np.array([.01,.02,.03]);raw[0,13:16]=T[:3,:3]@p+T[:3,3]
        back=grasps_camera_to_object(raw,T);np.testing.assert_allclose(back[0,4:13].reshape(3,3),np.eye(3),atol=1e-12);np.testing.assert_allclose(back[0,13:16],p,atol=1e-12);np.testing.assert_equal(back[0,:4],raw[0,:4])
    def test_table_crossing_and_clear_approach(self):
        self.assertTrue(segment_intersects_box([.5,0,.6],[.5,0,.75],[.68,0,.695],[.30,.45,.025]))
        self.assertFalse(segment_intersects_box([.5,0,.95],[.5,0,.75],[.68,0,.695],[.30,.45,.025]))
