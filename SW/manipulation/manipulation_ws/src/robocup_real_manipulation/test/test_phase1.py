import copy
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import tempfile
import unittest
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml
from robocup_real_manipulation.core import Fault, associate
from robocup_real_manipulation.description import prepare
from robocup_real_manipulation.wrist_geometry import surface_point
from robocup_real_manipulation.tuning import overlay

PACKAGE=Path(__file__).resolve().parents[1]
REPO=PACKAGE.parents[4]
CFG=yaml.safe_load((PACKAGE/'config/robot.yaml').read_text())


class Phase1GeometryTests(unittest.TestCase):
    def test_real_phase1_overlay_loads_without_changing_calibration_or_authorization(self):
        tuning=yaml.safe_load((PACKAGE/'config/phase1.tuning.yaml').read_text())
        actual=overlay(CFG,tuning)
        self.assertEqual(actual['max_observation_age_s'],3.)
        self.assertEqual(actual['handeye_transform'],CFG['handeye_transform'])
        self.assertFalse(actual['execution_enabled'])
        self.assertTrue(actual['external_sensor_tf'])
    def test_distorted_real_rgb_ray_is_undistorted_before_depth_projection(self):
        cfg=copy.deepcopy(CFG); cfg['camera_min_depth_m']=.1
        K=np.array([[300.,0,100],[0,300.,100],[0,0,1.]])
        D=np.array([.2,-.04,0,0,0.])
        true=np.array([.075,0.,.3])
        uv,_=cv2.projectPoints(true.reshape(1,3),np.zeros(3),np.zeros(3),K,D)
        x,y=np.rint(uv[0,0]).astype(int)
        mask=np.zeros((220,220),np.uint8); cv2.circle(mask,(x,y),12,1,-1)
        info=NS(width=220,height=220,k=K.ravel(),d=D,distortion_model='plumb_bob')
        p,std=surface_point(np.full(mask.shape,.3),mask,info,cfg,.003)
        np.testing.assert_allclose(p,true,atol=.001)
        self.assertGreaterEqual(std,.003)

    def test_invalid_depth_and_mismatched_camera_grid_are_not_accepted(self):
        info=NS(width=100,height=100,k=[200.,0,50,0,200.,50,0,0,1],d=[],distortion_model='plumb_bob')
        cfg=copy.deepcopy(CFG); cfg['camera_min_depth_m']=.1
        with self.assertRaises(Fault): surface_point(np.zeros((100,100)),np.ones((100,100)),info,cfg,.003)
        info.width=99
        with self.assertRaisesRegex(Fault,'GRID'): surface_point(np.full((100,100),.3),np.ones((100,100)),info,cfg,.003)

    def test_new_view_without_track_identity_uses_all_geometric_candidates(self):
        candidates=[(np.array([.01,0,0]),'','near'),(np.array([.09,0,0]),'','far')]
        self.assertEqual(associate(candidates,np.zeros(3),'head-track',CFG)[0],'near')
        candidates[1]=(np.array([.011,0,0]),'','ambiguous')
        with self.assertRaisesRegex(Fault,'AMBIGUOUS'): associate(candidates,np.zeros(3),'head-track',CFG)
        with self.assertRaises(Fault): associate([(np.zeros(3),'wrong-known-track','wrong')],np.zeros(3),'head-track',CFG)

    def test_export_keeps_collision_meshes_but_does_not_own_public_sensor_children(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch('robocup_real_manipulation.description.validate'):
                prepare(REPO,CFG,Path(directory))
            root=ET.parse(Path(directory)/'robot.urdf').getroot()
            children={joint.find('child').get('link') for joint in root.findall('joint')}
            for frame in ('camera_link','livox_frame','laser_frame'):
                self.assertNotIn(frame,children)
                self.assertIn('collision_'+frame,children)
            camera=root.find("link[@name='collision_camera_link']")
            self.assertTrue(camera.findall('collision'))
            self.assertIn('tcp',children)
            self.assertIn(CFG['wrist_optical_frame'],children)


if __name__=='__main__': unittest.main()
