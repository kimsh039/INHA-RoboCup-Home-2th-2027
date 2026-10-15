import copy
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import cv2
import numpy as np
import yaml
from robocup_real_manipulation.wrist_geometry import banana_point_axis
from robocup_real_manipulation.core import Fault, rotation, grasp_geometry, verify_axis_series

PACKAGE=Path(__file__).resolve().parents[1]
CFG=yaml.safe_load((PACKAGE/'config/robot.yaml').read_text())
SETTINGS=yaml.safe_load((PACKAGE/'config/wrist.yaml').read_text())['banana']


class BananaTests(unittest.TestCase):
    def arc(self):
        mask=np.zeros((240,240),np.uint8)
        cv2.ellipse(mask,(120,120),(65,85),0,50,310,255,22)
        depth=np.where(mask!=0,.3,np.nan)
        info=NS(width=240,height=240,k=[500.,0.,120.,0.,500.,120.,0.,0.,1.],d=[],distortion_model='plumb_bob')
        cfg=copy.deepcopy(CFG); cfg['camera_min_depth_m']=.2
        cfg['classes']['banana']['radius_m']=.3
        return mask,depth,info,cfg

    def test_concave_centroid_is_not_used_and_depth_is_taken_from_real_body(self):
        mask,depth,info,cfg=self.arc()
        ys,xs=np.nonzero(mask)
        self.assertFalse(mask[int(round(ys.mean())),int(round(xs.mean()))])
        p,std,yaw,details=banana_point_axis(depth,mask,info,cfg,.003,np.diag([1.,-1.,-1.]),SETTINGS)
        u,v=details['body_pixel']
        self.assertTrue(mask[v,u])
        actual_u=int(round(p[0]/p[2]*500+120)); actual_v=int(round(p[1]/p[2]*500+120))
        self.assertTrue(mask[actual_v,actual_u])
        self.assertAlmostEqual(p[2],.3)
        self.assertGreaterEqual(std,.003)
        self.assertGreater(abs(math.sin(yaw)),.8)

    def test_axis_is_transformed_to_robot_plane_and_uint8_sam_mask_is_supported(self):
        mask,depth,info,cfg=self.arc()
        R=rotation([0,0,math.sin(.3),math.cos(.3)])@np.diag([1.,-1.,-1.])
        a=banana_point_axis(depth,mask,info,cfg,.003,np.diag([1.,-1.,-1.]),SETTINGS)[2]
        b=banana_point_axis(depth,mask,info,cfg,.003,R,SETTINGS)[2]
        self.assertAlmostEqual(abs(math.cos(b-a-.6)),1.,places=6)

    def test_multiple_components_round_shape_and_bad_depth_fail_without_grasp(self):
        mask,depth,info,cfg=self.arc()
        split=np.zeros(mask.shape,np.uint8)
        cv2.rectangle(split,(20,40),(70,180),255,-1); cv2.rectangle(split,(150,40),(200,180),255,-1)
        with self.assertRaisesRegex(Fault,'SPLIT'): banana_point_axis(depth,split,info,cfg,.003,np.diag([1.,-1.,-1.]),SETTINGS)
        circle=np.zeros(mask.shape,np.uint8); cv2.circle(circle,(120,120),50,255,-1)
        with self.assertRaisesRegex(Fault,'PCA_AMBIGUOUS'): banana_point_axis(depth,circle,info,cfg,.003,np.diag([1.,-1.,-1.]),SETTINGS)
        with self.assertRaises(Fault): banana_point_axis(np.full(mask.shape,np.nan),mask,info,cfg,.003,np.diag([1.,-1.,-1.]),SETTINGS)

    def test_entire_banana_must_fit_configured_collision_radius_from_body_point(self):
        mask,depth,info,cfg=self.arc(); cfg['classes']['banana']['radius_m']=.005
        with self.assertRaisesRegex(Fault,'RADIUS'): banana_point_axis(depth,mask,info,cfg,.003,np.diag([1.,-1.,-1.]),SETTINGS)

    def test_cad_tcp_y_closure_is_perpendicular_to_pca_at_selected_point(self):
        cfg=copy.deepcopy(CFG); cfg['table_z_m']=.75
        rule=cfg['classes']['banana']; rule.update(center_height_m=.0135,grip_height_m=.0135)
        _,tcp,q=grasp_geometry(np.array([.5,.1,.777]),'SURFACE',rule,cfg,.6)
        closure=rotation(q)[:2,1]
        self.assertAlmostEqual(np.dot(closure,[math.cos(.6),math.sin(.6)]),0.,places=6)
        np.testing.assert_allclose(tcp[:2],[.5,.1])

    def test_three_axes_allow_sign_flip_but_reject_direction_instability(self):
        def q(angle):
            return np.array([math.cos(angle/2),math.sin(angle/2),0.,0.])
        verify_axis_series([q(.2),q(.21),q(.2+math.pi)],.15)
        with self.assertRaisesRegex(Fault,'UNSTABLE'): verify_axis_series([q(0),q(.7),q(-.6)],.15)


if __name__=='__main__': unittest.main()
