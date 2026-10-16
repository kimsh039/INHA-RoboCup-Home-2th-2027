"""Offline projection cases; no camera or ROS process is started."""
import sys
from pathlib import Path
import unittest
from types import SimpleNamespace
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from camera_frustum import in_frustum
from detection_approach import DetectionApproach


class CameraFrustumTests(unittest.TestCase):
    def test_distorted_border_rays_survive_and_side_wall_is_culled(self):
        k=[450.,0,320,0,450,240,0,0,1]; d=[-.15,.03,.001,-.002,0.]
        x,y=np.meshgrid(np.linspace(0,639,31),np.linspace(0,479,21))
        pixels=np.column_stack([x.ravel(),y.ravel()])
        rays=cv2.undistortPoints(pixels.reshape(-1,1,2),np.array(k).reshape(3,3),np.array(d)).reshape(-1,2)
        camera=np.column_stack([rays,np.ones(len(rays))])
        self.assertTrue(in_frustum(camera,640,480,k,d).all())
        outside=np.array([[3,0,1],[0,3,1],[0,0,-1],[0,0,.1],[0,0,6],[float('nan'),0,1]])
        self.assertFalse(in_frustum(outside,640,480,k,d).any())

    def test_calibration_changes_do_not_reuse_old_cone(self):
        camera=np.array([[.6,0,1.]])
        wide=[400.,0,320,0,400,240,0,0,1]
        tele=[1000.,0,320,0,1000,240,0,0,1]
        self.assertTrue(in_frustum(camera,640,480,wide,[])[0])
        self.assertFalse(in_frustum(camera,640,480,tele,[])[0])

    def test_stopped_docking_hands_off_measured_surface(self):
        from geometry_msgs.msg import PolygonStamped, Point32
        from builtin_interfaces.msg import Time
        published=[]; reports=[]
        node=SimpleNamespace(
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=10))))
        approach=SimpleNamespace(nav_busy=False,anchor={},relative=lambda _:np.array([.4,0,0]),
            plan={'dock':[0,0,0],'surface':{'corners':[[.3,-.5],[1.,-.5],[1.,.5],[.3,.5]],'z':.7}},
            finish_request={'kind':'dock','phase':'succeeded','message':'done'},
            planner=SimpleNamespace(errors=lambda pose,plan:(0,0,0)),
            grasp_gap_satisfied=lambda:True,heading_tolerance=.035,command=lambda *args:None,
            arrived_since=.1,active=True,PolygonStamped=PolygonStamped,Point32=Point32,
            node=node,surface_pub=SimpleNamespace(publish=published.append),
            report=lambda *args:reports.append(args))
        DetectionApproach.advance_heading(approach,1.,[0,0,0],{'linear':0.,'angular':0.})
        self.assertFalse(approach.active)
        self.assertEqual(published[0].header.frame_id,'odom')
        self.assertEqual(published[0].header.stamp.sec,10)
        self.assertEqual(len(published[0].polygon.points),4)
        self.assertEqual(reports[-1][0],'succeeded')


if __name__=='__main__': unittest.main()
