import unittest
import numpy as np
from scripts.diagnose_table_grasps import obb_intersects_aabb
class TableGraspGeometryTest(unittest.TestCase):
    def test_overlap_and_separation(self):
        self.assertTrue(obb_intersects_aabb(np.array([0,0,.01]),np.eye(3),np.array([.02]*3),np.zeros(3),np.array([.1,.1,.025])))
        self.assertFalse(obb_intersects_aabb(np.array([0,0,.06]),np.eye(3),np.array([.02]*3),np.zeros(3),np.array([.1,.1,.025])))
    def test_rotated_box_radius(self):
        a=np.pi/4;R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
        self.assertTrue(obb_intersects_aabb(np.array([0,0,.04]),R,np.array([.04,.005,.005]),np.zeros(3),np.array([.1,.1,.025])))
