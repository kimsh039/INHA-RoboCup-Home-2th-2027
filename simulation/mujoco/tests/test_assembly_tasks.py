import unittest
import json
from pathlib import Path
import numpy as np
from scripts.generate_dummy_pointclouds import generate
from manipulation.placement import place_targets
class AssemblyTasksTest(unittest.TestCase):
    def test_support_face_exclusion(self):
        cfg=json.loads((Path(__file__).resolve().parents[1]/'config/pointclouds.json').read_text())
        for name in ('cube','pringles','can'):
            p,meta=generate(name,cfg)
            bottom=-cfg['objects'][name].get('height_m',cfg['objects'][name].get('dimensions_m',[0,0,0])[2])/2
            self.assertFalse(np.isclose(p[:,2],bottom,atol=1e-8,rtol=0).any())
            self.assertTrue(meta['exclude_support_face'])
        self.assertEqual(cfg['objects']['cube']['dimensions_m'],[.04]*3)
    def test_place_relative_to_rotated_base(self):
        base=np.eye(4);base[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        grasp=np.eye(4);grasp[:3,3]=[.5,0,.74]
        targets=place_targets(grasp,base)
        for (side,T),sign in zip(targets,[1,-1]):
            np.testing.assert_allclose(T[:3,3]-grasp[:3,3],sign*.10*base[:3,1])
            np.testing.assert_allclose(T[:3,:3],grasp[:3,:3])
