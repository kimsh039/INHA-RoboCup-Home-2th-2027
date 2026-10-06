import unittest
import numpy as np
from manipulation.sim_model import target_tcp,transform


class SimFramesTest(unittest.TestCase):
    def test_grasp_depth_tip_mapping_and_world_composition(self):
        R=np.array([[0,-1,0],[1,0,0],[0,0,1]],dtype=float)
        grasp={"translation":[.01,.02,.03],"rotation_matrix":R.tolist(),"depth":.04}
        world=transform([.3,-.1,.73],R)
        target=target_tcp(grasp,world)
        pre=target_tcp(grasp,world,.2)
        expected=world@transform(np.array(grasp["translation"])+.04*R[:,0],R)
        np.testing.assert_allclose(target,expected,atol=1e-12)
        np.testing.assert_allclose(target[:3,3]-pre[:3,3],.2*target[:3,0],atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(target[:3,3]-pre[:3,3]),.2)

    def test_reject_reflected_rotation(self):
        with self.assertRaises(ValueError):
            target_tcp({"translation":[0,0,0],"rotation_matrix":np.diag([1,1,-1]),"depth":.04},np.eye(4))


if __name__=="__main__":unittest.main()
