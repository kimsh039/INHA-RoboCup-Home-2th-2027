import copy,unittest
import numpy as np
from manipulation.sim_model import config,load_model,body_transform
from manipulation.observation import top_down_pose,apply_observation
from manipulation.wrist_camera_cloud import scan_cube
from manipulation.mid360_cloud import apply_transform,validate_scene_metadata,validate_scene_status


class WristCameraTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m,cls.d=load_model();cls.initial=cls.d.qpos.copy()
        cls.plan=top_down_pose(cls.m,cls.d)
        np.testing.assert_array_equal(cls.d.qpos,cls.initial)
        apply_observation(cls.m,cls.d,cls.plan)
        cls.points,cls.camera,cls.network,cls.meta=scan_cube(cls.m,cls.d,config(),64)

    def test_actual_mounted_camera_top_down_at_40cm(self):
        T=body_transform(self.m,self.d,'wrist_camera_optical_frame');O=body_transform(self.m,self.d,'cube')
        np.testing.assert_allclose(T[:3,2],[0,0,-1],atol=1e-6)
        np.testing.assert_allclose(T[:3,3],O[:3,3]+[0,0,.30],atol=1e-6)
        self.assertAlmostEqual(self.meta['base_to_object_horizontal_distance_m'],.4,places=6)
        self.assertFalse(self.plan['motion_path_checked'])
        self.assertFalse(any(c.dist<-.001 for c in self.d.contact))

    def test_only_top_surface_and_optical_frame_roundtrip(self):
        self.assertGreater(len(self.points),1000)
        np.testing.assert_allclose(self.points[:,2],.02,atol=1e-6)
        np.testing.assert_allclose(apply_transform(np.array(self.meta['T_network_object']),self.points),self.camera,atol=1e-7)
        np.testing.assert_array_equal(self.camera,self.network)
        self.assertTrue(np.all(np.abs(self.points[:,:2])<=.020001))
        self.assertTrue(np.all(self.camera[:,2]>=self.meta['sensor']['near_m']))

    def test_first_hit_excludes_robot_and_table(self):
        import mujoco
        T=np.array(self.meta['T_world_camera']);g=np.array([-1],dtype=np.int32)
        for p in self.camera[::max(1,len(self.camera)//30)]:
            distance=np.linalg.norm(p)
            hit=mujoco.mj_ray(self.m,self.d,T[:3,3],T[:3,:3]@(p/distance),
                np.array([1,0,0,1,0,0],dtype=np.uint8),True,self.m.body('wrist_camera_cad_link').id,g)
            self.assertEqual(g[0],self.m.geom('cube_geom').id)
            self.assertAlmostEqual(hit,distance,places=6)

    def test_initial_camera_cannot_be_silently_aimed(self):
        import mujoco
        scratch=mujoco.MjData(self.m);scratch.qpos[:]=self.initial;mujoco.mj_forward(self.m,scratch)
        with self.assertRaisesRegex(ValueError,'OUTSIDE_FOV|BEHIND_CAMERA'):
            scan_cube(self.m,scratch,config(),16)

    def test_stale_scene_rejected(self):
        validate_scene_metadata(self.meta)
        status={k:copy.deepcopy(self.meta[k]) for k in ('T_world_base','T_world_object')}
        validate_scene_status(self.meta,status)
        status['T_world_base'][0][3]+=.1
        with self.assertRaisesRegex(ValueError,'POSE_MISMATCH'):validate_scene_status(self.meta,status)


if __name__=='__main__':unittest.main()
