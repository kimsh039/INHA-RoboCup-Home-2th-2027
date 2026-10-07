import math
import unittest
from pathlib import Path
import numpy as np
from robocup_head_detection.arm_kinematics import (Chain, ObservationConfig, Table, clearance, joint_path, look_at_ik,
                                                   observation_pose)

URDF = Path(__file__).resolve().parents[3]/'simulation'/'robot_description'/'robocup.urdf'
TABLE_Z = 0.72 - 0.1425          # table top in base_link (base_link is 0.1425 m above the floor)


def table_ahead(edge, depth=0.8, half_width=0.8, z=TABLE_Z):
    """Table whose near edge is `edge` m ahead of base_link; CCW corners."""
    return Table(((edge, half_width), (edge, -half_width), (edge+depth, -half_width), (edge+depth, half_width)), z)


class ArmKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.chain = Chain(URDF.read_text(), 'wrist_camera_optical_frame')

    def test_chain_joints_and_limits(self):
        self.assertEqual(self.chain.names, [f'piper_joint{i}' for i in range(1, 7)])
        self.assertEqual(self.chain.limits.shape, (6, 2))
        self.assertTrue(np.all(self.chain.limits[:, 0] < self.chain.limits[:, 1]))

    def test_table_orientation(self):
        t = table_ahead(0.2)
        self.assertTrue(t.over((0.5, 0.0), 0.0))
        self.assertFalse(t.over((0.1, 0.0), 0.0))
        clockwise = Table(t.corners[::-1], t.z)       # wrong winding must not pass silently
        self.assertFalse(clockwise.over((0.5, 0.0), 0.0))

    def test_look_at_reaches_pose(self):
        target = np.array([0.40, -0.06, TABLE_Z + 0.04])
        camera = target + np.array([0, 0, 0.30])
        for seed in (np.zeros(6), *ObservationConfig().seeds):   # current joints first, as the planner does
            q, pos_err, ang_err = look_at_ik(self.chain, camera, (0, 0, -1), seed)
            if pos_err < 2e-3 and ang_err < 1e-2:
                break
        self.assertLess(pos_err, 2e-3)
        self.assertLess(ang_err, 1e-2)
        T = self.chain.fk(q)
        np.testing.assert_allclose(T[:3, 3], camera, atol=2e-3)
        self.assertTrue(np.all(q >= self.chain.limits[:, 0]-1e-9) and np.all(q <= self.chain.limits[:, 1]+1e-9))

    def test_observation_pose_for_docked_target(self):
        table = table_ahead(0.2)
        target = (0.407, -0.062, TABLE_Z + 0.04)
        q, info = observation_pose(self.chain, target, table, np.zeros(6))
        self.assertIsNotNone(q, info)
        T = self.chain.fk(q)
        view = np.asarray(target) - T[:3, 3]
        self.assertLess(math.acos(T[:3, 2] @ view/np.linalg.norm(view)), 0.02)   # optical axis on the target
        self.assertGreaterEqual(info['clearance'], ObservationConfig().min_clearance)
        self.assertIsNotNone(joint_path(self.chain, np.zeros(6), q, table))

    def test_clearance_rejects_arm_below_table_top(self):
        table = table_ahead(0.2, z=1.2)                # a "table" above the arm's working height
        cfg = ObservationConfig(distances=(0.3,), elevations=(math.radians(90), math.radians(60)))
        q, info = observation_pose(self.chain, (0.4, 0.0, 1.0), table, np.zeros(6), cfg)
        self.assertIsNone(q)
        self.assertEqual(info, 'NO_OBSERVATION_POSE')


if __name__ == '__main__':
    unittest.main()
