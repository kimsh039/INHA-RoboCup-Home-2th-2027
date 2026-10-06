"""최종 복귀가 시작 관절각을 계획하고 실제 도달을 검사하는지 확인한다."""
import unittest
from types import SimpleNamespace
from run_grasp_moveit import Demo


class ReturnToInitialTest(unittest.TestCase):
    def demo(self, actual):
        node=SimpleNamespace(cfg={"initial_joints_rad":[0.]*6,
                                  "folded_candidate_rad":[-1.6,.02,-.02,0.,0.,0.],
                                  "arm_goal_tolerance_rad":.01},
                             status={"joint_positions":actual}, goals=[])
        node.move_goal=lambda constraints, cartesian:node.goals.append((constraints,cartesian))
        return node

    def test_final_goal_is_initial_not_legacy_transport(self):
        node=self.demo([0.]*6)
        Demo.return_to_initial(node)
        constraints,cartesian=node.goals[0]
        self.assertEqual([j.position for j in constraints.joint_constraints],[0.]*6)
        self.assertFalse(cartesian)
        self.assertEqual(node.final_return["max_joint_error_rad"],0.)

    def test_forward_facing_legacy_pose_does_not_pass(self):
        node=self.demo([-1.6,.02,-.02,0.,0.,0.])
        with self.assertRaisesRegex(RuntimeError,"RETURN_TO_INITIAL_FAILED"):
            Demo.return_to_initial(node)

    def test_verification_accepts_control_tolerance(self):
        node=self.demo([.002]*6)
        Demo.return_to_initial(node)
        self.assertLess(node.final_return["max_joint_error_rad"],.01)
