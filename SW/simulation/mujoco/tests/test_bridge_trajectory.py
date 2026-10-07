"""Verify soft measured-start handling without accepting unsafe later targets."""
import threading
from types import SimpleNamespace
import unittest
import numpy as np
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import GoalResponse
from run_mujoco_moveit_bridge import Bridge
from manipulation.sim_model import JOINTS,load_model

class BridgeTrajectoryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.model,cls.data=load_model()
    def make_bridge(self):
        model=self.model
        data=SimpleNamespace(qpos=self.data.qpos.copy())
        jq=[int(model.joint(n).qposadr[0]) for n in JOINTS]
        data.qpos[jq[1]]=-.00044;data.qpos[jq[2]]=.00079
        return SimpleNamespace(model=model,data=data,jq=jq,lock=threading.RLock(),active=None,failure=None,base_state='STOPPED',get_logger=lambda:SimpleNamespace(error=lambda _:None))
    def goal(self,bridge):
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=JOINTS
        start=JointTrajectoryPoint(positions=bridge.data.qpos[bridge.jq].tolist(),time_from_start=Duration(sec=0))
        end=JointTrajectoryPoint(positions=[0.]*6,time_from_start=Duration(sec=1))
        goal.trajectory.points=[start,end];return goal
    def test_measured_soft_limit_start_is_accepted(self):
        bridge=self.make_bridge();self.assertEqual(Bridge.accept(bridge,self.goal(bridge)),GoalResponse.ACCEPT)
    def test_later_out_of_limit_target_is_rejected(self):
        bridge=self.make_bridge();goal=self.goal(bridge);goal.trajectory.points[1].positions[1]=-.00044
        self.assertEqual(Bridge.accept(bridge,goal),GoalResponse.REJECT)
    def test_excessive_start_violation_is_rejected(self):
        bridge=self.make_bridge();goal=self.goal(bridge);goal.trajectory.points[0].positions[1]=-.01
        self.assertEqual(Bridge.accept(bridge,goal),GoalResponse.REJECT)
