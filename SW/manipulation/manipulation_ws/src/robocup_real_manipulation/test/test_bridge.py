"""Checks that real driver adapter rejects unsafe trajectories before publishing."""
import copy
import unittest
import numpy as np
from concurrent.futures import Future
from types import SimpleNamespace
from robocup_real_manipulation.ros_util import action_call
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from test_core import CFG
from robocup_real_manipulation.core import Fault
from robocup_real_manipulation.piper_bridge import trajectory_points,interpolate

class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.c=copy.deepcopy(CFG);self.c['joint_limits']=[[-2,2]]*6
        self.current=np.zeros(6)
    def trajectory(self,end=.01,seconds=1):
        return JointTrajectory(joint_names=self.c['joint_names'],points=[JointTrajectoryPoint(positions=[0.]*6,time_from_start=Duration()),JointTrajectoryPoint(positions=[end]*6,time_from_start=Duration(sec=seconds))])
    def test_aborted_plan_returns_failure_code_for_left_fallback(self):
        def done(value):
            future=Future();future.set_result(value);return future
        payload=SimpleNamespace(error_code=SimpleNamespace(val=-1))
        handle=SimpleNamespace(accepted=True,get_result_async=lambda:done(SimpleNamespace(status=6,result=payload)),cancel_goal_async=lambda:None)
        client=SimpleNamespace(wait_for_server=lambda timeout_sec:True,send_goal_async=lambda goal:done(handle))
        self.assertEqual(action_call(client,None,1,lambda:None,allow_aborted=True).error_code.val,-1)
        with self.assertRaisesRegex(Fault,'ACTION_NOT_SUCCEEDED'):action_call(client,None,1,lambda:None)
    def test_valid_path_interpolation_and_limits(self):
        times,points=trajectory_points(self.trajectory(),self.c,self.current)
        np.testing.assert_allclose(interpolate(times,points,self.current,.5),[.005]*6)
        for t in np.linspace(0,1,10):
            q=interpolate(times,points,self.current,t);self.assertTrue(np.all(q>=0) and np.all(q<=.01))
    def test_nonfinite_names_and_time_rejected(self):
        for traj in (self.trajectory(float('nan')),self.trajectory(seconds=0)):
            with self.assertRaises(Fault):trajectory_points(traj,self.c,self.current)
        traj=self.trajectory();traj.joint_names=list(reversed(traj.joint_names))
        with self.assertRaises(Fault):trajectory_points(traj,self.c,self.current)
    def test_speed_acceleration_and_jump_rejected(self):
        with self.assertRaisesRegex(Fault,'SPEED'):trajectory_points(self.trajectory(.2),self.c,self.current)
        with self.assertRaisesRegex(Fault,'ACCELERATION'):trajectory_points(self.trajectory(.06),self.c,self.current)
        traj=self.trajectory();traj.points[0].positions=[.1]*6
        with self.assertRaisesRegex(Fault,'START'):trajectory_points(traj,self.c,self.current)

if __name__=='__main__':unittest.main()
