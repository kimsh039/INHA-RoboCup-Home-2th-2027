"""Contract-only mock: no driver publishers, SDK calls or motion execution."""
import time
import numpy as np
import rclpy
from rclpy.action import GoalResponse
from .node import Coordinator
from .core import require
from .ros_util import spin

class MockMotion:
    right_blocked=False
    def pose(self,p,q):
        from geometry_msgs.msg import Pose
        msg=Pose(); msg.position.x,msg.position.y,msg.position.z=map(float,p)
        msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w=map(float,q);return msg
    def setup_scene(self,*args,**kwargs): pass
    def contacts(self,*args): pass
    def attach(self,*args): pass
    def detach(self,*args): pass
    def joint_move(self,joints,guard): guard()
    def pose_move(self,p,q,guard): guard()
    def plan_to(self,p,q,guard):
        guard();require(not self.right_blocked or p[1]>=0.,'PLAN_FAILED');return True
    def cart_plan(self,p,q,guard,start=None): guard(); return True
    def cart_move(self,p,q,guard): guard()
    def execute(self,traj,guard): guard()

class MockCoordinator(Coordinator):
    def __init__(self):
        super().__init__(); require(self.c.get('fixture_only') is True,'MOCK_REQUIRES_TEST_FIXTURE')
        self.motion=MockMotion(); self.declare_parameter('fail_stage',''); self.declare_parameter('right_blocked',False)
        self.motion.right_blocked=self.get_parameter('right_blocked').value
        self.get_logger().warning('MOCK: all motion results are synthetic; no hardware commands')
    def accept(self,goal):
        with self.mutex:
            try:
                require(not self.locked_task,'RECOVERY_LOCK_ACTIVE'); self.join.begin(goal.task_id)
                self.locked_task=goal.task_id;self.persist();self.lease=None;return GoalResponse.ACCEPT
            except Exception: return GoalResponse.REJECT
    def guard(self):
        # Preserve sequence/base/joint checks, skip physical contact sensor only.
        holding=self.seq.holding if self.seq else None
        if self.seq and holding=='HELD': self.seq.holding='UNKNOWN'
        try: super().guard()
        finally:
            if self.seq and holding=='HELD': self.seq.holding=holding
    def to_gravity(self,p,header,target_stamp=None):
        if header.frame_id!='base_link': return super().to_gravity(p,header,target_stamp)
        return np.array(p)
    def width_move(self,width):
        from robocup_real_manipulation_msgs.action import SetWidth
        self.guard();self.gripper_result=SetWidth.Result(success=True,measured_width_m=width,effort=1.)
    def stop(self): return True
    def perform(self,stage,seq):
        require(stage!=self.get_parameter('fail_stage').value,'MOCK_STAGE_FAILURE:'+stage)
        super().perform(stage,seq)

def main():
    rclpy.init();spin(MockCoordinator())
