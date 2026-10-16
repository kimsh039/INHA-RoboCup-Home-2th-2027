"""Explicit test traffic only. Never run on the robot DDS domain."""
import uuid
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from robocup_real_manipulation_msgs.msg import Target, ApproachResult, BaseLease, WristObservation, ObservationRequest
from robocup_real_manipulation_msgs.action import Manipulate
from .ros_util import spin

class Sender(Node):
    def __init__(self):
        if os.environ.get('ROS_DOMAIN_ID')!='187': raise RuntimeError('TEST_REQUIRES_ISOLATED_DOMAIN_187')
        super().__init__('manipulation_test_sender')
        self.declare_parameter('approach_first',False); self.declare_parameter('approach_outcome','SUCCESS')
        self.declare_parameter('base_stopped',True); self.declare_parameter('scenario','normal')
        self.task=uuid.uuid4().hex; self.lock=''; self.lease_task=''; self.pending=[]; self.result=None; self.started=False
        self.target=self.create_publisher(Target,'/manipulation/target',10)
        self.approach=self.create_publisher(ApproachResult,'/manipulation/approach_result',10)
        self.lease=self.create_publisher(BaseLease,'/manipulation/base_lease',10)
        self.joint=self.create_publisher(JointState,'/joint_states',10)
        self.wrist=self.create_publisher(WristObservation,'/manipulation/wrist/result',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(BaseLease,'/manipulation/base_lock_request',self.lock_request,qos)
        self.create_subscription(ObservationRequest,'/manipulation/wrist/request',self.observe,10)
        self.client=ActionClient(self,Manipulate,'/manipulation/run')
        self.create_timer(.05,self.tick)
        self.get_logger().warning('TEST INPUTS ONLY: isolate ROS_DOMAIN_ID from real robot')
    def lock_request(self,msg):
        self.lock=msg.task_id if msg.locked else ''
        self.lease_task=msg.task_id
    def stamp(self,msg): msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id='base_link'; return msg
    def observe(self,req):
        if self.get_parameter('scenario').value=='wrist_timeout': return
        self.pending.extend([req]*3)
    def tick(self):
        if self.lease_task and not (self.get_parameter('scenario').value=='unlock_missing' and not self.lock):
            self.lease.publish(self.stamp(BaseLease(task_id=self.lease_task,locked=bool(self.lock),stopped=self.get_parameter('base_stopped').value)))
        self.joint.publish(self.stamp(JointState(name=[f'piper_joint{i}' for i in range(1,7)],position=[0.]*6,velocity=[0.]*6)))
        if self.pending:
            req=self.pending.pop(0)
            msg=WristObservation(task_id=req.task_id,observation_id=req.observation_id,class_id=req.class_id,instance_id=req.instance_id,
                                 candidate_count=1,candidate_index=0,confidence=.9,position_kind='CENTER',has_position=True,position_std_m=.002)
            msg.position.x=.40; msg.position.y=0.; msg.position.z=.74
            self.stamp(msg)
            scenario=self.get_parameter('scenario').value
            if scenario=='stale_observation': msg.header.stamp.sec-=10
            if scenario=='missing_tf': msg.header.frame_id='missing_camera_frame'
            if scenario=='invalid_depth':
                import numpy as np
                from cv_bridge import CvBridge
                from sensor_msgs.msg import CameraInfo
                msg.has_position=False;msg.position_kind='SURFACE'
                msg.aligned_depth=CvBridge().cv2_to_imgmsg(np.zeros((40,40),np.float32),encoding='32FC1');msg.aligned_depth.header=msg.header
                msg.target_mask=CvBridge().cv2_to_imgmsg(np.full((40,40),255,np.uint8),encoding='mono8');msg.target_mask.header=msg.header
                msg.camera_info=CameraInfo(header=msg.header,width=40,height=40,k=[100.,0.,20.,0.,100.,20.,0.,0.,1.])
            if scenario=='ambiguous':
                import copy
                msg.candidate_count=2
                other=copy.deepcopy(msg);other.candidate_index=1;other.position.x=.41
                self.wrist.publish(msg);self.wrist.publish(other)
            else: self.wrist.publish(msg)
        if not self.started and self.client.server_is_ready() and self.target.get_subscription_count()>0 and self.approach.get_subscription_count()>0:
            self.started=True
            t=Target(task_id=self.task,class_id=0,instance_id='' if self.get_parameter('scenario').value=='ambiguous' else 'test_apple'); t.coarse_position.x=.4; t.coarse_position.z=.74; self.stamp(t)
            a=self.stamp(ApproachResult(task_id=self.task,outcome=self.get_parameter('approach_outcome').value,base_stopped=self.get_parameter('base_stopped').value))
            if self.get_parameter('approach_first').value: self.approach.publish(a); self.target.publish(t)
            else: self.target.publish(t); self.approach.publish(a)
            f=self.client.send_goal_async(Manipulate.Goal(task_id=self.task)); f.add_done_callback(self.accepted)
    def accepted(self,f):
        h=f.result()
        if not h.accepted: self.get_logger().error('TEST GOAL REJECTED'); return
        if self.get_parameter('scenario').value=='cancel': h.cancel_goal_async()
        h.get_result_async().add_done_callback(self.finished)
    def finished(self,f):
        self.result=f.result().result.status
        self.get_logger().info(f'TEST RESULT {self.result.outcome} stage={self.result.stage} holding={self.result.holding} place={self.result.place_success} return={self.result.return_success} reason={self.result.reason}')

def main():
    rclpy.init(); spin(Sender())
