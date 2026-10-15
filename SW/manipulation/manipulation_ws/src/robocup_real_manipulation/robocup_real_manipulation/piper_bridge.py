"""Physical-only adapter for checked piper_ros humble JointState interface.

Never enables motors. Sends complete 7-position commands: upstream missing joints
otherwise default to zero! A single bridge owns both arm and width actions.
"""
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from piper_msgs.msg import PiperStatusMsg
from std_srvs.srv import Trigger
from robocup_real_manipulation_msgs.msg import BaseLease
from robocup_real_manipulation_msgs.action import SetWidth
from .core import require, fresh, finite, validate, width_to_driver
from .ros_util import read_config, seconds, spin
from .startup_bounds import feedback_in_bounds, check_trajectory_bounds

class PiperBridge(Node):
    def __init__(self):
        super().__init__('real_piper_bridge'); self.c=validate(read_config(self),hardware=True)
        self.group=ReentrantCallbackGroup(); self.mutex=threading.RLock(); self.busy=False; self.tripped=False
        self.q=None; self.v=None; self.width=None; self.effort=None; self.joint_stamp=0.; self.grip_stamp=0.; self.lease=None
        self.status=None; self.status_received=0.; self.command_started=False
        self.last_command=None; self.active_task=None
        self.command_pub=self.create_publisher(JointState,self.c['command_topic'],1)
        self.state_pub=self.create_publisher(JointState,'/joint_states',10)
        self.create_subscription(JointState,self.c['feedback_topic'],self.feedback,qos_profile_sensor_data,callback_group=self.group)
        self.create_subscription(JointState,self.c['gripper_feedback_topic'],self.gripper_feedback,qos_profile_sensor_data,callback_group=self.group)
        self.create_subscription(PiperStatusMsg,self.c['arm_status_topic'],self.arm_status,10,callback_group=self.group)
        self.create_subscription(BaseLease,self.c['base_lease_topic'],self.base,10,callback_group=self.group)
        self.arm=ActionServer(self,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory',self.execute_arm,
                             goal_callback=self.accept_arm,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.gripper=ActionServer(self,SetWidth,'/manipulation/set_width',self.execute_gripper,
                                 goal_callback=self.accept_width,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.create_service(Trigger,'/manipulation/stop',self.stop_service,callback_group=self.group)
        self.create_timer(.02,self.watchdog,callback_group=self.group)
    def now(self): return self.get_clock().now().nanoseconds/1e9
    def feedback(self,msg):
        try:
            require(len(msg.name)==len(msg.position) and len(set(msg.name))==len(msg.name),'INVALID_JOINT_STATE')
            raw=dict(zip(msg.name,msg.position)); q=np.array([raw[n] for n in self.c['driver_joint_names']])*self.c['joint_signs']+self.c['joint_offsets']
            stamp=seconds(msg.header.stamp); fresh(stamp,self.now(),self.c['feedback_age_s'])
            require(finite(q),'NONFINITE_JOINTS')
            require(feedback_in_bounds(q,self.c),'JOINT_FEEDBACK_LIMIT:'+str(q.tolist()))
            with self.mutex:
                # Drop reordered samples. The watchdog still requires newer,
                # genuinely fresh feedback; never rewrite an acquisition stamp.
                if stamp<self.joint_stamp: return
                if stamp==self.joint_stamp: return
                self.v=(q-self.q)/(stamp-self.joint_stamp) if self.q is not None else None
                self.q=q; self.joint_stamp=stamp
                self.publish_state(msg.header)
        except Exception as e:
            if self.busy: self.get_logger().error('BRIDGE_TRIP_CAUSE: '+__import__('traceback').format_exc()+'\n'+''.join(__import__('traceback').format_stack(limit=4))); self.tripped=True
            age=self.now()-seconds(msg.header.stamp)
            self.get_logger().warning(f'Arm feedback rejected: {e}; age={age:.3f}s limit={self.c["feedback_age_s"]:.3f}s',throttle_duration_sec=5.)
    def gripper_feedback(self,msg):
        try:
            require(msg.name==['gripper'] and len(msg.position)==1 and len(msg.effort)==1,'GRIPPER_FEEDBACK')
            stamp=seconds(msg.header.stamp); fresh(stamp,self.now(),self.c['feedback_age_s'])
            require(finite(msg.position+msg.effort),'GRIPPER_NONFINITE')
            with self.mutex:
                if stamp<self.grip_stamp: return
                if stamp==self.grip_stamp: return
                self.width=msg.position[0]*self.c['width_scale']+self.c['width_bias_m']
                self.effort=msg.effort[0]; self.grip_stamp=stamp
                require(0<=self.width<=self.c['gripper_max_m']+.002,'GRIPPER_RANGE')
        except Exception:
            if self.busy: self.get_logger().error('BRIDGE_TRIP_CAUSE: '+__import__('traceback').format_exc()+'\n'+''.join(__import__('traceback').format_stack(limit=4))); self.tripped=True
    def publish_state(self,header):
        if self.width is None or self.v is None: return
        fresh(self.grip_stamp,self.now(),self.c['feedback_age_s'])
        self.state_pub.publish(JointState(header=header,name=self.c['joint_names']+self.c['gripper_joint_names'],
                               position=self.q.tolist()+[s*self.width/2 for s in self.c['gripper_joint_signs']],
                               velocity=self.v.tolist()+[0.,0.]))
    def arm_status(self,msg): self.status=msg; self.status_received=time.monotonic()
    def base(self,msg): self.lease=msg
    def valid(self):
        c=self.c; require(not self.tripped,'BRIDGE_TRIPPED')
        fresh(self.joint_stamp,self.now(),c['feedback_age_s']); fresh(self.grip_stamp,self.now(),c['feedback_age_s'])
        require(self.q is not None and self.v is not None and self.width is not None,'FEEDBACK_NOT_READY')
        require(self.status is not None and time.monotonic()-self.status_received<c['feedback_age_s'] and self.status.err_code==0,'PIPER_STATUS_STALE_OR_FAULT')
        for i in range(1,7):
            require(not getattr(self.status,f'joint_{i}_angle_limit') and not getattr(self.status,f'communication_status_joint_{i}'),'PIPER_JOINT_FAULT')
        lease=self.lease
        require(lease and lease.task_id and lease.locked and lease.stopped and (self.active_task is None or lease.task_id==self.active_task),'BASE_INTERLOCK')
        fresh(seconds(lease.header.stamp),self.now(),c['base_age_s'])
    def reserve(self):
        with self.mutex:
            try:
                self.valid(); require(not self.busy,'DRIVER_BUSY'); self.busy=True; self.active_task=self.lease.task_id; return True
            except Exception as e: self.get_logger().warning(str(e)); return False
    def accept_arm(self,goal):
        try:
            trajectory_points(goal.trajectory,self.c,self.q)
            return GoalResponse.ACCEPT if self.reserve() else GoalResponse.REJECT
        except Exception as e: self.get_logger().warning(str(e)); return GoalResponse.REJECT
    def accept_width(self,goal):
        try:
            limits=np.array(self.c['joint_limits'])
            require(self.q is not None and np.all(self.q>=limits[:,0]) and np.all(self.q<=limits[:,1]),'INITIAL_POSE_REQUIRED')
            width_to_driver(goal.width_m,self.c)
            require(0<goal.timeout_s<=self.c['stage_timeout_s'],'WIDTH_TIMEOUT')
            return GoalResponse.ACCEPT if self.reserve() else GoalResponse.REJECT
        except Exception as e: self.get_logger().warning(str(e)); return GoalResponse.REJECT
    def publish(self,q,width,hold=False):
        if not hold: self.valid()
        else:
            fresh(self.joint_stamp,self.now(),self.c['feedback_age_s']); fresh(self.grip_stamp,self.now(),self.c['feedback_age_s'])
        raw=(np.array(q)-self.c['joint_offsets'])/self.c['joint_signs']; gap=width_to_driver(width,self.c)
        msg=JointState(); msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=self.c['driver_joint_names']+['gripper']; msg.position=raw.tolist()+[float(gap)]
        # Upstream velocity[6] selects global speed percent, NOT a gripper speed.
        msg.velocity=[0.]*6+[float(self.c['driver_speed_percent'])]
        msg.effort=[0.]*6+[self.c['gripper_command_effort']]
        self.command_pub.publish(msg); self.last_command=np.array(q); self.command_started=True
    def hold(self):
        with self.mutex: self.publish(self.q.copy(),self.width,hold=True)
    def stopped(self):
        return self.v is not None and max(abs(self.v))<self.c['stable_speed_rad_s']
    def confirm_stop(self):
        self.get_logger().error('BRIDGE_TRIP_CAUSE: '+__import__('traceback').format_exc()+'\n'+''.join(__import__('traceback').format_stack(limit=4))); self.tripped=True
        try:
            self.hold(); deadline=time.monotonic()+self.c['stop_timeout_s']; since=None
            while time.monotonic()<deadline:
                fresh(self.joint_stamp,self.now(),self.c['feedback_age_s'])
                if self.stopped():
                    if since is None: since=time.monotonic()
                    if time.monotonic()-since>=self.c['joint_stable_s']: return True
                else: since=None
                time.sleep(.01)
        except Exception: pass
        return False
    def stop_service(self,request,response):
        response.success=self.confirm_stop(); response.message='HOLD_CONFIRMED' if response.success else 'STOP_UNCONFIRMED_PHYSICAL_ESTOP_REQUIRED'
        return response
    def watchdog(self):
        if not self.busy: return
        try: self.valid()
        except Exception:
            self.get_logger().error('BRIDGE_TRIP_CAUSE: '+__import__('traceback').format_exc()+'\n'+''.join(__import__('traceback').format_stack(limit=4))); self.tripped=True
            try: self.hold()
            except Exception: pass
    def guard(self,handle,deadline):
        require(not handle.is_cancel_requested,'CANCELED'); self.valid(); require(time.monotonic()<deadline,'MOTION_TIMEOUT')
    def execute_arm(self,handle):
        result=FollowJointTrajectory.Result(); deadline=time.monotonic()+self.c['stage_timeout_s']
        try:
            times,points=trajectory_points(handle.request.trajectory,self.c,self.q)
            q0=self.q.copy(); start=time.monotonic(); width=self.width
            while True:
                self.guard(handle,deadline); elapsed=time.monotonic()-start
                q=interpolate(times,points,q0,elapsed)
                require(max(abs(self.q-q))<=self.c['path_tolerance_rad'],'PATH_TOLERANCE')
                self.publish(q,width)
                fb=FollowJointTrajectory.Feedback(); fb.joint_names=self.c['joint_names']; fb.header.stamp=self.get_clock().now().to_msg()
                fb.actual.positions=self.q.tolist(); fb.desired.positions=q.tolist(); fb.error.positions=(q-self.q).tolist(); handle.publish_feedback(fb)
                if elapsed>=times[-1]: break
                time.sleep(.02)
            since=None
            while True:
                self.guard(handle,deadline)
                if max(abs(self.q-points[-1]))<=self.c['goal_tolerance_rad'] and self.stopped():
                    if since is None: since=time.monotonic()
                    if time.monotonic()-since>=self.c['joint_stable_s']: break
                else: since=None
                time.sleep(.01)
            result.error_code=FollowJointTrajectory.Result.SUCCESSFUL; handle.succeed()
        except Exception as e:
            confirmed=self.confirm_stop(); result.error_code=FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string=str(e)+('' if confirmed else ':STOP_UNCONFIRMED')
            if handle.is_cancel_requested: handle.canceled()
            else: handle.abort()
        finally: self.busy=False; self.active_task=None
        return result
    def execute_gripper(self,handle):
        result=SetWidth.Result(); deadline=time.monotonic()+handle.request.timeout_s
        try:
            q=self.q.copy(); width=handle.request.width_m; since=None; samples=[]
            while True:
                self.guard(handle,deadline); self.publish(q,width)
                require(max(abs(self.q-q))<=self.c['goal_tolerance_rad'],'ARM_MOVED_DURING_GRIPPER')
                handle.publish_feedback(SetWidth.Feedback(measured_width_m=float(self.width)))
                if abs(self.width-width)<=self.c['gripper_tolerance_m']:
                    if since is None: since=time.monotonic(); samples=[]
                    samples.append(self.effort)
                    if time.monotonic()-since>=self.c['gripper_settle_s']: break
                else: since=None
                time.sleep(.02)
            result.success=True; result.measured_width_m=float(self.width); result.effort=float(min(samples)); handle.succeed()
        except Exception as e:
            confirmed=self.confirm_stop(); result.reason=str(e)+('' if confirmed else ':STOP_UNCONFIRMED')
            if handle.is_cancel_requested: handle.canceled()
            else: handle.abort()
        finally: self.busy=False; self.active_task=None
        return result

def trajectory_points(traj,cfg,current):
    require(current is not None,'JOINTS_NOT_READY')
    require(list(traj.joint_names)==cfg['joint_names'] and len(traj.points)>0,'TRAJECTORY_NAMES')
    require(seconds(traj.header.stamp)==0,'SCHEDULED_TRAJECTORY_UNSUPPORTED')
    times=np.array([seconds(p.time_from_start) for p in traj.points]); points=np.array([p.positions for p in traj.points])
    require(points.shape==(len(times),6) and finite(points) and finite(times),'TRAJECTORY_SHAPE')
    require(times[0]>=0 and np.all(np.diff(times)>0) and times[-1]<cfg['stage_timeout_s']-cfg['joint_stable_s'],'TRAJECTORY_TIMES')
    check_trajectory_bounds(points,np.array(current),cfg)
    previous=np.array(current); last=0.
    for t,q in zip(times,points):
        dt=t-last; delta=abs(q-previous)
        if dt==0: require(max(delta)<=cfg['goal_tolerance_rad'],'TRAJECTORY_START')
        else:
            require(np.all(1.5*delta/dt<=cfg['max_joint_speed_rad_s']),'TRAJECTORY_SPEED')
            require(np.all(6*delta/dt**2<=cfg['max_joint_acceleration_rad_s2']),'TRAJECTORY_ACCELERATION')
        previous=q; last=t
    return times,points

def interpolate(times,points,current,t):
    previous=np.array(current); last=0.
    for end,q in zip(times,points):
        if t<=end and end>last:
            u=np.clip((t-last)/(end-last),0,1); return previous+(q-previous)*(3*u*u-2*u*u*u)
        previous=q; last=end
    return points[-1]

def main():
    rclpy.init(); spin(PiperBridge())
