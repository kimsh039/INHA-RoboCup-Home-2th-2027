"""Gazebo 전용 FollowJointTrajectory/SetGripper 경계. 실제 CAN/제조사 드라이버를 호출하지 않는다."""
import json
import threading
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from robocup_manipulation_msgs.action import SetGripper
from .generate import ARM,FINGERS

LIMIT_ROUNDOFF_RAD=1e-8

def seconds(t):return t.sec+t.nanosec/1e9


def validate_trajectory(trajectory,limits):
    names=list(trajectory.joint_names)
    if len(names)!=6 or set(names)!=set(ARM) or not trajectory.points:raise ValueError('ARM_JOINT_NAMES_REQUIRED')
    if seconds(trajectory.header.stamp)!=0:raise ValueError('ONLY_IMMEDIATE_TRAJECTORY_SUPPORTED')
    last=-1.
    for point in trajectory.points:
        t=seconds(point.time_from_start);q=np.array(point.positions)
        if not np.isfinite(t) or t<0 or t<=last or len(q)!=6 or not np.isfinite(q).all():raise ValueError('INVALID_TRAJECTORY')
        for n,v in zip(names,q):
            lower,upper=limits[n]
            if not lower-LIMIT_ROUNDOFF_RAD<=v<=upper+LIMIT_ROUNDOFF_RAD:
                raise ValueError(f'JOINT_LIMIT: {n}={v:.12g}, limits=[{lower},{upper}]')
        for values in (point.velocities,point.accelerations):
            if values and (len(values)!=6 or not np.isfinite(values).all()):raise ValueError('INVALID_DERIVATIVES')
        last=t
    if last>300:raise ValueError('TRAJECTORY_TOO_LONG')
    return names


class Controller(Node):
    def __init__(self):
        super().__init__('gazebo_manipulation_controller')
        self.declare_parameter('manifest','');self.declare_parameter('require_base_stopped',True);self.declare_parameter('allow_base_drift_experiment',False)
        self.declare_parameter('feedback_max_age_sim_s',.5)
        self.declare_parameter('feedback_stall_wall_s',5.)
        self.declare_parameter('execution_wall_timeout_s',1800.)
        self.declare_parameter('arm_settle_tolerance_rad',.003)
        self.limits=json.loads(Path(self.get_parameter('manifest').value).read_text())['joint_limits']
        self.lock=threading.Lock();self.busy=False;self.latched=False;self.cancel_generation=0
        self.js=None;self.received=0.;self.odom=None;self.odom_received=0.
        self.group=ReentrantCallbackGroup()
        self.pubs={n:self.create_publisher(Float64,'/gazebo/manipulation/'+n+'/target',1) for n in ARM+FINGERS}
        self.create_subscription(JointState,'/joint_states',self.feedback,10,callback_group=self.group)
        self.create_subscription(Odometry,'/odom',self.base_feedback,10,callback_group=self.group)
        self.gripper_pub=self.create_publisher(JointState,'/hardware/gripper_feedback',10)
        self.arm=ActionServer(self,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory',self.execute,
            goal_callback=self.accept,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.gripper=ActionServer(self,SetGripper,'/hardware/set_gripper',self.grip,
            goal_callback=self.accept_grip,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.create_service(Trigger,'/hardware/stop',self.stop,callback_group=self.group)
        self.create_service(Trigger,'/hardware/reset',self.reset,callback_group=self.group)
        self.create_timer(.05,self.watchdog,callback_group=self.group)

    def feedback(self,msg):
        if len(msg.name)!=len(msg.position) or not np.isfinite(msg.position).all():return
        self.js=msg;self.received=time.monotonic()
        q=dict(zip(msg.name,msg.position))
        if all(n in q for n in FINGERS):
            self.gripper_pub.publish(JointState(header=msg.header,name=['gripper_gap'],position=[q[FINGERS[0]]-q[FINGERS[1]]]))

    def base_feedback(self,msg):self.odom=msg;self.odom_received=time.monotonic()

    def state(self):
        if self.js is None:raise RuntimeError('NO_JOINT_FEEDBACK')
        self.fresh_feedback(self.js,self.received,'JOINT')
        q=dict(zip(self.js.name,self.js.position))
        if not all(n in q for n in ARM+FINGERS):raise RuntimeError('INCOMPLETE_JOINT_FEEDBACK')
        return q

    def fresh_feedback(self,msg,arrival,kind):
        # Gazebo의 느린 실행을 실제 장치 통신 단절과 혼동하지 않는다.
        # source age는 ROS sim clock, 완전한 단절/정지는 monotonic wall clock으로 검사한다.
        wall_age=time.monotonic()-arrival
        now=self.get_clock().now().nanoseconds/1e9
        source_age=now-seconds(msg.header.stamp)
        if wall_age>self.get_parameter('feedback_stall_wall_s').value:
            raise RuntimeError(f'STALE_{kind}_FEEDBACK: wall stall {wall_age:.3f}s')
        if now<=0 or source_age<-.01 or source_age>self.get_parameter('feedback_max_age_sim_s').value:
            raise RuntimeError(f'STALE_{kind}_FEEDBACK: sim source age {source_age:.3f}s')

    def check_base(self):
        if not self.get_parameter('require_base_stopped').value:return
        if self.odom is None:raise RuntimeError('NO_BASE_FEEDBACK')
        self.fresh_feedback(self.odom,self.odom_received,'BASE')
        if self.get_parameter('allow_base_drift_experiment').value:return
        v=self.odom.twist.twist
        if np.linalg.norm([v.linear.x,v.linear.y,v.linear.z])>.01 or np.linalg.norm([v.angular.x,v.angular.y,v.angular.z])>.01:
            raise RuntimeError('BASE_MOVING')

    def reserve(self):
        with self.lock:
            try:self.state();self.check_base()
            except RuntimeError as e:
                self.get_logger().warning('GOAL_REJECTED: '+str(e));return GoalResponse.REJECT
            if self.busy or self.latched:
                self.get_logger().warning('GOAL_REJECTED: '+('BUSY' if self.busy else 'STOP_LATCHED'));return GoalResponse.REJECT
            self.busy=True;return GoalResponse.ACCEPT

    def accept(self,goal):
        try:validate_trajectory(goal.trajectory,self.limits)
        except ValueError as e:self.get_logger().warning(str(e));return GoalResponse.REJECT
        return self.reserve()

    def accept_grip(self,goal):
        if not np.isfinite([goal.gap_m,goal.timeout_s]).all() or not 0<=goal.gap_m<=.07 or not 0<goal.timeout_s<=30:return GoalResponse.REJECT
        return self.reserve()

    def send(self,names,values):
        for n,v in zip(names,values):
            lower,upper=self.limits[n]
            if not np.isfinite(v) or not lower-LIMIT_ROUNDOFF_RAD<=v<=upper+LIMIT_ROUNDOFF_RAD:
                raise RuntimeError(f'JOINT_LIMIT: command {n}={v}')
            # 실제 명령은 항상 원래 URDF 한계 안에 둔다. 극소 반올림 오차만 정규화한다.
            self.pubs[n].publish(Float64(data=float(np.clip(v,lower,upper))))

    def hold(self):
        try:q=self.state()
        except RuntimeError:return False
        self.send(ARM,[q[n] for n in ARM]);return True

    def stop(self,req,res):
        self.latched=True;self.cancel_generation+=1;self.hold()
        # hold 발행만으로 정지 확인을 보고하지 않는다.
        deadline=time.monotonic()+2.;stable=0;previous=self.js
        while time.monotonic()<deadline:
            try:
                q=self.state();self.check_base()
                msg=self.js
                if msg is not previous:
                    stable=stable+1 if len(msg.velocity)==len(msg.name) and np.max(np.abs(msg.velocity))<.02 else 0
                    previous=msg
                if stable>=3:res.success=True;res.message='STOPPED_BY_FRESH_FEEDBACK';return res
            except RuntimeError:break
            time.sleep(.02)
        res.success=False;res.message='STOP_UNCONFIRMED';return res

    def reset(self,req,res):
        try:self.state();self.check_base()
        except RuntimeError as e:res.message=str(e);return res
        if self.busy:res.message='BUSY';return res
        self.latched=False;res.success=True;res.message='SIMULATION_ONLY_RESET';return res

    def watchdog(self):
        if self.busy:
            try:self.state();self.check_base()
            except RuntimeError:
                self.latched=True;self.cancel_generation+=1;self.hold()

    def check(self,handle,generation,last_clock,started):
        self.state();self.check_base()
        if handle.is_cancel_requested:raise RuntimeError('CANCELED')
        if self.latched or generation!=self.cancel_generation:raise RuntimeError('STOP_LATCHED')
        clock=self.get_clock().now().nanoseconds/1e9
        if clock<last_clock-1e-6:self.latched=True;raise RuntimeError('CLOCK_RESET')
        if time.monotonic()-started>self.get_parameter('execution_wall_timeout_s').value:raise RuntimeError('WALL_TIMEOUT')
        return clock

    def execute(self,handle):
        result=FollowJointTrajectory.Result();generation=self.cancel_generation;started=time.monotonic()
        try:
            trajectory=handle.request.trajectory;names=list(trajectory.joint_names)
            q=self.state();initial=np.array([q[n] for n in names])
            first=trajectory.points[0]
            if seconds(first.time_from_start)==0 and np.max(abs(np.array(first.positions)-initial))>.03:raise RuntimeError('START_STATE_MISMATCH')
            stamps=[0.];poses=[initial]
            for p in trajectory.points:
                if seconds(p.time_from_start)==0:poses[0]=np.array(p.positions)
                else:stamps.append(seconds(p.time_from_start));poses.append(np.array(p.positions))
            origin=self.get_clock().now().nanoseconds/1e9;last=origin;settle_start=None;last_progress=started
            while True:
                now=self.check(handle,generation,last,started);last=now;t=now-origin
                segment=min(len(stamps)-2,max(0,int(np.searchsorted(stamps,t)-1)))
                if t>=stamps[-1]:desired=poses[-1]
                else:
                    u=np.clip((t-stamps[segment])/(stamps[segment+1]-stamps[segment]),0,1)
                    desired=poses[segment]+u*(poses[segment+1]-poses[segment])
                self.send(names,desired);actual=self.state()
                wall=time.monotonic()
                if wall-last_progress>=5:
                    elapsed=wall-started;error=max(abs(actual[n]-v) for n,v in zip(names,poses[-1]))
                    progress=100*min(1.,t/max(stamps[-1],1e-9))
                    self.get_logger().info(f'ARM_PROGRESS: {progress:.1f}%, sim={t:.2f}/{stamps[-1]:.2f}s, wall={elapsed:.1f}s, real_time_factor={t/max(elapsed,1e-9):.3f}, final_joint_error={error:.4f}rad')
                    last_progress=wall
                feedback=FollowJointTrajectory.Feedback();feedback.header.stamp=self.get_clock().now().to_msg();feedback.joint_names=names
                feedback.desired.positions=desired.tolist();feedback.actual.positions=[actual[n] for n in names]
                feedback.error.positions=(desired-np.array(feedback.actual.positions)).tolist();handle.publish_feedback(feedback)
                if t>=stamps[-1]:
                    error=np.max(abs(np.array(feedback.actual.positions)-poses[-1]))
                    if error<self.get_parameter('arm_settle_tolerance_rad').value:
                        settle_start=now if settle_start is None else settle_start
                        if now-settle_start>=.2:break
                    else:settle_start=None
                    if t>stamps[-1]+10:raise RuntimeError('GOAL_TOLERANCE')
                time.sleep(.02)
            handle.succeed();result.error_code=0
        except Exception as e:
            self.hold();result.error_code=FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED;result.error_string=str(e)
            if handle.is_cancel_requested:handle.canceled()
            else:handle.abort()
        finally:self.busy=False
        return result

    def grip(self,handle):
        result=SetGripper.Result();generation=self.cancel_generation;started=time.monotonic();last=self.get_clock().now().nanoseconds/1e9;origin=last
        gap=handle.request.gap_m
        try:
            while True:
                now=self.check(handle,generation,last,started);last=now
                self.send(FINGERS,[gap/2,-gap/2]);q=self.state();result.measured_gap_m=q[FINGERS[0]]-q[FINGERS[1]]
                handle.publish_feedback(SetGripper.Feedback(measured_gap_m=result.measured_gap_m))
                # 닫기는 물체에 막힐 수 있다. 성공은 닫기 단계 종료이며 실제 파지는 별도 확인한다.
                if abs(result.measured_gap_m-gap)<.002 or (gap==0 and now-origin>=3):
                    result.success=True;handle.succeed();break
                if now-origin>handle.request.timeout_s:raise RuntimeError('GRIPPER_TIMEOUT')
                time.sleep(.02)
        except Exception as e:
            result.error_code=str(e)
            if handle.is_cancel_requested:handle.canceled()
            else:handle.abort()
        finally:self.busy=False
        return result


def main():
    rclpy.init();node=Controller();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
