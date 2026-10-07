"""Nav2→MuJoCo 성공 로직의 Gazebo backend를 연결하는 PickPlace action 서버."""
import json
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,ActionClient,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from robocup_manipulation_msgs.action import PickPlace
from robocup_manipulation_msgs.msg import TaskStatus


class Task(Node):
    def __init__(self):
        super().__init__('gazebo_pick_place_task')
        for name in ('repository','generated','task_config'):self.declare_parameter(name,'')
        self.root=Path(self.get_parameter('repository').value);self.generated=Path(self.get_parameter('generated').value)
        path=self.get_parameter('task_config').value
        self.cfg=json.loads(Path(path).read_text()) if path else {'navigation':None}
        self.group=ReentrantCallbackGroup();self.busy=False;self.lock=threading.Lock();self.odom=None
        self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),10,callback_group=self.group)
        self.status=self.create_publisher(TaskStatus,'/manipulation/status',10)
        self.stop=self.create_client(Trigger,'/hardware/stop',callback_group=self.group)
        self.nav=None
        if self.cfg.get('navigation'):
            from nav2_msgs.action import NavigateToPose
            self.nav_type=NavigateToPose;self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose',callback_group=self.group)
        self.server=ActionServer(self,PickPlace,'/manipulation/pick_place',self.execute,
            goal_callback=self.accept,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)

    def accept(self,request):
        with self.lock:
            p=request.destination.pose.position;o=request.destination.pose.orientation
            target=np.array([p.x,p.y,p.z]);limits=(np.array([.5,0,.7401]),np.array([.5,-.2,.7401]))
            if self.busy or request.plan_only or request.object_class!='cube' or request.destination.header.frame_id!='world':return GoalResponse.REJECT
            if not np.isfinite([*target,request.timeout_s]).all() or not 0<request.timeout_s<=3600:return GoalResponse.REJECT
            if min(np.linalg.norm(target-t) for t in limits)>.025 or np.linalg.norm([o.x,o.y,o.z,o.w-1])>1e-6:return GoalResponse.REJECT
            self.busy=True;return GoalResponse.ACCEPT

    def execute(self,handle):
        start=time.monotonic();request=handle.request;result=PickPlace.Result();process=None;nav_handle=None
        def feedback(stage,detail=''):
            status=TaskStatus(task_id=request.task_id,state=stage,elapsed_s=time.monotonic()-start,detail=detail)
            status.header.stamp=self.get_clock().now().to_msg();self.status.publish(status);handle.publish_feedback(PickPlace.Feedback(status=status))
        def check():
            if handle.is_cancel_requested:raise RuntimeError('CANCELED')
            if time.monotonic()-start>request.timeout_s:raise RuntimeError('TASK_TIMEOUT')
        def wait(f):
            while not f.done():check();time.sleep(.02)
            return f.result()
        try:
            if self.nav:
                feedback('NAVIGATE_TO_APPROACH')
                while not self.nav.server_is_ready():check();time.sleep(.1)
                cfg=self.cfg['navigation'];pose=PoseStamped();pose.header.frame_id=cfg['frame'];pose.header.stamp=self.get_clock().now().to_msg()
                pose.pose.position.x,pose.pose.position.y=map(float,cfg['xy']);yaw=float(cfg['yaw_rad'])
                pose.pose.orientation.z=float(np.sin(yaw/2));pose.pose.orientation.w=float(np.cos(yaw/2))
                nav_handle=wait(self.nav.send_goal_async(self.nav_type.Goal(pose=pose)))
                if not nav_handle.accepted:raise RuntimeError('NAV_GOAL_REJECTED')
                out=wait(nav_handle.get_result_async())
                if out.status!=4:raise RuntimeError('NAVIGATION_FAILED')
            feedback('BASE_STOPPED')
            stopped_since=None
            while True:
                check()
                if self.odom:
                    stamp=self.odom.header.stamp.sec+self.odom.header.stamp.nanosec/1e9;now=self.get_clock().now().nanoseconds/1e9;v=self.odom.twist.twist
                    stopped=0<=now-stamp<.5 and np.linalg.norm([v.linear.x,v.linear.y,v.linear.z])<.01 and np.linalg.norm([v.angular.x,v.angular.y,v.angular.z])<.01
                    stopped_since=time.monotonic() if stopped and stopped_since is None else (stopped_since if stopped else None)
                    if stopped_since is not None and time.monotonic()-stopped_since>=.5:break
                time.sleep(.05)
            directory=self.generated.parent/'task_runs'/str(time.time_ns());directory.mkdir(parents=True)
            report=directory/'result.json';messages=queue.Queue()
            remaining=max(1.,request.timeout_s-(time.monotonic()-start))
            process=subprocess.Popen([sys.executable,'-u','-m','robocup_gazebo_manipulation.tutorial','--repository',str(self.root),
                '--generated',str(self.generated),'--timeout',str(remaining),'--output',str(report),'--ros-args','-p','use_sim_time:=true'],
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
            def reader():
                with (directory/'execution.log').open('w') as log:
                    for line in process.stdout:log.write(line);log.flush();messages.put(line.strip())
            thread=threading.Thread(target=reader,daemon=True);thread.start()
            while process.poll() is None:
                check()
                try:line=messages.get(timeout=.1)
                except queue.Empty:continue
                if re.fullmatch('[A-Z][A-Z0-9_]+',line):feedback(line)
            thread.join(timeout=2)
            if process.returncode!=0 or not report.exists():raise RuntimeError('GAZEBO_BACKEND_FAILED:'+str(directory))
            data=json.loads(report.read_text())
            if data.get('result')!='PICK_AND_PLACE_PASS':raise RuntimeError(data.get('detail','GAZEBO_TASK_FAILED'))
            actual=np.array(data['placement']['actual_object_position_m']);p=request.destination.pose.position
            if np.linalg.norm(actual-[p.x,p.y,p.z])>.025:raise RuntimeError('REQUESTED_DESTINATION_NOT_REACHED')
            feedback('SUCCEEDED');result.success=True;result.final_state='SUCCEEDED';result.detail=str(report);handle.succeed()
        except BaseException as e:
            if nav_handle is not None:nav_handle.cancel_goal_async()
            if process and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    try:process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
            confirmed=False
            if self.stop.service_is_ready():
                f=self.stop.call_async(Trigger.Request());until=time.monotonic()+3
                while not f.done() and time.monotonic()<until:time.sleep(.02)
                confirmed=bool(f.done() and f.result().success)
            result.success=False;result.error_code='CANCELED' if handle.is_cancel_requested else 'GAZEBO_TASK_FAILED'
            if not confirmed:result.error_code='STOP_UNCONFIRMED'
            result.final_state='FAILED';result.detail=str(e);feedback(result.final_state,result.detail)
            if handle.is_cancel_requested:handle.canceled()
            else:handle.abort()
        finally:self.busy=False
        return result


def main():
    rclpy.init();node=Task();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
