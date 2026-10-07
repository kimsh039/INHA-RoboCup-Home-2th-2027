"""MoveIt→Gazebo 실동작 시험. pick은 보존 후보 재사용이며 새 영상/GPU E2E라고 주장하지 않는다."""
import argparse
import copy
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger
from moveit_msgs.action import MoveGroup,ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene,GetCartesianPath
from moveit_msgs.msg import Constraints,JointConstraint,PositionConstraint,OrientationConstraint,CollisionObject,PlanningScene,RobotState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from robocup_manipulation_msgs.action import SetGripper
from scipy.spatial.transform import Rotation
from .generate import ARM,FINGERS


class Demo(Node):
    def __init__(self,repository,timeout):
        super().__init__('gazebo_manipulation_demo');self.repository=Path(repository);self.timeout=timeout;self.started=time.monotonic()
        self.js=None;self.poses={};self.events=[];self.active=None
        self.create_subscription(JointState,'/joint_states',lambda m:setattr(self,'js',m),10)
        self.create_subscription(TFMessage,'/gazebo/dynamic_poses',self.on_poses,10)
        self.move=ActionClient(self,MoveGroup,'/move_action');self.execute=ActionClient(self,ExecuteTrajectory,'/execute_trajectory')
        self.gripper=ActionClient(self,SetGripper,'/hardware/set_gripper')
        self.scene=self.create_client(ApplyPlanningScene,'/apply_planning_scene');self.cart=self.create_client(GetCartesianPath,'/compute_cartesian_path')
        self.stop=self.create_client(Trigger,'/hardware/stop')
        self.base_world=np.array(json.loads((self.repository/'simulation/mujoco/config/grasp_simulation.json').read_text())['base_world_m'])

    def on_poses(self,msg):
        for t in msg.transforms:
            self.poses[t.child_frame_id]=t
            if t.child_frame_id=='robocup':self.base_world=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])

    def tick(self):
        if time.monotonic()-self.started>self.timeout:raise RuntimeError('DEMO_TIMEOUT')
        rclpy.spin_once(self,timeout_sec=.02)

    def wait(self,predicate):
        while not predicate():self.tick()

    def future(self,f):self.wait(f.done);return f.result()

    def action(self,client,goal):
        self.wait(client.server_is_ready)
        handle=self.future(client.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('GOAL_REJECTED')
        self.active=handle
        try:return self.future(handle.get_result_async()).result
        except BaseException:
            handle.cancel_goal_async()
            raise
        finally:self.active=None

    def stage(self,name):
        print(name,flush=True);self.events.append({'stage':name,'wall_elapsed_s':time.monotonic()-self.started})

    def robot_state(self):
        self.wait(lambda:self.js is not None and all(n in self.js.name for n in ARM))
        # Gazebo는 preserveFixedJoint 관절도 발행한다. 고정 관절은 MoveIt variable이 아니다.
        # 현재 MoveIt 상태에 팔/그리퍼 실제 피드백만 diff로 갱신한다.
        source=self.js
        indices=[i for i,n in enumerate(source.name) if n in ARM+FINGERS]
        filtered=JointState(header=copy.deepcopy(source.header),
            name=[source.name[i] for i in indices],position=[source.position[i] for i in indices])
        if len(source.velocity)==len(source.name):filtered.velocity=[source.velocity[i] for i in indices]
        if len(source.effort)==len(source.name):filtered.effort=[source.effort[i] for i in indices]
        return RobotState(joint_state=filtered,is_diff=True)

    def apply_scene(self):
        self.wait(self.scene.service_is_ready)
        scene=PlanningScene(is_diff=True)
        table=CollisionObject(id='gazebo_table',operation=CollisionObject.ADD);table.header.frame_id='base_link'
        table.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.6,.9,.05])]
        p=Pose();p.orientation.w=1.;p.position.x,p.position.y,p.position.z=(np.array([.68,0,.695])-self.base_world).tolist();table.primitive_poses=[p]
        scene.world.collision_objects=[table]
        if not self.future(self.scene.call_async(ApplyPlanningScene.Request(scene=scene))).success:raise RuntimeError('SCENE_FAILED')

    def constraints(self,q=None,T=None):
        c=Constraints()
        if q is not None:
            c.joint_constraints=[JointConstraint(joint_name=n,position=float(v),tolerance_above=.005,tolerance_below=.005,weight=1.) for n,v in zip(ARM,q)]
        else:
            p=Pose();p.position.x,p.position.y,p.position.z=T[:3,3].tolist();quat=Rotation.from_matrix(T[:3,:3]).as_quat()
            p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=quat.tolist()
            position=PositionConstraint(link_name='manipulation_tcp',weight=1.);position.header.frame_id='base_link'
            position.constraint_region.primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[.003])];position.constraint_region.primitive_poses=[p]
            orientation=OrientationConstraint(link_name='manipulation_tcp',orientation=p.orientation,weight=1.)
            orientation.header.frame_id='base_link';orientation.absolute_x_axis_tolerance=orientation.absolute_y_axis_tolerance=orientation.absolute_z_axis_tolerance=.04
            c.position_constraints=[position];c.orientation_constraints=[orientation]
        return c

    def plan(self,q=None,T=None,plan_only=False):
        goal=MoveGroup.Goal();req=goal.request;req.group_name='arm';req.pipeline_id='ompl';req.planner_id='RRTConnectkConfigDefault'
        req.start_state=self.robot_state();req.goal_constraints=[self.constraints(q,T)];req.allowed_planning_time=10.;req.num_planning_attempts=3
        req.max_velocity_scaling_factor=.3;req.max_acceleration_scaling_factor=.3;goal.planning_options.plan_only=True
        result=self.action(self.move,goal)
        if result.error_code.val!=1:raise RuntimeError('PLAN_FAILED:'+str(result.error_code.val))
        if not plan_only:self.run_trajectory(result.planned_trajectory)
        return result.planned_trajectory

    def run_trajectory(self,trajectory):
        trajectory.joint_trajectory.header.stamp.sec=trajectory.joint_trajectory.header.stamp.nanosec=0
        result=self.action(self.execute,ExecuteTrajectory.Goal(trajectory=trajectory))
        if result.error_code.val!=1:raise RuntimeError('EXECUTION_FAILED:'+str(result.error_code.val))

    def grip(self,gap):
        result=self.action(self.gripper,SetGripper.Goal(gap_m=float(gap),timeout_s=10.))
        if not result.success:raise RuntimeError(result.error_code)
        return result.measured_gap_m

    def home(self):
        self.stage('RETURN_TO_INITIAL');self.plan(q=[0]*6)
        q=dict(zip(self.js.name,self.js.position));error=max(abs(q[n]) for n in ARM)
        if error>.02:raise RuntimeError('RETURN_TO_INITIAL_FAILED')
        self.stage('VERIFY_INITIAL_POSTURE');return error

    def cube(self):
        t=self.poses.get('manipulation_cube')
        if t is None:raise RuntimeError('NO_CUBE_GROUND_TRUTH')
        return np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])

    def pause(self,duration):
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:self.tick()

    def run(self,mode,plan_only):
        self.robot_state();self.pause(1.);self.apply_scene()
        if mode=='arm':
            self.stage('PLAN_ARM_TEST');self.plan(q=[0,.1,-.15,0,.05,0],plan_only=plan_only)
            if plan_only:return {'result':'PLANNED'}
            self.stage('OPEN_GRIPPER');self.grip(.07);self.stage('CLOSE_GRIPPER');self.grip(0.)
            return {'result':'PASS_ARM_GRIPPER_RETURN','initial_error_rad':self.home()}
        obs=json.loads((self.repository/'simulation/mujoco/reports/live_observation.json').read_text())['measured_status']['joint_positions'][:6]
        self.stage('MOVE_TO_OBSERVATION');self.plan(q=obs,plan_only=plan_only)
        if plan_only:return {'result':'PLANNED'}
        self.grip(.07)
        if mode=='observation':return {'result':'OBSERVATION_REACHED','detail':'Wrist RGB-D may now be inspected; no grasp claim'}
        recorded=json.loads((self.repository/'simulation/mujoco/reports/updated_urdf_pick_place.json').read_text())
        T=np.array(recorded['selected']['T_world_grasp_tcp']);T[:3,3]-=self.base_world
        self.stage('APPROACH_RECORDED_GRASP');self.plan(T=T)
        before=self.cube();self.stage('CLOSE_GRIPPER');gap=self.grip(0.)
        if not .005<gap<.065:raise RuntimeError('EMPTY_OR_INVALID_GRASP_GAP')
        lift=T.copy();lift[2,3]+=.2;self.stage('LIFT');self.plan(T=lift);self.pause(.5)
        actual=self.cube()
        if actual[2]-before[2]<.15:raise RuntimeError('GRASP_NOT_VERIFIED_NO_PHYSICAL_LIFT')
        self.stage('VERIFY_HOLD');self.pause(1.);held=self.cube()
        if np.linalg.norm(held-actual)>.015:raise RuntimeError('OBJECT_SLIPPED')
        destination=lift.copy();destination[1,3]-=.1;self.stage('MOVE_TO_PLACE');self.plan(T=destination)
        destination[2,3]-=.2;self.stage('LOWER_OBJECT');self.plan(T=destination)
        self.stage('RELEASE');self.grip(.07);self.pause(1.)
        released=self.cube();expected=before+np.array([0,-.1,0])
        if np.linalg.norm(released-expected)>.025:raise RuntimeError('PLACEMENT_NOT_VERIFIED')
        retreat=destination.copy();retreat[2,3]+=.1;self.stage('RETREAT');self.plan(T=retreat)
        return {'result':'PASS_RECORDED_GAZEBO_PICK_PLACE','initial_error_rad':self.home(),'placed_cube_world_m':released.tolist()}


def main():
    p=argparse.ArgumentParser();p.add_argument('--repository',required=True);p.add_argument('--mode',choices=['arm','observation','pick'],default='arm')
    p.add_argument('--plan-only',action='store_true');p.add_argument('--timeout',type=float,default=300);p.add_argument('--output',type=Path,required=True)
    args,ros=rclpy.utilities.remove_ros_args()[1:],None
    args=p.parse_args(args);rclpy.init();node=Demo(args.repository,args.timeout)
    report={'scope':'Gazebo simulation only. pick uses preserved nominal candidate, no live GraspNet/servo claim','mode':args.mode,'result':'FAIL'}
    try:report.update(node.run(args.mode,args.plan_only))
    except BaseException as e:
        report['detail']=str(e)
        if node.active:node.active.cancel_goal_async()
        if node.stop.service_is_ready():
            f=node.stop.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=3.)
            report['stop_confirmed']=bool(f.done() and f.result().success)
        raise
    finally:
        report['events']=node.events;args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2));node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
