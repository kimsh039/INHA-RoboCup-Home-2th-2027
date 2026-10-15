"""Collision-aware MoveIt planning; measured-feedback execution through Piper bridge."""
import copy
import numpy as np
import uuid
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene, GetStateValidity
from moveit_msgs.msg import (Constraints, JointConstraint, PositionConstraint, OrientationConstraint,
                            BoundingVolume, CollisionObject, AttachedCollisionObject, PlanningScene,
                            AllowedCollisionEntry)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from .core import require
from .ros_util import action_call, wait_future
from .startup_bounds import feedback_in_bounds, check_trajectory_bounds

class Motion:
    def __init__(self,node,cfg,group):
        self.n=node; self.c=cfg
        self.plan=ActionClient(node,MoveGroup,'/move_action',callback_group=group)
        self.run=ActionClient(node,ExecuteTrajectory,'/execute_trajectory',callback_group=group)
        self.cart=node.create_client(GetCartesianPath,'/compute_cartesian_path',callback_group=group)
        self.scene=node.create_client(ApplyPlanningScene,'/apply_planning_scene',callback_group=group)
        self.validity=node.create_client(GetStateValidity,'/check_state_validity',callback_group=group)
        self.allowed=set()
    def pose(self,p,q):
        pose=Pose(); pose.position.x,pose.position.y,pose.position.z=map(float,p)
        pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,q)
        return pose
    def apply(self,scene,guard):
        require(self.scene.wait_for_service(timeout_sec=1),'SCENE_UNAVAILABLE')
        request=ApplyPlanningScene.Request(); request.scene=scene
        require(wait_future(self.scene.call_async(request),5,guard).success,'SCENE_REJECTED')
    def box(self,id,center,size):
        obj=CollisionObject(); obj.id=id; obj.header.frame_id=self.c['gravity_frame']; obj.operation=CollisionObject.ADD
        primitive=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(map(float,size)))
        obj.primitives=[primitive]; obj.primitive_poses=[self.pose(center,[0,0,0,1])]
        return obj
    def setup_scene(self,center,rule,guard,padding=0.):
        c=self.c; x0,x1,y0,y1=c['table_bounds_xy']; z=c['table_z_m']
        scene=PlanningScene(is_diff=True)
        scene.world.collision_objects=[self.box('manip_table',[(x0+x1)/2,(y0+y1)/2,z-.025],[x1-x0,y1-y0,.05])]
        scene.world.collision_objects += [self.box(o['id'],o['center'],o['size']) for o in c['obstacles']]
        if center is not None:
            scene.world.collision_objects.append(self.box('manip_target',center,[2*(rule['radius_m']+padding),2*(rule['radius_m']+padding),2*(rule['center_height_m']+padding)]))
        self.apply(scene,guard)
    def contacts(self,links,guard):
        # Explicit small ACM overrides: leave unrelated/self collision checks intact.
        scene=PlanningScene(is_diff=True)
        names=['manip_target']+list(links)
        scene.allowed_collision_matrix.entry_names=names
        scene.allowed_collision_matrix.entry_values=[AllowedCollisionEntry(enabled=[i!=j and (i==0 or j==0) for j in range(len(names))]) for i in range(len(names))]
        self.apply(scene,guard)
    def attach(self,center,rule,guard):
        scene=PlanningScene(is_diff=True); scene.robot_state.is_diff=True
        obj=self.box('manip_target',center,[2*rule['radius_m'],2*rule['radius_m'],2*rule['center_height_m']])
        attached=AttachedCollisionObject(link_name=self.c['tcp_frame'],object=obj,touch_links=self.c['touch_links'])
        scene.robot_state.attached_collision_objects=[attached]
        removal=CollisionObject(id='manip_target',operation=CollisionObject.REMOVE)
        scene.world.collision_objects=[removal]; self.apply(scene,guard)
    def detach(self,center,rule,guard):
        scene=PlanningScene(is_diff=True); scene.robot_state.is_diff=True
        attached=AttachedCollisionObject(link_name=self.c['tcp_frame'],object=CollisionObject(id='manip_target',operation=CollisionObject.REMOVE))
        scene.robot_state.attached_collision_objects=[attached]
        scene.world.collision_objects=[self.box('placed_'+uuid.uuid4().hex,center,[2*rule['radius_m'],2*rule['radius_m'],2*rule['center_height_m']])]
        self.apply(scene,guard)
    def plan_to(self,p=None,q=None,joints=None,guard=lambda:None):
        prefix=self.check_startup_prefix(joints,guard)
        c=self.c; goal=MoveGroup.Goal(); req=goal.request
        req.group_name=c['planning_group']; req.num_planning_attempts=1; req.allowed_planning_time=5.0
        req.max_velocity_scaling_factor=c['velocity_scale']; req.max_acceleration_scaling_factor=c['acceleration_scale']
        req.start_state.is_diff=True
        if prefix is not None:
            req.start_state.joint_state=JointState(name=list(c['joint_names']),position=prefix[1].tolist())
        constraint=Constraints()
        if joints is not None:
            constraint.joint_constraints=[JointConstraint(joint_name=n,position=float(v),tolerance_above=c['goal_tolerance_rad'],tolerance_below=c['goal_tolerance_rad'],weight=1.0) for n,v in zip(c['joint_names'],joints)]
        else:
            pc=PositionConstraint(); pc.header.frame_id=c['gravity_frame']; pc.link_name=c['tcp_frame']; pc.weight=1.0
            pc.constraint_region=BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[c['position_tolerance_m']])],primitive_poses=[self.pose(p,[0,0,0,1])])
            oc=OrientationConstraint(); oc.header.frame_id=c['gravity_frame']; oc.link_name=c['tcp_frame']; oc.orientation=self.pose(p,q).orientation; oc.weight=1.0
            oc.absolute_x_axis_tolerance=oc.absolute_y_axis_tolerance=oc.absolute_z_axis_tolerance=c['orientation_tolerance_rad']
            constraint.position_constraints=[pc]; constraint.orientation_constraints=[oc]
        req.goal_constraints=[constraint]; goal.planning_options.plan_only=True
        result=action_call(self.plan,goal,c['stage_timeout_s'],guard,allow_aborted=True)
        require(result.error_code.val==1,'PLAN_FAILED')
        traj=result.planned_trajectory
        if prefix is not None:
            points=traj.joint_trajectory.points
            require(bool(points) and np.max(np.abs(np.array(points[0].positions)-prefix[1]))<=.001,'STARTUP_PREFIX_NOT_INWARD')
            first=copy.deepcopy(points[0]); first.positions=prefix[0].tolist()
            first.time_from_start.sec=0; first.time_from_start.nanosec=0
            points.insert(0,first)
        return traj
    def check_startup_prefix(self,joints,guard):
        c=self.c; guard()
        msg=self.n.joint
        require(msg is not None,'INITIAL_FEEDBACK_NOT_READY')
        values=dict(zip(msg.name,msg.position)); current=np.array([values[n] for n in c['joint_names']])
        limits=np.array(c['joint_limits']); bounded=np.clip(current,limits[:,0],limits[:,1])
        if np.array_equal(current,bounded): return
        require(feedback_in_bounds(current,c),'STARTUP_DROOP_TOO_LARGE')
        require(joints is not None and c.get('observation_joints') is not None and
                np.max(np.abs(np.array(joints)-c['observation_joints']))<=.001,'INITIAL_POSE_REQUIRED')
        require(self.validity.wait_for_service(timeout_sec=2),'STARTUP_COLLISION_SERVICE_UNAVAILABLE')
        # Check actual (unclipped) geometry and the inward prefix against the
        # complete current scene. No ACM changes or zero-offset changes.
        steps=max(2,int(np.ceil(np.max(np.abs(bounded-current))/.002))+1)
        for u in np.linspace(0.,1.,steps):
            guard(); candidate=current+(bounded-current)*u
            request=GetStateValidity.Request(); request.robot_state.is_diff=True
            positions=dict(values); positions.update(zip(c['joint_names'],candidate.tolist()))
            request.robot_state.joint_state=JointState(name=list(positions),position=list(positions.values()))
            response=wait_future(self.validity.call_async(request),5,guard)
            if not response.valid:
                contacts='; '.join(
                    f'{contact.contact_body_1} <-> {contact.contact_body_2} '
                    f'depth={contact.depth*1000:.3f}mm'
                    for contact in response.contacts)
                self.n.get_logger().error(
                    f'Startup prefix collision: fraction={u:.3f}; '
                    f'joints={candidate.tolist()}; contacts={contacts or "not reported"}')
            require(response.valid,'STARTUP_PREFIX_COLLISION')
        self.n.get_logger().info('Measured droop prefix collision check passed; restoring initial pose')
        return current,bounded
    def execute(self,traj,guard):
        require(bool(traj.joint_trajectory.points),'EMPTY_PLAN')
        msg=self.n.joint; values=dict(zip(msg.name,msg.position))
        check_trajectory_bounds(np.array([p.positions for p in traj.joint_trajectory.points]),
                                np.array([values[n] for n in self.c['joint_names']]),self.c)
        self.check_planned_startup_prefix(traj,values,guard)
        points=traj.joint_trajectory.points
        elapsed=0.; previous=None; old_time=0.
        for point in points:
            original=point.time_from_start.sec+point.time_from_start.nanosec/1e9
            dt=max(0.,original-old_time)
            if previous is not None:
                delta=np.abs(np.array(point.positions)-previous)
                dt=max(dt,float(np.max(1.5*delta/self.c['max_joint_speed_rad_s'])),float(np.max(np.sqrt(6*delta/self.c['max_joint_acceleration_rad_s2']))))
                # Leave time margin below the limits after nanosecond encoding.
                dt=max(dt,1e-6)*1.02
            elapsed+=dt; point.time_from_start.sec=int(elapsed); point.time_from_start.nanosec=int((elapsed-int(elapsed))*1e9)
            point.velocities=[]; point.accelerations=[]
            previous=np.array(point.positions); old_time=original
        require(elapsed<self.c['stage_timeout_s']-self.c['joint_stable_s'],'RETIME_EXCEEDS_STAGE_TIMEOUT')
        goal=ExecuteTrajectory.Goal(trajectory=traj)
        result=action_call(self.run,goal,self.c['stage_timeout_s'],guard)
        require(result.error_code.val==1,'EXECUTION_FAILED')
    def check_planned_startup_prefix(self,traj,values,guard):
        c=self.c; limits=np.array(c['joint_limits'])
        previous=np.array([values[n] for n in c['joint_names']])
        points=[np.array(p.positions) for p in traj.joint_trajectory.points]
        def inside(q): return bool(np.all(q>=limits[:,0]-1e-8) and np.all(q<=limits[:,1]+1e-8))
        if inside(previous) and all(inside(q) for q in points): return
        require(self.validity.wait_for_service(timeout_sec=2),'STARTUP_COLLISION_SERVICE_UNAVAILABLE')
        # Check the actual planned prefix, including the measured-to-first
        # connection. The bridge's smoothstep follows these same joint lines.
        for target in points:
            steps=max(2,int(np.ceil(np.max(np.abs(target-previous))/.002))+1)
            for u in np.linspace(0.,1.,steps):
                guard(); q=previous+(target-previous)*u
                positions=dict(values); positions.update(zip(c['joint_names'],q.tolist()))
                request=GetStateValidity.Request(); request.robot_state.is_diff=True
                request.robot_state.joint_state=JointState(name=list(positions),position=list(positions.values()))
                response=wait_future(self.validity.call_async(request),5,guard)
                if not response.valid:
                    self.n.get_logger().error('Planned startup prefix collision: '+ '; '.join(
                        f'{x.contact_body_1}<->{x.contact_body_2} depth={x.depth*1000:.3f}mm'
                        for x in response.contacts))
                require(response.valid,'STARTUP_PREFIX_COLLISION')
            previous=target
            if inside(target): break
    def joint_move(self,joints,guard): self.execute(self.plan_to(joints=joints,guard=guard),guard)
    def pose_move(self,p,q,guard): self.execute(self.plan_to(p,q,guard=guard),guard)
    def cart_plan(self,p,q,guard,start=None):
        require(self.cart.wait_for_service(timeout_sec=1),'CARTESIAN_UNAVAILABLE')
        req=GetCartesianPath.Request(); req.header.frame_id=self.c['gravity_frame']; req.group_name=self.c['planning_group']; req.link_name=self.c['tcp_frame']
        req.start_state.is_diff=True
        if start is not None:
            req.start_state.joint_state.name=start.joint_trajectory.joint_names
            req.start_state.joint_state.position=start.joint_trajectory.points[-1].positions
        req.waypoints=[self.pose(p,q)]; req.max_step=self.c['cartesian_step_m']; req.jump_threshold=1.5; req.avoid_collisions=True
        result=wait_future(self.cart.call_async(req),5,guard)
        require(result.error_code.val==1 and result.fraction>=0.999,'CARTESIAN_PATH_INCOMPLETE')
        # Humble Cartesian service has no scaling fields. Stretch timestamps,
        # velocities and accelerations; driver separately enforces measured bounds.
        traj=result.solution; scale=self.c['velocity_scale']
        for point in traj.joint_trajectory.points:
            t=(point.time_from_start.sec+point.time_from_start.nanosec/1e9)/scale
            point.time_from_start.sec=int(t); point.time_from_start.nanosec=int((t-int(t))*1e9)
            point.velocities=[v*scale for v in point.velocities]
            point.accelerations=[a*scale*scale for a in point.accelerations]
        return traj
    def cart_move(self,p,q,guard): self.execute(self.cart_plan(p,q,guard),guard)
