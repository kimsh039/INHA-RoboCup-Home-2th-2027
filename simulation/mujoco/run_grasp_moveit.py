"""MoveIt plans; MuJoCo executes and verifies finger contact and an actual 20 cm lift."""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK, GetCartesianPath, GetStateValidity
from moveit_msgs.msg import (PlanningScene, CollisionObject, AttachedCollisionObject,
                            AllowedCollisionEntry, Constraints, PositionConstraint,
                            OrientationConstraint, JointConstraint, RobotState)
from scipy.spatial.transform import Rotation

from manipulation.placement import place_targets
from manipulation.approach_checks import segment_intersects_box
from manipulation.grasp_opening import cube_opening_check
from manipulation.mid360_cloud import validate_scene_metadata, validate_scene_status
from manipulation.sim_model import ROOT, JOINTS, config, target_tcp, transform


def pose(T):
    p=Pose();p.position.x,p.position.y,p.position.z=map(float,T[:3,3])
    q=Rotation.from_matrix(T[:3,:3]).as_quat()
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q)
    return p


class Demo(Node):
    def __init__(self, grasp_path=None, metadata_path=None, fixed_open_width=False):
        super().__init__("piper_grasp_demo")
        self.grasp_path=grasp_path or ROOT/"data/grasps/cube/grasps.json"
        self.metadata_path=metadata_path or ROOT/"data/pointclouds/cube/metadata.json"
        self.fixed_open_width=fixed_open_width
        self.attached=None;self.support_allowed=False;self.cfg=config();self.status=None;self.joints=None;self.events=[];self.rejected=[]
        self.create_subscription(String,"/simulation/grasp_status",self.status_callback,10)
        self.create_subscription(JointState,"/joint_states",lambda msg:setattr(self,"joints",msg),10)
        self.gripper=self.create_publisher(Float64,"/gripper_command",10)
        self.stage_pub=self.create_publisher(String,"/manipulation_stage",10)
        self.base_pub=self.create_publisher(String,"/base_state",10)
        self.markers=self.create_publisher(PoseArray,"/grasp_targets",10)
        self.scene=self.create_client(ApplyPlanningScene,"/apply_planning_scene")
        self.ik=self.create_client(GetPositionIK,"/compute_ik")
        self.cart=self.create_client(GetCartesianPath,"/compute_cartesian_path")
        self.valid=self.create_client(GetStateValidity,"/check_state_validity")
        self.move=ActionClient(self,MoveGroup,"/move_action")
        self.execute=ActionClient(self,ExecuteTrajectory,"/execute_trajectory")

    def status_callback(self,msg):self.status=json.loads(msg.data)

    def tick(self,seconds=0.1):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(self,timeout_sec=min(0.05,max(0,until-time.monotonic())))
            if self.status and self.status["failure"]:raise RuntimeError(self.status["failure"])

    def wait_future(self,future,timeout=60):
        until=time.monotonic()+timeout
        while not future.done():
            self.tick(0.02)
            if time.monotonic()>until:raise RuntimeError("ROS_RESPONSE_TIMEOUT")
        if future.exception():raise future.exception()
        return future.result()

    def wait_simulation(self,seconds,timeout=60):
        # CPU가 느려도 닫기 동작을 중간에 끝내지 않도록 물리 시간을 기준으로 기다린다.
        if 'simulation_time_s' not in self.status:
            raise RuntimeError('RESTART_MUJOCO_BRIDGE_REQUIRED: simulation time missing')
        start=self.status['simulation_time_s'];deadline=time.monotonic()+timeout
        while self.status['simulation_time_s']-start<seconds:
            self.tick(.05)
            if time.monotonic()>deadline:raise RuntimeError('SIMULATION_WAIT_TIMEOUT')

    def state(self):
        s=RobotState();s.joint_state=self.joints;s.is_diff=False
        if self.attached is not None:s.attached_collision_objects=[self.attached]
        return s

    def stage(self,name):
        self.stage_pub.publish(String(data=name));self.events.append({"state":name,"time_wall":time.time()})
        print(name,flush=True);self.tick(0.1)

    def action(self,client,goal):
        handle=self.wait_future(client.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError("ACTION_GOAL_REJECTED")
        try:
            # CPU 시뮬레이션의 벽시계 지연을 허용하되 무한 대기는 하지 않는다.
            envelope=self.wait_future(handle.get_result_async(),timeout=600)
        except BaseException:
            handle.cancel_goal_async()
            raise
        result=envelope.result
        if result.error_code.val!=1:raise RuntimeError(f"MOVEIT_FAILED: {result.error_code.val}")
        return result

    def move_goal(self,constraints,plan_only):
        goal=MoveGroup.Goal();req=goal.request
        req.group_name="arm";req.pipeline_id="ompl";req.planner_id="RRTConnectkConfigDefault"
        req.start_state=self.state();req.goal_constraints=[constraints]
        req.allowed_planning_time=8.;req.num_planning_attempts=5
        req.max_velocity_scaling_factor=self.cfg["planning_velocity_scale"]
        req.max_acceleration_scaling_factor=self.cfg["planning_velocity_scale"]
        goal.planning_options.plan_only=plan_only;goal.planning_options.replan=False
        return self.action(self.move,goal)

    def pose_constraints(self,T):
        c=Constraints();c.name="TCP_goal"
        pos=PositionConstraint();pos.header.frame_id="world";pos.link_name="tcp";pos.weight=1.
        sphere=SolidPrimitive();sphere.type=SolidPrimitive.SPHERE;sphere.dimensions=[self.cfg["tcp_position_tolerance_m"]]
        pos.constraint_region.primitives=[sphere];pos.constraint_region.primitive_poses=[pose(T)]
        orientation=OrientationConstraint();orientation.header.frame_id="world";orientation.link_name="tcp"
        orientation.orientation=pose(T).orientation;orientation.weight=1.
        orientation.absolute_x_axis_tolerance=orientation.absolute_y_axis_tolerance=orientation.absolute_z_axis_tolerance=self.cfg["tcp_orientation_tolerance_rad"]
        c.position_constraints=[pos];c.orientation_constraints=[orientation];return c

    def apply_scene(self,scene):
        # 장면의 부분 갱신은 현재 관절 상태와 연결된 물체를 유지해야 한다.
        # 기본값 False이면 ACM만 갱신해도 연결 물체가 전체 교체될 수 있다.
        if scene.is_diff:scene.robot_state.is_diff=True
        request=ApplyPlanningScene.Request();request.scene=scene
        if not self.wait_future(self.scene.call_async(request)).success:raise RuntimeError("PLANNING_SCENE_UPDATE_FAILED")

    def collision_object(self,name,T,dimensions):
        obj=CollisionObject();obj.id=name;obj.header.frame_id="world";obj.operation=CollisionObject.ADD
        primitive=SolidPrimitive();primitive.type=SolidPrimitive.BOX;primitive.dimensions=list(map(float,dimensions))
        obj.primitives=[primitive];obj.primitive_poses=[pose(T)];return obj

    def world_scene(self):
        scene=PlanningScene();scene.is_diff=True
        table=self.collision_object("table",transform(self.cfg["table_center_world_m"],np.eye(3)),2*np.array(self.cfg["table_half_size_m"]))
        cube=self.collision_object("cube",np.array(self.status["T_world_object"]),json.loads((ROOT/"config/pointclouds.json").read_text())["objects"]["cube"]["dimensions_m"])
        floor=self.collision_object("floor",transform([0,0,-.025],np.eye(3)),[4.,4.,.05])
        scene.world.collision_objects=[table,cube,floor]
        center=np.array(self.cfg["table_center_world_m"]);half=np.array(self.cfg["table_half_size_m"]);height=center[2]-half[2]
        for i,(sx,sy) in enumerate(((-1,-1),(-1,1),(1,-1),(1,1))):
            T=transform([center[0]+sx*(half[0]-.04),center[1]+sy*(half[1]-.04),height/2],np.eye(3))
            scene.world.collision_objects.append(self.collision_object(f"table_leg_{i}",T,[.05,.05,height]))
        self.apply_scene(scene);self.allow_fingers(False)

    def allow_fingers(self,allowed):
        # Include the robot adjacency filters exported into SRDF; avoid replacing them with false entries.
        import xml.etree.ElementTree as ET
        pairs={(x.attrib["link1"],x.attrib["link2"]) for x in ET.parse(ROOT/"simulation/generated/piper.srdf").findall("disable_collisions")}
        if self.cfg.get("robot_model")!="inha":pairs.add(("base_link","table"))
        else:
            pairs.add(("base_link","floor"))
            pairs.update((x.get("name"),"floor") for x in ET.parse(ROOT/"simulation/generated/piper.urdf").findall("link") if "wheel_link" in x.get("name"))
        if self.support_allowed:pairs.add(("cube","table"))
        if allowed:pairs.update((f,"cube") for f in ("link7","link8"))
        names=[x.get("name") for x in ET.parse(ROOT/"simulation/generated/piper.urdf").findall("link")]+["table","cube","floor",*[f"table_leg_{i}" for i in range(4)]]
        scene=PlanningScene();scene.is_diff=True
        scene.allowed_collision_matrix.entry_names=names
        for a in names:
            scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[a==b or (a,b) in pairs or (b,a) in pairs for b in names]))
        self.apply_scene(scene)

    def check_state(self,state):
        req=GetStateValidity.Request();req.robot_state=state;req.group_name="arm"
        return self.wait_future(self.valid.call_async(req)).valid

    def ik_state(self,T,start):
        req=GetPositionIK.Request();req.ik_request.group_name="arm";req.ik_request.ik_link_name="tcp"
        req.ik_request.robot_state=start;req.ik_request.avoid_collisions=True
        req.ik_request.pose_stamped.header.frame_id="world";req.ik_request.pose_stamped.pose=pose(T)
        req.ik_request.timeout=Duration(sec=1)
        response=self.wait_future(self.ik.call_async(req))
        if response.error_code.val!=1:raise RuntimeError(f"IK_FAILED: {response.error_code.val}")
        return response.solution

    def cartesian(self,start,T):
        req=GetCartesianPath.Request();req.header.frame_id="world";req.start_state=start
        req.group_name="arm";req.link_name="tcp";req.waypoints=[pose(T)]
        req.max_step=self.cfg["cartesian_step_m"];req.jump_threshold=2.0;req.avoid_collisions=True
        req.max_velocity_scaling_factor=self.cfg["planning_velocity_scale"]
        req.max_acceleration_scaling_factor=self.cfg["planning_velocity_scale"]
        result=self.wait_future(self.cart.call_async(req))
        if result.error_code.val!=1 or result.fraction<0.999999:raise RuntimeError(f"CARTESIAN_PATH_FAILED: fraction={result.fraction:.4f}, code={result.error_code.val}")
        trajectory=result.solution
        # Conservative per-segment time parameterization with zero velocities at each waypoint.
        # Cubic Hermite interpolation in the bridge then respects the configured v/a bounds.
        t=0.;previous=None
        v=self.cfg["joint_velocity_limit_rad_s"]*self.cfg["planning_velocity_scale"]
        a=self.cfg["joint_acceleration_limit_rad_s2"]*self.cfg["planning_velocity_scale"]
        for point in trajectory.joint_trajectory.points:
            current=np.array(point.positions)
            if previous is not None:
                delta=float(np.max(np.abs(current-previous)))
                t+=max(0.05,1.5*delta/v,np.sqrt(6*delta/a))
            point.time_from_start=Duration(sec=int(t),nanosec=int((t-int(t))*1e9))
            point.velocities=[0.]*len(current);point.accelerations=[];previous=current
        return trajectory

    def end_state(self,start,trajectory):
        state=RobotState();state.joint_state=JointState()
        positions=dict(zip(start.joint_state.name,start.joint_state.position))
        positions.update(zip(trajectory.joint_trajectory.joint_names,trajectory.joint_trajectory.points[-1].positions))
        state.joint_state.name=list(positions);state.joint_state.position=list(positions.values())
        if self.attached is not None:state.attached_collision_objects=[self.attached]
        return state

    def execute_trajectory(self,trajectory):
        goal=ExecuteTrajectory.Goal();goal.trajectory=trajectory
        self.action(self.execute,goal);self.tick(0.2)

    def tcp_reached(self,T):
        actual=np.array(self.status["T_world_tcp"])
        error=float(np.linalg.norm(actual[:3,3]-T[:3,3]))
        angle=float(Rotation.from_matrix(actual[:3,:3].T@T[:3,:3]).magnitude())
        if error>self.cfg["tcp_position_tolerance_m"] or angle>self.cfg["tcp_orientation_tolerance_rad"]:
            raise RuntimeError(f"TCP_TARGET_NOT_REACHED: {error:.5f} m, {angle:.5f} rad")

    def attach_planning_object(self, tcp=None, obj=None):
        # Planning collision attachment only. MuJoCo remains entirely contact-driven.
        relative=np.linalg.inv(np.array(self.status["T_world_tcp"]) if tcp is None else tcp)@(np.array(self.status["T_world_object"]) if obj is None else obj)
        scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True
        # MoveIt은 물체 연결 시 같은 이름의 월드 물체를 자동 제거한다.
        # 같은 메시지에 REMOVE도 넣으면 이중 삭제가 되어 서비스가 실패한다.
        attached=AttachedCollisionObject();attached.link_name="tcp";attached.touch_links=["link7","link8"]
        attached.object=self.collision_object("cube",relative,json.loads((ROOT/"config/pointclouds.json").read_text())["objects"]["cube"]["dimensions_m"]);attached.object.header.frame_id="tcp"
        # 서비스가 일부만 적용되어 실패하더라도 finally에서 연결을 해제할 수 있게 기록한다.
        self.attached=attached
        scene.robot_state.attached_collision_objects=[attached];self.apply_scene(scene)

    def detach_planning_object(self, world_object):
        scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True
        a=AttachedCollisionObject();a.link_name="tcp";a.object.id="cube";a.object.operation=CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects=[a]
        scene.world.collision_objects=[self.collision_object("cube",world_object,json.loads((ROOT/"config/pointclouds.json").read_text())["objects"]["cube"]["dimensions_m"])]
        self.apply_scene(scene);self.attached=None

    def placement_plan(self,start,grasp_tcp,lift_tcp,world_object):
        """Check attached object, carry, descent and table bounds; left before right."""
        relative=np.linalg.inv(grasp_tcp)@world_object
        errors=[]
        for side,lower in place_targets(grasp_tcp,np.array(self.status["T_world_base"]),self.cfg["place_lateral_offset_m"]):
            try:
                expected=lower@relative
                half=np.array(json.loads((ROOT/"config/pointclouds.json").read_text())["objects"]["cube"]["dimensions_m"])/2
                center=np.array(self.cfg["table_center_world_m"]);extent=np.array(self.cfg["table_half_size_m"])
                if np.any(np.abs(expected[:2,3]-center[:2])+half[:2]>extent[:2]):raise RuntimeError("PLACE_OUTSIDE_TABLE")
                above=lower.copy();above[2,3]+=self.cfg["lift_distance_m"]
                self.support_allowed=False;self.allow_fingers(True)
                carry=self.cartesian(start,above)
                self.support_allowed=True;self.allow_fingers(True)
                descend=self.cartesian(self.end_state(start,carry),lower)
                return side,above,lower,carry,descend
            except RuntimeError as error:errors.append({"side":side,"reason":str(error)})
            finally:self.support_allowed=False;self.allow_fingers(True)
        self.place_rejections=errors
        raise RuntimeError(f"NO_FEASIBLE_PLACEMENT: {errors}")

    def fold_arm(self):
        c=Constraints()
        for name,value in zip(JOINTS,self.cfg["folded_candidate_rad"]):
            c.joint_constraints.append(JointConstraint(joint_name=name,position=float(value),tolerance_above=.005,tolerance_below=.005,weight=1.))
        self.move_goal(c,False)
        if max(abs(a-b) for a,b in zip(self.status["joint_positions"][:6],self.cfg["folded_candidate_rad"]))>self.cfg["arm_goal_tolerance_rad"]:raise RuntimeError("RETURN_HOME_FAILED")

    def run(self):
        self.stage("INIT")
        until=time.monotonic()+15
        while self.status is None or self.joints is None:
            self.tick(0.1)
            if time.monotonic()>until:raise RuntimeError("START_MUJOCO_BRIDGE_FIRST")
        source_path=self.grasp_path
        if not source_path.exists():raise RuntimeError("NEW_GRASPNET_OUTPUT_REQUIRED: run Colab with the new colab_input.zip")
        source=json.loads(source_path.read_text())
        local=json.loads(self.metadata_path.read_text())
        if source.get("input_metadata")!=local:raise RuntimeError("GRASP_INPUT_MODEL_MISMATCH: re-run Colab with the 4 cm support-face-excluded cloud, then import its output ZIP")
        validate_scene_metadata(local)
        validate_scene_status(local,self.status)
        for client in (self.scene,self.ik,self.cart,self.valid):
            if not client.wait_for_service(timeout_sec=10):raise RuntimeError("START_MOVEIT_FIRST")
        for client in (self.move,self.execute):
            if not client.wait_for_server(timeout_sec=10):raise RuntimeError("MOVEIT_ACTION_UNAVAILABLE")
        self.stage("OPEN_GRIPPER")
        self.gripper.publish(Float64(data=self.cfg.get("hardware_max_jaw_opening_m",.07)/2))
        self.tick(self.cfg["grasp_close_duration_s"]+.3)
        if self.status["jaw_opening_m"]<self.cfg.get("hardware_max_jaw_opening_m",.07)-.002:raise RuntimeError("GRIPPER_OPEN_FAILED")
        self.stage("LOAD_OBJECT");self.world_scene()
        if local.get('pipeline') == 'wrist_camera_mujoco_roi_v1':
            # Navigation/close approach preceded the stopped wrist observation.
            if self.status['base_state']!='STOPPED':raise RuntimeError('STOP_BASE_BEFORE_GRASP')
            self.stage('BASE_STOPPED')
        else:
            # Verify and reach folded candidate before simulating base motion.
            self.stage("FOLD_ARM")
            self.fold_arm()
            self.base_pub.publish(String(data="MOVING"));self.stage("BASE_MOVING");self.tick(1.)
            self.base_pub.publish(String(data="STOPPED"));self.stage("BASE_STOPPED")
        if self.status["base_state"]!="STOPPED":raise RuntimeError("BASE_STOP_NOT_CONFIRMED")
        self.stage("SELECT_GRASP")
        source=json.loads(self.grasp_path.read_text())
        if source["units"]!="meters" or source["point_cloud_frame"]!="object":raise RuntimeError("GRASP_FRAME_INVALID")
        local=json.loads(self.metadata_path.read_text())
        if source["input_metadata"]!=local:raise RuntimeError("GRASP_INPUT_MODEL_MISMATCH")
        validate_scene_status(local,self.status)
        selected=None
        use_observation_start=local.get('pipeline')=='wrist_camera_mujoco_roi_v1'
        T_world_object=np.array(self.status["T_world_object"])
        opening=min(self.status["jaw_opening_m"],self.status["max_jaw_opening_m"],self.cfg.get("hardware_max_jaw_opening_m",.07))
        dimensions=json.loads((ROOT/"config/pointclouds.json").read_text())["objects"]["cube"]["dimensions_m"]
        for grasp in sorted(source["grasps"],key=lambda g:-g["score"]):
            try:
                if self.fixed_open_width:
                    # 예측 폭 대신 실제 개구에서 큐브 전체와 양쪽 1mm 여유가 들어가는지 확인한다.
                    opening_check=cube_opening_check(grasp,dimensions,opening)
                    if not opening_check['fits']:raise RuntimeError(f"OBJECT_OUTSIDE_ACTUAL_OPENING: {opening_check}")
                elif not 0<grasp["width"]<=min(self.status["max_jaw_opening_m"],self.cfg.get("hardware_max_jaw_opening_m",.07)):raise RuntimeError("GRIPPER_WIDTH_INVALID")
                pre=(np.array(self.status['T_world_tcp']).copy() if use_observation_start
                     else target_tcp(grasp,T_world_object,self.cfg["pregrasp_distance_m"]))
                target=target_tcp(grasp,T_world_object)
                if segment_intersects_box(pre[:3,3],target[:3,3],self.cfg["table_center_world_m"],self.cfg["table_half_size_m"]):
                    raise RuntimeError("APPROACH_INTERSECTS_TABLE: TCP path crosses tabletop")
                if use_observation_start:
                    preplan=None
                    start=self.state()
                else:
                    try:self.ik_state(pre,self.state())
                    except RuntimeError as error:raise RuntimeError(f"PREGRASP_{error}") from error
                    preplan=self.move_goal(self.pose_constraints(pre),True).planned_trajectory
                    start=self.end_state(self.state(),preplan)
                self.allow_fingers(True)
                self.ik_state(target,start)
                approach=self.cartesian(start,target)
                # Check lift IK before committing. A full attached-object lift plan follows actual contact verification.
                lift=target.copy();lift[2,3]+=self.cfg["lift_distance_m"]
                grasp_state=self.end_state(start,approach)
                self.ik_state(lift,grasp_state)
                self.attach_planning_object(target,T_world_object)
                self.support_allowed=True;self.allow_fingers(True)
                grasp_state.attached_collision_objects=[self.attached]
                liftplan=self.cartesian(grasp_state,lift)
                self.placement_plan(self.end_state(grasp_state,liftplan),target,lift,T_world_object)
                selected=(grasp,pre,target,preplan,approach,lift)
                break
            except RuntimeError as error:
                self.rejected.append({"candidate_id":grasp["candidate_id"],"reason":str(error)})
                print(f"Rejected {grasp['candidate_id']}: {error}",flush=True)
            finally:
                if self.attached is not None:self.detach_planning_object(T_world_object)
                self.support_allowed=False;self.allow_fingers(False)
        if selected is None:raise RuntimeError("NO_FEASIBLE_GRASP")
        grasp,pre,target,preplan,approach,lift=selected
        self.selected={"candidate_id":grasp["candidate_id"],"score":grasp["score"],"width_m":grasp["width"],
                       "width_policy":"actual maximum opening" if self.fixed_open_width else "predicted width limit",
                       "actual_opening_m":opening,
                       "opening_check":cube_opening_check(grasp,dimensions,opening) if self.fixed_open_width else None,
                       "T_world_pre_tcp":pre.tolist(),"T_world_grasp_tcp":target.tolist(),
                       "approach_start":"measured wrist observation" if use_observation_start else "20cm pre-grasp"}
        markers=PoseArray();markers.header.frame_id="world";markers.poses=[pose(target),pose(pre)];self.markers.publish(markers)
        print("Selected:",self.selected,flush=True)
        if not use_observation_start:
            self.stage("MOVE_TO_PREGRASP");self.execute_trajectory(preplan);self.tcp_reached(pre)
            self.stage("WAIT_AT_PREGRASP");self.tick(self.cfg["pregrasp_wait_s"])
        self.stage("APPROACH_GRASP");self.allow_fingers(True)
        # Recompute from measured state, rather than blindly using a stale preflight trajectory.
        self.execute_trajectory(self.cartesian(self.state(),target));self.tcp_reached(target)
        self.stage("CLOSE_GRIPPER");self.gripper.publish(Float64(data=0.0));self.wait_simulation(self.cfg["grasp_close_duration_s"]+1.)
        self.stage("VERIFY_GRASP")
        self.grasp_contact_status={k:self.status.get(k) for k in ('simulation_time_s','jaw_opening_m','finger_contacts','finger_contact_forces','gripper_actuator_force_N')}
        print('Grasp contact:',self.grasp_contact_status,flush=True)
        if set(self.status["finger_contacts"])!={"link7","link8"}:raise RuntimeError("GRASP_VERIFICATION_FAILED: no bilateral finger contact")
        initial=np.array(self.status["T_world_object"])
        relative=np.linalg.inv(np.array(self.status["T_world_tcp"]))@initial
        self.attach_planning_object();self.support_allowed=True;self.allow_fingers(True);self.stage("LIFT_OBJECT")
        actual_tcp=np.array(self.status["T_world_tcp"]);actual_lift=actual_tcp.copy();actual_lift[2,3]+=self.cfg["lift_distance_m"]
        self.execute_trajectory(self.cartesian(self.state(),actual_lift));self.tcp_reached(actual_lift)
        samples=[];self.lift_samples=samples
        for _ in range(20):
            self.tick(0.1)
            observed=np.linalg.inv(np.array(self.status["T_world_tcp"]))@np.array(self.status["T_world_object"])
            position_error=float(np.linalg.norm(observed[:3,3]-relative[:3,3]))
            orientation_error=float(Rotation.from_matrix(relative[:3,:3].T@observed[:3,:3]).magnitude())
            rise=float(self.status["T_world_object"][2][3]-initial[2,3])
            samples.append({"rise_m":rise,"relative_position_error_m":position_error,"relative_orientation_error_rad":orientation_error,
                            "finger_contacts":self.status["finger_contacts"],"finger_contact_forces":self.status.get('finger_contact_forces')})
            if (set(self.status["finger_contacts"])!={"link7","link8"} or position_error>self.cfg["relative_position_tolerance_m"]
                or orientation_error>self.cfg["relative_orientation_tolerance_rad"]
                or abs(rise-self.cfg["lift_distance_m"])>self.cfg["lift_height_tolerance_m"]):
                raise RuntimeError(f"LIFT_VERIFICATION_FAILED: {samples[-1]}")
        self.lift_samples=samples
        self.stage("PLAN_PLACE_LEFT")
        side,above,lower,carry,descend=self.placement_plan(self.state(),target,actual_lift,initial)
        markers.poses=[pose(target),pose(pre),pose(lower)];self.markers.publish(markers)
        self.placement={"side":side,"offset_m":self.cfg["place_lateral_offset_m"],"T_world_place_tcp":lower.tolist()}
        if side=="right":self.stage("PLAN_PLACE_RIGHT")
        self.stage("MOVE_TO_PLACE");self.execute_trajectory(carry);self.tcp_reached(above)
        self.stage("LOWER_OBJECT");self.support_allowed=True;self.allow_fingers(True)
        self.execute_trajectory(self.cartesian(self.state(),lower));self.tcp_reached(lower);self.tick(.5)
        if not self.status["object_support_contact"]:raise RuntimeError("PLACE_SUPPORT_NOT_CONFIRMED")
        self.stage("OPEN_GRIPPER");self.gripper.publish(Float64(data=.035));self.tick(self.cfg["grasp_close_duration_s"]+1)
        placed=np.array(self.status["T_world_object"])
        self.detach_planning_object(placed)
        self.stage("RETREAT");self.execute_trajectory(self.cartesian(self.state(),above));self.tcp_reached(above)
        self.allow_fingers(False);self.tick(1)
        expected=initial[:3,3]+(self.cfg["place_lateral_offset_m"]*(1 if side=="left" else -1))*np.array(self.status["T_world_base"])[:3,1]
        actual=np.array(self.status["T_world_object"])[:3,3]
        if np.linalg.norm(actual-expected)>.02 or not self.status["object_support_contact"] or self.status["finger_contacts"]:raise RuntimeError("PLACEMENT_VERIFICATION_FAILED")
        self.placement.update(expected_object_position_m=expected.tolist(),actual_object_position_m=actual.tolist())
        self.support_allowed=False;self.allow_fingers(False)
        self.stage("FOLD_ARM");self.fold_arm();self.stage("DONE")


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grasps',type=Path,default=ROOT/'data/grasps/cube/grasps.json')
    parser.add_argument('--metadata',type=Path,default=ROOT/'data/pointclouds/cube/metadata.json')
    parser.add_argument('--result',type=Path,default=ROOT/'reports/grasp_simulation_result.json')
    parser.add_argument('--fixed-open-width',action='store_true',help='예측 폭 제한 대신 실제 최대 개구와 큐브 기하를 검사')
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args);node=Demo(args.grasps,args.metadata,args.fixed_open_width);result={"result":"FAIL","planner":"MoveIt 2 OMPL/KDL","scope":"assembled robot pick and place, contact-driven physics","grasp_file":str(args.grasps),"metadata_file":str(args.metadata),"fixed_open_width":args.fixed_open_width}
    try:
        node.run();result["result"]="PICK_AND_PLACE_PASS"
    except (RuntimeError,ValueError,KeyError) as error:
        print(f"FAIL: {error}. MuJoCo viewer stays open in terminal 1.",flush=True)
        node.stage_pub.publish(String(data="FAIL"));result["failure_reason"]=str(error)
    finally:
        result.update({"events":node.events,"rejected_candidates":node.rejected,
                       "grasp_contact_status":getattr(node,"grasp_contact_status",None),
                       "selected":getattr(node,"selected",None),"lift_samples":getattr(node,"lift_samples",None),
                       "placement":getattr(node,"placement",None),"place_rejections":getattr(node,"place_rejections",None),"last_simulation_status":node.status})
        args.result.parent.mkdir(parents=True,exist_ok=True)
        args.result.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print("Result:",args.result,flush=True)
        node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()
