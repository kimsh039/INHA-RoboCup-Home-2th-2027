"""Visible MuJoCo physics and an actual FollowJointTrajectory server for MoveIt 2."""
from __future__ import annotations

import json
import argparse
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from scipy.spatial.transform import Rotation

from manipulation.sim_model import JOINTS, ROOT, config, load_model, jaw_opening, tcp_transform, body_transform, write_moveit_files, verify_exported_fk


class Bridge(Node):
    def __init__(self, model, data):
        super().__init__("piper_mujoco_bridge")
        self.model,self.data,self.cfg=model,data,config()
        self.lock=threading.RLock()
        self.active=None
        self.failure=None
        self.last_simulation_time=float(data.time)
        self.clock_discontinuity=None
        self.base_state="STOPPED"
        self.stage="INIT"
        self.jq=[int(model.joint(n).qposadr[0]) for n in JOINTS]
        self.jv=[int(model.joint(n).dofadr[0]) for n in JOINTS]
        self.act=[model.actuator(n).id for n in JOINTS]
        # 관절 목표와 서보 입력을 분리한다. 보상 입력을 목표 오차로 오인하지 않는다.
        self.arm_target=data.ctrl[self.act].copy()
        self.gripper=model.actuator("gripper").id
        self.gripper_target=0.035
        self.group=ReentrantCallbackGroup()
        self.js=self.create_publisher(JointState,"/joint_states",10)
        self.status_pub=self.create_publisher(String,"/simulation/grasp_status",10)
        self.create_subscription(Float64,"/gripper_command",self.set_gripper,10,callback_group=self.group)
        self.create_subscription(String,"/base_state",self.set_base,10,callback_group=self.group)
        self.create_subscription(String,"/manipulation_stage",self.set_stage,10,callback_group=self.group)
        self.create_subscription(PoseArray,"/grasp_targets",self.set_markers,10,callback_group=self.group)
        self.server=ActionServer(self,FollowJointTrajectory,"/arm_controller/follow_joint_trajectory",
                                execute_callback=self.execute,goal_callback=self.accept,
                                cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.create_timer(0.02,self.publish,callback_group=self.group)
        self.max_width=jaw_opening(model,data)
        # Measure closed calibration in scratch data; do not move/teleport the executed robot.
        scratch=mujoco.MjData(model);scratch.qpos[:]=data.qpos
        for n in ("joint7","joint8"):scratch.qpos[model.joint(n).qposadr[0]]=0
        mujoco.mj_forward(model,scratch)
        self.closed_width=jaw_opening(model,scratch)
        if not (0.06 < self.max_width < 0.08 and abs(self.closed_width)<0.001):
            raise RuntimeError(f"PAD_CALIBRATION_FAILED: {self.closed_width}, {self.max_width}")
        self.get_logger().info(f"Measured jaw surface gap: closed={self.closed_width:.6f} m, open={self.max_width:.6f} m")

    def set_gripper(self,msg):
        with self.lock:
            if not np.isfinite(msg.data) or not 0<=msg.data<=0.035:
                self.get_logger().error("GRIPPER_WIDTH_INVALID: command is joint7 displacement in m")
                return
            self.gripper_target=msg.data

    def set_base(self,msg):
        with self.lock:
            if msg.data not in ("MOVING","STOPPED"):return
            if msg.data=="MOVING" and np.max(np.abs(self.data.qpos[self.jq]-self.cfg["folded_candidate_rad"]))>self.cfg["arm_goal_tolerance_rad"]:
                self.get_logger().error("Cannot mark base MOVING: folded pose not reached")
                return
            self.base_state=msg.data

    def set_stage(self,msg):
        with self.lock:self.stage=msg.data

    def set_markers(self,msg):
        if msg.header.frame_id!="world" or len(msg.poses) not in (2,3):return
        with self.lock:
            for name,pose in zip(("grasp_marker","pregrasp_marker","place_marker"),msg.poses):
                i=self.model.body_mocapid[self.model.body(name).id]
                self.data.mocap_pos[i]=[pose.position.x,pose.position.y,pose.position.z]

    def accept(self,goal):
        with self.lock:
            trajectory=goal.trajectory
            if self.active is not None or self.failure or not trajectory.points or len(trajectory.joint_names)!=6 or set(trajectory.joint_names)!=set(JOINTS):
                return GoalResponse.REJECT
            if self.base_state=="MOVING":return GoalResponse.REJECT
            times=[p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in trajectory.points]
            if times[0]<0 or any(b<=a for a,b in zip(times,times[1:])):return GoalResponse.REJECT
            order=[trajectory.joint_names.index(n) for n in JOINTS]
            ranges=self.model.jnt_range[[self.model.joint(n).id for n in JOINTS]]
            for index,p in enumerate(trajectory.points):
                if len(p.positions)!=6:return GoalResponse.REJECT
                if len(p.velocities) not in (0,6) or not np.isfinite(p.velocities).all():return GoalResponse.REJECT
                q=np.array(p.positions)[order]
                if not np.isfinite(q).all():return GoalResponse.REJECT
                outside=np.maximum(ranges[:,0]-q,q-ranges[:,1])
                if np.any(outside>1e-6):
                    # MuJoCo soft limits can settle slightly outside a hard URDF bound.
                    # Accept only a measured initial state; all later waypoints stay in bounds.
                    measured=self.data.qpos[self.jq]
                    if index!=0 or np.max(outside)>0.002 or np.max(np.abs(q-measured))>0.002:
                        self.get_logger().error(f"TRAJECTORY_REJECTED: joint limits at point {index}, q={q.tolist()}")
                        return GoalResponse.REJECT
            # Reserve before execution callback to reject overlapping goals.
            self.active={"reserved":True}
            return GoalResponse.ACCEPT

    def execute(self,handle):
        traj=handle.request.trajectory
        order=[traj.joint_names.index(n) for n in JOINTS]
        times=np.array([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in traj.points])
        positions=np.array([p.positions for p in traj.points])[:,order]
        velocities=np.array([p.velocities if len(p.velocities)==6 else [0.]*6 for p in traj.points])[:,order]
        with self.lock:
            if times[0]>0:
                times=np.r_[0,times];positions=np.vstack((self.data.qpos[self.jq],positions));velocities=np.vstack((np.zeros(6),velocities))
            pending={"t0":self.data.time,"times":times,"q":positions,"v":velocities,"event":threading.Event(),"error":None}
            out=ROOT/"reports/runtime_diagnostics";out.mkdir(parents=True,exist_ok=True)
            trajectory_file=out/f"trajectory_{time.time_ns()}.json"
            pending["diagnostic_file"]=str(trajectory_file)
            # 시간 보간과 추종 오차를 동일한 계획으로 재검증할 수 있게 원본 궤적을 저장한다.
            trajectory_file.write_text(json.dumps({"stage":self.stage,"start_simulation_time_s":float(self.data.time),
                "joint_names":JOINTS,"times_s":times.tolist(),"positions_rad":positions.tolist(),
                "velocities_rad_s":velocities.tolist(),"measured_start_rad":self.data.qpos[self.jq].tolist()},indent=2)+"\n")
            self.active=pending
        wall_started=time.monotonic()
        self.get_logger().info(f"Trajectory started: duration={pending['times'][-1]:.3f} simulation seconds")
        while rclpy.ok() and not pending["event"].wait(0.02):
            if handle.is_cancel_requested:
                with self.lock:
                    self.arm_target=self.data.qpos[self.jq].copy()
                    self.active=None
                self.get_logger().info(f"Trajectory canceled: wall={time.monotonic()-wall_started:.3f}s, simulation={self.data.time-pending['t0']:.3f}s")
                handle.canceled()
                return FollowJointTrajectory.Result(error_code=-1,error_string="Canceled; holding measured arm state")
        with self.lock:
            self.active=None
        self.get_logger().info(f"Trajectory finished: wall={time.monotonic()-wall_started:.3f}s, simulation={self.data.time-pending['t0']:.3f}s, error={pending['error']}")
        if pending["error"]:
            handle.abort()
            return FollowJointTrajectory.Result(error_code=-5,error_string=pending["error"])
        handle.succeed()
        return FollowJointTrajectory.Result(error_code=0)

    def contacts(self):
        fingers=set();unexpected=[]
        for c in self.data.contact:
            names=[self.model.body(int(self.model.geom_bodyid[g])).name or "world" for g in (c.geom1,c.geom2)]
            geoms=[self.model.geom(g).name or "unnamed" for g in (c.geom1,c.geom2)]
            if "cube" in names:
                fingers.update(n for n in names if n in ("link7","link8"))
                if any(n in ("link7","link8") for n in names):continue
                if all(n in ("cube","world") for n in names):continue
            if "table" in geoms and "base_link" in names:continue
            if c.dist < -self.cfg["penetration_tolerance_m"]:
                unexpected.append({"bodies":names,"geoms":geoms,"penetration_m":float(-c.dist)})
        return sorted(fingers),unexpected

    def save_runtime_incident(self, kind, details):
        # 최초 오류 당시의 상태를 보존해, 이후 정지 상태를 원인 상태로 오인하지 않는다.
        p=self.active
        record={"kind":kind,"stage":self.stage,"wall_time":time.time(),
                "simulation_time_s":float(self.data.time),"details":details,
                "joint_positions_rad":self.data.qpos[self.jq].tolist(),
                "arm_target_rad":self.arm_target.tolist(),
                "gripper_target_m":float(self.gripper_target),
                "physics_warnings":[{"index":i,"count":int(w.number),"last_info":int(w.lastinfo)} for i,w in enumerate(self.data.warning) if w.number],
                "trajectory_file":p.get("diagnostic_file") if p and not p.get("reserved") else None}
        out=ROOT/"reports/runtime_diagnostics";out.mkdir(parents=True,exist_ok=True)
        text=json.dumps(record,indent=2)+"\n"
        (out/f"incident_{time.time_ns()}.json").write_text(text)
        (ROOT/"reports/runtime_incident_latest.json").write_text(text)

    def check_simulation_clock(self, location):
        # MuJoCo 시간이 되돌아가면 이전 궤적의 t0/목표를 새 상태에 적용할 수 없다.
        # 물리 step 안의 자동 초기화와 viewer 등 step 사이 변경을 구분해 기록한다.
        if self.clock_discontinuity is not None:return False
        current=float(self.data.time)
        if current+1e-9 < self.last_simulation_time:
            details={"location":location,"previous_time_s":self.last_simulation_time,
                     "current_time_s":current,"previous_arm_target_rad":self.arm_target.tolist()}
            self.clock_discontinuity=details
            self.failure=f"SIMULATION_TIME_RESET: {details}"
            self.save_runtime_incident("SIMULATION_TIME_RESET",details)
            self.arm_target=self.data.qpos[self.jq].copy()
            p=self.active
            if p and not p.get("reserved"):
                p["error"]=self.failure;p["event"].set()
            self.get_logger().error(self.failure)
            # 재시작 전까지 physics를 진행하지 않는다. 충돌 검사를 우회하지 않는다.
            return False
        self.last_simulation_time=current
        return True

    def step(self):
        with self.lock:
            if not self.check_simulation_clock("before_physics_step"):return
            p=self.active
            if p and not p.get("reserved"):
                t=self.data.time-p["t0"]
                if t>=p["times"][-1]:target=p["q"][-1]
                else:
                    i=max(0,int(np.searchsorted(p["times"],t,side="right")-1))
                    h=p["times"][i+1]-p["times"][i];s=max(0,(t-p["times"][i])/h)
                    target=(2*s**3-3*s**2+1)*p["q"][i]+(s**3-2*s**2+s)*h*p["v"][i]+(-2*s**3+3*s**2)*p["q"][i+1]+(s**3-s**2)*h*p["v"][i+1]
                limits=self.model.jnt_range[[self.model.joint(n).id for n in JOINTS]]
                self.arm_target=np.clip(target,limits[:,0],limits[:,1])
                error=float(np.max(np.abs(self.data.qpos[self.jq]-target)))
                if t>0.5 and error>self.cfg["arm_path_tolerance_rad"]:
                    p["error"]=f"PATH_TRACKING_FAILED: {error:.4f} rad"
                if t>=p["times"][-1]+1:
                    if error>self.cfg["arm_goal_tolerance_rad"]:p["error"]=f"GOAL_NOT_REACHED: {error:.4f} rad"
                    p["event"].set()
            rate=0.035/self.cfg["grasp_close_duration_s"]*self.model.opt.timestep
            self.data.ctrl[self.gripper]+=np.clip(self.gripper_target-self.data.ctrl[self.gripper],-rate,rate)
            # 중력 및 속도 편향 토크를 기존 위치 서보의 입력 오프셋으로 보상한다.
            # 서보 게인, URDF 관절 한계와 구동력 제한은 그대로 유지한다.
            compensated=self.arm_target+self.data.qfrc_bias[self.jv]/self.model.actuator_gainprm[self.act,0]
            ranges=self.model.actuator_ctrlrange[self.act]
            self.data.ctrl[self.act]=np.clip(compensated,ranges[:,0],ranges[:,1])
            mujoco.mj_step(self.model,self.data)
            if not self.check_simulation_clock("inside_physics_step"):return
            _,unexpected=self.contacts()
            if unexpected and not self.failure:
                self.failure=f"UNEXPECTED_COLLISION: {unexpected[0]}"
                self.save_runtime_incident("UNEXPECTED_COLLISION",{"contacts":unexpected})
                self.get_logger().error(self.failure)
            if not np.isfinite(self.data.qpos).all() or any(w.number for w in self.data.warning):self.failure="PHYSICS_UNSTABLE"
            if self.base_state=="MOVING" and np.max(np.abs(self.data.qpos[self.jq]-self.cfg["folded_candidate_rad"]))>self.cfg["arm_goal_tolerance_rad"]:
                self.failure="FOLDED_HOLD_FAILED"
            if self.failure or (p and p.get("error")):
                self.arm_target=self.data.qpos[self.jq].copy()
                if p and not p.get("reserved"):
                    p["error"]=self.failure or p["error"];p["event"].set()

    def publish(self):
        with self.lock:
            names=[f"joint{i}" for i in range(1,9)]
            message=JointState();message.header.stamp=self.get_clock().now().to_msg();message.name=names
            message.position=[float(self.data.qpos[self.model.joint(n).qposadr[0]]) for n in names]
            message.velocity=[float(self.data.qvel[self.model.joint(n).dofadr[0]]) for n in names]
            self.js.publish(message)
            fingers,collisions=self.contacts()
            # 실제 손가락 접촉의 누르는 힘과 접선 방향 힘을 진단용으로 기록한다.
            contact_forces=[]
            for index,c in enumerate(self.data.contact):
                bodies=[self.model.body(int(self.model.geom_bodyid[g])).name for g in (c.geom1,c.geom2)]
                if 'cube' not in bodies or not any(n in ('link7','link8') for n in bodies):continue
                force=np.zeros(6);mujoco.mj_contactForce(self.model,self.data,index,force)
                contact_forces.append({'finger':next(n for n in bodies if n in ('link7','link8')),
                                       'normal_force_N':float(force[0]),
                                       'tangential_force_N':float(np.linalg.norm(force[1:3]))})
            payload={"stage":self.stage,"base_state":self.base_state,"failure":self.failure,
                     "simulation_time_s":float(self.data.time),
                     "finger_contact_forces":contact_forces,
                     "gripper_actuator_force_N":float(self.data.actuator_force[self.gripper]),
                     "T_world_tcp":tcp_transform(self.model,self.data).tolist(),
                     "T_world_object":body_transform(self.model,self.data,"cube").tolist(),
                     "jaw_opening_m":jaw_opening(self.model,self.data),"max_jaw_opening_m":self.max_width,
                     "T_world_base":body_transform(self.model,self.data,"base_link").tolist(),
                     "object_support_contact":any("table" in [self.model.geom(g).name for g in (c.geom1,c.geom2)] and "cube_geom" in [self.model.geom(g).name for g in (c.geom1,c.geom2)] and c.dist<0.0005 for c in self.data.contact),
                     "finger_contacts":fingers,"unexpected_contacts":collisions,"joint_positions":list(message.position)}
            self.status_pub.publish(String(data=json.dumps(payload)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare-only",action="store_true",help="Generate/check MoveIt model without opening the viewer")
    parser.add_argument("--headless",action="store_true",help="Execute the same physics and ROS loop without a viewer")
    args=parser.parse_args()
    model,data=load_model()
    print("Generated matching MoveIt model:",write_moveit_files(model),flush=True)
    print("FK consistency check:",verify_exported_fk(model,data),flush=True)
    if args.prepare_only:
        return
    rclpy.init();node=Bridge(model,data)
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    executor_errors=[]
    def spin_ros():
        try:
            executor.spin()
        except Exception as error:
            executor_errors.append(str(error))
            node.get_logger().error(f"ROS_EXECUTOR_FAILED: {error}; trajectory execution unavailable")
    thread=threading.Thread(target=spin_ros,daemon=True);thread.start()
    try:
        if args.headless:
            # 화면 유무와 무관하게 기존 브리지의 물리 step·접촉·추종 검사를 그대로 실행한다.
            print("MuJoCo headless bridge ready",flush=True)
            while rclpy.ok():
                if executor_errors and not node.failure:
                    node.failure=f"ROS_EXECUTOR_FAILED: {executor_errors[0]}"
                start=time.monotonic();node.step()
                time.sleep(max(0,model.opt.timestep-(time.monotonic()-start)))
            return
        with mujoco.viewer.launch_passive(model,data) as viewer:
            viewer.cam.lookat[:]=[0.30,0,0.75];viewer.cam.distance=2.6;viewer.cam.azimuth=135;viewer.cam.elevation=-25
            print("MuJoCo viewer ready. Keep this terminal open; start MoveIt in terminal 2.",flush=True)
            sync_at=0.
            while viewer.is_running() and rclpy.ok():
                if executor_errors and not node.failure:
                    node.failure=f"ROS_EXECUTOR_FAILED: {executor_errors[0]}"
                start=time.monotonic();node.step()
                if start-sync_at>1/30:
                    viewer.sync();sync_at=start
                time.sleep(max(0,model.opt.timestep-(time.monotonic()-start)))
    finally:
        with node.lock:
            if node.active and not node.active.get("reserved"):
                node.active["error"]="VIEWER_CLOSED";node.active["event"].set()
        executor.shutdown(timeout_sec=2);node.destroy_node();rclpy.try_shutdown()


if __name__=="__main__":main()
