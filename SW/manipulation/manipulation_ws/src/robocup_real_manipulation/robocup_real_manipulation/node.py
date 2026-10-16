import json
import os
import copy
from collections import deque
import hashlib
import math
import threading
import time
import uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from sensor_msgs.msg import JointState, CameraInfo
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String
from robocup_real_manipulation_msgs.msg import Target, ApproachResult, BaseLease, ObservationRequest, WristObservation, TaskStatus
from robocup_real_manipulation_msgs.action import Manipulate, SetWidth
from .core import (Fault, require, fresh, validate, EventJoin, rule_for, robust_depth,
                   transform_point, rotation, associate, grasp_geometry, support, observation_geometry, verify_axis_series)
from .integration import ProcessOwner, TerminalIDs, grasp_contact, hold_contact, lease_evidence, full_view_distance, scene_at_arrival, validate_integration
from .timing import Timing
from .storage import append_event
from .sequence import Sequence
from .wrist_geometry import mask_scene
from .motion import Motion
from .ros_util import read_config, seconds, wait_future, action_call, spin

class Coordinator(Node):
    def __init__(self):
        super().__init__('real_manipulation')
        self.c=validate(read_config(self)); self.scene_source=copy.deepcopy(self.c); self.declare_parameter('hardware_mode',False); self.group=ReentrantCallbackGroup(); self.mutex=threading.RLock()
        self.lease_requested_at=0.; self.camera_info=None; self.camera_info_received=0.
        self.timing=Timing(self)
        self.goal_handle=None; self.seq=None; self.last_status=None; self.lease=None; self.joint=None
        self.joint_stamp=0; self.gripper_feedback=None; self.still_since=None; self.observations=deque(maxlen=64); self.request_id=None
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self); self.cv=CvBridge()
        self.journal=Path(self.c['journal_path']).expanduser(); self.journal.parent.mkdir(parents=True,exist_ok=True)
        self.owner=ProcessOwner(str(self.journal)+'.owner')
        if not self.c.get('fixture_only',False):
            self.node_owner=ProcessOwner(self.c.get('coordinator_owner_path','~/.ros/robocup_real_manipulation/coordinator.owner'))
        saved=json.loads(self.journal.read_text()) if self.journal.exists() else {'completed':[],'locked_task':''}
        require(not saved.get('terminal_ledger') or self.journal.with_suffix('.sqlite3').is_file(),'TERMINAL_LEDGER_MISSING')
        self.join=EventJoin(); self.join.completed=TerminalIDs(self.journal.with_suffix('.sqlite3'),saved.get('completed',[])); self.locked_task=saved.get('locked_task','')
        self.lock_requested=bool(self.locked_task); self.release_task=''
        self.log=Path(self.c['log_path']).expanduser(); self.log.parent.mkdir(parents=True,exist_ok=True)
        self.config_sha256=hashlib.sha256(json.dumps(self.c,sort_keys=True).encode()).hexdigest()
        snapshot=self.log.parent/('config_'+self.config_sha256+'.json')
        if not snapshot.exists(): snapshot.write_text(json.dumps(self.c,ensure_ascii=False,indent=2))
        for old in sorted(self.log.parent.glob('config_*.json'),key=lambda p:p.stat().st_mtime,reverse=True)[20:]:
            if old!=snapshot: old.unlink()
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.lock_pub=self.create_publisher(BaseLease,self.c['base_lock_topic'],qos)
        self.status_pub=self.create_publisher(TaskStatus,'/manipulation/status',qos)
        self.result_pub=self.create_publisher(TaskStatus,'/manipulation/result',qos)
        self.request_pub=self.create_publisher(ObservationRequest,'/manipulation/wrist/request',10)
        self.create_subscription(Target,'/manipulation/target',lambda m:self.event('targets',m),10,callback_group=self.group)
        self.create_subscription(ApproachResult,'/manipulation/approach_result',lambda m:self.event('approaches',m),10,callback_group=self.group)
        self.create_subscription(BaseLease,self.c['base_lease_topic'],self.base,10,callback_group=self.group)
        self.create_subscription(CameraInfo,self.c.get('wrist_info_topic','/wrist/wrist/color/camera_info'),self.intrinsics,qos_profile_sensor_data,callback_group=self.group)
        self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data,callback_group=self.group)
        self.create_subscription(JointState,self.c['gripper_feedback_topic'],lambda m:setattr(self,'gripper_feedback',m),qos_profile_sensor_data,callback_group=self.group)
        self.create_subscription(WristObservation,'/manipulation/wrist/result',self.observation,10,callback_group=self.group)
        self.width=ActionClient(self,SetWidth,'/manipulation/set_width',callback_group=self.group)
        self.stop_client=self.create_client(Trigger,'/manipulation/stop',callback_group=self.group)
        self.server=ActionServer(self,Manipulate,'/manipulation/run',self.execute,goal_callback=self.accept,
                                 cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.create_timer(0.1,self.publish_lock,callback_group=self.group)
        # External pipeline can hand off a completed approach without another operator goal.
        self.declare_parameter('auto_start_on_approach',False)
        self.auto_attempted=set(); self.input_errors={}
        self.self_client=ActionClient(self,Manipulate,'/manipulation/run',callback_group=self.group)
        self.create_timer(0.1,self.auto_start,callback_group=self.group)
        self.create_timer(1.,self.prune_events,callback_group=self.group)
        self.motion=Motion(self,self.c,self.group)
        self.publish_lock()
        self.readiness_pub=self.create_publisher(String,'/manipulation/readiness',1)
        self.create_timer(1.,self.publish_readiness,callback_group=self.group)
        if not self.c['execution_enabled']: self.get_logger().info('READ-ONLY: physical goals rejected; fill config before commissioning')
    def publish_readiness(self):
        errors=[]
        if not self.get_parameter('hardware_mode').value: errors.append('READ_ONLY_MODE')
        try:
            validate(copy.deepcopy(self.c),hardware=True); validate_integration(self.c)
        except Exception as exc: errors.append(str(exc))
        for topic in (self.c['feedback_topic'],self.c['gripper_feedback_topic'],self.c['arm_status_topic'],
                      '/manipulation/wrist/result','/manipulation/base_lease'):
            if self.count_publishers(topic)!=1: errors.append('FEEDBACK_NOT_CONNECTED:'+topic)
        if not self.width.server_is_ready(): errors.append('GRIPPER_ACTION_NOT_CONNECTED')
        if self.joint_stamp<=0 or not -.05<=self.now()-self.joint_stamp<=self.c['feedback_age_s']:
            errors.append('MODEL_JOINT_FEEDBACK_STALE')
        if self.locked_task and self.goal_handle is None: errors.append('BASE_LOCK_RETAINED')
        self.readiness_pub.publish(String(data=json.dumps(dict(ready=not errors,errors=errors,
            hardware_mode=self.get_parameter('hardware_mode').value,locked_task=self.locked_task,
            base_completion_policy=self.c.get('base_completion_policy','hold'),config_sha256=self.config_sha256))))
    def prune_events(self):
        with self.mutex:
            for kind,age in (('targets',self.c['max_target_age_s']),('approaches',self.c['max_event_age_s'])):
                cache=getattr(self.join,kind)
                for task,msg in list(cache.items()):
                    if task!=self.join.active and self.now()-seconds(msg.header.stamp)>age:
                        del cache[task]; self.auto_attempted.discard(task)
    def intrinsics(self,msg):
        self.camera_info=msg; self.camera_info_received=self.now()
    def now(self): return self.get_clock().now().nanoseconds/1e9
    def auto_start(self):
        if not self.get_parameter('auto_start_on_approach').value or not self.get_parameter('hardware_mode').value:
            return
        from .external_input import auto_candidate
        with self.mutex:
            if self.locked_task or self.goal_handle or not self.self_client.server_is_ready(): return
            task_id=auto_candidate(self.join.targets,self.join.approaches,self.auto_attempted)
            if task_id is None: return
            self.auto_attempted.add(task_id)
            future=self.self_client.send_goal_async(Manipulate.Goal(task_id=task_id))
            def accepted(f):
                try:
                    if not f.result().accepted: self.get_logger().warning('Automatic handoff rejected: '+task_id)
                except Exception as e: self.get_logger().warning('Automatic handoff error: '+str(e))
            future.add_done_callback(accepted)
    def persist(self):
        data={'completed':[],'terminal_ledger':str(self.journal.with_suffix('.sqlite3')),'locked_task':self.locked_task}
        tmp=self.journal.with_suffix('.tmp')
        with tmp.open('w') as f:
            json.dump(data,f); f.flush(); os.fsync(f.fileno())
        tmp.replace(self.journal)
        directory=os.open(str(self.journal.parent),os.O_DIRECTORY)
        try: os.fsync(directory)
        finally: os.close(directory)
    def publish_lock(self):
        with self.mutex:
            msg=BaseLease(task_id=self.locked_task or self.release_task,locked=bool(self.locked_task) and self.lock_requested); msg.header.stamp=self.get_clock().now().to_msg()
            self.lock_pub.publish(msg)
    def event(self,kind,msg):
        try:
            fresh(seconds(msg.header.stamp),self.now(),self.c['max_target_age_s'] if kind=='targets' else self.c['max_event_age_s'])
            if kind=='targets':
                require(msg.header.frame_id and np.isfinite([msg.coarse_position.x,msg.coarse_position.y,msg.coarse_position.z]).all(),'INVALID_TARGET')
                rule_for(self.c,msg.class_id)
                # Class support comes from enabled geometry rules, not past trial success.
            else: require(msg.outcome in ('SUCCESS','FAILED','CANCELED'),'APPROACH_OUTCOME')
            with self.mutex: self.join.put(kind,msg.task_id,msg)
        except Exception as e:
            if msg.task_id and len(msg.task_id)<=128 and str(e) not in ('DUPLICATE_EVENT','PREVIOUS_TASK_EVENT','BUSY'):
                with self.mutex:
                    if len(self.input_errors)>=64: self.input_errors.pop(next(iter(self.input_errors)))
                    self.input_errors[msg.task_id]=str(e)
            self.get_logger().warning('event rejected: '+str(e))
    def base(self,msg):
        with self.mutex:
            if msg.task_id==self.locked_task: self.lease=msg
    def joints(self,msg):
        self.timing.sample('coordinator_arm',seconds(msg.header.stamp),self.now())
        try:
            fresh(seconds(msg.header.stamp),self.now(),self.c['feedback_age_s'])
            values=dict(zip(msg.name,msg.velocity)); positions=dict(zip(msg.name,msg.position))
            require(all(n in positions and n in values for n in self.c['joint_names']),'JOINT_DATA_MISSING')
            require(np.isfinite(list(positions.values())+list(values.values())).all(),'JOINT_DATA_INVALID')
            with self.mutex:
                stamp=seconds(msg.header.stamp)
                require(stamp>self.joint_stamp,'JOINT_STAMP_NOT_ADVANCING')
                self.joint=msg; self.joint_stamp=stamp
                if max(abs(values[n]) for n in self.c['joint_names'])<self.c['stable_speed_rad_s']:
                    if self.still_since is None: self.still_since=time.monotonic()
                else: self.still_since=None
        except Exception: self.still_since=None
    def observation(self,msg):
        self.timing.sample('wrist_result',seconds(msg.header.stamp),self.now(),observation_id=msg.observation_id)
        with self.mutex:
            if self.goal_handle and msg.task_id==self.goal_handle.request.task_id and msg.observation_id==self.request_id:
                if len(self.observations)<32: self.observations.append(msg)
    def accept(self,goal):
        with self.mutex:
            try:
                require(self.get_parameter('hardware_mode').value is True,'READ_ONLY_MODE')
                validate(self.c,hardware=True)
                validate_integration(self.c)
                require(not self.locked_task,'RECOVERY_LOCK_ACTIVE')
                self.join.begin(goal.task_id)
                self.locked_task=goal.task_id; self.persist(); self.lease=None; self.lock_requested=False; self.lease_requested_at=0.
                self.publish_lock()
                return GoalResponse.ACCEPT
            except Exception as e:
                self.get_logger().warning('goal rejected: '+str(e)); return GoalResponse.REJECT
    def guard(self):
        require(rclpy.ok(),'SHUTDOWN')
        require(not self.goal_handle or not self.goal_handle.is_cancel_requested,'CANCELED')
        # During WAIT_FOR_APPROACH, acquisition of base lock is a completion condition.
        if self.seq and self.seq.stage!='WAIT_FOR_APPROACH':
            with self.mutex: lease=self.lease
            lease_evidence(lease,self.locked_task,self.now(),self.c,self.lease_requested_at)
            fresh(self.joint_stamp,self.now(),self.c['feedback_age_s'])
            if self.seq.holding=='HELD' and self.seq.stage not in ('RELEASE','RETREAT','RETURN_ARM','COMPLETE'):
                grip=self.gripper_feedback
                require(grip and grip.name==['gripper'] and len(grip.position)==1 and len(grip.effort)==1,'HOLD_FEEDBACK_MISSING')
                fresh(seconds(grip.header.stamp),self.now(),self.c['feedback_age_s'])
                width=grip.position[0]*self.c['width_scale']+self.c['width_bias_m']
                try: hold_contact(width,grip.effort[0],self.held_width,self.c)
                except Fault:
                    self.seq.holding='UNKNOWN'
                    raise
    def until(self,predicate,timeout):
        deadline=time.monotonic()+timeout
        while True:
            self.guard()
            if predicate(): return
            require(time.monotonic()<deadline,'TIMEOUT'); time.sleep(0.01)
    def execute(self,handle):
        self.goal_handle=handle
        for attr in ('center','tcp','q','target','observation_tcp','observation_q','held_width','gripper_result','precise_observation'):
            if hasattr(self,attr): delattr(self,attr)
        self.seq=Sequence(self); outcome,reason=self.seq.run()
        result=Manipulate.Result(status=self.last_status)
        with self.mutex:
            self.join.finish()
            self.auto_attempted.discard(handle.request.task_id)
            self.input_errors.pop(handle.request.task_id,None)
            if outcome=='SUCCESS' and self.c.get('base_completion_policy','hold')=='release':
                self.release_task=self.locked_task; self.locked_task=''; self.lock_requested=False
            self.publish_lock()
            self.persist()
        self.result_pub.publish(self.last_status)
        if handle.is_cancel_requested: handle.canceled()
        elif outcome=='SUCCESS': handle.succeed()
        else: handle.abort()
        self.goal_handle=None; self.seq=None
        return result
    def finish_success(self,seq):
        if self.c.get('base_completion_policy','hold')=='hold':
            # Terminal pick/place has no following navigation. Require the real
            # fresh lock/stop evidence, then keep requesting that same lease.
            self.guard()
            self.lock_requested=True
            self.publish_lock()
            return 'BASE_LOCK_RETAINED'
        # Optional following-navigation flow requires a real release ACK.
        requested=self.now()
        self.lock_requested=False; self.lease=None; self.publish_lock()
        self.status(seq,'RUNNING','BASE_UNLOCK_REQUESTED')
        deadline=time.monotonic()+self.c.get('base_unlock_timeout_s',3.)
        try:
            while time.monotonic()<deadline:
                require(rclpy.ok(),'SHUTDOWN')
                require(not self.goal_handle.is_cancel_requested,'CANCELED')
                lease=self.lease
                if lease and lease.task_id==self.locked_task and not lease.locked and lease.stopped:
                    stamp=seconds(lease.header.stamp)
                    fresh(stamp,self.now(),self.c['base_age_s'])
                    if stamp>=requested: return
                time.sleep(.01)
            raise Fault('BASE_UNLOCK_UNCONFIRMED')
        except Exception:
            self.lock_requested=True; self.publish_lock()
            raise
    def status(self,seq,outcome,reason):
        msg=TaskStatus(task_id=self.goal_handle.request.task_id,stage=seq.stage,outcome=outcome,reason=reason,
                       holding=seq.holding,place_success=seq.place_success,return_success=seq.return_success,stop_confirmed=seq.stop_confirmed)
        msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id=self.c['gravity_frame']
        msg.class_id=self.target.class_id if hasattr(self,'target') else -1
        if hasattr(self,'observation_tcp'):
            msg.observation_tcp.header=msg.header; msg.observation_tcp.pose=self.motion.pose(self.observation_tcp,self.observation_q)
        if hasattr(self,'center'):
            msg.precise_center=self.point(self.center); msg.grasp_tcp.header=msg.header; msg.grasp_tcp.pose=self.motion.pose(self.tcp,self.q)
            msg.association_error_m=self.association_error
        self.last_status=msg; self.status_pub.publish(msg)
        self.goal_handle.publish_feedback(Manipulate.Feedback(status=msg))
        record=dict(task_id=msg.task_id,stage=msg.stage,outcome=outcome,reason=reason,holding=seq.holding,
                    config_sha256=self.config_sha256,stamp=self.now(),place_success=seq.place_success,return_success=seq.return_success,stop_confirmed=seq.stop_confirmed)
        if hasattr(self,'observation_tcp'): record.update(observation_tcp=self.observation_tcp.tolist())
        if hasattr(self,'center'): record.update(center=self.center.tolist(),grasp_tcp=self.tcp.tolist(),association_error_m=self.association_error)
        with append_event(self.log) as f: f.write(json.dumps(record)+'\n')
    def point(self,p):
        msg=PointStamped(); msg.header.frame_id=self.c['gravity_frame']; msg.header.stamp=self.get_clock().now().to_msg()
        msg.point.x,msg.point.y,msg.point.z=map(float,p); return msg
    def to_gravity(self,p,header,target_stamp=None):
        if target_stamp is None and header.frame_id==self.c['gravity_frame']: return np.array(p)
        require(bool(header.frame_id),'FRAME_MISSING')
        try:
            if target_stamp is None:
                tf=self.tf.lookup_transform(self.c['gravity_frame'],header.frame_id,Time.from_msg(header.stamp),timeout=Duration(seconds=0.1))
            else:
                tf=self.tf.lookup_transform_full(self.c['gravity_frame'],Time.from_msg(target_stamp),header.frame_id,Time.from_msg(header.stamp),self.c['coarse_fixed_frame'],timeout=Duration(seconds=0.1))
        except Exception as e: raise Fault('TF_MISSING_AT_CAPTURE') from e
        self.timing.sample('tf',seconds(header.stamp),self.now(),target_frame=self.c['gravity_frame'])
        t=tf.transform.translation; q=tf.transform.rotation
        return transform_point(p,[t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
    def width_move(self,width):
        result=action_call(self.width,SetWidth.Goal(width_m=width,timeout_s=self.c['stage_timeout_s']),self.c['stage_timeout_s'],self.guard)
        require(result.success,'GRIPPER_FAILED:'+result.reason); self.gripper_result=result
        with append_event(self.log) as f:
            f.write(json.dumps(dict(task_id=self.goal_handle.request.task_id,stage=self.seq.stage,event='GRIPPER_FEEDBACK',command_width_m=width,measured_width_m=result.measured_width_m,effort=result.effort,config_sha256=self.config_sha256,stamp=self.now()))+'\n')
    def stop(self):
        if not self.stop_client.wait_for_service(timeout_sec=0.5): return False
        return wait_future(self.stop_client.call_async(Trigger.Request()),self.c['stop_timeout_s']).success
    def wrist(self):
        c=self.c
        search=c.get('wrist_search_offsets_m',[[0.,0.],[.05,0.],[-.05,0.],[0.,.05]])
        for attempt in range(c['observation_retries']+1):
            if attempt:
                offset=np.array(search[attempt%len(search)],float)
                require(offset.shape==(2,) and np.linalg.norm(offset)<=c.get('wrist_search_radius_m',.08),'WRIST_SEARCH_BOUND')
                target=self.coarse.copy(); target[:2]+=offset
                require(support(target,self.rule['radius_m'],c),'WRIST_SEARCH_OUTSIDE_SUPPORT')
                position,q=observation_geometry(target,self.view_config,self.rule)
                self.motion.pose_move(position,q,self.guard)
            self.until(lambda:self.still_since is not None and time.monotonic()-self.still_since>=c['joint_stable_s'],c['stage_timeout_s'])
            self.request_id=uuid.uuid4().hex; self.observations=deque(maxlen=64); acquired_after=self.now()
            req=ObservationRequest(task_id=self.goal_handle.request.task_id,observation_id=self.request_id,class_id=self.target.class_id,instance_id=self.target.instance_id)
            req.header.stamp=self.get_clock().now().to_msg(); req.coarse_position=self.point(self.coarse); self.request_pub.publish(req)
            deadline=time.monotonic()+c['observation_timeout_s']; errors=[]; frames={}; stable=[]
            while time.monotonic()<deadline:
                self.guard()
                with self.mutex: batch=list(self.observations); self.observations.clear()
                complete=[]
                for obs in batch:
                    key=seconds(obs.header.stamp)
                    if not 1<=obs.candidate_count<=32 or obs.candidate_index>=obs.candidate_count: continue
                    entries=frames.setdefault(key,{})
                    entries[obs.candidate_index]=obs
                    if len(entries)==obs.candidate_count and all(o.candidate_count==obs.candidate_count for o in entries.values()):
                        complete.append(list(entries.values())); del frames[key]
                    if len(frames)>16: del frames[min(frames)]
                if len(complete)>1:
                    with self.mutex:
                        for rest in complete[1:]: self.observations.extend(rest)
                candidates=[]
                for obs in (complete[0] if complete else []):
                    try:
                        fresh(seconds(obs.header.stamp),self.now(),c['max_observation_age_s'])
                        require(seconds(obs.header.stamp)>=acquired_after,'IMAGE_BEFORE_REQUEST')
                        require(obs.class_id==self.target.class_id and c['min_confidence']<=obs.confidence<=1,'CLASS_OR_CONFIDENCE')
                        require(self.still_since is not None and seconds(obs.header.stamp)<=self.joint_stamp+c['future_tolerance_s'],'ARM_NOT_STABLE_AT_CAPTURE')
                        if obs.has_position:
                            require(math.isfinite(obs.position_std_m) and 0<obs.position_std_m<=c['max_position_std_m'],'POSITION_QUALITY')
                            p=np.array([obs.position.x,obs.position.y,obs.position.z]); kind=obs.position_kind
                        else:
                            for image in (obs.aligned_depth,obs.target_mask):
                                require(image.header.frame_id==obs.header.frame_id and abs(seconds(image.header.stamp)-seconds(obs.header.stamp))<0.025,'UNSYNCHRONIZED_DEPTH_MASK')
                            info=obs.camera_info
                            require(info.header.frame_id==obs.header.frame_id and abs(seconds(info.header.stamp)-seconds(obs.header.stamp))<0.025,'UNSYNCHRONIZED_INTRINSICS')
                            require(obs.aligned_depth.encoding in ('16UC1','32FC1') and obs.target_mask.encoding=='mono8','DEPTH_ENCODING')
                            depth=self.cv.imgmsg_to_cv2(obs.aligned_depth).astype(float)
                            if obs.aligned_depth.encoding=='16UC1': depth*=0.001
                            mask=self.cv.imgmsg_to_cv2(obs.target_mask)
                            require(info.width==depth.shape[1] and info.height==depth.shape[0] and not any(info.d),'RECTIFIED_INTRINSICS_REQUIRED')
                            p,_=robust_depth(depth,mask,info.k,c); kind='SURFACE'
                        if kind=='SURFACE':
                            # Camera optical +Z must face table vertically in the initial experiment.
                            tf=self.tf.lookup_transform(c['gravity_frame'],obs.header.frame_id,Time.from_msg(obs.header.stamp),timeout=Duration(seconds=.1))
                            q=tf.transform.rotation
                            require(np.dot(rotation([q.x,q.y,q.z,q.w])[:,2],[0,0,-1])>math.cos(math.radians(10)),'SURFACE_VIEW_NOT_TOPDOWN')
                        p=self.to_gravity(p,obs.header)
                        center,tcp,q=grasp_geometry(p,kind,self.rule,c,obs.axis_yaw if obs.axis_valid else None)
                        candidates.append((center,obs.instance_id,(center,tcp,q,obs)))
                    except Exception as e: errors.append(str(e))
                if candidates:
                    try: (center,tcp,q,selected_observation),error=associate(candidates,self.coarse,self.target.instance_id,c)
                    except Fault as exc:
                        errors.append(str(exc)); stable=[]; continue
                    stamp=seconds(complete[0][0].header.stamp)
                    if not stable or stamp>stable[-1][0]: stable.append((stamp,center,tcp,q,error))
                    stable=stable[-c['observation_stable_frames']:]
                    if len(stable)>=c['observation_stable_frames']:
                        positions=np.array([item[1] for item in stable])
                        try:
                            require(np.max(np.std(positions,axis=0))<=c['max_position_std_m'],'UNSTABLE_POSITION_SERIES')
                            if self.rule.get('axis_required'):
                                verify_axis_series([item[3] for item in stable],c.get('axis_stability_tolerance_rad',.15))
                        except Fault as exc:
                            errors.append(str(exc)); stable=[]; continue
                        self.center=np.mean(positions,axis=0); self.tcp=np.mean([item[2] for item in stable],axis=0)
                        self.q=q; self.association_error=error; self.precise_observation=selected_observation; return
                errors=errors[-16:]
                time.sleep(.01)
            self.get_logger().warning(f'wrist attempt {attempt}: '+','.join(errors[-5:]))
        raise Fault('WRIST_REDETECT_TIMEOUT'+(':'+errors[-1] if errors else ''))
    def perform(self,stage,seq):
        c=self.c; up=np.array([0,0,1.])
        if stage=='WAIT_FOR_APPROACH':
            for attr in ('center','tcp','q'):
                if hasattr(self,attr): delattr(self,attr)
            def ready():
                with self.mutex:
                    require(self.join.active not in self.input_errors,self.input_errors.get(self.join.active,''))
                    if not self.join.ready(): return False
                    t=self.join.targets[self.join.active]; a=self.join.approaches[self.join.active]
                    require(a.outcome=='SUCCESS','APPROACH_'+a.outcome)
                    require(a.base_stopped,'APPROACH_BASE_NOT_STOPPED')
                    fresh(seconds(t.header.stamp),self.now(),c['max_target_age_s']); fresh(seconds(a.header.stamp),self.now(),c['max_event_age_s'])
                    if not self.lock_requested:
                        self.lock_requested=True; self.lease_requested_at=self.now(); self.lease=None
                        self.publish_lock()
                    lease=self.lease
                    if not lease or not lease.locked or not lease.stopped: return False
                    lease_evidence(lease,self.locked_task,self.now(),c,self.lease_requested_at)
                    # Startup may precede the first valid measured arm feedback.
                    # Wait here; retain strict freshness checks during motion.
                    if self.joint is None: return False
                    try: fresh(self.joint_stamp,self.now(),c['feedback_age_s'])
                    except Fault: return False
                    require(t.header.frame_id==c['coarse_fixed_frame'],'COARSE_NOT_FROZEN_BEFORE_APPROACH')
                    require(self.scene_source.get('scene_frame')==c['coarse_fixed_frame'],'SCENE_NOT_CONFIGURED_IN_FIXED_FRAME')
                    tf=self.tf.lookup_transform(c['gravity_frame'],c['coarse_fixed_frame'],Time.from_msg(lease.header.stamp),timeout=Duration(seconds=.1))
                    p=tf.transform.translation; q=tf.transform.rotation
                    arrived=scene_at_arrival(self.scene_source,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
                    c.update(arrived)
                    self.target=t; self.rule=rule_for(c,t.class_id)
                    self.coarse=self.to_gravity([t.coarse_position.x,t.coarse_position.y,t.coarse_position.z],t.header,lease.header.stamp)
                    return True
            self.until(ready,c['approach_timeout_s'])
        elif stage=='MOVE_TO_OBSERVATION':
            self.motion.setup_scene(self.coarse,self.rule,self.guard,padding=c['coarse_collision_padding_m'])
            if c['observation_joints'] is not None:
                self.status(seq,'RUNNING','MOVE_TO_INITIAL_POSE')
                with self.mutex:
                    values=dict(zip(self.joint.name,self.joint.position))
                require(all(math.isfinite(values[n]) for n in c['joint_names']),
                        'INVALID_INITIAL_POSE_FEEDBACK')
                if any(abs(values[n]-q)>c['goal_tolerance_rad'] for n,q in
                       zip(c['joint_names'],c['observation_joints'])):
                    self.motion.joint_move(c['observation_joints'],self.guard)
            require(self.camera_info is not None,'WRIST_CAMERA_INFO_MISSING')
            fresh(seconds(self.camera_info.header.stamp),self.now(),c['max_observation_age_s'])
            if not c.get('fixture_only',False):
                info=self.camera_info
                tf=self.tf.lookup_transform('piper_link6',info.header.frame_id,Time.from_msg(info.header.stamp),timeout=Duration(seconds=.1))
                t=tf.transform.translation; q=tf.transform.rotation
                actual=np.array([q.x,q.y,q.z,q.w]); expected=np.array(c['handeye_transform']['quaternion_xyzw'])
                require(np.linalg.norm(np.array([t.x,t.y,t.z])-c['handeye_transform']['xyz'])<=.002 and abs(np.dot(actual,expected))>=math.cos(.02/2),'LIVE_HAND_EYE_CONFIG_MISMATCH')
            view=dict(c); view['observation_camera_distance_m']=full_view_distance(self.camera_info,self.rule,c)
            self.view_config=view
            position,q=observation_geometry(self.coarse,view,self.rule)
            self.observation_tcp=position; self.observation_q=q
            self.status(seq,'RUNNING','COARSE_TOPDOWN_OBSERVATION_TARGET')
            self.motion.pose_move(position,q,self.guard)
        elif stage=='WRIST_REDETECT': self.wrist()
        elif stage=='COMPUTE_GRASP':
            # The wrist class was independently checked against the selected target.
            self.rule=rule_for(c,self.target.class_id)
            require(support(self.center,self.rule['radius_m'],c),'GRASP_OUTSIDE_SUPPORT')
            obs=self.precise_observation
            fresh(seconds(obs.header.stamp),self.now(),c['max_observation_age_s'])
            require(obs.aligned_depth.encoding in ('16UC1','32FC1') and obs.target_mask.encoding=='mono8','MEASURED_SCENE_MISSING')
            require(all(m.header.frame_id==obs.header.frame_id and abs(seconds(m.header.stamp)-seconds(obs.header.stamp))<=.025 for m in (obs.aligned_depth,obs.target_mask,obs.camera_info)),'SCENE_CAPTURE_MISMATCH')
            depth=self.cv.imgmsg_to_cv2(obs.aligned_depth).astype(float)
            if obs.aligned_depth.encoding=='16UC1': depth*=.001
            tf=self.tf.lookup_transform(c['gravity_frame'],obs.header.frame_id,Time.from_msg(obs.header.stamp),timeout=Duration(seconds=.1))
            t=tf.transform.translation; q=tf.transform.rotation
            c['wrist_obstacles']=mask_scene(depth,self.cv.imgmsg_to_cv2(obs.target_mask),obs.camera_info,rotation([q.x,q.y,q.z,q.w]),[t.x,t.y,t.z],self.center,self.rule,c)
            self.motion.setup_scene(self.center,self.rule,self.guard)
        elif stage=='MOVE_TO_PREGRASP':
            self.width_move(self.rule['open_m']); self.motion.pose_move(self.tcp+up*c['pregrasp_m'],self.q,self.guard)
        elif stage=='DESCEND':
            self.motion.contacts(c['touch_links'],self.guard); self.motion.cart_move(self.tcp,self.q,self.guard)
        # [TUNE:GRIPPER] Physical width targets are classes.<name>.open_m / close_m.
        elif stage=='CLOSE_GRIPPER': self.width_move(self.rule['close_m'])
        elif stage=='VERIFY_GRASP':
            # Close completion isn't contact evidence. Calibrated effort AND width are mandatory.
            r=self.gripper_result
            grasp_contact(r.measured_width_m,r.effort,self.rule,c)
            self.held_width=r.measured_width_m
            self.motion.contacts(c['touch_links']+['manip_table'],self.guard)
            self.motion.attach(self.center,self.rule,self.guard)
        elif stage=='LIFT':
            self.motion.cart_move(self.tcp+up*c['lift_m'],self.q,self.guard)
            self.motion.contacts(c['touch_links'],self.guard)
        elif stage=='MOVE_TO_PLACE':
            self.place_tcp=None
            for sign in (-1,1):  # right=-Y in the level robot frame
                candidate=self.tcp+np.array([0,sign*c['place_offset_m'],0])
                center=self.center+np.array([0,sign*c['place_offset_m'],0])
                if not support(center,self.rule['radius_m'],c): continue
                try:
                    self.motion.contacts(c['touch_links'],self.guard)
                    hover=self.motion.plan_to(candidate+up*c['lift_m'],self.q,guard=self.guard)
                    self.motion.contacts(c['touch_links']+['manip_table'],self.guard)
                    descent=self.motion.cart_plan(candidate,self.q,self.guard,start=hover)
                    self.motion.contacts(c['touch_links'],self.guard)
                except Fault as e:
                    if str(e) not in ('PLAN_FAILED','CARTESIAN_PATH_INCOMPLETE'): raise
                    continue
                self.motion.execute(hover,self.guard)
                self.motion.contacts(c['touch_links']+['manip_table'],self.guard)
                # Replan descent from measured current state, don't execute stale preflight.
                self.motion.cart_move(candidate,self.q,self.guard)
                self.place_tcp=candidate; self.place_center=center; break
            require(self.place_tcp is not None,'NO_FEASIBLE_PLACE_HOLDING_OBJECT')
        elif stage=='RELEASE':
            self.width_move(self.rule['open_m']); seq.holding='EMPTY'; seq.place_success=True
            self.motion.detach(self.place_center,self.rule,self.guard)
        elif stage=='RETREAT': self.motion.cart_move(self.place_tcp+up*c['retreat_m'],self.q,self.guard)
        elif stage=='RETURN_ARM': self.motion.joint_move(c['return_joints'],self.guard)
        elif stage=='COMPLETE':
            self.until(lambda:self.still_since is not None and time.monotonic()-self.still_since>=c['joint_stable_s'],c['stop_timeout_s'])
            seq.stop_confirmed=True

def main():
    rclpy.init(); spin(Coordinator())
