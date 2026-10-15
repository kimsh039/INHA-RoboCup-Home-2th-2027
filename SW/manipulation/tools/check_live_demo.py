#!/usr/bin/env python3
"""Read-only config/ROS graph/feedback/TF report. Never enables or moves a robot."""
import argparse
import copy
import hashlib
import json
import time
from pathlib import Path
import numpy as np
import yaml

# Source-tree use before colcon install is intentionally supported.
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'manipulation_ws/src/robocup_real_manipulation'))
from robocup_real_manipulation.core import validate, rotation
from robocup_real_manipulation.tuning import overlay

FLAGS = ('execution_enabled','commissioned','calibration_verified','scene_verified',
         'model_names_verified','gravity_frame_verified','width_mapping_verified')
REQUIRED = ('joint_signs','joint_offsets','return_joints','topdown_quaternion',
            'tcp_to_grip_center_m','table_z_m','table_bounds_xy','base_arm_transform',
            'handeye_transform','link6_tcp_transform','obstacles','width_scale',
            'width_bias_m','driver_width_max','grasp_min_effort','can_port','firmware',
            'wrist_camera_model','wrist_camera_serial','camera_min_depth_m','coarse_z_error_bound_m')


def configuration(cfg):
    missing = [k for k in REQUIRED if cfg.get(k) is None]
    for name, rule in cfg.get('classes',{}).items():
        if not rule.get('enabled'): continue
        for k in ('class_id','center_height_m','grip_height_m','radius_m'):
            if rule.get(k) is None: missing.append('classes.'+name+'.'+k)
        if rule.get('axis_required') and rule.get('closing_axis_yaw_offset_rad') is None:
            missing.append('classes.'+name+'.closing_axis_yaw_offset_rad')
    flags = [k for k in FLAGS if cfg.get(k) is not True]
    if cfg.get('width_mapping_mode') == 'nominal_trial':
        flags = [k for k in flags if k != 'width_mapping_verified']
    issues = []
    try: validate(cfg)
    except Exception as e: issues.append(str(e))
    if not missing:
        # Inspect numeric settings independently of operator authorization flags.
        # This copy is NEVER written, launched, or sent to a robot.
        numeric = copy.deepcopy(cfg)
        for k in FLAGS: numeric[k] = True
        try: validate(numeric,hardware=True)
        except Exception as e: issues.append(str(e))
    calibrated = {}
    for key in ('base_arm_transform','handeye_transform','link6_tcp_transform'):
        tf = cfg.get(key)
        try:
            assert isinstance(tf,dict) and len(tf['xyz']) == 3 and np.isfinite(tf['xyz']).all()
            rotation(tf['quaternion_xyzw'])
            calibrated[key] = tf
        except Exception as e: issues.append('TRANSFORM:'+key+':'+str(e))
    return dict(missing_values=missing, unacknowledged_flags=flags, issues=issues,
                width_mapping_mode=cfg.get('width_mapping_mode','measured'),
                width_mapping_verified=cfg.get('width_mapping_verified',False),
                applied_transforms=calibrated, input_class_schema=cfg.get('model_names'),
                enabled_class_ids={k:v['class_id'] for k,v in cfg.get('classes',{}).items() if v.get('enabled')},
                hardware_config_ready=not (missing or flags or issues))


def ros_report(cfg, seconds):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState, CameraInfo, Image
    from piper_msgs.msg import PiperStatusMsg
    from robocup_real_manipulation_msgs.msg import Target, ApproachResult, BaseLease, ObservationRequest, WristObservation
    from robocup_real_manipulation_msgs.action import Manipulate, SetWidth
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.action import MoveGroup
    from tf2_ros import Buffer, TransformListener
    rclpy.init(); node = Node('manipulation_read_only_check')
    latest = {}; subscriptions = []; errors = []; snapshot = {}
    expected = {cfg['feedback_topic']:JointState, cfg['gripper_feedback_topic']:JointState,
                cfg['arm_status_topic']:PiperStatusMsg, '/manipulation/target':Target,
                '/manipulation/approach_result':ApproachResult,
                cfg['base_lease_topic']:BaseLease,
                '/manipulation/wrist/request':ObservationRequest,
                '/manipulation/wrist/result':WristObservation,
                '/wrist_camera/color/camera_info':CameraInfo,
                '/wrist_camera/aligned_depth_to_color/camera_info':CameraInfo,
                '/wrist_camera/depth/image_rect_raw':Image}
    def cb(topic, msg):
        latest[topic] = (time.monotonic(), msg)
    for topic, message_type in expected.items():
        # Runtime graph type check below reports incompatible external adapters.
        subscriptions.append(node.create_subscription(message_type,topic,lambda m,t=topic:cb(t,m),qos_profile_sensor_data))
    buffer = Buffer(); listener = TransformListener(buffer,node)
    clients = [(n,ActionClient(node,t,n)) for t,n in [(Manipulate,'/manipulation/run'),
               (SetWidth,'/manipulation/set_width'),(FollowJointTrajectory,'/arm_controller/follow_joint_trajectory'),
               (MoveGroup,'/move_action')]]
    started = time.monotonic()
    try:
        while time.monotonic()-started < seconds: rclpy.spin_once(node,timeout_sec=.05)
        now = node.get_clock().now().nanoseconds*1e-9
        topic_types = dict(node.get_topic_names_and_types())
        for topic, message_type in expected.items():
            expected_name = message_type.__module__.split('.')[0]+'/msg/'+message_type.__name__
            found = topic_types.get(topic,[])
            publishers=node.get_publishers_info_by_topic(topic)
            entry = dict(expected_type=expected_name, graph_types=found,
                         publishers=len(publishers),publisher_types=[x.topic_type for x in publishers],
                         observed=topic in latest)
            # Our subscriptions also appear in topic_types; count actual publishers.
            if any(x.topic_type != expected_name for x in publishers): errors.append('TYPE_MISMATCH:'+topic)
            if topic in latest:
                arrival, msg = latest[topic]; entry['receive_age_s'] = time.monotonic()-arrival
                if hasattr(msg,'header'):
                    stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                    entry.update(frame_id=msg.header.frame_id,stamp=stamp,source_age_s=now-stamp)
                if isinstance(msg,JointState):
                    entry.update(names=list(msg.name),positions=list(msg.position),efforts=list(msg.effort))
                elif isinstance(msg,CameraInfo):
                    entry.update(width=msg.width,height=msg.height,distortion_model=msg.distortion_model)
                elif isinstance(msg,Image):
                    entry.update(width=msg.width,height=msg.height,encoding=msg.encoding)
                elif isinstance(msg,PiperStatusMsg): entry['err_code'] = msg.err_code
                elif isinstance(msg,BaseLease): entry.update(task_id=msg.task_id,locked=msg.locked,stopped=msg.stopped)
                elif isinstance(msg,(Target,WristObservation)): entry.update(task_id=msg.task_id,class_id=msg.class_id)
            snapshot[topic] = entry
        for topic in (cfg['feedback_topic'],cfg['gripper_feedback_topic']):
            entry = snapshot[topic]
            if not entry['observed']: errors.append('NO_FEEDBACK:'+topic)
            elif not 0 <= entry.get('source_age_s',float('inf')) <= cfg['feedback_age_s']:
                errors.append('STALE_OR_FUTURE_FEEDBACK:'+topic)
        if cfg['feedback_topic'] in latest and cfg.get('joint_signs') is not None and cfg.get('joint_offsets') is not None:
            msg = latest[cfg['feedback_topic']][1]
            try:
                raw=np.array([msg.position[msg.name.index(n)] for n in cfg['driver_joint_names']])
                mapped=np.array(cfg['joint_signs'])*raw+cfg['joint_offsets']; limits=np.array(cfg['joint_limits'])
                snapshot['mapped_joint_positions_rad']=mapped.tolist()
                if not (np.isfinite(mapped).all() and np.all(mapped>=limits[:,0]) and np.all(mapped<=limits[:,1])):
                    errors.append('MAPPED_JOINT_OUTSIDE_LIMITS')
            except Exception as e: errors.append('JOINT_MAPPING:'+str(e))
        tf_records = {}
        for target,source in [(cfg['gravity_frame'],'piper_base_link'),('piper_link6',cfg['tcp_frame']),
                              ('piper_link6',cfg['wrist_optical_frame']),(cfg['gravity_frame'],'piper_link6')]:
            key=target+'<-'+source
            try:
                t=buffer.lookup_transform(target,source,Time())
                v=t.transform.translation; q=t.transform.rotation
                tf_records[key]=dict(xyz=[v.x,v.y,v.z],quaternion_xyzw=[q.x,q.y,q.z,q.w])
                expected_tf=None
                if target=='piper_link6' and source==cfg['tcp_frame']: expected_tf=cfg.get('link6_tcp_transform')
                elif target=='piper_link6' and source==cfg['wrist_optical_frame']: expected_tf=cfg.get('handeye_transform')
                elif target==cfg['gravity_frame'] and source=='piper_base_link': expected_tf=cfg.get('base_arm_transform')
                if expected_tf is not None:
                    actual=tf_records[key]
                    delta=float(np.linalg.norm(np.array(actual['xyz'])-expected_tf['xyz']))
                    Rdiff=rotation(actual['quaternion_xyzw'])@rotation(expected_tf['quaternion_xyzw']).T
                    angle=float(np.arccos(np.clip((np.trace(Rdiff)-1)/2,-1,1)))
                    actual.update(config_translation_difference_m=delta,config_rotation_difference_rad=angle)
                    if delta>.001 or angle>.001: errors.append('TF_CONFIG_MISMATCH:'+key)
            except Exception as e: tf_records[key]=dict(error=str(e)); errors.append('TF_MISSING:'+key)
        return dict(topics=snapshot,actions={n:c.server_is_ready() for n,c in clients},
                    tf=tf_records,errors=errors,
                    note='Idle detection/lease topics may have no sample until an actual task. Graph discovery is not execution proof.')
    finally:
        for _,client in clients: client.destroy()
        node.destroy_node(); rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True); p.add_argument('--tuning-file',type=Path)
    p.add_argument('--live',action='store_true'); p.add_argument('--seconds',type=float,default=5.)
    p.add_argument('--output',type=Path,required=True); a=p.parse_args()
    if not .5<=a.seconds<=30: raise SystemExit('seconds must be 0.5..30')
    cfg=yaml.safe_load(a.config.read_text())
    if a.tuning_file: cfg=overlay(cfg,yaml.safe_load(a.tuning_file.read_text()))
    report=dict(config_path=str(a.config.resolve()),
                config_sha256=hashlib.sha256(json.dumps(cfg,sort_keys=True).encode()).hexdigest(),
                configuration=configuration(cfg),read_only=True)
    if a.live: report['ros']=ros_report(cfg,a.seconds)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    def clean(value):
        if isinstance(value,float) and not np.isfinite(value): return str(value)
        if isinstance(value,dict): return {k:clean(v) for k,v in value.items()}
        if isinstance(value,list): return [clean(v) for v in value]
        return value
    report=clean(report)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2,allow_nan=False))


if __name__ == '__main__': main()
