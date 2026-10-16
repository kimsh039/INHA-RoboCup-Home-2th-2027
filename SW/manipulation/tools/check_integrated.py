#!/usr/bin/env python3
"""Fail-closed physical dependency check. Does not publish or call any robot service/action."""
import argparse
import json
from pathlib import Path
import time
import yaml
from robocup_real_manipulation.core import validate
from robocup_real_manipulation.integration import validate_integration

p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--vision',required=True)
p.add_argument('--repo',required=True);p.add_argument('--runtime',action='store_true');a=p.parse_args()
c=yaml.safe_load(Path(a.config).read_text());v=yaml.safe_load(Path(a.vision).read_text());errors=[]
try:validate(c,hardware=True);validate_integration(c)
except Exception as e:errors.append(str(e))
if c.get('scene_frame')!=c.get('coarse_fixed_frame') or c.get('coarse_fixed_frame') not in ('odom','map'):errors.append('FIXED_SCENE_FRAME_REQUIRED')
if c.get('startup_prefix_allowed_contacts'):errors.append('UNREVIEWED_STARTUP_COLLISION_EXCEPTIONS')
if v.get('mask_source') not in ('sam2','yolo_seg'):errors.append('CLASS_VERIFIED_MASK_SOURCE_REQUIRED:sam2_auto_is_not_class_redetection')
for key in ('yolo_model','sam_checkpoint'):
    if key=='sam_checkpoint' and v.get('mask_source')!='sam2':continue
    path=Path(v.get(key,'')).expanduser();path=path if path.is_absolute() else Path(a.repo)/path
    if not path.is_file():errors.append('MODEL_MISSING:'+str(path))
if v.get('color_topic','').startswith('/wrist/calibrated') and '/aligned_depth_to_color/' in v.get('depth_topic',''):errors.append('CORRECTED_RGB_FACTORY_ALIGNED_DEPTH_MISMATCH')
if a.runtime:
    import rclpy
    from rclpy.node import Node
    rclpy.init(args=[]);n=Node('manipulation_dependency_check')
    end=time.monotonic()+2.
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    required={v['color_topic']:'sensor_msgs/msg/Image',v['depth_topic']:'sensor_msgs/msg/Image',v['camera_info_topic']:'sensor_msgs/msg/CameraInfo',
        '/manipulation/target':'robocup_real_manipulation_msgs/msg/Target',
        '/manipulation/approach_result':'robocup_real_manipulation_msgs/msg/ApproachResult',c['base_lease_topic']:'robocup_real_manipulation_msgs/msg/BaseLease',
        c['feedback_topic']:'sensor_msgs/msg/JointState',c['gripper_feedback_topic']:'sensor_msgs/msg/JointState',c['arm_status_topic']:'piper_msgs/msg/PiperStatusMsg'}
    topics=dict(n.get_topic_names_and_types())
    for topic,typ in required.items():
        if typ not in topics.get(topic,[]):errors.append('TOPIC_TYPE_MISSING:'+topic+':'+typ)
        elif n.count_publishers(topic)!=1:errors.append('PUBLISHER_COUNT_NOT_ONE:'+topic+':'+str(n.count_publishers(topic)))
    for topic in (c['command_topic'],'/arm_controller/follow_joint_trajectory/_action/status','/manipulation/wrist/result'):
        if n.count_publishers(topic):errors.append('EXISTING_CONTROLLER_OR_WRIST_OWNER:'+topic)
    n.destroy_node();rclpy.shutdown()
print(json.dumps(dict(ready=not errors,errors=errors,enabled_classes=[name for name,rule in c['classes'].items() if rule['enabled']],physical_validation=c.get('physical_validation',{})),ensure_ascii=False,indent=2))
raise SystemExit(1 if errors else 0)
