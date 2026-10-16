#!/usr/bin/env python3
"""Passive graph/time probe. Creates subscriptions only; no motor/action/service calls."""
import argparse
import json
import time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

parser=argparse.ArgumentParser(); parser.add_argument('--seconds',type=float,default=6.)
a=parser.parse_args(); rclpy.init(args=[]); n=Node('manipulation_passive_audit')
tf=Buffer(); listener=TransformListener(tf,n)
samples={}; latest={}; tuples={k:deque(maxlen=6) for k in ('rgb','depth','info')}; pairs=[]
def sec(stamp): return stamp.sec+stamp.nanosec/1e9
def receive(topic,msg):
    now=n.get_clock().now().nanoseconds/1e9
    data=samples.setdefault(topic,deque(maxlen=100)); latest[topic]=msg
    if hasattr(msg,'header'):
        data.append(dict(stamp=sec(msg.header.stamp),received=now,age=now-sec(msg.header.stamp),frame=msg.header.frame_id))
    elif hasattr(msg,'data'):
        try: value=json.loads(msg.data)
        except Exception: value=msg.data[:1000]
        data.append(value)
    kind={'/wrist/wrist/color/image_raw':'rgb','/wrist/wrist/aligned_depth_to_color/image_raw':'depth','/wrist/wrist/color/camera_info':'info'}.get(topic)
    if kind:
        tuples[kind].append(msg)
        if all(tuples.values()):
            rgb=tuples['rgb'][-1]; stamp=sec(rgb.header.stamp)
            dep=min(tuples['depth'],key=lambda x:abs(sec(x.header.stamp)-stamp))
            info=min(tuples['info'],key=lambda x:abs(sec(x.header.stamp)-stamp))
            pairs.append((stamp,abs(sec(dep.header.stamp)-stamp),abs(sec(info.header.stamp)-stamp)))
            if len(pairs)>100: del pairs[0]
for topic,typ in [('/detection/head/objects',String),('/piper/joint_states_feedback',JointState),
                  ('/joint_states_feedback',JointState),('/gripper_states_feedback',JointState),('/odom',Odometry),
                  ('/wrist/wrist/color/image_raw',Image),('/wrist/wrist/aligned_depth_to_color/image_raw',Image),
                  ('/wrist/wrist/color/camera_info',CameraInfo)]:
    n.create_subscription(typ,topic,lambda m,t=topic:receive(t,m),qos_profile_sensor_data)
deadline=time.monotonic()+max(1,min(a.seconds,20))
while time.monotonic()<deadline: rclpy.spin_once(n,timeout_sec=.05)
report={'captured_at':n.get_clock().now().nanoseconds/1e9,'duration_s':a.seconds,'topics':{},'samples':{}}
for topic,types in n.get_topic_names_and_types():
    if any(key in topic for key in ('detection','manipulation','piper','joint','cmd_vel','wrist')):
        pubs=n.get_publishers_info_by_topic(topic)
        report['topics'][topic]={'types':types,'publishers':[dict(node=p.node_namespace+'/'+p.node_name,reliability=str(p.qos_profile.reliability),durability=str(p.qos_profile.durability),depth=p.qos_profile.depth) for p in pubs]}
for topic,values in samples.items():
    report['samples'][topic]={'count':len(values),'latest':list(values)[-1]}
    ages=[x['age'] for x in values if isinstance(x,dict) and 'age' in x]
    if ages: report['samples'][topic].update(min_age_s=min(ages),max_age_s=max(ages))
report['wrist_pairs']={'count':len(pairs),'within_25ms':sum(max(d,i)<=.025 for _,d,i in pairs)}
info=latest.get('/wrist/wrist/color/camera_info')
if info: report['intrinsics']={'width':info.width,'height':info.height,'k':list(info.k),'d':list(info.d),'frame':info.header.frame_id}
report['tf']={}
for parent,child in [('odom','base_link'),('base_link','wrist_color_optical_frame'),('map','base_link')]:
    try:
        tr=tf.lookup_transform(parent,child,Time())
        p=tr.transform.translation; q=tr.transform.rotation
        report['tf'][parent+'<-'+child]={'stamp':sec(tr.header.stamp),'xyz':[p.x,p.y,p.z],'q':[q.x,q.y,q.z,q.w]}
    except Exception as exc: report['tf'][parent+'<-'+child]={'error':str(exc)}
print(json.dumps(report,ensure_ascii=False,indent=2))
n.destroy_node(); rclpy.shutdown()
