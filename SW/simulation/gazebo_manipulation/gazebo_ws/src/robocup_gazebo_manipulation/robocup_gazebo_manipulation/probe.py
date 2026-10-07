"""실제 Gazebo bridge의 joint/odom/RGB-D/CameraInfo/TF 수신을 검증한다."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo,JointState
from nav_msgs.msg import Odometry
from tf2_ros import Buffer,TransformListener
from .generate import ARM,FINGERS


class Probe(Node):
    def __init__(self):
        super().__init__('gazebo_manipulation_probe');self.messages={};self.counts={};self.arrivals={};self.subs=[]
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        topics={'joints':('/joint_states',JointState),'odom':('/odom',Odometry)}
        for camera in ('head','wrist'):
            root='/'+camera+'_camera'
            topics.update({camera+'_rgb':(root+'/color/image_raw',Image),camera+'_depth':(root+'/aligned_depth_to_color/image_raw',Image),camera+'_info':(root+'/color/camera_info',CameraInfo)})
        from rclpy.qos import qos_profile_sensor_data
        for key,(topic,typ) in topics.items():
            self.subs.append(self.create_subscription(typ,topic,lambda m,k=key:self.capture(k,m),qos_profile_sensor_data))

    def capture(self,key,msg):
        self.messages[key]=msg;self.counts[key]=self.counts.get(key,0)+1;self.arrivals[key]=time.monotonic()

    def report(self):
        out={'scope':'actual Gazebo ROS bridge, not real D435/PiPER','result':'FAIL','counts':self.counts,'cameras':{}}
        required=['joints','odom']+[c+'_'+k for c in ('head','wrist') for k in ('rgb','depth','info')]
        missing=[k for k in required if self.counts.get(k,0)<3 or time.monotonic()-self.arrivals.get(k,0)>2]
        out['missing_or_stale']=missing
        if missing:return out
        q=dict(zip(self.messages['joints'].name,self.messages['joints'].position))
        out['joint_positions']={n:q.get(n) for n in ARM+FINGERS}
        if any(n not in q for n in ARM+FINGERS):return out
        valid=True
        for c in ('head','wrist'):
            rgb,depth,info=(self.messages[c+'_'+k] for k in ('rgb','depth','info'))
            dtype=np.dtype('>f4' if depth.is_bigendian else '<f4') if depth.encoding=='32FC1' else np.dtype('>u2' if depth.is_bigendian else '<u2')
            if depth.encoding not in ('32FC1','16UC1'):valid=False;continue
            values=np.frombuffer(bytes(depth.data),dtype=dtype).reshape(depth.height,depth.step//dtype.itemsize)[:,:depth.width].astype(float)
            if depth.encoding=='16UC1':values*=.001
            mask=np.isfinite(values)&(values>0)
            good=rgb.width==depth.width==info.width and rgb.height==depth.height==info.height and info.k[0]>0 and info.k[4]>0 and mask.any()
            stamps=[m.header.stamp.sec+m.header.stamp.nanosec/1e9 for m in (rgb,depth,info)]
            good=bool(good and max(stamps)-min(stamps)<.15 and rgb.header.frame_id==depth.header.frame_id==info.header.frame_id)
            out['cameras'][c]={'width':rgb.width,'height':rgb.height,'depth_encoding':depth.encoding,'optical_frame':info.header.frame_id,
                'valid_depth_fraction':float(mask.mean()),'median_depth_m':float(np.median(values[mask])) if mask.any() else None,
                'source_sync_span_s':max(stamps)-min(stamps),'valid':good}
            valid&=good
        try:
            t=self.buffer.lookup_transform('manipulation_tcp','wrist_camera_optical_frame',rclpy.time.Time())
            out['tcp_to_wrist']={'translation_m':[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
                'quaternion_xyzw':[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]}
        except Exception as e:out['tf_error']=str(e);valid=False
        out['result']='PASS' if valid else 'FAIL';return out


def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=15);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(rclpy.utilities.remove_ros_args()[1:]);rclpy.init();node=Probe()
    try:
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        report=node.report();args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
        if report['result']!='PASS':raise SystemExit(1)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
