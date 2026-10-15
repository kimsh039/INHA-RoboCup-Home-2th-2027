"""Manipulation-side receiver for the existing Head/fusion topics."""
from collections import OrderedDict
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import copy
import numpy as np
from robocup_detection_msgs.msg import HeadTarget
from robocup_real_manipulation_msgs.msg import Target
from .external_input import capture_key, check_pair
from .ros_util import read_config
from .core import require, transform_point
from .wrist_geometry import surface_point


class HeadInput(Node):
    def __init__(self):
        super().__init__('manipulation_head_input')
        self.c = read_config(self)
        self.heads, self.points = OrderedDict(), OrderedDict()
        self.emitted = OrderedDict()
        # [INTERFACE:EXTERNAL_TOPICS] Only subscribers; never launches the detector/Nav.
        head_topic = self.declare_parameter('head_topic', '/detection/head/target').value
        point_topic = self.declare_parameter('point_topic', '/detection/target_map_point').value
        self.age = float(self.declare_parameter('source_max_age_s', 2.).value)
        # Optional measured RGB-D conversion of existing Head boxes. Not a new
        # detector; use only when the external module publishes no 3D point.
        self.mode=self.declare_parameter('point_source','fusion').value
        self.freeze=self.declare_parameter('freeze_to_fixed_frame',True).value
        require(self.mode in ('fusion','aligned_depth'),'POINT_SOURCE')
        self.depths=OrderedDict(); self.infos=OrderedDict()
        if self.freeze or self.mode=='aligned_depth':
            self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        if self.mode=='aligned_depth':
            self.cv=CvBridge()
            depth_topic=self.declare_parameter('head_depth_topic','/head/camera/aligned_depth_to_color/image_raw').value
            info_topic=self.declare_parameter('head_info_topic','/head/camera/color/camera_info').value
            self.create_subscription(Image,depth_topic,lambda m:self.depth_input('depth',m),qos_profile_sensor_data)
            self.create_subscription(CameraInfo,info_topic,lambda m:self.depth_input('info',m),qos_profile_sensor_data)
        output_topic = self.declare_parameter('output_topic', '/manipulation/target').value
        self.pub = self.create_publisher(Target, output_topic, QoSProfile(depth=6,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(HeadTarget, head_topic, lambda m: self.receive('head', m), 6)
        if self.mode=='fusion':
            self.create_subscription(PointStamped, point_topic, lambda m: self.receive('point', m), 6)

    def depth_input(self,kind,msg):
        cache=self.depths if kind=='depth' else self.infos
        key=capture_key(msg.header); cache[key]=msg
        if len(cache)>6: cache.popitem(last=False)
        if key in self.heads: self.make_depth_point(key)

    def make_depth_point(self,key):
        if key not in self.heads or key not in self.depths or key not in self.infos: return
        head=self.heads[key]; depth_msg=self.depths.pop(key); info=self.infos.pop(key)
        try:
            require(head.header.frame_id==depth_msg.header.frame_id==info.header.frame_id,'HEAD_DEPTH_FRAME')
            require(depth_msg.encoding in ('16UC1','32FC1'),'HEAD_DEPTH_ENCODING')
            depth=self.cv.imgmsg_to_cv2(depth_msg).astype(float)
            if depth_msg.encoding=='16UC1': depth*=.001
            mask=np.zeros(depth.shape,np.uint8)
            x0=max(0,int(head.x)); y0=max(0,int(head.y))
            x1=min(depth.shape[1],int(head.x+head.width)); y1=min(depth.shape[0],int(head.y+head.height))
            require(x1>x0 and y1>y0,'HEAD_BBOX')
            mask[y0:y1,x0:x1]=1
            config=copy.deepcopy(self.c); config['camera_min_depth_m']=.2; config['depth_max_m']=4.
            point,_=surface_point(depth,mask,info,config,.003)
            tf=self.tf.lookup_transform(self.c['gravity_frame'],head.header.frame_id,
                Time.from_msg(head.header.stamp),timeout=Duration(seconds=.1))
            t=tf.transform.translation; q=tf.transform.rotation
            position=transform_point(point,[t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
            rule=self.c['classes'].get(head.class_name)
            require(rule and rule['enabled'] and rule['center_height_m'] is not None,'HEAD_CLASS_GEOMETRY')
            z=self.c['table_z_m']; height=2*rule['center_height_m']
            require(z is not None and z+.005<=position[2]<=z+height+.06,'HEAD_POINT_NOT_OBJECT_ON_TABLE')
            # Coarse XY is measured visible surface, not a precision center. For
            # tabletop objects use known center height; wrist remeasurement is mandatory.
            position[2]=z+rule['center_height_m']
            output=PointStamped(header=copy.deepcopy(head.header)); output.header.frame_id=self.c['gravity_frame']
            output.point.x,output.point.y,output.point.z=map(float,position)
            self.receive('point',output)
        except Exception as exc: self.get_logger().warning('Head RGB-D input rejected: '+str(exc))

    def receive(self, kind, msg):
        key = capture_key(msg.header)
        cache = self.heads if kind == 'head' else self.points
        cache[key] = msg
        if len(cache) > 6:
            cache.popitem(last=False)
        if kind=='head' and self.mode=='aligned_depth':
            self.make_depth_point(key)
        if key not in self.heads or key not in self.points:
            return
        head, point = self.heads.pop(key), self.points.pop(key)
        try:
            task_id = check_pair(head, point, self.c, self.get_clock().now().nanoseconds / 1e9, self.age)
            if task_id in self.emitted:
                return
            if self.freeze:
                fixed=self.c['coarse_fixed_frame']
                if point.header.frame_id!=fixed:
                    tf=self.tf.lookup_transform(fixed,point.header.frame_id,Time.from_msg(point.header.stamp),
                        timeout=Duration(seconds=.1))
                    point=freeze_point(point,fixed,tf.transform)
            # Keep the fusion frame and SOURCE acquisition time, not receipt time.
            output = Target(header=point.header, task_id=task_id, instance_id=head.target_id,
                            class_id=head.class_id, coarse_position=point.point)
            self.pub.publish(output)
            self.emitted[task_id] = None
            if len(self.emitted) > 1024:
                self.emitted.popitem(last=False)
        except Exception as exc:
            self.get_logger().warning('External input rejected: ' + str(exc))


def freeze_point(point,frame,transform):
    """Snapshot at ACQUISITION into odom; retain source timestamp, no aging reset."""
    output=copy.deepcopy(point)
    t=transform.translation; q=transform.rotation; p=point.point
    value=transform_point([p.x,p.y,p.z],[t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
    output.header.frame_id=frame
    output.point.x,output.point.y,output.point.z=map(float,value)
    return output


def main():
    rclpy.init()
    node = HeadInput()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
