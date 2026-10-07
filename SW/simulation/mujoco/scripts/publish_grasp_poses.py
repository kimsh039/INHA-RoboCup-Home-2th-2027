"""RViz-only inspection of untouched model grasps; never commands the robot."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import Point,TransformStamped
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
from scripts.publish_dummy_pointcloud import load_message
from manipulation.sim_model import ROOT,config,transform,target_tcp
from scripts.derive_pregrasps import derive
from scripts.diagnose_table_grasps import gripper_boxes,obb_intersects_aabb


def point(v):return Point(x=float(v[0]),y=float(v[1]),z=float(v[2]))
def box_edges(center,half):
    from itertools import product
    vertices=np.array(list(product((-1,1),repeat=3)))*half+center
    lines=[]
    for i in range(8):
        for j in range(i+1,8):
            if np.count_nonzero(vertices[i]!=vertices[j])==1:lines.extend([vertices[i],vertices[j]])
    return np.array(lines)


def build_markers(grasps,cfg,cube_dimensions=None,sensor_metadata=None,cloud_points=None):
    """All source poses/widths untouched. Gripper outline is the API visualization template."""
    markers=[]
    def marker(ns,kind,frame='object',color=(1,1,1,1)):
        m=Marker();m.header.frame_id=frame;m.ns=ns;m.id=len(markers);m.type=kind;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.color.r,m.color.g,m.color.b,m.color.a=map(float,color)
        markers.append(m);return m
    def lines(ns,points,color,thickness=.001,frame='object'):
        m=marker(ns,Marker.LINE_LIST,frame=frame,color=color);m.scale.x=thickness;m.points=[point(p) for p in points];return m
    def axes(ns,p,R,length=.025,alpha=1.,frame='object'):
        for i,color in enumerate(((1,0,0,alpha),(0,1,0,alpha),(0,0,1,alpha))):
            m=marker(ns,Marker.ARROW,frame=frame,color=color);m.scale.x=.0015;m.scale.y=.0035;m.scale.z=.005;m.points=[point(p),point(p+length*R[:,i])]
    if sensor_metadata is not None and sensor_metadata.get('pipeline') in ('mid360_mujoco_roi_v1','wrist_camera_mujoco_roi_v1'):
        wrist=sensor_metadata['pipeline']=='wrist_camera_mujoco_roi_v1'
        T=np.array(sensor_metadata['T_world_camera' if wrist else 'T_world_lidar']);O=np.array(sensor_metadata['T_world_object'])
        axes('lidar_axes',T[:3,3],T[:3,:3],.08,1.,'world')
        origin=marker('lidar_origin',Marker.SPHERE,'world',(.3,.5,1,1))
        origin.pose.position=point(T[:3,3]);origin.scale.x=origin.scale.y=origin.scale.z=.025
        label=marker('lidar_label',Marker.TEXT_VIEW_FACING,'world');label.pose.position=point(T[:3,3]+[0,0,.07]);label.scale.z=.025;label.text='Wrist camera: URDF optical frame' if wrist else 'MID360: URDF livox_frame'
        if cloud_points is not None:
            rays=[]
            for p in cloud_points[::max(1,len(cloud_points)//24)]:rays.extend([T[:3,3],O[:3,:3]@p+O[:3,3]])
            lines('lidar_rays',rays,(.6,.7,1,.35),.0005,'world')
    if cube_dimensions is not None:
        size=np.asarray(cube_dimensions,dtype=float)
        if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0):raise ValueError('CUBE_DIMENSIONS_INVALID')
        lines('reference_cube',box_edges(np.zeros(3),size/2),(.7,.7,.7,.6),.0005)
    for g in grasps:
        p=np.array(g['translation']);R=np.array(g['rotation_matrix']);pre=derive(R,p,cfg['pregrasp_distance_m']);pre_p=np.array(pre['T_object_pregrasp'])[:3,3]
        name=f"candidate_{g['candidate_id']}";axes('grasp_axes',p,R);axes('pregrasp_axes',pre_p,R,.018,.5)
        lines('approach_segments',[pre_p,p],(.3,.8,1,.65),.0007)
        w=g['width'];depth=g['depth'];finger=.004;back=.02;height=.004
        # Exact local box bounds used in GraspNetAPI plot_gripper_pro_max.
        parts=[([-back-finger,-w/2-finger,-height/2],[depth,-w/2,height/2]),
               ([-back-finger,w/2,-height/2],[depth,w/2+finger,height/2]),
               ([-back-finger,-w/2,-height/2],[-back,w/2,height/2]),
               ([-.04-back-finger,-finger/2,-height/2],[-back-finger,finger/2,height/2])]
        vertices=[]
        for low,high in parts:
            low=np.array(low);high=np.array(high);vertices.extend(box_edges((low+high)/2,(high-low)/2))
        color=(1,.2,.8,.9) if w>cfg['hardware_max_jaw_opening_m'] else (.2,1,.7,.9)
        lines('model_gripper',np.array(vertices)@R.T+p,color)
        label=marker('labels',Marker.TEXT_VIEW_FACING);label.pose.position=point(p+[0,0,.04]);label.scale.z=.009
        label.text=f"id={g['candidate_id']} score={g['score']:.3f}\nw={w*1000:.1f}mm d={depth*1000:.0f}mm"
        # Keep the unverified tutorial TCP mapping visually separate and optional in RViz.
        tcp=target_tcp(g,np.eye(4));axes('mapped_tcp_axes',tcp[:3,3],tcp[:3,:3],.018,.65)
    table=marker('table',Marker.CUBE,'world',(.5,.35,.2,.25));table.pose.position=point(cfg['table_center_world_m']);table.scale.x,table.scale.y,table.scale.z=map(float,2*np.array(cfg['table_half_size_m']))
    origin=marker('object_center',Marker.SPHERE,color=(1,1,1,1));origin.scale.x=origin.scale.y=origin.scale.z=.003
    return MarkerArray(markers=markers)


class Preview(Node):
    def __init__(self,cloud,markers,cfg):
        super().__init__('grasp_pose_preview');self.cloud=cloud;self.markers=markers
        self.T=transform(cfg['object_world_m'],np.eye(3));self.cfg=cfg
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub=self.create_publisher(type(cloud),'/grasp_preview/points',qos)
        self.marker_pub=self.create_publisher(MarkerArray,'/grasp_preview/markers',qos)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(String,'/simulation/grasp_status',self.status,10)
        self.create_timer(.2,self.publish)
    def status(self,msg):
        value=json.loads(msg.data);T=np.array(value['T_world_object'])
        if T.shape==(4,4) and np.isfinite(T).all():self.T=T
    def publish(self):
        stamp=self.get_clock().now().to_msg();tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='world';tf.child_frame_id='object'
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,self.T[:3,3]);q=Rotation.from_matrix(self.T[:3,:3]).as_quat()
        tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,q);self.tf.sendTransform(tf)
        self.cloud.header.stamp=Time();self.cloud_pub.publish(self.cloud)
        for m in self.markers.markers:
            m.header.stamp=Time();m.frame_locked=True
        self.marker_pub.publish(self.markers)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--top',type=int,default=5);parser.add_argument('--candidate-id',type=int);parser.add_argument('--width-valid',action='store_true');parser.add_argument('--table-clear',action='store_true');parser.add_argument('--raw',action='store_true',help='Show unfiltered raw model candidates; default checks width AND table');parser.add_argument('--check',action='store_true')
    parser.add_argument('--grasps',type=Path,default=ROOT/'data/grasps/cube/grasps.json')
    parser.add_argument('--metadata',type=Path,default=ROOT/'data/pointclouds/cube/metadata.json')
    parser.add_argument('--cloud',type=Path,default=ROOT/'data/pointclouds/cube/points.npy')
    args=parser.parse_args()
    source=json.loads(args.grasps.read_text());meta=json.loads(args.metadata.read_text());cfg=config()
    if source['input_metadata']!=meta or source['units']!='meters' or source['point_cloud_frame']!='object':raise ValueError('GRASP_INPUT_MODEL_MISMATCH')
    if args.top<1:raise ValueError('--top must be positive')
    grasps=sorted(source['grasps'],key=lambda g:-g['score'])
    if args.candidate_id is not None:grasps=[g for g in grasps if g['candidate_id']==args.candidate_id]
    if not args.raw or args.width_valid:
        grasps=[g for g in grasps if 0<g['width']<=cfg['hardware_max_jaw_opening_m']]
    if not args.raw or args.table_clear:
        def table_clear(g):
            R=np.array(g['rotation_matrix']);p=np.array(cfg['object_world_m'])+g['translation']
            for lo,hi in gripper_boxes(g):
                lo=np.array(lo);hi=np.array(hi)
                if obb_intersects_aabb(p+R@((lo+hi)/2),R,(hi-lo)/2,np.array(cfg['table_center_world_m']),np.array(cfg['table_half_size_m'])):return False
            return True
        grasps=[g for g in grasps if table_clear(g)]
    print('Candidates after display filters:',len(grasps), 'raw mode:',args.raw, flush=True)
    grasps=grasps[:args.top]
    if not grasps:raise ValueError('NO_CANDIDATE_FOR_DISPLAY: both hardware width and template/table tests must pass')
    cloud=load_message(args.cloud);markers=build_markers(grasps,cfg,meta.get('dimensions',{}).get('dimensions_m'),meta,np.load(args.cloud,allow_pickle=False))
    print('Display only; no robot control. Candidates:',[(g['candidate_id'],round(g['width']*1000,2)) for g in grasps],flush=True)
    print('RGB axes = grasp XYZ; X is approach, Y is jaw opening. Magenta gripper >70mm; green <=70mm. Cyan line = 20cm approach. Mapped TCP is unverified.',flush=True)
    if args.check:print('CHECK_PASS',len(markers.markers),'markers,',cloud.width,'points');return
    rclpy.init();node=Preview(cloud,markers,cfg)
    if 'T_world_object' in meta:node.T=np.array(meta['T_world_object'])
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
