from pathlib import Path
import subprocess
import argparse
import tempfile
import xml.etree.ElementTree as E
parser=argparse.ArgumentParser()
parser.add_argument('--test-wall',action='store_true',help='Add a wall 2 m forward for sensor validation')
parser.add_argument('--world',choices=['empty','room'],default='empty',help='room: walls, partition and table for SLAM/Nav2 tests (see README)')
args=parser.parse_args()
HERE=Path(__file__).resolve().parent
DESCRIPTION=HERE.parent/'robot_description'
P=HERE/'build'
P.mkdir(exist_ok=True)
r=E.parse(DESCRIPTION/'robocup.urdf')
for m in r.findall('.//mesh'): m.set('filename',str((DESCRIPTION/m.get('filename')).resolve()))
# Wheel contact for the sim only; masses/inertias come from the URDF (HW/URDF/tracer, 30 kg).
# With the source mesh collisions all six wheels touch the floor at once, so the drive wheels
# slipped against the casters (in-place turns lost ~20-30% of the commanded yaw).
def set_collision(link,shape,**size):
 c=r.find(f"link[@name='{link}']/collision")
 c.find('origin').attrib.update(xyz='0 0 0',rpy='0 0 0')
 g=c.find('geometry'); g.clear(); E.SubElement(g,shape,{k:str(v) for k,v in size.items()})
def friction(link,mu):
 g=E.SubElement(r.getroot(),'gazebo',reference=link)
 for k in ('mu1','mu2'): E.SubElement(g,k).text=str(mu)
for side in ('left','right'):
 # Sphere of the 60.5 mm wheel radius: one smooth contact point on the tyre centre line.
 # The faceted mesh and a full-width cylinder both scrubbed during in-place turns.
 set_collision(f'{side}_wheel_link','sphere',radius=0.0605)
 friction(f'{side}_wheel_link',1.0)
for c in ('fl','fr','rl','rr'):
 # 1 mm smaller than the 37.5 mm caster wheel so the drive wheels carry the load (the real
 # drive wheels are sprung); low-friction casters only catch the body when it pitches.
 set_collision(f'{c}_wheel_link','sphere',radius=0.0365)
 friction(f'{c}_wheel_link',0.01)
# Convert through a temporary file; robocup.urdf remains the only stored URDF.
with tempfile.TemporaryDirectory(prefix='robocup-sdf-') as temporary:
 local=Path(temporary)/'robot.urdf'
 r.write(local,encoding='unicode')
 s=subprocess.run(['gz','sdf','-p',str(local)],capture_output=True,text=True,check=True)
m=E.fromstring(s.stdout).find('model'); m.set('name','robocup')
E.SubElement(m,'pose').text='0 0 0.145 0 0 0'
# Cancel the source left wheel joint's half-turn so both drive axes point +Y.
m.find("joint[@name='left_wheel']/axis/xyz").text='0 -1 0'
def plugin(parent,filename,name,**params):
 p=E.SubElement(parent,'plugin',filename=filename,name=name)
 for k,v in params.items(): E.SubElement(p,k).text=str(v)
 return p
for i in range(1,7):
 plugin(m,'gz-sim-joint-position-controller-system','gz::sim::systems::JointPositionController',joint_name=f'piper_joint{i}',use_velocity_commands='true',cmd_max='0.5')
for i in (1,2):
 plugin(m,'gz-sim-joint-position-controller-system','gz::sim::systems::JointPositionController',joint_name=f'piper_gripper_joint{i}',use_velocity_commands='true',cmd_max='0.02')
plugin(m,'gz-sim-joint-state-publisher-system','gz::sim::systems::JointStatePublisher',topic='/robocup/joint_states')
plugin(m,'gz-sim-diff-drive-system','gz::sim::systems::DiffDrive',left_joint='left_wheel',right_joint='right_wheel',wheel_separation='0.34',wheel_radius='0.0605',topic='/robocup/cmd_vel',odom_topic='/robocup/odometry',odom_publish_frequency='10',max_linear_velocity='1.6',min_linear_velocity='-1.6',max_linear_acceleration='1.0',min_linear_acceleration='-1.0',max_angular_acceleration='2.0',min_angular_acceleration='-2.0',frame_id='odom',child_frame_id='base_link',tf_topic='/model/robocup/tf')
sdf=E.Element('sdf',version='1.10'); w=E.SubElement(sdf,'world',name='robocup_motion')
E.SubElement(w,'gravity').text='0 0 -9.81'
ph=E.SubElement(w,'physics',name='physics',type='ignored'); E.SubElement(ph,'max_step_size').text='0.001'; E.SubElement(ph,'real_time_factor').text='1'
for f,n in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')]: plugin(w,f'gz-sim-{f}-system',f'gz::sim::systems::{n}')
plugin(w,'gz-sim-sensors-system','gz::sim::systems::Sensors',render_engine='ogre2')
w.append(m)
ground=E.SubElement(w,'model',name='ground'); E.SubElement(ground,'static').text='true'; l=E.SubElement(ground,'link',name='ground')
for tag in ('collision','visual'):
 e=E.SubElement(l,tag,name='ground_'+tag); g=E.SubElement(e,'geometry'); pl=E.SubElement(g,'plane'); E.SubElement(pl,'normal').text='0 0 1'; E.SubElement(pl,'size').text='100 100'
if args.test_wall:
 wall=E.SubElement(w,'model',name='sensor_test_wall'); E.SubElement(wall,'static').text='true'; E.SubElement(wall,'pose').text='2 0 1 0 0 0'; wl=E.SubElement(wall,'link',name='wall')
 for tag in ('visual','collision'):
  e=E.SubElement(wl,tag,name='wall_'+tag); g=E.SubElement(e,'geometry'); box=E.SubElement(g,'box'); E.SubElement(box,'size').text='0.2 4 2'
def box(name,pose,size,color='0.75 0.75 0.75 1'):
 b=E.SubElement(w,'model',name=name); E.SubElement(b,'static').text='true'; E.SubElement(b,'pose').text=pose; bl=E.SubElement(b,'link',name='link')
 for tag in ('visual','collision'):
  e=E.SubElement(bl,tag,name=tag); g=E.SubElement(e,'geometry'); E.SubElement(E.SubElement(g,'box'),'size').text=size
  if tag=='visual': mat=E.SubElement(e,'material'); E.SubElement(mat,'ambient').text=color; E.SubElement(mat,'diffuse').text=color
if args.world=='room':
 # Inner room 6 x 6 m centred on the spawn point; walls 0.1 m thick, 1.0 m high.
 for name,pose,size in [('wall_north','0 3.05 0.5 0 0 0','6.2 0.1 1.0'),('wall_south','0 -3.05 0.5 0 0 0','6.2 0.1 1.0'),
                        ('wall_east','3.05 0 0.5 0 0 0','0.1 6.0 1.0'),('wall_west','-3.05 0 0.5 0 0 0','0.1 6.0 1.0'),
                        # Partition breaks the square's symmetry so scan matching has a unique fit.
                        ('wall_partition','-1.5 2.25 0.5 0 0 0','0.1 1.5 1.0')]:
  box(name,pose,size)
 # Table 1.6 x 0.8 m, top 0.03 m thick with surface at 0.72 m; legs 0.05 m square inset 0.05 m from the edges.
 tx,ty=1.8,-1.0
 box('table_top',f'{tx} {ty} 0.705 0 0 0','1.6 0.8 0.03','0.55 0.35 0.2 1')
 for i,(dx,dy) in enumerate([(.725,.325),(.725,-.325),(-.725,.325),(-.725,-.325)]):
  box(f'table_leg{i}',f'{tx+dx} {ty+dy} 0.345 0 0 0','0.05 0.05 0.69','0.3 0.3 0.3 1')
light=E.SubElement(w,'light',name='sun',type='directional'); E.SubElement(light,'pose').text='0 0 5 0 0 0'; E.SubElement(light,'direction').text='-0.5 -0.5 -1'; E.SubElement(light,'diffuse').text='0.8 0.8 0.8 1'
scene=E.SubElement(w,'scene'); E.SubElement(scene,'ambient').text='0.6 0.6 0.6 1'
gui=E.SubElement(w,'gui',fullscreen='false'); p=E.SubElement(gui,'plugin',filename='MinimalScene',name='3D View'); E.SubElement(p,'engine').text='ogre2'; E.SubElement(p,'scene').text='scene'; E.SubElement(p,'camera_pose').text='2.2 2.2 1.9 0 0.32 -2.35619'
for f in ('GzSceneManager','InteractiveViewControl','CameraTracking','WorldControl','WorldStats','EntityTree','JointPositionController'): E.SubElement(gui,'plugin',filename=f,name=f)
E.indent(sdf); E.ElementTree(sdf).write(P/'motion.world.sdf',encoding='unicode',xml_declaration=True)
print(P/'motion.world.sdf')
