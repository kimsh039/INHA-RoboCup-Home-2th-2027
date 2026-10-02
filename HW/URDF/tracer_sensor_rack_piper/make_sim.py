from pathlib import Path
import subprocess
import xml.etree.ElementTree as E
P=Path(__file__).resolve().parent
r=E.parse(P/'tracer_sensor_rack_piper_gazebo.urdf')
for m in r.findall('.//mesh'): m.set('filename',str((P/m.get('filename')).resolve()))
r.write(P/'sim_local.urdf',encoding='unicode')
s=subprocess.run(['gz','sdf','-p',str(P/'sim_local.urdf')],capture_output=True,text=True,check=True)
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
plugin(m,'gz-sim-diff-drive-system','gz::sim::systems::DiffDrive',left_joint='left_wheel',right_joint='right_wheel',wheel_separation='0.34',wheel_radius='0.0605',topic='/robocup/cmd_vel',odom_topic='/robocup/odometry',odom_publish_frequency='10')
sdf=E.Element('sdf',version='1.10'); w=E.SubElement(sdf,'world',name='robocup_motion')
E.SubElement(w,'gravity').text='0 0 -9.81'
ph=E.SubElement(w,'physics',name='physics',type='ignored'); E.SubElement(ph,'max_step_size').text='0.001'; E.SubElement(ph,'real_time_factor').text='1'
for f,n in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')]: plugin(w,f'gz-sim-{f}-system',f'gz::sim::systems::{n}')
w.append(m)
ground=E.SubElement(w,'model',name='ground'); E.SubElement(ground,'static').text='true'; l=E.SubElement(ground,'link',name='ground')
for tag in ('collision','visual'):
 e=E.SubElement(l,tag,name='ground_'+tag); g=E.SubElement(e,'geometry'); pl=E.SubElement(g,'plane'); E.SubElement(pl,'normal').text='0 0 1'; E.SubElement(pl,'size').text='100 100'
light=E.SubElement(w,'light',name='sun',type='directional'); E.SubElement(light,'pose').text='0 0 5 0 0 0'; E.SubElement(light,'direction').text='-0.5 -0.5 -1'; E.SubElement(light,'diffuse').text='0.8 0.8 0.8 1'
scene=E.SubElement(w,'scene'); E.SubElement(scene,'ambient').text='0.6 0.6 0.6 1'
gui=E.SubElement(w,'gui',fullscreen='false'); p=E.SubElement(gui,'plugin',filename='MinimalScene',name='3D View'); E.SubElement(p,'engine').text='ogre2'; E.SubElement(p,'scene').text='scene'; E.SubElement(p,'camera_pose').text='2.2 2.2 1.9 0 0.32 -2.35619'
for f in ('GzSceneManager','InteractiveViewControl','CameraTracking','WorldControl','WorldStats','EntityTree','JointPositionController'): E.SubElement(gui,'plugin',filename=f,name=f)
E.indent(sdf); E.ElementTree(sdf).write(P/'motion.world.sdf',encoding='unicode',xml_declaration=True)
print(P/'motion.world.sdf')
