"""기존 Gazebo 생성 함수를 재사용하고 별도 출력에 manipulation 장면을 만든다."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

ARM=[f'piper_joint{i}' for i in range(1,7)]
FINGERS=['piper_gripper_joint1','piper_gripper_joint2']
NOMINAL_SHA='3b31339f0af58a2e81d9abcf7213b0255848ed9d7b5b02b02102fa0a21ba8732'


def generate(repository,output,room=False,mid360=False,head_camera=False,g2=False):
    repository=Path(repository).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    source=repository/'simulation/robot_description/robocup.urdf'
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    canonical=source.read_bytes().replace(b'../../../HW/',b'../../HW/')
    if hashlib.sha256(canonical).hexdigest()!=NOMINAL_SHA: raise ValueError('NOMINAL_MODEL_CHANGED: review generator before accepting another baseline')
    script=repository/'simulation/gazebo/make_sim.py'
    spec=importlib.util.spec_from_file_location('existing_gazebo_generator',script)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    urdf=module.load_robot_urdf()
    # MuJoCo와 동일한 명목 TCP 위치/축. 카메라 고정 변환은 공용 URDF를 유지한다.
    tcp_cfg=json.loads((repository/'simulation/mujoco/config/grasp_simulation.json').read_text())
    robot=urdf.getroot()
    ET.SubElement(robot,'link',name='manipulation_tcp')
    j=ET.SubElement(robot,'joint',name='manipulation_tcp_fixed',type='fixed')
    ET.SubElement(j,'parent',link='piper_link6');ET.SubElement(j,'child',link='manipulation_tcp')
    ET.SubElement(j,'origin',xyz=' '.join(map(str,tcp_cfg['tcp_tip_link6_m'])),
        rpy=' '.join(map(str,Rotation.from_matrix(tcp_cfg['tcp_axes_in_link6']).as_euler('xyz'))))
    # Planning 모델은 원본 collision을 보존한다. wheel 접촉 단순화는 SDF에만 적용한다.
    planning=ET.fromstring(ET.tostring(robot))
    for mesh in planning.findall('.//mesh'): mesh.set('filename',Path(mesh.get('filename')).as_uri())
    for gazebo in list(planning.findall('gazebo')): planning.remove(gazebo)
    ET.ElementTree(planning).write(output/'robot.urdf',encoding='unicode')
    # Disable only sensor generation; nominal links, mounts and TF remain intact.
    disabled=set()
    if not mid360:disabled.add('livox_mid360s')
    if not g2:disabled.add('ydlidar_g2')
    if not head_camera:disabled.update(('d435f_depth','d435f_color'))
    for parent in robot.iter():
        for sensor in list(parent.findall('sensor')):
            if sensor.get('name') in disabled:parent.remove(sensor)
    module.set_wheel_contacts(urdf)
    for sensor in robot.findall('.//sensor'):
        if sensor.get('type') in ('camera','depth_camera'):
            sensor.find('update_rate').text='5'
            sensor.find('camera/image/width').text='320';sensor.find('camera/image/height').text='240'
            if sensor.get('type')=='depth_camera': sensor.find('camera/clip/near').text='0.05'
    # 기존 navigation topic/world 이름을 유지하며 로봇 한 대만 생성한다.
    module.SPAWN_POSE=' '.join(map(str,tcp_cfg['base_world_m']))+' 0 0 0'
    model=module.urdf_to_model(urdf);module.add_robot_plugins(model)
    for p in model.findall('plugin'):
        name=p.findtext('joint_name')
        if name in FINGERS:
            # fingers는 힘 PID로 제어한다. 물체 관통으로 파지 성공을 만들지 않는다.
            p.find('use_velocity_commands').text='false'
            p.find('cmd_max').text='10'
            for key,value in {'cmd_min':'-10','p_gain':'500','i_gain':'0','d_gain':'5'}.items():module.sub(p,key,value)
    sdf=module.build_world(model,room=room)
    world=sdf.find('world')
    scene=world.find('scene')
    if scene is not None:module.sub(scene,'shadows','false')
    module.plugin(world,'gz-sim-contact-system','gz::sim::systems::Contact')
    def contact_sensor(link,key):
        if link is None:raise ValueError('MISSING_CONTACT_LINK:'+key)
        collisions=link.findall('collision')
        if not collisions:raise ValueError('MISSING_CONTACT_COLLISION:'+key)
        sensor=module.sub(link,'sensor',name='manipulation_contact_'+key,type='contact')
        module.sub(sensor,'always_on','true');module.sub(sensor,'update_rate','50')
        module.sub(sensor,'topic','/robocup/contacts/'+key)
        contact=module.sub(sensor,'contact')
        # gz-sim8 Contact reads topic from the contact element, not sensor/topic.
        module.sub(contact,'topic','/robocup/contacts/'+key)
        for collision in collisions:module.sub(contact,'collision',collision.get('name'))
    for i in (1,2):contact_sensor(model.find(f"link[@name='piper_gripper_link{i}']"),'finger'+str(i))
    # 기준 장면: 테이블의 40mm cube. 물체를 gripper에 고정하는 plugin은 사용하지 않는다.
    module.static_box(world,'manipulation_table','0.68 0 0.695 0 0 0','0.6 0.9 0.05','0.5 0.3 0.15 1')
    for i,(sx,sy) in enumerate(((-1,-1),(-1,1),(1,-1),(1,1))):
        module.static_box(world,f'manipulation_table_leg_{i}',f'{.68+sx*.26} {sy*.41} .335 0 0 0','.05 .05 .67','0.5 0.3 0.15 1')
    cube=module.sub(world,'model',name='manipulation_cube');module.sub(cube,'pose','0.5 -0.1 0.7401 0 0 0')
    link=module.sub(cube,'link',name='cube_link')
    inertial=module.sub(link,'inertial');module.sub(inertial,'mass','.04')
    inertia=module.sub(inertial,'inertia')
    for key in ('ixx','iyy','izz'):module.sub(inertia,key,str(.04*.04**2/6))
    for key in ('ixy','ixz','iyz'):module.sub(inertia,key,'0')
    for kind in ('visual','collision'):
        element=module.sub(link,kind,name=kind);module.sub(module.sub(module.sub(element,'geometry'),'box'),'size','.04 .04 .04')
        if kind=='visual':
            material=module.sub(element,'material');module.sub(material,'diffuse','0.9 0.1 0.05 1');module.sub(material,'ambient','0.9 0.1 0.05 1')
        else:
            friction=module.sub(module.sub(element,'surface'),'friction');ode=module.sub(friction,'ode');module.sub(ode,'mu','1');module.sub(ode,'mu2','1')
    contact_sensor(link,'cube')
    # GUI scene-broadcaster is not an authoritative stamped model-pose source.
    # Publish each model's actual world pose with explicit frame names and sim stamp.
    for body in (model,cube):
        module.plugin(body,'gz-sim-pose-publisher-system','gz::sim::systems::PosePublisher',
            publish_model_pose='true',publish_link_pose='false',publish_nested_model_pose='false',
            publish_visual_pose='false',publish_collision_pose='false',publish_sensor_pose='false',
            use_pose_vector_msg='true',static_publisher='false',update_frequency='50',
            topic='/robocup/manipulation/model_poses')
    ET.indent(sdf);ET.ElementTree(sdf).write(output/'world.sdf',encoding='unicode',xml_declaration=True)
    ET.ElementTree(model).write(output/'robot.sdf',encoding='unicode',xml_declaration=True)
    srdf=ET.Element('robot',name=planning.get('name'));group=ET.SubElement(srdf,'group',name='arm')
    ET.SubElement(group,'chain',base_link='piper_base_link',tip_link='manipulation_tcp')
    g=ET.SubElement(srdf,'group',name='gripper')
    for n in FINGERS: ET.SubElement(g,'joint',name=n)
    ET.SubElement(srdf,'end_effector',name='gripper',parent_link='manipulation_tcp',group='gripper',parent_group='arm')
    # odom 부모의 floating joint로 주행 TF를 연결한다. Nav2의 map→odom과 중복하지 않는다.
    ET.SubElement(srdf,'virtual_joint',name='odom_base',type='floating',parent_frame='odom',child_link='base_link')
    state=ET.SubElement(srdf,'group_state',name='initial',group='arm')
    for n in ARM:ET.SubElement(state,'joint',name=n,value='0')
    parents={l.get('name'):l.get('name') for l in planning.findall('link')}
    def root(x):
        while parents[x]!=x:x=parents[x]
        return x
    adjacent=set()
    for j in planning.findall('joint'):
        a,b=j.find('parent').get('link'),j.find('child').get('link');adjacent.add(frozenset((a,b)))
        if j.get('type')=='fixed':parents[root(b)]=root(a)
    # 성공본 simulation/mujoco/manipulation/inha_model.py와 같은 조립체 인접 규칙.
    # link6에 고정된 flange/gripper/camera도 link5의 인접 moving component다.
    # raw URDF joint 두 링크만 제외하면 고정 부품이 잘못된 비인접 충돌로 보고된다.
    component_neighbors=set()
    for j in planning.findall('joint'):
        if j.get('type')!='fixed':
            component_neighbors.add(frozenset((root(j.find('parent').get('link')),root(j.find('child').get('link')))))
    names=list(parents)
    for i,a in enumerate(names):
        for b in names[i+1:]:
            same=root(a)==root(b)
            if same or frozenset((root(a),root(b))) in component_neighbors:
                ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='SameRigidBody' if same else 'AdjacentRigidComponents')
    ET.ElementTree(srdf).write(output/'robot.srdf',encoding='unicode')
    bridges=yaml.safe_load((repository/'simulation/ros2/ros_bridge.yaml').read_text())
    if not mid360:bridges=[b for b in bridges if not b['ros_topic_name'].startswith('/mid360/') and '/mid360s/' not in b['gz_topic_name']]
    if not g2:bridges=[b for b in bridges if b['ros_topic_name']!='/scan' and '/g2/' not in b['gz_topic_name']]
    if not head_camera:bridges=[b for b in bridges if not b['ros_topic_name'].startswith('/head_camera/')]
    # Tutorial constructs its own cloud from depth; do not bridge unused sensor output.
    bridges=[b for b in bridges if b['ros_topic_name'] not in ('/wrist_camera/depth/points','/wrist_camera/depth/camera_info')]
    # 동일 화각과 pose의 시뮬 depth. 실제 D435 정렬 검증과 구분한다.
    for b in bridges:
        if b['ros_topic_name'] in ('/head_camera/depth/image_raw','/wrist_camera/depth/image_raw'):
            b['ros_topic_name']=b['ros_topic_name'].replace('/depth/','/aligned_depth_to_color/')
    for n in ARM+FINGERS:
        bridges.append(dict(ros_topic_name='/gazebo/manipulation/'+n+'/target',
            gz_topic_name=f'/model/robocup/joint/{n}/0/cmd_pos',ros_type_name='std_msgs/msg/Float64',gz_type_name='gz.msgs.Double',direction='ROS_TO_GZ'))
    bridges.append(dict(ros_topic_name='/gazebo/dynamic_poses',gz_topic_name='/robocup/manipulation/model_poses',
        ros_type_name='tf2_msgs/msg/TFMessage',gz_type_name='gz.msgs.Pose_V',direction='GZ_TO_ROS'))
    for key in ('finger1','finger2','cube'):
        bridges.append(dict(ros_topic_name='/gazebo/contacts/'+key,gz_topic_name='/robocup/contacts/'+key,
            ros_type_name='ros_gz_interfaces/msg/Contacts',gz_type_name='gz.msgs.Contacts',direction='GZ_TO_ROS'))
    (output/'bridge.yaml').write_text(yaml.safe_dump(bridges,sort_keys=False))
    limits={n:[float(planning.find(f"joint[@name='{n}']/limit").get(k)) for k in ('lower','upper')] for n in ARM+FINGERS}
    manifest=dict(nominal_sha256=digest,source_repository=str(repository),gazebo_version='Harmonic / gz-sim8',
        mid360_enabled=mid360,head_camera_enabled=head_camera,g2_enabled=g2,joint_limits=limits,world='robocup_motion',model='robocup',finger_control='effort PID, no fixed grasp attachment',
        collision_policy='same nominal MuJoCo rigid-assembly / adjacent-moving-component exclusions; nonadjacent and world collisions retained',
        validation='UNVERIFIED_GENERATED',files={n:hashlib.sha256((output/n).read_bytes()).hexdigest() for n in ('robot.urdf','robot.srdf','world.sdf','robot.sdf','bridge.yaml')})
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return manifest


def main():
    p=argparse.ArgumentParser();p.add_argument('--repository',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--room',action='store_true');p.add_argument('--mid360',action='store_true');p.add_argument('--head-camera',action='store_true');p.add_argument('--g2',action='store_true');a=p.parse_args()
    print(json.dumps(generate(a.repository,a.output,a.room,a.mid360,a.head_camera,a.g2),indent=2))

if __name__=='__main__':main()
