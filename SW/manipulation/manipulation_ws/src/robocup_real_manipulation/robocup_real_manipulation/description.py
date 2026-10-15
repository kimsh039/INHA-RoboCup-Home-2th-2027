"""Prepare physical description from CAD meshes + measured transforms, no Gazebo."""
import argparse
import copy
import itertools
import json
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import yaml
from .core import validate, require, rotation

def apply_transform(joint,parent,measurement):
    joint.find('parent').set('link',parent)
    xyz=measurement['xyz']; q=measurement['quaternion_xyzw']; R=rotation(q)
    pitch=np.arcsin(np.clip(-R[2,0],-1,1)); roll=np.arctan2(R[2,1],R[2,2]); yaw=np.arctan2(R[1,0],R[0,0])
    origin=joint.find('origin')
    if origin is None: origin=ET.SubElement(joint,'origin')
    origin.set('xyz',' '.join(map(str,xyz))); origin.set('rpy',' '.join(map(str,[roll,pitch,yaw])))

def prepare(repo,cfg,out):
    validate(cfg,hardware=True); out.mkdir(parents=True,exist_ok=True)
    path=repo/'SW/simulation/robot_description/robocup.urdf'; tree=ET.parse(path); root=tree.getroot()
    for element in list(root):
        if element.tag in ('gazebo','ros2_control','transmission'): root.remove(element)
    joints={j.get('name'):j for j in root.findall('joint')}; links={l.get('name') for l in root.findall('link')}
    require(set(cfg['joint_names']).issubset(joints) and set(cfg['touch_links']).issubset(links),'CAD_NAME_MISMATCH')
    require(cfg['gravity_frame']=='base_link' and cfg['tcp_frame']=='tcp' and cfg['wrist_optical_frame']=='wrist_camera_optical_frame','MODEL_FRAME_NAMES_REQUIRE_EXTENSION')
    apply_transform(joints['rack_to_piper_base'],'base_link',cfg['base_arm_transform'])
    apply_transform(joints['wrist_camera_optical_joint'],'piper_link6',cfg['handeye_transform'])
    require('tcp' not in links,'CAD_ALREADY_HAS_TCP')
    ET.SubElement(root,'link',name='tcp')
    tcp=ET.SubElement(root,'joint',name='physical_tcp_joint',type='fixed')
    ET.SubElement(tcp,'parent',link='piper_link6'); ET.SubElement(tcp,'child',link='tcp')
    apply_transform(tcp,'piper_link6',cfg['link6_tcp_transform'])
    if cfg.get('external_sensor_tf',False):
        # CAD meshes remain in the collision model. Real sensor_tf/RealSense own
        # these public sensor frames; do not give the same child two TF parents.
        rename={name:'collision_'+name for name in ('camera_link','livox_frame','laser_frame')}
        for link in root.findall('link'):
            link.set('name',rename.get(link.get('name'),link.get('name')))
        for j in root.findall('joint'):
            for tag in ('parent','child'):
                item=j.find(tag); item.set('link',rename.get(item.get('link'),item.get('link')))
    # Uncommanded mobile-base wheels do not enter the manipulation state monitor.
    for j in root.findall('joint'):
        if j.get('type')=='continuous': j.set('type','fixed')
    # [MODEL:WRIST_MOUNT_MESH] A solid bounding box fills the mount's cutouts
    # and can falsely intersect the moving wrist (link5). Use the CAD surface
    # at its original visual origin; do not retain the old box-centroid origin.
    mount=root.find("link[@name='wrist_camera_mount_link']")
    require(mount is not None,'WRIST_MOUNT_LINK_MISSING')
    visual=mount.find('visual')
    require(visual is not None and visual.find('geometry/mesh') is not None,'WRIST_MOUNT_MESH_MISSING')
    for collision in mount.findall('collision'): mount.remove(collision)
    collision=ET.SubElement(mount,'collision')
    if visual.find('origin') is not None: collision.append(copy.deepcopy(visual.find('origin')))
    collision.append(copy.deepcopy(visual.find('geometry')))
    for mesh in root.iter('mesh'):
        filename=mesh.get('filename')
        if not filename.startswith('package://'):
            asset=(path.parent/filename).resolve(); require(asset.exists(),'MESH_MISSING:'+str(asset))
            mesh.set('filename',asset.as_uri())
    urdf=ET.tostring(root,encoding='unicode'); (out/'robot.urdf').write_text(urdf)
    srdf=ET.Element('robot',name=root.get('name')); group=ET.SubElement(srdf,'group',name=cfg['planning_group'])
    ET.SubElement(group,'chain',base_link='piper_base_link',tip_link='tcp')
    # Adjacent links and one rigid fixed assembly cannot collide by joint motion.
    # Retain collisions between different assemblies and all environment objects.
    fixed_parent={link.get('name'):link.get('name') for link in root.findall('link')}
    def component(name):
        while fixed_parent[name]!=name:
            fixed_parent[name]=fixed_parent[fixed_parent[name]]; name=fixed_parent[name]
        return name
    disabled=set()
    for j in root.findall('joint'):
        a=j.find('parent').get('link'); b=j.find('child').get('link')
        disabled.add(tuple(sorted((a,b))))
        ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='Adjacent')
        if j.get('type')=='fixed': fixed_parent[component(b)]=component(a)
    groups={}
    for link in fixed_parent: groups.setdefault(component(link),[]).append(link)
    for members in groups.values():
        for a,b in itertools.combinations(sorted(members),2):
            if (a,b) not in disabled:
                ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='RigidAssembly')
    # Empty width=0 is a valid physical state: opposing fingers touch there.
    # Keep each finger checked against the arm, sensors, table and target.
    fingers=('piper_gripper_link1','piper_gripper_link2')
    require(set(fingers).issubset(fixed_parent),'GRIPPER_CONTACT_LINKS_MISSING')
    ET.SubElement(srdf,'disable_collisions',link1=fingers[0],link2=fingers[1],reason='GripperClosureContact')
    # The mount is a fixed decoration of link6 at the joint6 housing interface.
    # CAD contact depth at start/home is <0.3 mm throughout the joint6 sweep.
    # Treat this one housing/attachment pair as adjacent, not the whole wrist
    # or other arm links. Its geometry remains checked against the environment.
    require(component('wrist_camera_mount_link')==component('piper_link6') and
            joints['piper_joint6'].find('parent').get('link')=='piper_link5',
            'WRIST_MOUNT_ADJACENCY_CHANGED')
    ET.SubElement(srdf,'disable_collisions',link1='piper_link5',link2='wrist_camera_mount_link',reason='AdjacentMountInterface')
    (out/'robot.srdf').write_text(ET.tostring(srdf,encoding='unicode'))
    (out/'config_sha256.txt').write_text(hashlib.sha256(json.dumps(cfg,sort_keys=True).encode()).hexdigest())
    return urdf,ET.tostring(srdf,encoding='unicode')

def main():
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    prepare(a.repo,yaml.safe_load(a.config.read_text()),a.output)
    print(a.output/'robot.urdf')
