#!/usr/bin/env python3
"""Merge the checked-in wrist camera module with the existing robot URDF."""
from pathlib import Path
import argparse
import copy
import json
import math
import os
import sys
import xml.etree.ElementTree as E

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
DESCRIPTION = ROOT/'simulation/robot_description'


def add_wrist_camera(robot, output_directory, gazebo=False):
    config = json.loads((HERE/'placement.json').read_text())
    module = E.parse(HERE/'wrist_camera.urdf').getroot()
    existing_links = {l.get('name') for l in robot.findall('link')}
    existing_joints = {j.get('name') for j in robot.findall('joint')}
    if config['parent_link'] not in existing_links:
        raise ValueError('Missing parent link: '+config['parent_link'])
    new_links = {l.get('name') for l in module.findall('link')}
    new_joints = {j.get('name') for j in module.findall('joint')} | {'piper_to_wrist_camera_mount'}
    if existing_links & new_links or existing_joints & new_joints:
        raise ValueError('Wrist camera links/joints already exist or names conflict')
    for child in module:
        child = copy.deepcopy(child)
        for mesh in child.findall('.//mesh'):
            path = HERE/mesh.get('filename')
            if not path.is_file():
                raise FileNotFoundError(path)
            mesh.set('filename',os.path.relpath(path, output_directory).replace(os.sep,'/'))
        robot.append(child)
    joint = E.SubElement(robot,'joint',name='piper_to_wrist_camera_mount',type='fixed')
    E.SubElement(joint,'parent',link=config['parent_link'])
    E.SubElement(joint,'child',link='wrist_camera_mount_link')
    E.SubElement(joint,'origin',xyz=' '.join(map(str,config['xyz_m'])),rpy=' '.join(map(str,config['rpy_rad'])))
    if gazebo:
        sys.path.insert(0,str(ROOT/'simulation/gazebo'))
        from sensors import sensor, tag
        extension = E.SubElement(robot,'gazebo',reference='wrist_camera_link')
        for name,kind,stream,fov,w,h,near,far in [
            ('wrist_d435f_depth','depth_camera','depth',87,1280,720,.2,3),
            ('wrist_d435f_color','camera','color',69,1920,1080,.01,100)]:
            s=sensor(extension,name,kind,'0 0 0 0 0 0',
                     '/robocup/wrist_camera/'+stream+'/image','wrist_camera_optical_frame',30)
            tag(s,'optical_frame_id','wrist_camera_optical_frame')
            camera=E.SubElement(s,'camera'); tag(camera,'horizontal_fov',math.radians(fov))
            image=E.SubElement(camera,'image'); tag(image,'width',w); tag(image,'height',h); tag(image,'format','R8G8B8')
            clip=E.SubElement(camera,'clip'); tag(clip,'near',near); tag(clip,'far',far)


def validate(robot, directory):
    names=[l.get('name') for l in robot.findall('link')]
    joints=robot.findall('joint')
    assert len(names)==len(set(names))
    assert len(joints)==len({j.get('name') for j in joints})
    parents={j.find('child').get('link'):j.find('parent').get('link') for j in joints}
    assert len(parents)==len(joints)
    assert set(names)-set(parents)=={'base_link'}
    for name in names:
        seen=set()
        while name in parents:
            assert name not in seen
            seen.add(name); name=parents[name]
        assert name=='base_link'
    for mesh in robot.findall('.//mesh'):
        assert (directory/mesh.get('filename')).is_file()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--replace-main',action='store_true',help='Update standard integrated URDF filenames instead of separate wrist-camera copies')
    args=p.parse_args()
    for gazebo in (False,True):
        suffix='_gazebo' if gazebo else ''
        source=DESCRIPTION/f'tracer_sensor_rack_piper{suffix}.urdf'
        output=source if args.replace_main else DESCRIPTION/f'tracer_sensor_rack_piper_wrist_camera{suffix}.urdf'
        robot=E.parse(source).getroot()
        add_wrist_camera(robot,output.parent,gazebo)
        validate(robot,output.parent)
        E.indent(robot)
        E.ElementTree(robot).write(output,encoding='utf-8',xml_declaration=True)
        print(output.relative_to(ROOT))


if __name__=='__main__':
    main()
