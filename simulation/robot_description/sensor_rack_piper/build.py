#!/usr/bin/env python3
"""Combine sensor rack and Piper; keep original meshes and joint kinematics."""
from pathlib import Path
import copy
import os
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2] / 'HW/URDF'

def build(source, output):
    rack = ET.parse(ROOT/'sensor_rack_description/urdf/sensor_rack.urdf').getroot()
    rack.set('name', 'sensor_rack_piper')
    for mesh in rack.findall('.//mesh'):
        mesh.set('filename', os.path.relpath(ROOT/'sensor_rack_description'/mesh.get('filename').split('package://sensor_rack_description/')[1], HERE).replace(os.sep, '/'))
    arm = ET.parse(ROOT/'piper'/source).getroot()
    for node in arm:
        if node.tag == 'link' and node.get('name') == 'world':
            continue
        if node.tag == 'joint' and node.get('name') == 'world_to_base_link':
            continue
        node = copy.deepcopy(node)
        for e in node.iter():
            if e.tag in ('link','joint','material') and e.get('name'):
                e.set('name', 'piper_' + e.get('name'))
            for attr in ('link','joint','reference'):
                if e.get(attr):
                    e.set(attr, 'piper_' + e.get(attr))
            if e.tag == 'mesh':
                e.set('filename', os.path.relpath(ROOT/'piper'/e.get('filename'), HERE).replace(os.sep, '/'))
        rack.append(node)
    joint = ET.SubElement(rack,'joint',name='rack_to_piper_base',type='fixed')
    ET.SubElement(joint,'parent',link='cad_Manipulator_mount_1')
    ET.SubElement(joint,'child',link='piper_base_link')
    # CAD mounting origin Z=.7952; plate upper surface Z=.799.
    ET.SubElement(joint,'origin',xyz='0 0 0.0038',rpy='0 0 0')
    links = {e.get('name') for e in rack.findall('link')}
    joints = rack.findall('joint')
    children = [j.find('child').get('link') for j in joints]
    assert len(children) == len(set(children))
    assert links - set(children) == {'base_link'}
    assert len(joints) == len(links)-1
    for j in joints:
        assert j.find('parent').get('link') in links
    for m in rack.findall('.//mesh'):
        assert (HERE/m.get('filename')).is_file()
    ET.indent(rack)
    ET.ElementTree(rack).write(HERE/output,encoding='utf-8',xml_declaration=True)
    print(output, len(links), 'links;',len(joints),'joints; fixed mount verified')

if __name__ == '__main__':
    build('piper_with_gripper.urdf', 'sensor_rack_piper.urdf')
    build('piper_with_gripper_gazebo.urdf', 'sensor_rack_piper_gazebo.urdf')
