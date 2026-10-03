#!/usr/bin/env python3
"""Attach the rack bottom center to the Tracer top mounting rails."""
from pathlib import Path
import argparse
import subprocess
import sys
import xml.etree.ElementTree as E

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent

def build(variant, xyz, yaw):
    tracer=E.parse(ROOT/'tracer/tracer_v1.urdf').getroot()
    tracer.set('name','tracer_sensor_rack_piper')
    for m in tracer.findall('.//mesh'):
        m.set('filename','../tracer/'+m.get('filename'))
    suffix='_gazebo' if variant else ''
    rack=E.parse(ROOT/f'sensor_rack_piper/sensor_rack_piper{suffix}.urdf').getroot()
    # Preserve Piper and sensor frame names; only the duplicate root needs renaming.
    for node in rack:
        for e in node.iter():
            for attr in ('name','link','reference'):
                if e.get(attr)=='base_link': e.set(attr,'rack_base_link')
        tracer.append(node)
    j=E.SubElement(tracer,'joint',name='tracer_to_rack',type='fixed')
    E.SubElement(j,'parent',link='base_link')
    E.SubElement(j,'child',link='rack_base_link')
    E.SubElement(j,'origin',xyz=' '.join(map(str,xyz)),rpy=f'0 0 {yaw}')
    links=[l.get('name') for l in tracer.findall('link')]
    joints=tracer.findall('joint'); names=[j.get('name') for j in joints]
    assert len(set(links))==len(links) and len(set(names))==len(names)
    parents={j.find('child').get('link'):j.find('parent').get('link') for j in joints}
    assert len(parents)==len(joints) and set(links)-set(parents)=={'base_link'}
    for link in links:
        visited=set()
        while link in parents:
            assert link not in visited
            visited.add(link); link=parents[link]
        assert link=='base_link'
    for m in tracer.findall('.//mesh'): assert (HERE/m.get('filename')).is_file()
    if variant:
        from sensors import add_sensors
        add_sensors(tracer)
    E.indent(tracer)
    out=HERE/f'tracer_sensor_rack_piper{suffix}.urdf'
    E.ElementTree(tracer).write(out,encoding='utf-8',xml_declaration=True)
    print(out.name, len(links),'links;',len(joints),'joints; tree and meshes verified')

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--xyz',type=float,nargs=3,default=[0,0,0.01611])
    p.add_argument('--yaw',type=float,default=0)
    a=p.parse_args()
    subprocess.run([sys.executable,str(ROOT/'sensor_rack_piper/build.py')],check=True)
    build(False,a.xyz,a.yaw)
    build(True,a.xyz,a.yaw)
