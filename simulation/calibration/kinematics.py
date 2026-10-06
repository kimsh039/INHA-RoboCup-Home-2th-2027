"""URDF forward kinematics from measured joints; no camera truth used for hand-eye samples."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

def fk(urdf,positions,base,child):
    root=ET.parse(urdf).getroot()
    parents={j.find('child').get('link'):j for j in root.findall('joint')}
    chain=[];frame=child;seen=set()
    while frame!=base:
        if frame in seen or frame not in parents:raise ValueError(f'No chain {base} -> {child}')
        seen.add(frame);joint=parents[frame];chain.append(joint);frame=joint.find('parent').get('link')
    t=np.eye(4)
    for joint in reversed(chain):
        origin=joint.find('origin');local=np.eye(4)
        if origin is not None:
            local[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            local[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
        motion=np.eye(4);kind=joint.get('type')
        if kind!='fixed':
            name=joint.get('name')
            if name not in positions:raise ValueError('Missing measured joint: '+name)
            axis=joint.find('axis');axis=np.fromstring(axis.get('xyz','1 0 0') if axis is not None else '1 0 0',sep=' ')
            axis=axis/np.linalg.norm(axis);q=positions[name]
            if kind in ('revolute','continuous'):motion[:3,:3]=Rotation.from_rotvec(q*axis).as_matrix()
            elif kind=='prismatic':motion[:3,3]=q*axis
            else:raise ValueError('Unsupported joint '+kind)
        t=t@local@motion
    return t
