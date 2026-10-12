#!/usr/bin/env python3
"""Apply the accepted Koide RGB-optical estimate; no solver or validation runs."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[4]
RECORD = REPO / 'HW/calibration/head_mid360/calib.json'


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def inverse(a):
    out = identity()
    for i in range(3):
        for j in range(3):
            out[i][j] = a[j][i]
        out[i][3] = -sum(out[i][j] * a[j][3] for j in range(3))
    return out


def from_quaternion(xyz, xyzw):
    x, y, z, w = xyzw
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), xyz[0]],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), xyz[1]],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), xyz[2]],
        [0., 0., 0., 1.],
    ]


def from_origin(origin):
    if origin is None:
        return identity()
    xyz = list(map(float, origin.get('xyz', '0 0 0').split()))
    r, p, y = map(float, origin.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
        [-sp, cp*sr, cp*cr, xyz[2]], [0., 0., 0., 1.],
    ]


def fixed_transform(root, parent, child):
    by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
    out = identity()
    visited = set()
    while child != parent:
        if child in visited:
            raise ValueError('Cycle in fixed URDF chain')
        visited.add(child)
        joint = by_child[child]
        if joint.get('type') != 'fixed':
            raise ValueError('Requested sensor transform crosses a moving joint')
        out = multiply(from_origin(joint.find('origin')), out)
        child = joint.find('parent').get('link')
    return out


def quaternion(a):
    # Stable conversion, including the inverted Mid360 mount near 180 degrees.
    trace = a[0][0] + a[1][1] + a[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.) * 2
        q = [(a[2][1]-a[1][2])/s, (a[0][2]-a[2][0])/s, (a[1][0]-a[0][1])/s, s/4]
    else:
        i = max(range(3), key=lambda k: a[k][k])
        j, k = (i+1) % 3, (i+2) % 3
        s = math.sqrt(1. + a[i][i] - a[j][j] - a[k][k]) * 2
        q = [0., 0., 0., (a[k][j]-a[j][k])/s]
        q[i], q[j], q[k] = s/4, (a[j][i]+a[i][j])/s, (a[k][i]+a[i][k])/s
    norm = math.sqrt(sum(v*v for v in q))
    return [v/norm for v in q]


def result(matrix, parent, child):
    return {'parent_frame': parent, 'child_frame': child, 'translation_unit': 'm',
            'matrix4x4': matrix, 'translation_xyz_m': [matrix[i][3] for i in range(3)],
            'quaternion_xyzw': quaternion(matrix), 'status': 'user_accepted_physical_koide',
            'method': 'Koide four-bag RGB-optical estimate composed with existing Mid360 mount',
            'contains_nominal_reference': True, 'independent_measurement': False,
            'source': 'HW/calibration/head_mid360/calib.json',
            'post_work_verification_run': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, default=REPO / 'SW/simulation/robot_description/robocup.calibrated.urdf')
    parser.add_argument('--bundle', type=Path, default=REPO / 'SW/simulation/calibration/records/integrated_calibration')
    args = parser.parse_args()
    source = RECORD.read_bytes()
    values = json.loads(source)['results']['T_lidar_camera']
    lidar_rgb = from_quaternion(values[:3], values[3:])
    original = args.urdf.read_bytes()
    text = original.decode()
    root = ET.fromstring(text)
    joint = root.find("joint[@name='camera_link_joint']")
    mount_parent = joint.find('parent').get('link')
    body_optical = fixed_transform(root, 'camera_link', 'camera_optical_frame')
    mount = multiply(multiply(fixed_transform(root, mount_parent, 'livox_frame'), lidar_rgb), inverse(body_optical))
    xyz = [mount[i][3] for i in range(3)]
    rpy = [math.atan2(mount[2][1], mount[2][2]),
           math.atan2(-mount[2][0], math.hypot(mount[0][0], mount[1][0])),
           math.atan2(mount[1][0], mount[0][0])]
    origin = '<origin xyz="' + ' '.join(format(v, '.17g') for v in xyz) + '" rpy="' + ' '.join(format(v, '.17g') for v in rpy) + '" />'
    pattern = r'(<joint\b[^>]*\bname="camera_link_joint"[^>]*>)(.*?)(</joint>)'
    text = re.sub(pattern, lambda m: m[1] + re.sub(r'<origin\b[^>]*/>', lambda _: origin, m[2], count=1) + m[3], text, count=1, flags=re.S)
    payload = text.encode()
    previous_hash = hashlib.sha256(original).hexdigest()
    args.urdf.write_bytes(payload)
    sidecar = Path(str(args.urdf) + '.application.json')
    metadata = json.loads(sidecar.read_text()) if sidecar.exists() else {}
    metadata.update({
        'status': 'physical_koide_head_applied',
        'previous_runtime_urdf_sha256': previous_hash,
        'runtime_urdf_sha256': hashlib.sha256(payload).hexdigest(),
        'head_selection': 'physical_koide_four_bags_20261012',
        'head_solver_metrics_basis': 'Historical, superseded Head hand-eye metrics; not Koide accuracy metrics.',
        'head_mount_delta_applied_to_koide': False,
        'ros_runtime_launched': False, 'post_work_verification_run': False,
        'koide_head_application': {
            'source': 'HW/calibration/head_mid360/calib.json',
            'source_sha256': hashlib.sha256(source).hexdigest(),
            'transform_direction': 'livox_frame <- camera_color_optical_frame',
            'translation_m_quaternion_xyzw': values,
            'joint': 'camera_link_joint', 'origin_xyz': xyz, 'origin_rpy': rpy,
            'acceptance': 'User accepted the calibration on 2026-10-12',
            'model_optical_frame': 'camera_optical_frame',
            'model_camera_link_definition': 'Virtual RGB body frame; real RealSense root uses driver internal extrinsics at runtime',
            'base_mid360_basis': 'Existing calibrated URDF fixed chain; no new Base–Mid360 measurement',
        },
    })
    metadata['remaining_independent_measurements'] = [
        'physical Base–Mid360 mount', 'physical arm mount', 'physical TCP and tool axes',
        'joint zero offsets and link geometry', 'gripper opening/zero', 'physical wrist camera calibration',
    ]
    encoded = json.dumps(metadata, indent=2, ensure_ascii=False) + '\n'
    sidecar.write_text(encoded)
    args.bundle.mkdir(parents=True, exist_ok=True)
    (args.bundle / 'summary.json').write_text(encoded)
    results = args.bundle / 'results'
    results.mkdir(exist_ok=True)
    base_rgb = multiply(fixed_transform(root, 'base_link', 'livox_frame'), lidar_rgb)
    (results / 'base_head_koide.json').write_text(json.dumps(result(base_rgb, 'base_link', 'camera_optical_frame'), indent=2) + '\n')
    (results / 'head_mid360_koide.json').write_text(json.dumps(result(inverse(lidar_rgb), 'camera_optical_frame', 'livox_frame'), indent=2) + '\n')
    print(json.dumps({'applied': str(args.urdf), 'origin_xyz': xyz, 'origin_rpy': rpy, 'verification_run': False}))


if __name__ == '__main__':
    main()
