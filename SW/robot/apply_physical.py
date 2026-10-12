#!/usr/bin/env python3
"""Generate Jetson runtime parameters from committed physical measurements."""
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

REPO = Path(__file__).resolve().parents[2]
HERE = REPO / 'SW/robot'
PHYSICAL = REPO / 'HW/calibration/physical'
sys.path.insert(0, str(REPO / 'HW/calibration/head_mid360/tools'))
from apply_koide import fixed_transform


def main():
    config = HERE / 'config'
    config.mkdir(exist_ok=True)
    odom = json.loads((PHYSICAL / 'tracer_odom/odom_calibration.json').read_text())
    mapping = {
        'linear_velocity_scale': 'linear_velocity_scale',
        'linear_velocity_offset': 'linear_velocity_offset_m_s',
        'linear_offset_threshold': 'linear_offset_threshold_m_s',
        'angular_velocity_scale': 'angular_velocity_scale',
        'angular_velocity_offset': 'angular_velocity_offset_rad_s',
        'angular_offset_threshold': 'angular_offset_threshold_rad_s',
        'angular_velocity_bias': 'angular_velocity_bias_rad_s',
        'straight_yaw_deadband': 'straight_yaw_deadband_rad_s',
        'straight_yaw_deadband_min_linear': 'straight_yaw_deadband_min_linear_m_s',
    }
    params = {name: odom[key] for name, key in mapping.items()}
    params.update({'port_name': 'can2', 'odom_frame': 'odom', 'base_frame': 'base_link',
                   'odom_topic_name': 'odom', 'is_tracer_mini': False,
                   'simulated_robot': False, 'use_sim_time': False})
    (config / 'tracer.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}, sort_keys=False))
    mid = json.loads((PHYSICAL / 'base_mid360/base_mid360.json').read_text())
    rotation = [mid['matrix4x4'][i][j] for i in range(3) for j in range(3)]
    (config / 'imu_base.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': {'r_base_imu': rotation}}}))
    ekf = yaml.safe_load((PHYSICAL / 'tracer_odom/ekf_odom_imu.yaml').read_text())['ekf_filter_node']['ros__parameters']
    ekf.update({'publish_tf': True, 'odom_frame': 'odom', 'world_frame': 'odom'})
    (config / 'ekf.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': ekf}}, sort_keys=False))

    # Use CAD geometry and arm kinematics only. No simulated sensor/TCP fits.
    nominal = REPO / 'SW/simulation/robot_description/robocup.urdf'
    original = nominal.read_bytes()
    source = ET.fromstring(original)
    robot = ET.Element('robot', {'name': 'robocup_physical_arm'})
    robot.append(copy.deepcopy(source.find("link[@name='base_link']")))
    arm_links = {link.get('name') for link in source.findall('link') if link.get('name').startswith('piper_')}
    for link in source.findall('link'):
        if link.get('name') in arm_links:
            robot.append(copy.deepcopy(link))
    for joint in source.findall('joint'):
        if joint.find('parent').get('link') in arm_links and joint.find('child').get('link') in arm_links:
            robot.append(copy.deepcopy(joint))
    mount = fixed_transform(source, 'base_link', 'piper_base_link')
    joint = ET.SubElement(robot, 'joint', {'name': 'physical_base_to_piper', 'type': 'fixed'})
    ET.SubElement(joint, 'parent', {'link': 'base_link'})
    ET.SubElement(joint, 'child', {'link': 'piper_base_link'})
    # Nominal PiPER mounting reference has no rotation.
    ET.SubElement(joint, 'origin', {'xyz': ' '.join(format(mount[i][3], '.17g') for i in range(3)), 'rpy': '0 0 0'})
    for mesh in robot.findall('.//mesh'):
        filename = mesh.get('filename', '')
        if filename and not filename.startswith(('package:', 'file:', 'http:', 'https:')):
            mesh.set('filename', os.path.relpath((nominal.parent / filename).resolve(), HERE))
    ET.indent(robot)
    payload = ET.tostring(robot, encoding='utf-8', xml_declaration=True) + b'\n'
    (HERE / 'robot.physical.urdf').write_bytes(payload)
    inputs = [
        ('Base–Mid360', PHYSICAL / 'base_mid360/base_mid360.json'),
        ('G2–Mid360 and range scale', PHYSICAL / 'base_2dlidar/vs_mid360/calibration.json'),
        ('Base–G2 record', PHYSICAL / 'base_2dlidar/calibration.json'),
        ('Head–Mid360', REPO / 'HW/calibration/head_mid360/calib.json'),
        ('Wrist hand-eye', PHYSICAL / 'wrist_handeye/calibration.json'),
        ('Head RGB intrinsics', REPO / 'HW/calibration/camera_intrinsics/head/ost.yaml'),
        ('Wrist RGB intrinsics', REPO / 'HW/calibration/camera_intrinsics/wrist/ost.yaml'),
        ('Tracer odometry v4', PHYSICAL / 'tracer_odom/odom_calibration.json'),
    ]
    manifest = {
        'measurement_selection': 'physical_robot_only',
        'inputs': [{'kind': kind, 'path': str(path.relative_to(REPO)),
                    'sha256': hashlib.sha256(path.read_bytes()).hexdigest()} for kind, path in inputs],
        'branch_sources': {'g2_mid360_odom': 'b8be83d9108b076b398a13e515e0998b6db2c79d',
                           'wrist_handeye': '0bac1e558075600d21c6b7273f26a6858266ea0d'},
        'base_piper_reference': {'kind': 'CAD_geometry_not_a_physical_calibration',
                                 'matrix4x4': mount, 'nominal_model_sha256': hashlib.sha256(original).hexdigest()},
        'arm_kinematics': 'Nominal geometry; piper_joint1 retains +1.6 rad model rotation, feedback bridge subtracts 1.6 rad from measured q1.',
        'simulated_calibrations_applied': [],
        'tcp_calibration_applied': False,
        'post_work_verification_run': False, 'runtime_launched': False,
    }
    (HERE / 'physical_application.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    print('Generated physical arm model, Tracer/IMU/EKF parameters and calibration provenance. No runtime or verification was run.')


if __name__ == '__main__':
    main()
