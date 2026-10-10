#!/usr/bin/env python3
"""Assemble recorded sensor estimates and explicit URDF references into one runtime model.

This performs the requested transform composition/application. It does not run ROS,
Gazebo, tests, or an independent accuracy check. Original measurements are preserved.
"""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

import calibration_workflow as workflow

HERE = Path(__file__).resolve().parent
DESCRIPTION = HERE.parent / 'robot_description'
RECORDS = HERE / 'records'
HW_CALIBRATION = HERE.parents[2] / 'HW' / 'calibration'


def invoke(name, **kwargs):
    getattr(workflow, name)(SimpleNamespace(**kwargs))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=RECORDS / 'integrated_calibration')
    parser.add_argument('--runtime', type=Path, default=DESCRIPTION / 'robocup.calibrated.urdf')
    parser.add_argument('--replace', action='store_true', help='Replace the selected generated bundle/runtime, preserving recorded inputs')
    args = parser.parse_args()
    output = args.output_dir.resolve()
    runtime = args.runtime.resolve()
    if output.exists() and not args.replace:
        raise ValueError('Output bundle already exists; choose a new path or explicitly use --replace')
    nominal = DESCRIPTION / 'robocup.urdf'
    head_arm = RECORDS / 'head_piper/results/piper_head.json'
    handeye = workflow.read(RECORDS / 'head_piper/results/handeye.json')
    if handeye.get('synthetic') or handeye.get('mode') != 'eye_on_base':
        raise ValueError('Expected measured eye-on-base Head–PiPER input')
    # The hand-eye solver already reports held-out residuals as part of calibration.
    # Use those recorded values, without reprocessing images or launching a validation job.
    head_arm_usable = handeye['max_holdout_translation_m'] <= 0.005 and handeye['max_holdout_rotation_deg'] <= 1.0
    # Physical G2 estimate against the Mid-360S (2026-10-10). The Gazebo record
    # records/20261005_base_2dlidar is preserved as history.
    lidar_input = HW_CALIBRATION / 'base_2dlidar/calibration.json'
    lidar = workflow.read(lidar_input)
    if lidar.get('measurement') != 'physical_robot' or lidar.get('status') != 'physical_holdout_validated':
        raise ValueError('Expected the held-out validated physical Base–2D LiDAR record')
    mid_input = RECORDS / '20261006_base_mid360/results/auto_room_20261006_031114/base_mid360.json'
    head_input = RECORDS / '20261006_head_mid360/results/automated_01/head_mid360.json'
    wrist_input = RECORDS / '20261006_wrist_d435/results/flange_wrist.json'
    tcp_input = RECORDS / 'link6_tcp/results/flange_tcp.json'
    tcp = workflow.read(tcp_input)
    if (tcp.get('status') != 'passed_provisional_limits'
            or tcp.get('input_kind') != 'simulated_constrained_joint_observations'
            or not tcp.get('simulation') or tcp.get('independent_measurement') is not False):
        raise ValueError('Expected the completed, explicitly marked simulated TCP pivot result')
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='calibration_', dir=output.parent) as temp:
        bundle = Path(temp)
        results = bundle / 'results'
        results.mkdir()
        steps = bundle / 'application'
        steps.mkdir()
        invoke('normalize_base', input=lidar_input, output=results / 'base_lidar.json')
        for source, name in [(mid_input, 'base_mid360.json'), (head_input, 'head_mid360_recorded.json'), (wrist_input, 'flange_wrist.json'), (head_arm, 'piper_head.json')]:
            shutil.copy2(source, results / name)
        invoke('urdf_reference', urdf=nominal, parent='base_link', child='piper_base_link',
               reference_child=None, offset_xyz=[0., 0., 0.], offset_rpy=[0., 0., 0.],
               definition='PiPER mounting reference from the CAD/URDF fixed chain; no independent measurement',
               output=results / 'base_piper.json')
        root = ET.parse(nominal).getroot()
        jaw_origins = []
        for name in ('piper_gripper_joint1', 'piper_gripper_joint2'):
            joint = root.find(f"joint[@name='{name}']")
            if joint is None or joint.find('parent').get('link') != 'piper_gripper_base':
                raise ValueError('Expected two gripper joints below piper_gripper_base')
            jaw_origins.append(np.fromstring(joint.find('origin').get('xyz', '0 0 0'), sep=' '))
        offset = np.mean(jaw_origins, axis=0).tolist()
        invoke('urdf_reference', urdf=nominal, parent='piper_link6', child='tcp',
               reference_child='piper_gripper_base', offset_xyz=offset, offset_rpy=[0., 0., 0.],
               definition='Nominal midpoint of the two jaw joint origins at symmetric opening; axes follow gripper base. Not a measured fingertip/contact point.',
               output=results / 'flange_tcp_nominal.json')
        # Preserve the old CAD reference separately; use actual stepped joint observations.
        shutil.copy2(tcp_input, results / 'flange_tcp.json')
        # The archived result's orientation-source path is relative to its original session.
        copied_tcp = workflow.read(results / 'flange_tcp.json')
        import os
        copied_tcp['orientation_source'] = os.path.relpath((tcp_input.parent / tcp['orientation_source']).resolve(), results)
        (results / 'flange_tcp.json').write_text(json.dumps(copied_tcp, indent=2, allow_nan=False) + '\n')
        invoke('compose', left=results / 'base_mid360.json', right=results / 'head_mid360_recorded.json',
               inverse_left=False, inverse_right=True, output=results / 'base_head_via_lidar.json')
        invoke('compose', left=results / 'base_piper.json', right=results / 'piper_head.json',
               inverse_left=False, inverse_right=False, output=results / 'base_head_via_arm.json')
        invoke('compare', left=results / 'base_head_via_lidar.json', right=results / 'base_head_via_arm.json',
               output=results / 'head_path_difference.json')
        selected_head = 'base_head_via_arm.json' if head_arm_usable else 'base_head_via_lidar.json'
        shutil.copy2(results / selected_head, results / 'base_head.json')
        # Recorded hand-eye observations predate the selected CAD mount. Move their
        # Head frame by the same rack-space rigid delta used for the nominal model.
        # Verify the observation identities so a future new calibration is never
        # silently tilted a second time.
        head_mount = None
        mount_path = DESCRIPTION / 'head_mount_delta.json'
        if mount_path.exists():
            head_mount = workflow.read(mount_path)
            for relative, expected in head_mount['head_measurement_sources'].items():
                canonical = json.dumps(workflow.read(RECORDS / relative), sort_keys=True,
                                       separators=(',', ':'), allow_nan=False).encode()
                if hashlib.sha256(canonical).hexdigest() != expected:
                    raise ValueError('Head measurements changed; update/remove head_mount_delta.json for the new mount calibration')
            camera = workflow.fixed_fk(root, 'rack_base_link', 'camera_link')
            if not np.allclose(camera, workflow.se3(head_mount['camera_new_rack_frame']), atol=1e-9, rtol=0):
                raise ValueError('Nominal Head mount differs from head_mount_delta.json')
            rack = workflow.fixed_fk(root, 'base_link', 'rack_base_link')
            delta = rack @ workflow.se3(head_mount['old_to_new_rack_transform']) @ np.linalg.inv(rack)
            recorded, measured = workflow.result(results / 'base_head.json')
            (results / 'base_head.json').rename(results / 'base_head_recorded.json')
            workflow.write(results / 'base_head.json', workflow.transform(
                delta @ measured, recorded['parent_frame'], recorded['child_frame'],
                status='cad_mount_delta_applied', independent_measurement=False,
                method='Rigid CAD mount delta applied to historical Head hand-eye; not a new calibration',
                recorded_transform='base_head_recorded.json', mount_change=head_mount))
        # Express a coherent Head–Mid360 transform from the selected Head runtime chain.
        # Keep the independently fitted plane estimate under its recorded name.
        invoke('compose', left=results / 'base_head.json', right=results / 'base_mid360.json',
               inverse_left=True, inverse_right=False, output=results / 'head_mid360_runtime.json')
        source = nominal
        applications = []
        for name, target in [('base_lidar', None), ('base_mid360', None), ('base_piper', None),
                             ('base_head', 'camera_link'), ('flange_wrist', 'wrist_camera_mount_link')]:
            destination = steps / (name + '.urdf')
            invoke('patch', urdf=source, result=results / (name + '.json'), output=destination,
                   mount_child=target, portable_meshes=True)
            applications.append(workflow.read(str(destination) + '.application.json'))
            source = destination
        # Write directly beside the final runtime so relative mesh paths remain portable.
        runtime.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='runtime_', dir=runtime.parent) as runtime_temp:
            staged_runtime = Path(runtime_temp) / runtime.name
            invoke('add_tcp', urdf=source, result=results / 'flange_tcp.json', output=staged_runtime, portable_meshes=True)
            tree = ET.parse(staged_runtime)
            import os
            for mesh in tree.findall('.//mesh'):
                name = mesh.get('filename', '')
                if name and not name.startswith(('file:', 'package:', 'http:', 'https:')):
                    mesh.set('filename', os.path.relpath((staged_runtime.parent / name).resolve(), runtime.parent))
            tree.write(staged_runtime, encoding='utf-8', xml_declaration=True)
            tcp_application = workflow.read(str(staged_runtime) + '.application.json')
            tcp_application['runtime_urdf_sha256'] = workflow.sha(staged_runtime)
            head_selection = 'independent_head_arm_handeye' if head_arm_usable else 'recorded_head_lidar_plane_estimate'
            if head_mount:
                head_selection = 'recorded_head_arm_handeye_with_cad_mount_delta' if head_arm_usable else 'recorded_head_lidar_with_cad_mount_delta'
            summary = {
                'status': 'simulation_calibration_integrated',
                'transform_convention': 'parent <- child, metres',
                'runtime': 'SW/simulation/robot_description/' + runtime.name, 'runtime_path_basis': 'repository_root', 'runtime_urdf_sha256': workflow.sha(staged_runtime),
                'nominal_urdf_sha256': workflow.sha(nominal),
                'head_selection': head_selection,
                'head_mount_change': head_mount,
                'head_solver_metrics_basis': 'Historical original-mount measurements; no new mount calibration.' if head_mount else 'Recorded selected Head measurements.',
                'head_arm_solver_holdout': {
                    'train_count': handeye['training_count'], 'holdout_count': handeye['holdout_count'],
                    'max_translation_mm': handeye['max_holdout_translation_m'] * 1000,
                    'max_rotation_deg': handeye['max_holdout_rotation_deg'],
                    'within_provisional_limits': head_arm_usable, 'additional_validation_run': False,
                },
                'head_path_difference': workflow.read(results / 'head_path_difference.json'),
                'head_paths_share_nominal_arm_mount': True,
                'base_lidar_input': os.path.relpath(lidar_input, HERE.parents[2]).replace(os.sep, '/'),
                'base_lidar_input_kind': 'physical_static_multipose_vs_mid360',
                'base_lidar_holdout': {k: lidar['metrics']['holdout']['est'][k] for k in ('med_cm', 'p90_cm', 'in2cm')},
                'base_piper_input_kind': 'urdf_reference', 'tcp_input_kind': tcp['input_kind'],
                'tcp_definition': workflow.read(results / 'flange_tcp.json')['definition'],
                'tcp_solver_holdout': {
                    'train_count': tcp['training_count'], 'holdout_count': tcp['holdout_count'],
                    'max_translation_mm': tcp['max_holdout_mm'], 'max_allowed_mm': tcp['max_allowed_holdout_mm'],
                    'condition_number': tcp['condition_number'], 'orientation_estimated': False,
                    'within_provisional_limits': True, 'additional_validation_run': False,
                },
                'tcp_translation_xyz_m': tcp['translation_xyz_m'],
                'tcp_result_source_sha256': workflow.sha(tcp_input),
                'tcp_simulation': True, 'tcp_independent_measurement': False,
                'tcp_fixture_kind': tcp['fixture_kind'],
                'hardware_accuracy_established': False, 'ros_runtime_launched': False,
                'post_work_verification_run': False,
                'remaining_independent_measurements': ['physical arm mount', 'physical TCP contact/pivot and tool axes',
                    'joint zero offsets, axes and link geometry', 'gripper opening/zero',
                    'real sensor calibration (Base–2D LiDAR x/y/yaw done relative to Mid-360S; Mid-360S and cameras pending)'],
                'historical_head_plane_bias': 'Recorded Head–Mid360 plane estimate has 11.720 mm Gazebo GT position error; retained and not silently corrected.',
                'applications': applications, 'tcp_application': tcp_application,
            }
            workflow.write(bundle / 'summary.json', summary)
            if output.exists():
                shutil.rmtree(output)
            shutil.copytree(bundle, output)
            staged_runtime.replace(runtime)
            application = runtime.with_name(runtime.name + '.application.json')
            application.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
        print(json.dumps({k: summary[k] for k in ('status', 'head_selection', 'head_arm_solver_holdout', 'tcp_solver_holdout', 'tcp_translation_xyz_m', 'runtime')}, indent=2))


if __name__ == '__main__':
    main()
