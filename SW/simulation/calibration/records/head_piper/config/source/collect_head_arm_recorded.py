"""Collect image PnP and measured joints for a fixed Head camera and a flange tag."""
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

PROJECT = Path('/Users/seoneum/Desktop/RoboCup/calibration/project').resolve()
REPO = Path('/Users/seoneum/Desktop/RoboCup/repositories/INHA-RoboCup-Home-2th-2027')
SESSION = PROJECT / 'calibration_data/sim/head_piper'
PYTHON = PROJECT / '.venv/bin/python'
URDF = REPO / 'simulation/robot_description/robocup.urdf'
sys.path.insert(0, str(PROJECT))
from kinematics import fk

ENV = os.environ.copy()
ENV.update(GZ_IP='127.0.0.1', GZ_PARTITION='robocup_head_arm_completion')
ENV.pop('PYTHONPATH', None)
ENV.pop('PYTHONHOME', None)


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def command(argv, timeout=45):
    argv = [str(x) for x in argv]
    with (SESSION / 'commands.jsonl').open('a') as stream:
        stream.write(json.dumps({'argv': argv, 'partition': ENV['GZ_PARTITION']}) + '\n')
    result = subprocess.run(argv, cwd=PROJECT, env=ENV, text=True, capture_output=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout


def make_scene():
    spec = importlib.util.spec_from_file_location('world_builder', REPO / 'simulation/gazebo/make_sim.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    builder.DESCRIPTION = URDF.parent
    builder.ROBOT_URDF = URDF
    urdf = builder.load_robot_urdf()
    # This collection requires Head RGB and actual joints only. Preserve the RGB grid.
    for gazebo in urdf.getroot().findall('gazebo'):
        for sensor in list(gazebo.findall('sensor')):
            if sensor.get('name') != 'd435f_color':
                gazebo.remove(sensor)
            else:
                sensor.find('update_rate').text = '10'
    builder.set_wheel_contacts(urdf)
    model = builder.urdf_to_model(urdf)
    builder.add_robot_plugins(model)
    world = builder.build_world(model)
    zero = {f'piper_joint{i}': 0.0 for i in range(1, 7)}
    camera = fk(URDF, zero, 'base_link', 'camera_optical_frame')
    flange = fk(URDF, zero, 'base_link', 'piper_link6')
    tag_camera = np.eye(4)
    tag_camera[:3, :3] = np.diag([1.0, -1.0, -1.0])
    tag_camera[2, 3] = 0.35
    flange_tag = np.linalg.inv(flange) @ camera @ tag_camera
    pose = ' '.join(str(x) for x in [*flange_tag[:3, 3], *Rotation.from_matrix(flange_tag[:3, :3]).as_euler('xyz')])
    link = model.find("link[@name='piper_link6']")
    if link is None:
        raise ValueError('Moving flange link missing from converted SDF')
    visual = ET.SubElement(link, 'visual', name='head_arm_apriltag')
    ET.SubElement(visual, 'pose').text = pose
    plane = ET.SubElement(ET.SubElement(visual, 'geometry'), 'plane')
    ET.SubElement(plane, 'normal').text = '0 0 1'
    ET.SubElement(plane, 'size').text = '0.1 0.1'
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = '1 1 1 1'
    ET.SubElement(material, 'diffuse').text = '1 1 1 1'
    metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
    ET.SubElement(metal, 'albedo_map').text = str(PROJECT / 'cli/board/apriltag_36h11_id0.png')
    ET.SubElement(metal, 'metalness').text = '0'
    ET.SubElement(metal, 'roughness').text = '1'
    path = SESSION / 'config/head_arm.world.sdf'
    ET.indent(world)
    ET.ElementTree(world).write(path, encoding='utf-8', xml_declaration=True)
    save(SESSION / 'config/scene.json', {
        'urdf_sha256': hashlib.sha256(URDF.read_bytes()).hexdigest(),
        'urdf': str(URDF), 'scene_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'fixture_placement_only': True, 'T_flange_tag_for_fixture_only': flange_tag.tolist(),
        'tag_black_edge_m': 0.08, 'collector_inputs': 'RGB + CameraInfo + measured joints',
        'camera_transform_not_used_in_solver': True, 'simulation': True,
    })
    return path


def main():
    SESSION.mkdir(parents=True, exist_ok=True)
    (SESSION / 'config').mkdir(exist_ok=True)
    if (SESSION / 'dataset.json').exists():
        raise ValueError('Preserve existing captures; choose a new session')
    world = make_scene()
    processes = []
    handles = []
    try:
        for mode, name in [('-s', 'server'), ('-g', 'gui')]:
            stream = (SESSION / (name + '.log')).open('w')
            handles.append(stream)
            argv = ['gz', 'sim', mode, '--render-engine-api-backend', 'metal']
            if mode == '-s':
                argv += ['-r', str(world)]
            processes.append(subprocess.Popen(argv, cwd=PROJECT, env=ENV, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
        for attempt in range(12):
            if processes[0].poll() is not None:
                raise RuntimeError('Gazebo server exited: ' + (SESSION / 'server.log').read_text()[-4000:])
            try:
                output = command(['gz', 'service', '-s', '/world/robocup_motion/control', '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean', '--timeout', '1000', '--req', 'pause: false'], timeout=4)
                if 'true' in output:
                    break
            except (RuntimeError, subprocess.TimeoutExpired):
                pass
            time.sleep(0.5)
        else:
            raise RuntimeError('Gazebo did not become available')
        time.sleep(2)
        records = []
        for name, split in [('wrist_train.csv', 'train'), ('wrist_holdout10.csv', 'holdout')]:
            table = PROJECT / 'cli/poses' / name
            for row in csv.DictReader(table.open()):
                command([PYTHON, PROJECT / 'move_pose.py', '--table', table, '--sample', row['sample_id']])
                time.sleep(0.8)
                try:
                    output = command([PYTHON, PROJECT / 'capture_pose.py', '--camera', 'head', '--partition', ENV['GZ_PARTITION'], '--urdf', URDF, '--dataset', SESSION / 'dataset.json', '--split', split, '--timeout', '22'], timeout=35)
                    capture = json.loads(output.strip().splitlines()[-1])
                    sample = json.loads((SESSION / 'dataset.json').read_text())['samples'][-1]
                    targets = np.array([float(row[f'joint{i}_rad']) for i in range(1, 7)])
                    actual = np.array([sample['measured_joint_positions'][f'piper_joint{i}'] for i in range(1, 7)])
                    if np.max(np.abs(actual - targets)) > 0.015:
                        raise RuntimeError('Actual joints differ from command; collection stopped')
                    record = {'split': split, 'sample_id': row['sample_id'], 'image_id': sample['id'], 'joint_targets_rad': targets.tolist(), 'measured_joint_positions': sample['measured_joint_positions'], 'max_command_error_rad': float(np.max(np.abs(actual - targets))), 'capture': capture}
                    records.append(record)
                    save(SESSION / split / row['sample_id'] / 'capture_record.json', record)
                    print(json.dumps({'stage': 'capture', 'split': split, 'sample': row['sample_id'], 'captured': len(records)}), flush=True)
                except (RuntimeError, subprocess.TimeoutExpired) as exc:
                    save(SESSION / split / row['sample_id'] / 'failure.json', {'error': str(exc), 'command': row, 'not_used_as_successful_observation': True})
                    print(json.dumps({'stage': 'capture_failure', 'split': split, 'sample': row['sample_id'], 'error': str(exc)[-700:]}), flush=True)
        save(SESSION / 'capture_manifest.json', {'records': records, 'simulation': True})
    finally:
        for process in reversed(processes):
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=5)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for handle in handles:
            handle.close()
    (SESSION / 'results').mkdir(exist_ok=True)
    print(command([PYTHON, PROJECT / 'solve_handeye.py', SESSION / 'dataset.json', '--output', SESSION / 'results/handeye.json']), flush=True)
    print(command([PYTHON, PROJECT / 'calibration_workflow.py', 'normalize-handeye', '--input', SESSION / 'results/handeye.json', '--parent', 'piper_base_link', '--child', 'camera_optical_frame', '--output', SESSION / 'results/piper_head.json']), flush=True)
    print(json.dumps({'stage': 'done', 'session': str(SESSION)}), flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        save(SESSION / 'failure.json', {'error': str(exc)})
        raise
