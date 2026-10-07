#!/usr/bin/env python3
"""Collect a simulated TCP pivot from stepped dynamics and actual joint positions.

The fixed socket and virtual probe are a MuJoCo point equality constraint, not
physical contact measurements. CAD defines the fixture; only measured qpos/FK
enters the pivot solver. No joint state is teleported after initialization.
"""
import argparse
import csv
import json
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from calibration_workflow import fixed_fk, sha, transform, write

HERE = Path(__file__).resolve().parent
JOINTS = [f'piper_joint{i}' for i in range(1, 7)]
DEFINITION = ('Virtual probe at the symmetric CAD jaw-tip centre, attached rigidly '
              'to Link6 and seated in a fixed simulated point socket. '
              'TCP axes are defined parallel to Link6; orientation is not fitted.')


def numbers(values):
    return ' '.join(format(float(x), '.17g') for x in values)


def origin(element):
    o = element.find('origin')
    t = np.eye(4)
    if o is not None:
        t[:3, 3] = np.fromstring(o.get('xyz', '0 0 0'), sep=' ')
        t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(o.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return t


class Arm:
    def __init__(self, urdf):
        self.root = ET.parse(urdf).getroot()
        self.joints = [self.root.find(f"joint[@name='{name}']") for name in JOINTS]
        self.origins = [origin(j) for j in self.joints]
        self.axes = [np.fromstring(j.find('axis').get('xyz'), sep=' ') for j in self.joints]
        self.limits = np.array([[float(j.find('limit').get(k)) for k in ('lower', 'upper')] for j in self.joints])
        parent = 'piper_base_link'
        for joint in self.joints:
            if joint.find('parent').get('link') != parent:
                raise ValueError('Expected the six-joint PiPER serial chain')
            parent = joint.find('child').get('link')

    def fk(self, q):
        t = np.eye(4)
        for local, axis, value in zip(self.origins, self.axes, q):
            motion = np.eye(4)
            motion[:3, :3] = Rotation.from_rotvec(value * axis / np.linalg.norm(axis)).as_matrix()
            t = t @ local @ motion
        return t

    def command(self, seed, rotation, pivot, probe):
        def residual(q):
            t = self.fk(q)
            return np.r_[20 * (t[:3, :3] @ probe + t[:3, 3] - pivot),
                         Rotation.from_matrix(rotation.T @ t[:3, :3]).as_rotvec()]
        solution = least_squares(residual, seed, bounds=(self.limits[:, 0] + .01, self.limits[:, 1] - .01),
                                 xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=150)
        # This is a command feasibility gate, never the recorded joint observation.
        if np.linalg.norm(residual(solution.x)) > 1e-5:
            return None
        return solution.x


def probe_from_cad(arm, urdf, config):
    """Place the virtual probe at the CAD fingertip axial extent, centred on the jaws."""
    extents = []
    sources = []
    for name in ('piper_gripper_link1', 'piper_gripper_link2'):
        link = arm.root.find(f"link[@name='{name}']")
        collision = link.find('collision')
        mesh = collision.find('geometry/mesh')
        source = (urdf.parent / mesh.get('filename')).resolve()
        raw = source.read_bytes()
        count = struct.unpack('<I', raw[80:84])[0]
        if len(raw) != 84 + 50 * count:
            raise ValueError('Expected the supplied binary STL jaw mesh')
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        vertices = np.frombuffer(raw, dtype=dtype, offset=84, count=count)['vertices'].reshape(-1, 3).astype(float)
        vertices *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        joint = next(j for j in arm.root.findall('joint') if j.find('child').get('link') == name)
        local = origin(joint) @ origin(collision)  # symmetric closed opening, q=0
        # Explicit contraction avoids the platform BLAS status warnings on STL batches.
        points = np.einsum('ij,kj->ki', local[:3, :3], vertices) + local[:3, 3]
        if not np.isfinite(points).all():
            raise ValueError('Nonfinite CAD fingertip coordinates')
        extents.append(float(points[:, 2].max()))
        shutil.copy2(source, config / source.name)
        sources.append({'path': source.name, 'sha256': sha(source), 'tip_z_gripper_m': extents[-1]})
    probe = fixed_fk(arm.root, 'piper_link6', 'piper_gripper_base') @ np.r_[0., 0., np.mean(extents), 1.]
    return probe[:3], sources


def scene(arm, probe, pivot):
    root = ET.Element('mujoco', model='inha_tcp_pivot_fixture')
    ET.SubElement(root, 'compiler', angle='radian', fusestatic='false', inertiafromgeom='false')
    ET.SubElement(root, 'option', timestep='.001', gravity='0 0 -9.81', integrator='implicitfast',
                  iterations='100', tolerance='1e-10')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'site', name='socket', pos=numbers(pivot), size='.006', rgba='1 .3 .1 1')
    links = {l.get('name'): l for l in arm.root.findall('link')}
    joints = arm.root.findall('joint')
    bodies = {}

    def body(name, parent, joint=None):
        t = origin(joint) if joint is not None else np.eye(4)
        b = ET.SubElement(parent, 'body', name=name, pos=numbers(t[:3, 3]),
                          quat=numbers(Rotation.from_matrix(t[:3, :3]).as_quat()[[3, 0, 1, 2]]))
        bodies[name] = b
        inertial = links[name].find('inertial')
        if inertial is not None and float(inertial.find('mass').get('value')) > 0:
            it = origin(inertial)
            i = inertial.find('inertia').attrib
            matrix = np.array([[float(i['ixx']), float(i['ixy']), float(i['ixz'])],
                               [float(i['ixy']), float(i['iyy']), float(i['iyz'])],
                               [float(i['ixz']), float(i['iyz']), float(i['izz'])]])
            matrix = it[:3, :3] @ matrix @ it[:3, :3].T
            ET.SubElement(b, 'inertial', pos=numbers(it[:3, 3]), mass=inertial.find('mass').get('value'),
                          fullinertia=numbers([matrix[0, 0], matrix[1, 1], matrix[2, 2], matrix[0, 1], matrix[0, 2], matrix[1, 2]]))
        if joint is not None and joint.get('name') in JOINTS:
            limit = joint.find('limit')
            ET.SubElement(b, 'joint', name=joint.get('name'), type='hinge', axis=joint.find('axis').get('xyz'),
                          range=f"{limit.get('lower')} {limit.get('upper')}", damping='2', armature='.01')
        for child in (j for j in joints if j.find('parent').get('link') == name):
            if child.get('type') != 'fixed' and child.get('name') not in JOINTS + ['piper_gripper_joint1', 'piper_gripper_joint2']:
                raise ValueError('Unexpected moving descendant: ' + child.get('name'))
            body(child.find('child').get('link'), b, child)

    body('piper_base_link', world)
    ET.SubElement(bodies['piper_link6'], 'site', name='probe', pos=numbers(probe), size='.003', rgba='0 .8 .2 1')
    equality = ET.SubElement(root, 'equality')
    ET.SubElement(equality, 'connect', name='fixed_socket', site1='probe', site2='socket',
                  solref='.002 1', solimp='.999 .999 .001')
    actuators = ET.SubElement(root, 'actuator')
    for j in arm.joints:
        limit = j.find('limit')
        ET.SubElement(actuators, 'position', name=j.get('name'), joint=j.get('name'), kp='1200', kv='70',
                      ctrlrange=f"{limit.get('lower')} {limit.get('upper')}",
                      forcerange=f"-{limit.get('effort')} {limit.get('effort')}")
    ET.indent(root)
    return ET.tostring(root, encoding='unicode')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', required=True, type=Path, help='New, empty session directory')
    p.add_argument('--urdf', type=Path, default=HERE.parent / 'robot_description/robocup.urdf')
    p.add_argument('--train', type=int, default=25)
    p.add_argument('--holdout', type=int, default=10)
    p.add_argument('--seed', type=int, default=73)
    args = p.parse_args()
    if args.train < 6 or args.holdout < 3:
        raise ValueError('Need at least six train and three holdout poses')
    session, urdf = args.session.resolve(), args.urdf.resolve()
    session.mkdir(parents=True, exist_ok=False)
    config = session / 'config'
    config.mkdir()
    shutil.copy2(urdf, config / 'source.urdf')
    shutil.copy2(Path(__file__), config / 'collector.py')
    arm = Arm(urdf)
    probe, mesh_sources = probe_from_cad(arm, urdf, config)
    q_initial = np.array([-.6, 1.2, -1.6, .15, .7, 0.])
    initial = arm.fk(q_initial)
    pivot = initial[:3, :3] @ probe + initial[:3, 3]
    xml = scene(arm, probe, pivot)
    (config / 'scene.xml').write_text(xml)
    (config / 'tcp_orientation.md').write_text('# TCP orientation definition\n\n' + DEFINITION + '\n\n'
        '+X/+Y/+Z parallel to piper_link6. RPY = (0, 0, 0) rad. CAD axes, not a pivot orientation estimate.\n')
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    qadr = [int(model.joint(name).qposadr[0]) for name in JOINTS]
    vadr = [int(model.joint(name).dofadr[0]) for name in JOINTS]
    actuators = [model.actuator(name).id for name in JOINTS]
    data.qpos[qadr] = q_initial  # initialization only; no reset/teleport in the collection loop
    data.ctrl[actuators] = q_initial
    mujoco.mj_forward(model, data)
    probe_site, socket_site = model.site('probe').id, model.site('socket').id
    rng = np.random.default_rng(args.seed)
    commands = []
    for attempt in range(1000):
        delta = rng.uniform(-.42, .42, 3)
        desired = Rotation.from_rotvec(delta).as_matrix() @ initial[:3, :3]
        command = arm.command(q_initial, desired, pivot, probe)
        if command is not None and all(np.linalg.norm(command - q) > .08 for q in commands):
            commands.append(command)
        if len(commands) == args.train + args.holdout:
            break
    if len(commands) != args.train + args.holdout:
        raise RuntimeError('Not enough feasible, distinct pivot orientation commands')
    provenance = dict(input_kind='simulated_constrained_joint_observations', simulation=True,
        independent_measurement=False, hardware_accuracy_established=False, definition=DEFINITION,
        simulator='MuJoCo', simulator_version=mujoco.__version__, urdf_sha256=sha(urdf),
        scene_sha256=sha(config / 'scene.xml'), collector_sha256=sha(Path(__file__)),
        fixture_kind='site-to-site point equality; virtual ball socket, not frictional surface contact',
        mesh_sources=mesh_sources, probe_offset_for_scene_only_m=probe.tolist(),
        socket_for_scene_only_m=pivot.tolist(), joint_names=JOINTS,
        initial_joints_rad=q_initial.tolist(), gripper_opening='symmetric closed, frozen at q=0',
        collision_geometry_in_dynamics=False, gravity_m_s2=[0, 0, -9.81],
        seed=args.seed, timestep_s=float(model.opt.timestep), contact_limit_mm=.2,
        velocity_limit_rad_s=.002, train_requested=args.train, holdout_requested=args.holdout,
        observation_source='qpos after mj_step, then nominal URDF FK; IK values are commands only',
        limitations='Shared CAD kinematics and an imposed virtual constraint; not independent hardware calibration')
    write(config / 'provenance.json', provenance)
    dataset = dict(translation_unit='m', base_frame='piper_base_link', flange_frame='piper_link6',
                   **{k: provenance[k] for k in ('input_kind', 'simulation', 'independent_measurement',
                       'hardware_accuracy_established', 'definition', 'fixture_kind')}, samples=[])
    write(config / 'commands.json', {'purpose': 'IK actuator commands only; not pivot observations',
                                    'joint_names': JOINTS, 'positions_rad': [q.tolist() for q in commands]})
    trace_file = (session / 'dynamics.csv').open('x', newline='')
    trace = csv.writer(trace_file)
    trace.writerow(['simulation_time_s', *JOINTS, 'probe_socket_distance_mm', 'max_joint_velocity_rad_s'])
    step_count = 0

    def advance(steps):
        nonlocal step_count
        for _ in range(steps):
            mujoco.mj_step(model, data)
            step_count += 1
            if step_count % 20 == 0:
                trace.writerow([float(data.time), *data.qpos[qadr].tolist(),
                    float(np.linalg.norm(data.site_xpos[probe_site] - data.site_xpos[socket_site]) * 1000),
                    float(np.max(np.abs(data.qvel[vadr])))])
        if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):
            raise RuntimeError('Simulation instability; refusing a pivot observation')

    try:
        advance(2000)
        for index, command in enumerate(commands):
            start_command = data.ctrl[actuators].copy()
            for part in range(1, 41):
                f = part / 40
                data.ctrl[actuators] = start_command + (command - start_command) * (3*f*f - 2*f*f*f)
                advance(25)
            stable = 0
            for _ in range(120):
                advance(50)
                mujoco.mj_forward(model, data)
                distance = float(np.linalg.norm(data.site_xpos[probe_site] - data.site_xpos[socket_site]) * 1000)
                speed = float(np.max(np.abs(data.qvel[vadr])))
                stable = stable + 1 if distance <= .2 and speed <= .002 else 0
                if stable >= 10:
                    break
            if stable < 10:
                raise RuntimeError(f'Pose {index}: fixture/settling gate failed: {distance} mm, {speed} rad/s')
            actual = data.qpos[qadr].copy()
            if np.any(actual < arm.limits[:, 0]) or np.any(actual > arm.limits[:, 1]):
                raise RuntimeError('Actual joint position outside URDF limits')
            split = 'train' if index < args.train else 'holdout'
            folder = session / split / f'{index:03d}'
            measured = dict(zip(JOINTS, actual.tolist()))
            raw = dict(simulator='MuJoCo', simulation_stamp=float(data.time), positions=measured,
                velocities=dict(zip(JOINTS, data.qvel[vadr].tolist())), commanded_positions=dict(zip(JOINTS, command.tolist())),
                probe_socket_distance_mm=distance, max_joint_velocity_rad_s=speed,
                equality_force=data.efc_force[np.asarray(data.efc_type) == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)].tolist(),
                constraint_active=bool(data.eq_active[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, 'fixed_socket')]),
                scene_sha256=provenance['scene_sha256'])
            write(folder / 'joint_observation.json', raw)
            pose = transform(arm.fk(actual), 'piper_base_link', 'piper_link6', measured_joint_positions=measured,
                             simulation_stamp=float(data.time), observation_sha256=sha(folder / 'joint_observation.json'))
            write(folder / 'flange_pose.json', pose)
            dataset['samples'].append(dict(id=index, split=split, T_base_flange=pose['matrix4x4'],
                measured_joint_positions=measured, simulation_stamp=float(data.time),
                pose_sha256=sha(folder / 'flange_pose.json'), observation_sha256=sha(folder / 'joint_observation.json'),
                contact_note='Fixed simulated point equality active; settled actual qpos, no teleport',
                probe_socket_distance_mm=distance, max_joint_velocity_rad_s=speed))
            temp = session / 'dataset.json.tmp'
            temp.write_text(json.dumps(dataset, indent=2, allow_nan=False) + '\n')
            temp.replace(session / 'dataset.json')
            print(json.dumps({'sample': index, 'split': split, 'simulation_time_s': float(data.time),
                              'fixture_residual_mm': distance, 'speed_rad_s': speed}), flush=True)
        write(session / 'capture_summary.json', dict(status='collected_simulated_constrained_observations',
            train_count=args.train, holdout_count=args.holdout, dynamics_steps=step_count,
            simulation_duration_s=float(data.time), max_sample_fixture_residual_mm=max(s['probe_socket_distance_mm'] for s in dataset['samples']),
            dataset_sha256=sha(session / 'dataset.json'), provenance=provenance,
            post_work_verification_run=False))
    finally:
        trace_file.close()


if __name__ == '__main__':
    main()
