"""Read-only TCP collection, CAD inspection and fixed-point pivot calibration.

No motor commands, enabling, or automatic movement. Position pivot does NOT
estimate tool orientation or correct arm joint zeros/link geometry.
"""
import argparse
import hashlib
import json
import struct
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from .core import require, finite, rotation, fresh


def vec(value, length, reason):
    value = np.asarray(value, float)
    require(value.shape == (length,) and finite(value), reason)
    return value


def axis_rotation(axis, angle):
    axis = vec(axis, 3, 'AXIS'); require(np.linalg.norm(axis) > 0, 'ZERO_AXIS')
    x, y, z = axis / np.linalg.norm(axis)
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*(K@K)


def origin(element):
    result = np.eye(4); o = element.find('origin')
    if o is not None:
        result[:3, 3] = vec(np.fromstring(o.get('xyz', '0 0 0'), sep=' '), 3, 'ORIGIN')
        r, p, y = vec(np.fromstring(o.get('rpy', '0 0 0'), sep=' '), 3, 'RPY')
        result[:3, :3] = axis_rotation([0, 0, 1], y) @ axis_rotation([0, 1, 0], p) @ axis_rotation([1, 0, 0], r)
    return result


class Model:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.sha256 = hashlib.sha256(self.path.read_bytes()).hexdigest()
        self.root = ET.parse(self.path).getroot()
        self.parents = {j.find('child').get('link'): j for j in self.root.findall('joint')}

    def fk(self, parent, child, q=None):
        q = q or {}; chain = []; seen = set()
        while child != parent:
            require(child not in seen and child in self.parents, 'NO_URDF_CHAIN')
            seen.add(child); j = self.parents[child]; chain.append(j)
            child = j.find('parent').get('link')
        result = np.eye(4)
        for j in reversed(chain):
            T = origin(j); kind = j.get('type'); name = j.get('name')
            if kind != 'fixed':
                require(name in q and finite(q[name]), 'MISSING_JOINT:'+name)
                axis = vec(np.fromstring(j.find('axis').get('xyz'), sep=' '), 3, 'AXIS')
                motion = np.eye(4)
                if kind in ('revolute', 'continuous'):
                    motion[:3, :3] = axis_rotation(axis, q[name])
                elif kind == 'prismatic':
                    motion[:3, 3] = axis*q[name]
                else:
                    raise ValueError('UNSUPPORTED_JOINT:'+kind)
                T = T @ motion
            result = result @ T
        return result

    def mesh(self, link):
        collision = self.root.find("link[@name='%s']/collision" % link)
        require(collision is not None, 'NO_COLLISION_MESH')
        mesh = collision.find('geometry/mesh')
        path = (self.path.parent / mesh.get('filename')).resolve()
        data = path.read_bytes(); count = struct.unpack('<I', data[80:84])[0]
        require(len(data) == 84+50*count, 'EXPECTED_BINARY_STL')
        dtype = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
        points = np.frombuffer(data, dtype=dtype, count=count, offset=84)['v'].reshape(-1, 3).astype(float)
        points *= vec(np.fromstring(mesh.get('scale', '1 1 1'), sep=' '), 3, 'MESH_SCALE')
        T = origin(collision)
        return points @ T[:3, :3].T + T[:3, 3]


def inspect_cad(model):
    """Reproduce existing jaw-tip datum, NOT the contact-pad centroid."""
    clouds = {}; tips = []
    for link in ('piper_link6', 'piper_flange_link', 'piper_gripper_base',
                 'piper_gripper_link1', 'piper_gripper_link2'):
        T = model.fk('piper_link6', link, {'piper_gripper_joint1': 0., 'piper_gripper_joint2': 0.})
        points = model.mesh(link) @ T[:3, :3].T + T[:3, 3]
        clouds[link] = points
        if link.startswith('piper_gripper_link'):
            tips.append(float(points[:, 2].max()))
    require(abs(tips[0]-tips[1]) < 0.001, 'ASYMMETRIC_TIP_HEIGHT')
    report = dict(parent_frame='piper_link6', child_frame='tcp',
                  link6_tcp_transform={'xyz': [0., 0., float(np.mean(tips))],
                                       'quaternion_xyzw': [0., 0., 0., 1.]},
                  jaw_tip_z_m=tips, urdf_sha256=model.sha256,
                  source='CAD symmetric closed jaw-tip height on center axis',
                  independent_measurement=False, hardware_accuracy_established=False,
                  orientation_estimated=False,
                  note='A is link6 coordinate origin, not an assumed physical plate surface. B is the jaw-tip datum, not pad centroid.')
    return report, clouds


def plot_cad(report, clouds, path):
    # Optional offline visualization dependency; not required on Jetson.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 7))
    colors = ['#6366f1', '#a78bfa', '#64748b', '#0d9488', '#f59e0b']
    B = np.asarray(report['link6_tcp_transform']['xyz'])*1000
    for ax, horizontal, title in zip(axes, [0, 1], ['Side view X-Z', 'Front view Y-Z']):
        for (name, points), color in zip(clouds.items(), colors):
            points = np.unique(np.round(points, 7), axis=0)*1000
            ax.scatter(points[:, horizontal], points[:, 2], s=1, color=color, alpha=.45, label=name.replace('piper_', ''))
        ax.scatter([0, B[horizontal]], [0, B[2]], s=70, c=['#dc2626', '#2563eb'], zorder=5)
        ax.annotate('A: link6 origin', (0, 0), xytext=(8, -18), textcoords='offset points')
        ax.annotate('B: jaw-tip TCP', (B[horizontal], B[2]), xytext=(8, 10), textcoords='offset points')
        ax.plot([0, B[horizontal]], [0, B[2]], '--', color='#2563eb')
        ax.set(xlabel=('X' if horizontal == 0 else 'Y')+' in link6 (mm)', ylabel='Z in link6 (mm)', title=title)
        ax.set_aspect('equal'); ax.grid(alpha=.2); ax.invert_yaxis()
    axes[1].legend(loc='upper left', fontsize=8)
    fig.suptitle('Actual repository meshes | A -> B = %.3f mm | CAD datum, not hardware calibration' % B[2])
    fig.tight_layout(); fig.savefig(path, dpi=180); plt.close(fig)


def pose(sample):
    T = np.asarray(sample['base_link6_matrix'], float)
    require(T.shape == (4, 4) and finite(T) and np.allclose(T[3], [0, 0, 0, 1]), 'POSE_MATRIX')
    R = T[:3, :3]
    require(np.allclose(R.T@R, np.eye(3), atol=1e-6) and abs(np.linalg.det(R)-1) < 1e-6, 'POSE_ROTATION')
    return R, T[:3, 3]


def solve(dataset, quaternion, probe_offset, max_error_m=.003, max_condition=100.):
    """R_i p_probe + t_i = fixed_point. Independent holdout never enters fit."""
    rotation(quaternion); offset = vec(probe_offset, 3, 'PROBE_OFFSET')
    require(0 < max_error_m <= .01 and 1 < max_condition <= 1000, 'PIVOT_LIMITS')
    require(all(s.get('split') in ('train', 'holdout') for s in dataset['samples']), 'INVALID_SPLIT')
    train = [s for s in dataset['samples'] if s['split'] == 'train']
    held = [s for s in dataset['samples'] if s['split'] == 'holdout']
    require(len(train) >= 12 and len(held) >= 4, 'NEED_12_TRAIN_4_HOLDOUT')
    A = []; b = []
    for s in train:
        R, t = pose(s); A.append(np.c_[R, -np.eye(3)]); b.append(-t)
    A = np.vstack(A); b = np.concatenate(b)
    fit, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    condition = float(np.linalg.cond(A))
    require(rank == 6 and condition <= max_condition, 'INSUFFICIENT_ORIENTATION_DIVERSITY')
    p, center = fit[:3], fit[3:]
    def errors(samples):
        return [float(np.linalg.norm(pose(s)[0]@p+pose(s)[1]-center)) for s in samples]
    train_error, hold_error = errors(train), errors(held)
    passed = max(train_error+hold_error) <= max_error_m
    return dict(status='passed' if passed else 'failed_residual', parent_frame='piper_link6', child_frame='tcp',
                link6_tcp_transform={'xyz': (p-rotation(quaternion)@offset).tolist(),
                                     'quaternion_xyzw': list(quaternion)},
                probe_position_link6_m=p.tolist(), probe_offset_tcp_m=offset.tolist(),
                fixed_point_base_m=center.tolist(), train_errors_m=train_error, holdout_errors_m=hold_error,
                max_error_m=max_error_m, max_condition=max_condition, condition_number=condition,
                orientation_estimated=False, hardware_accuracy_established=False,
                input_kind=dataset.get('input_kind'), urdf_sha256=dataset.get('urdf_sha256'),
                joint_signs=dataset.get('joint_signs'), joint_offsets=dataset.get('joint_offsets'),
                fixture_id=dataset.get('fixture_id'),
                note='Residual is internal consistency under arm FK; not independent absolute arm accuracy.')


def apply_result(config, result, verified):
    require(result['status'] == 'passed' and result.get('input_kind') == 'physical_manual_contact', 'NOT_PHYSICAL_PASSED_RESULT')
    require(result.get('contact_geometry_verified') is True and result.get('axes_verified') is True and verified, 'TCP_GEOMETRY_NOT_VERIFIED')
    for key in ('joint_signs', 'joint_offsets'):
        value = vec(result.get(key), 6, 'MISSING_RESULT_'+key)
        require(np.array_equal(value, vec(config.get(key), 6, 'MISSING_CONFIG_'+key)), 'CALIBRATION_JOINT_MAPPING_CHANGED')
    tf = result['link6_tcp_transform']; vec(tf['xyz'], 3, 'TCP_XYZ'); rotation(tf['quaternion_xyzw'])
    require(np.linalg.norm(tf['xyz']) < .5, 'TCP_TOO_FAR')
    # [CALIB:TCP] Apply ONLY this transform. No commissioning or unrelated flags.
    config = dict(config); config['link6_tcp_transform'] = tf
    return config


def save_new(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as f:
        json.dump(data, f, indent=2, allow_nan=False); f.write('\n')


def stable_window(history, stable_s, max_spread):
    if len(history) < 10 or history[-1][0]-history[0][0] < stable_s: return False
    times = np.array([s[0] for s in history]); qs = np.array([s[1] for s in history])
    dt = np.diff(times)
    if not finite(qs) or np.any(dt <= 0) or np.max(dt) > .2: return False
    if np.max(np.ptp(qs, axis=0)) > max_spread: return False
    return np.max(np.abs(np.diff(qs, axis=0)/dt[:, None])) <= .01


def record(a, model):
    """One user-seated pose per invocation, using unique fresh feedback frames."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    cfg = yaml.safe_load(a.config.read_text())
    require(not cfg.get('fixture_only'), 'MOCK_CONFIG_NOT_PHYSICAL')
    signs = vec(cfg['joint_signs'], 6, 'FILL_JOINT_SIGNS')
    require(all(x in (-1, 1) for x in signs), 'JOINT_SIGNS')
    offsets = vec(cfg['joint_offsets'], 6, 'FILL_JOINT_OFFSETS')
    names = ['joint%d' % i for i in range(1, 7)]
    require(a.contact_confirmed, 'SEAT_PROBE_BEFORE_RECORD')
    require(.5 <= a.stable_s <= 5 and .0001 <= a.max_spread <= .01, 'CAPTURE_LIMITS')
    metadata = dict(input_kind='physical_manual_contact', fixture_id=a.fixture_id,
                    urdf_sha256=model.sha256, joint_signs=signs.tolist(), joint_offsets=offsets.tolist())
    dataset = json.loads(a.dataset.read_text()) if a.dataset.exists() else dict(metadata, samples=[])
    require(all(dataset.get(k) == v for k, v in metadata.items()), 'SESSION_METADATA_CHANGED')
    rclpy.init(); node = Node('tcp_calibration_recorder'); history = []; last_stamp = 0.; error = []
    def callback(msg):
        nonlocal last_stamp
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            now = node.get_clock().now().nanoseconds*1e-9
            fresh(stamp, now, .3)
            require(stamp >= last_stamp, 'BACKWARD_FEEDBACK_TIME')
            if stamp == last_stamp: return
            require(len(msg.name) == len(set(msg.name)) and len(msg.position) == len(msg.name), 'FEEDBACK_FORMAT')
            raw = vec([msg.position[msg.name.index(n)] for n in names], 6, 'FEEDBACK_JOINTS')
            q = signs*raw+offsets
            limits = np.asarray(cfg['joint_limits'], float)
            require(limits.shape == (6, 2) and np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]), 'JOINT_LIMIT')
            history.append((stamp, q, raw)); last_stamp = stamp
            history[:] = [s for s in history if stamp-s[0] <= a.stable_s+.15]
        except Exception as e:
            error.append(str(e)); history.clear()
    node.create_subscription(JointState, cfg['feedback_topic'], callback, qos_profile_sensor_data)
    deadline = time.monotonic()+20
    try:
        print('READ ONLY: no motion commands. Keep the same probe tip seated; collecting stable actual joints.')
        accepted = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if not stable_window(history, a.stable_s, a.max_spread): continue
            qs = np.array([s[1] for s in history])
            fresh(history[-1][0], node.get_clock().now().nanoseconds*1e-9, .3)
            accepted = (qs.mean(axis=0), list(history)); break
        require(accepted is not None, 'NO_STABLE_FRESH_FEEDBACK:'+','.join(error[-3:]))
        q, frames = accepted
        T = model.fk('piper_base_link', 'piper_link6', dict(zip(cfg['joint_names'], q)))
        dataset['samples'].append(dict(split=a.split, base_link6_matrix=T.tolist(), q_urdf_rad=q.tolist(),
                                       user_contact_confirmed=True,
                                       feedback=[dict(stamp=s[0], q_driver_rad=s[2].tolist()) for s in frames]))
        # Exclusive sidecar prevents accidental concurrent session updates.
        lock = a.dataset.with_suffix(a.dataset.suffix+'.lock'); lock.parent.mkdir(parents=True, exist_ok=True)
        with lock.open('x'):
            try:
                if a.dataset.exists():
                    current = json.loads(a.dataset.read_text())
                    require(current['samples'] == dataset['samples'][:-1], 'CONCURRENT_SESSION_UPDATE')
                temporary = a.dataset.with_suffix(a.dataset.suffix+'.tmp')
                with temporary.open('x') as f: json.dump(dataset, f, indent=2, allow_nan=False)
                temporary.replace(a.dataset)
            finally: lock.unlink()
        print('Recorded', len(dataset['samples']), a.split, 'pose; this does not verify physical contact automatically.')
    finally:
        node.destroy_node(); rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='command', required=True)
    cad = subs.add_parser('cad'); cad.add_argument('--urdf', type=Path, required=True)
    cad.add_argument('--output', type=Path, required=True); cad.add_argument('--plot', type=Path)
    capture = subs.add_parser('record'); capture.add_argument('--urdf', type=Path, required=True)
    capture.add_argument('--config', type=Path, required=True); capture.add_argument('--dataset', type=Path, required=True)
    capture.add_argument('--fixture-id', required=True); capture.add_argument('--split', choices=['train', 'holdout'], required=True)
    capture.add_argument('--contact-confirmed', action='store_true')
    capture.add_argument('--stable-s', type=float, default=1.); capture.add_argument('--max-spread', type=float, default=.001)
    fit = subs.add_parser('solve'); fit.add_argument('--dataset', type=Path, required=True)
    fit.add_argument('--quaternion', type=float, nargs=4, required=True)
    fit.add_argument('--probe-offset-tcp', type=float, nargs=3, required=True)
    fit.add_argument('--orientation-source', required=True); fit.add_argument('--axes-verified', action='store_true')
    fit.add_argument('--contact-geometry-verified', action='store_true')
    fit.add_argument('--max-error-mm', type=float, default=3.); fit.add_argument('--max-condition', type=float, default=100.)
    fit.add_argument('--output', type=Path, required=True)
    apply = subs.add_parser('apply'); apply.add_argument('--result', type=Path, required=True)
    apply.add_argument('--urdf', type=Path, required=True)
    apply.add_argument('--config', type=Path, required=True); apply.add_argument('--output', type=Path, required=True)
    apply.add_argument('--tcp-verified', action='store_true')
    a = parser.parse_args()
    if a.command == 'cad':
        report, clouds = inspect_cad(Model(a.urdf)); save_new(a.output, report)
        if a.plot:
            require(not a.plot.exists(), 'PLOT_EXISTS'); a.plot.parent.mkdir(parents=True, exist_ok=True)
            plot_cad(report, clouds, a.plot)
        print(json.dumps(report['link6_tcp_transform']))
    elif a.command == 'record': record(a, Model(a.urdf))
    elif a.command == 'solve':
        dataset = json.loads(a.dataset.read_text())
        result = solve(dataset, a.quaternion, a.probe_offset_tcp, a.max_error_mm/1000, a.max_condition)
        result.update(orientation_source=a.orientation_source, axes_verified=a.axes_verified,
                      contact_geometry_verified=a.contact_geometry_verified,
                      dataset_sha256=hashlib.sha256(a.dataset.read_bytes()).hexdigest())
        save_new(a.output, result); print(result['status'], 'holdout max mm:', max(result['holdout_errors_m'])*1000)
        if result['status'] != 'passed': raise SystemExit(2)
    else:
        result = json.loads(a.result.read_text())
        require(result.get('urdf_sha256') == Model(a.urdf).sha256, 'CALIBRATION_URDF_CHANGED')
        cfg = apply_result(yaml.safe_load(a.config.read_text()), result, a.tcp_verified)
        a.output.parent.mkdir(parents=True, exist_ok=True)
        with a.output.open('x') as f: yaml.safe_dump(cfg, f, sort_keys=False)
        print('Written new config; original unchanged. Commissioning flags unchanged:', a.output)


if __name__ == '__main__': main()
