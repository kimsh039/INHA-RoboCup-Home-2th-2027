"""Known-room, three-parameter LiDAR calibration. Does not control Gazebo.

prepare freezes room geometry; record reads captured messages; solve fits train;
validate applies the frozen result. z/roll/pitch are fixed CAD inputs, not fitted.
"""
import argparse
import csv
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
import scipy
from scipy.spatial.transform import Rotation, Slerp
import yaml


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    path = Path(path)
    text = gzip.open(path, 'rt').read() if path.suffix == '.gz' else path.read_text()
    return yaml.safe_load(text) if path.suffix in ('.yaml', '.yml') else json.loads(text)


def save(path, data):
    """Replace generated metadata atomically; never called on sensor inputs."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(data, allow_unicode=True, sort_keys=False) if path.suffix == '.yaml' else json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as f:
        f.write(text)
        temp = Path(f.name)
    temp.replace(path)


def utc_id(prefix):
    return prefix + datetime.now(timezone.utc).strftime('_%Y%m%dT%H%M%S_%fZ')


def stamp(message):
    s = message['header']['stamp']
    ns = int(s.get('nsec', 0))
    if not 0 <= ns < 1_000_000_000:
        raise ValueError('invalid simulation timestamp nanoseconds')
    return int(s.get('sec', 0))*1_000_000_000 + ns


def pose_message(message):
    if message.get('world') != 'robocup_motion' or message.get('reference_frame') != 'world' or message.get('base_frame') != 'base_link' or message.get('base_equals_model_origin') is not True:
        raise ValueError('pose must be world -> base_link from the robocup model origin')
    p = message['pose']
    if p.get('name') != 'robocup':
        raise ValueError('pose/info base_link relative pose is not a world pose; select robocup')
    xyz = np.array([p.get('position', {}).get(k, 0) for k in ('x', 'y', 'z')], float)
    q = np.array([p.get('orientation', {}).get(k, 0) for k in ('x', 'y', 'z', 'w')], float)
    if not np.all(np.isfinite(np.r_[xyz, q])) or np.linalg.norm(q) < 1e-9:
        raise ValueError('invalid position/quaternion')
    return xyz, Rotation.from_quat(q)


def captured_pose(folder, policy):
    before, scan, after = (load(folder/n) for n in ('base_pose_before.json', 'scan.json.gz', 'base_pose_after.json'))
    tb, ts, ta = map(stamp, (before, scan, after))
    pb, rb = pose_message(before)
    pa, ra = pose_message(after)
    delta_m = float(np.linalg.norm(pa-pb))
    delta_deg = float(np.rad2deg((rb.inv()*ra).magnitude()))
    reasons = []
    if not tb <= ts <= ta:
        reasons.append('scan timestamp is not between the two pose timestamps')
    if not 0 <= ta-tb <= policy['max_span_s']*1e9:
        reasons.append('pose time span exceeds policy or simulation clock went backwards')
    if delta_m > policy['max_translation_m']:
        reasons.append('base moved during capture')
    if delta_deg > policy['max_rotation_deg']:
        reasons.append('base rotated during capture')
    if tb == ta:
        p, r = pb, rb
    else:
        alpha = float(np.clip((ts-tb)/(ta-tb), 0, 1))
        p = (1-alpha)*pb + alpha*pa
        r = Slerp([0, 1], Rotation.from_quat([rb.as_quat(), ra.as_quat()]))([alpha])[0]
    check = {'before_sim_ns': tb, 'scan_sim_ns': ts, 'after_sim_ns': ta,
             'scan_is_between_poses': tb <= ts <= ta,
             'pose_span_sim_s': (ta-tb)/1e9,
             'translation_change_mm': delta_m*1000, 'rotation_change_deg': delta_deg,
             'stationary_capture_candidate': not reasons, 'reasons': reasons,
             'policy': policy, 'pose_at_scan': {'position_xyz_m': p.tolist(), 'quaternion_xyzw': r.as_quat().tolist()},
             'pose_method': 'linear translation + quaternion Slerp between recorded poses; stationary captures only'}
    return scan, p, r, check


def sdf_pose(element):
    p = element.find('pose')
    if p is None:
        return np.zeros(6)
    if p.attrib:
        raise ValueError('relative_to or alternative SDF pose format is not supported by this room solver')
    a = np.array([float(v) for v in p.text.split()])
    if a.shape != (6,) or not np.all(np.isfinite(a)):
        raise ValueError('SDF pose must be six finite xyz/rpy values')
    return a


def world_config(path):
    w = ET.parse(path).getroot().find('world')
    if w is None or w.get('name') != 'robocup_motion':
        raise ValueError('expected robocup_motion room world')
    robot = w.find("model[@name='robocup']")
    base = robot.find("link[@name='base_link']") if robot is not None else None
    if base is None or np.linalg.norm(sdf_pose(base)) > 1e-10:
        raise ValueError('this solver requires base_link to coincide with robocup model origin')
    sensor = base.find("sensor[@name='ydlidar_g2']")
    if sensor is None:
        raise ValueError('ydlidar_g2 must be directly attached to base_link')
    sensor_pose = sdf_pose(sensor)
    limits = {}
    for name, axis, side in [('wall_east', 0, -1), ('wall_west', 0, 1), ('wall_north', 1, -1), ('wall_south', 1, 1)]:
        wall = w.find(f"model[@name='{name}']")
        if wall is None or wall.findtext('static') != 'true':
            raise ValueError(f'missing static outer wall: {name}; use room world')
        wp = sdf_pose(wall)
        link = wall.find('link')
        collision = link.find('collision') if link is not None else None
        if collision is None or np.linalg.norm(wp[3:]) > 1e-10 or np.linalg.norm(sdf_pose(link)) > 1e-10 or np.linalg.norm(sdf_pose(collision)) > 1e-10:
            raise ValueError('room walls must be axis-aligned boxes without link/collision pose offsets')
        size = np.array([float(v) for v in collision.findtext('geometry/box/size').split()])
        if size.shape != (3,) or np.any(size <= 0):
            raise ValueError('invalid wall box dimensions')
        limits[name] = float(wp[axis]+side*size[axis]/2)
    if not limits['wall_west'] < limits['wall_east'] or not limits['wall_south'] < limits['wall_north']:
        raise ValueError('invalid room bounds')
    return {'world': w.get('name'), 'model': 'robocup', 'base_frame': 'base_link',
            'lidar_frame': sensor.findtext('gz_frame_id'), 'walls': limits,
            'fixed_z_roll_pitch': sensor_pose[[2, 3, 4]].tolist(),
            'fixed_input_source': 'sensor z/roll/pitch in archived SDF (CAD); not estimated by 2D calibration',
            'ground_truth_reference_only_xyz_rpy': sensor_pose.tolist(),
            'capture_policy': {'max_span_s': .5, 'max_translation_m': .001, 'max_rotation_deg': .02}}


def prepare(args):
    session = args.session.resolve()
    config = world_config(args.world)
    run_id = args.run or utc_id('run')
    if Path(run_id).name != run_id or run_id in ('.', '..'):
        raise ValueError('run id must be a single directory name')
    for name in ('train', 'validation', 'config', 'results'):
        (session/name).mkdir(parents=True, exist_ok=True)
    run = session/'config'/run_id
    run.mkdir()  # Refuse replacing a previous run snapshot.
    shutil.copyfile(args.world, run/'world.sdf')
    config.update({'run_id': run_id, 'world_source': str(args.world.resolve()), 'world_sha256': digest(run/'world.sdf'),
                   'calculator_sha256': digest(__file__),
                   'versions': {'python': sys.version.split()[0], 'numpy': np.__version__, 'scipy': scipy.__version__, 'pyyaml': yaml.__version__}})
    save(run/'run.json', config)
    (session/'config'/'active_run.txt').write_text(run_id+'\n')
    observations = session/'observations.md'
    if not observations.exists():
        observations.write_text('# 내 관찰 메모\n\n화면에서 본 가림·움직임·환경 변경만 자유롭게 적으세요. 수치·파일 목록·계산 결과는 자동 생성됩니다.\n')
    summary(session)
    print(f'Run prepared: {run_id}\nWorld snapshot: {run / "world.sdf"}\nActive run: {session / "config/active_run.txt"}')


def sample_config(session, folder):
    run_id = (folder/'run_id.txt').read_text().strip()
    if not run_id or Path(run_id).name != run_id or run_id in ('.', '..'):
        raise ValueError('invalid run_id.txt')
    config = load(session/'config'/run_id/'run.json')
    if digest(session/'config'/run_id/'world.sdf') != config['world_sha256']:
        raise ValueError('archived world has been modified')
    return config


def record(args):
    session, folder = args.session.resolve(), args.sample.resolve()
    rel = folder.relative_to(session)
    if len(rel.parts) != 2 or rel.parts[0] not in ('train', 'validation'):
        raise ValueError('sample must be SESSION/train/ID or SESSION/validation/ID')
    config = sample_config(session, folder)
    scan, _, _, check = captured_pose(folder, config['capture_policy'])
    points(scan, config)
    save(folder/'capture_check.json', check)
    note = {'sample_id': rel.parts[1], 'split': rel.parts[0], 'run_id': config['run_id'],
            'scan_sim_ns': stamp(scan), 'frame': scan['frame'], 'range_count': len(scan['ranges']),
            'pose_source': '/world/robocup_motion/pose/info : robocup',
            'capture_eligible': check['stationary_capture_candidate'], 'capture_reasons': check['reasons'],
            'requested_pose': load(folder/'requested_pose.json') if (folder/'requested_pose.json').exists() else None,
            'input_sha256': {n: digest(folder/n) for n in ('scan.json.gz', 'base_pose_before.json', 'base_pose_after.json', 'run_id.txt')}}
    save(folder/'sample.json', note)
    save(folder/'scan_header.json', {k: v for k, v in scan.items() if k not in ('ranges', 'intensities')})
    observation = folder/'observations.md'
    if not observation.exists():
        observation.write_text('# 내 관찰 메모\n\n필요할 때만 적으세요. 가림·이동 중 촬영·특이사항이 없다면 비워 두어도 됩니다.\n')
    summary(session)
    print(f'Record generated: {folder}\nCapture eligible: {note["capture_eligible"]}\nReasons: {check["reasons"]}')
    if not note['capture_eligible']:
        raise ValueError('capture is not eligible; keep this record and recapture into a new sample directory')


def summary(session):
    records = []
    for split in ('train', 'validation'):
        for path in sorted((session/split).glob('*/sample.json')):
            row = load(path)
            row['sample_path'] = str(path.parent.relative_to(session))
            records.append(row)
    counts = {s: sum(r['split'] == s and r['capture_eligible'] for r in records) for s in ('train', 'validation')}
    status = 'recorded_not_calibrated' if records else 'prepared_not_collected'
    latest = session/'results/latest_calibration.txt'
    if latest.exists():
        status = 'computed_validation_pending'
        validation = session/'results/latest_validation.txt'
        if validation.exists():
            v = load(Path(validation.read_text().strip()).parent/'validation.json')
            if v['calibration_sha256'] == digest(Path(latest.read_text().strip())):
                status = 'computed_' + v['status']
    save(session/'session.yaml', {'status': status,
                                'session': str(session), 'eligible_counts': counts, 'samples': records})
    text = ['# 자동 생성 실험 기록', '', f'상태: {status}', '',
            '측정 수치와 목록은 자동 생성됩니다. 내 관찰은 observations.md에 적습니다.', '',
            f'Train 후보 {counts["train"]} / validation 후보 {counts["validation"]}', '',
            '| 자료 | run | scan 시각 (ns) | 수집 조건 충족 |', '|---|---|---:|---|']
    for r in records:
        text.append(f'| {r["sample_path"]} | {r["run_id"]} | {r["scan_sim_ns"]} | {r["capture_eligible"]} |')
    for label, name in [('캘리브레이션', 'latest_calibration.txt'), ('검증', 'latest_validation.txt')]:
        path = session/'results'/name
        if path.exists():
            text += ['', f'{label} 결과: `{path.read_text().strip()}`']
    (session/'session_summary.md').write_text('\n'.join(text)+'\n')


def points(scan, config):
    if scan.get('frame') != config['lidar_frame'] or int(scan.get('verticalCount', 0)) != 1:
        raise ValueError('expected a single-layer laser_frame scan')
    n = int(scan['count'])
    ranges = np.array(scan['ranges'], float)
    if ranges.shape != (n,) or n < 50:
        raise ValueError('range count mismatch or insufficient rays')
    angle_min, angle_max, step, vertical = map(float, (scan['angleMin'], scan['angleMax'], scan['angleStep'], scan['verticalAngleMin']))
    if not np.all(np.isfinite([angle_min, angle_max, step, vertical])) or step <= 0 or abs(angle_min+(n-1)*step-angle_max) > 1e-7:
        raise ValueError('invalid scan angle metadata')
    valid = np.isfinite(ranges) & (ranges >= float(scan['rangeMin'])) & (ranges < float(scan['rangeMax']))
    if valid.sum() < 50:
        raise ValueError('insufficient finite scan returns')
    angles = angle_min+np.arange(n)[valid]*step
    r = ranges[valid]
    return np.column_stack((r*np.cos(vertical)*np.cos(angles), r*np.cos(vertical)*np.sin(angles), r*np.sin(vertical)))


def samples(session, split, excluded=()):
    result = []
    for folder in sorted((session/split).iterdir()):
        if not folder.is_dir() or str(folder.relative_to(session)) in excluded:
            continue
        config = sample_config(session, folder)
        scan, p, r, check = captured_pose(folder, config['capture_policy'])
        if not check['stationary_capture_candidate']:
            raise ValueError(f'{folder}: capture policy failed; exclude explicitly or recapture')
        note = load(folder/'sample.json')
        for name, sha in note['input_sha256'].items():
            if digest(folder/name) != sha:
                raise ValueError(f'{folder}: recorded input changed: {name}; inspect and regenerate record')
        result.append({'folder': folder, 'config': config, 'scan': scan, 'p': p, 'r': r,
                       'points': points(scan, config), 'check': check, 'scan_sha256': digest(folder/'scan.json.gz'),
                       'input_sha256': {n: digest(folder/n) for n in ('scan.json.gz', 'base_pose_before.json', 'base_pose_after.json', 'run_id.txt')}})
    if not result:
        raise ValueError(f'no usable {split} samples with recorded base poses')
    first = result[0]['config']
    for row in result:
        for key in ('walls', 'fixed_z_roll_pitch', 'lidar_frame', 'base_frame'):
            if row['config'][key] != first[key]:
                raise ValueError('room or fixed sensor geometry changed between runs; start a separate session')
    return result


def transform(row, x):
    z, roll, pitch = row['config']['fixed_z_roll_pitch']
    # R = Rz(yaw) Ry(fixed_pitch) Rx(fixed_roll).
    sensor_r = Rotation.from_euler('xyz', [roll, pitch, x[2]])
    base_points = np.einsum('ij,nj->ni', sensor_r.as_matrix(), row['points'])+np.array([x[0], x[1], z])
    return np.einsum('ij,nj->ni', row['r'].as_matrix(), base_points)+row['p']


def residual(row, x):
    q = transform(row, x)
    w = row['config']['walls']
    distances = np.column_stack((q[:, 0]-w['wall_east'], q[:, 0]-w['wall_west'], q[:, 1]-w['wall_north'], q[:, 1]-w['wall_south']))
    labels = np.argmin(abs(distances), axis=1)
    return distances[np.arange(len(q)), labels], labels


def metrics(row, x, gate):
    e, labels = residual(row, x)
    mask = abs(e) < gate
    per_wall = {}
    for i, name in enumerate(('east', 'west', 'north', 'south')):
        ee = e[mask & (labels == i)]
        per_wall[name] = {'points': len(ee), 'rms_mm': float(np.sqrt(np.mean(ee**2))*1000) if len(ee) else None}
    wall_error = e[mask]
    return {'sample': str(row['folder']), 'scan_sha256': row['scan_sha256'], 'input_sha256': row['input_sha256'],
            'scan_sim_ns': stamp(row['scan']), 'run_id': row['config']['run_id'], 'world_sha256': row['config']['world_sha256'],
            'base_pose_at_scan': row['check']['pose_at_scan'], 'valid_returns': len(e), 'wall_points': int(mask.sum()),
            'wall_fraction': float(mask.mean()), 'excluded_points': int((~mask).sum()),
            'wall_rms_mm': float(np.sqrt(np.mean(wall_error**2))*1000) if len(wall_error) else None,
            'wall_p95_abs_mm': float(np.percentile(abs(wall_error), 95)*1000) if len(wall_error) else None,
            'all_returns_median_nearest_wall_abs_mm': float(np.median(abs(e))*1000), 'per_wall': per_wall}


def coverage(m):
    return m['wall_fraction'] >= .6 and all(v['points'] >= 10 for v in m['per_wall'].values())


def solve(args):
    session = args.session.resolve()
    rows = samples(session, 'train', args.exclude)
    initial = np.array([0, 0, np.deg2rad(args.initial_yaw_deg)])
    bound = np.array([.5, .5, np.deg2rad(40)])
    def all_residual(x):
        return np.concatenate([residual(row, x)[0] for row in rows])
    candidates = []
    for offset in (-25, 0, 25):
        seed = initial+np.array([0, 0, np.deg2rad(offset)])
        fit = least_squares(all_residual, seed, bounds=(initial-bound, initial+bound), loss='soft_l1', f_scale=.02, x_scale=[.1, .1, .1])
        masks = []
        for _ in range(6):
            masks = [abs(residual(row, fit.x)[0]) < args.wall_gate_m for row in rows]
            if any(mask.sum() < 50 for mask in masks):
                break
            fit = least_squares(lambda x: np.concatenate([residual(row, x)[0][mask] for row, mask in zip(rows, masks)]), fit.x,
                                bounds=(initial-bound, initial+bound), x_scale=[.1, .1, .1])
        mm = [metrics(row, fit.x, args.wall_gate_m) for row in rows]
        if fit.success and all(coverage(m) for m in mm):
            candidates.append((sum(m['wall_rms_mm'] for m in mm), fit, mm))
    if not candidates:
        raise ValueError('no fit with sufficient support on all four walls; check room, scan, pose or heading prior')
    _, fit, mm = min(candidates, key=lambda x: x[0])
    singular = np.linalg.svd(fit.jac, compute_uv=False)
    if len(singular) < 3 or singular[-1] < 1e-6 or singular[0]/singular[-1] > 1e6:
        raise ValueError('training geometry is not observable for x/y/yaw')
    if np.any(abs(fit.x-initial) > bound*.98):
        raise ValueError('fit is on the prior boundary; check heading correspondence')
    config = rows[0]['config']
    z, roll, pitch = config['fixed_z_roll_pitch']
    rotation = Rotation.from_euler('xyz', [roll, pitch, fit.x[2]])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation.as_matrix()
    matrix[:3, 3] = [fit.x[0], fit.x[1], z]
    result = {'schema': 'base_lidar_known_room/v1', 'status': 'computed_validation_pending',
              'parent_frame': config['base_frame'], 'child_frame': config['lidar_frame'], 'transform_direction': 'base_link <- laser_frame',
              'estimated_parameters': ['x_m', 'y_m', 'yaw_rad'],
              'estimated_xy_yaw': fit.x.tolist(), 'yaw_deg': float(np.rad2deg(fit.x[2])),
              'translation_m': matrix[:3, 3].tolist(), 'quaternion_xyzw': rotation.as_quat().tolist(), 'matrix_4x4': matrix.tolist(),
              'fixed_z_roll_pitch': config['fixed_z_roll_pitch'], 'fixed_input_source': config['fixed_input_source'],
              'room_walls': config['walls'], 'heading_prior_deg': args.initial_yaw_deg, 'heading_search_halfwidth_deg': 40,
              'wall_gate_m': args.wall_gate_m, 'training_samples': mm,
              'method': 'robust known-wall least squares, train only; actual base xyz/quaternion interpolated at scan stamp; fixed CAD z/roll/pitch',
              'jacobian_rank': 3, 'jacobian_condition': float(singular[0]/singular[-1]),
              'limitations': ['z/roll/pitch are not calibrated', 'pose pairs are stationary samples, not a hardware time-sync guarantee',
                              'square room symmetry is resolved with a heading prior within +/-40 degrees',
                              'wall residual metrics exclude nonwall/outlier returns; support counts are reported',
                              'known walls must remain at archived positions; do not move walls in GUI']}
    gt = np.array(config['ground_truth_reference_only_xyz_rpy'])
    yaw_difference = np.arctan2(np.sin(fit.x[2]-gt[5]), np.cos(fit.x[2]-gt[5]))
    result['ground_truth_comparison'] = {
        'source': 'archived SDF sensor pose; used for comparison after fitting only',
        'reference_xy_yaw': [float(gt[0]), float(gt[1]), float(gt[5])],
        'xy_error_mm': float(np.linalg.norm(fit.x[:2]-gt[:2])*1000),
        'yaw_error_deg': float(np.rad2deg(yaw_difference))}
    out = session/'results'/utc_id('calibration')
    out.mkdir()
    result['calibration_id'] = out.name
    path = out/'base_2dlidar_mount.yaml'
    save(path, result)
    save(out/'calibration.json', result)
    np.savetxt(out/'T_base_lidar.csv', matrix, delimiter=',', fmt='%.12g')
    table_report(out/'calibration_report.md', 'train으로 계산한 Base–LiDAR', result, mm)
    (session/'results/latest_calibration.txt').write_text(str(path)+'\n')
    summary(session)
    print(f'CALIBRATION: {path}\nx={fit.x[0]*1000:.3f} mm, y={fit.x[1]*1000:.3f} mm, yaw={np.rad2deg(fit.x[2]):.6f} deg\nValidation is pending.')


def table_report(path, title, result, rows):
    text = [f'# {title}', '', f'상태: {result["status"]}', '',
            '결과 방향: base_link ← laser_frame. 거리 m, 회전 quaternion xyzw.', '']
    if 'estimated_xy_yaw' in result:
        x = result['estimated_xy_yaw']
        text += [f'추정 결과: **x={x[0]*1000:.3f} mm, y={x[1]*1000:.3f} mm, yaw={result["yaw_deg"]:.6f}°**', '',
                 f'현재 SDF 기준값과 비교: 평면 위치 오차 {result["ground_truth_comparison"]["xy_error_mm"]:.3f} mm, yaw 오차 {result["ground_truth_comparison"]["yaw_error_deg"]:.6f}°.', '',
                 '다음 작업: 별도 validation 자세에서 scan과 실제 base pose를 저장하고 validate 명령을 실행합니다.', '']
    text += ['| 자료 | 외벽 점 / 유효 거리 | 외벽 RMS (mm) | 절대오차 95백분위 (mm) | 판정 |', '|---|---:|---:|---:|---|']
    for m in rows:
        decision = ('통과' if m['passed'] else '; '.join(m['reasons'])) if 'passed' in m else 'train fitting'
        text.append(f'| {m["sample"]} | {m["wall_points"]}/{m["valid_returns"]} | {m["wall_rms_mm"]} | {m["wall_p95_abs_mm"]} | {decision} |')
    text += ['', 'z/roll/pitch는 archived SDF에서 고정한 입력이며 추정한 값이 아닙니다.',
             '외벽 외 점(책상·로봇 자체·칸막이 등)은 gate로 제외합니다. 전체 점의 오차가 아닙니다.',
             'validation에서는 calibration 변환을 재추정하지 않습니다.', '', '세부 수치·입력 SHA-256·실제 pose·run·판정 사유는 같은 폴더의 JSON/YAML에 있습니다.']
    if 'limits' in result:
        text += ['', '튜토리얼의 임시 검증 기준: '+json.dumps(result['limits'], ensure_ascii=False),
                 'passed는 아래 기록된 scan과 임시 기준에 한정한 판단입니다. 전체 센서 보정 완료를 뜻하지 않습니다.']
    path.write_text('\n'.join(text)+'\n')


def validate(args):
    session = args.session.resolve()
    path = args.calibration
    if path is None:
        path = Path((session/'results/latest_calibration.txt').read_text().strip())
    result = load(path)
    if result.get('schema') != 'base_lidar_known_room/v1':
        raise ValueError('unsupported calibration schema')
    rows = samples(session, 'validation', args.exclude)
    training_hashes = {m['scan_sha256'] for m in result['training_samples']}
    x = np.array(result['estimated_xy_yaw'], float)
    checks = []
    for row in rows:
        if row['scan_sha256'] in training_hashes:
            raise ValueError('validation scan duplicates a training input')
        if row['config']['walls'] != result['room_walls'] or row['config']['fixed_z_roll_pitch'] != result['fixed_z_roll_pitch']:
            raise ValueError('validation room or fixed geometry differs from calibration')
        m = metrics(row, x, result['wall_gate_m'])
        reasons = []
        if not coverage(m):
            reasons.append('insufficient outer-wall support')
        if m['wall_rms_mm'] is None or m['wall_rms_mm'] > args.max_rms_mm:
            reasons.append('outer-wall RMS exceeds limit')
        if m['wall_p95_abs_mm'] is None or m['wall_p95_abs_mm'] > args.max_p95_mm:
            reasons.append('outer-wall P95 exceeds limit')
        pose = row['check']['pose_at_scan']
        separated = all(np.linalg.norm(np.array(pose['position_xyz_m'])-t['base_pose_at_scan']['position_xyz_m']) >= .1 or
                        np.rad2deg((Rotation.from_quat(t['base_pose_at_scan']['quaternion_xyzw']).inv()*row['r']).magnitude()) >= 5
                        for t in result['training_samples'])
        if not separated:
            reasons.append('validation pose is too close to a training pose (<0.1m and <5deg)')
        m.update({'passed': not reasons, 'reasons': reasons})
        checks.append(m)
    report = {'status': 'passed_provisional_limits' if all(m['passed'] for m in checks) else 'failed_provisional_limits',
              'calibration_file': str(Path(path).resolve()), 'calibration_sha256': digest(path),
              'calibration_was_refitted': False, 'limits': {'max_wall_rms_mm': args.max_rms_mm, 'max_wall_p95_mm': args.max_p95_mm,
              'min_wall_fraction': .6, 'min_points_each_wall': 10, 'wall_gate_m': result['wall_gate_m']}, 'samples': checks}
    out = session/'results'/utc_id('validation')
    out.mkdir()
    save(out/'validation.json', report)
    table_report(out/'validation_report.md', '고정값으로 검증', report, checks)
    with (out/'residuals.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['sample', 'valid_return_index', 'world_x_m', 'world_y_m', 'world_z_m', 'nearest_wall', 'signed_error_m', 'wall_gate_selected'])
        for row in rows:
            q = transform(row, x)
            e, labels = residual(row, x)
            for i in range(len(e)):
                writer.writerow([str(row['folder']), i, *q[i], ('east', 'west', 'north', 'south')[labels[i]], e[i], abs(e[i]) < result['wall_gate_m']])
    (session/'results/latest_validation.txt').write_text(str(out/'validation_report.md')+'\n')
    summary(session)
    print(f'VALIDATION: {report["status"]}\nReport: {out / "validation_report.md"}')
    return 0 if all(m['passed'] for m in checks) else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for command in ('prepare', 'record', 'solve', 'validate'):
        p = sub.add_parser(command)
        p.add_argument('--session', type=Path, required=True)
        if command == 'prepare':
            p.add_argument('--world', type=Path, required=True)
            p.add_argument('--run')
        elif command == 'record':
            p.add_argument('--sample', type=Path, required=True)
        else:
            p.add_argument('--exclude', action='append', default=[], help='relative sample path, e.g. train/002')
            if command == 'solve':
                p.add_argument('--initial-yaw-deg', type=float, default=0)
                p.add_argument('--wall-gate-m', type=float, default=.05)
            else:
                p.add_argument('--calibration', type=Path)
                p.add_argument('--max-rms-mm', type=float, default=15)
                p.add_argument('--max-p95-mm', type=float, default=25)
    args = parser.parse_args()
    for name in ('wall_gate_m', 'max_rms_mm', 'max_p95_mm'):
        value = getattr(args, name, 1.)
        if not np.isfinite(value) or value <= 0:
            parser.error(f'{name} must be finite and positive')
    if not np.isfinite(getattr(args, 'initial_yaw_deg', 0.)):
        parser.error('initial yaw must be finite')
    try:
        status = globals()[args.command](args)
    except (ValueError, KeyError, OSError, TypeError, ET.ParseError) as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 2
    return status or 0


if __name__ == '__main__':
    raise SystemExit(main())
