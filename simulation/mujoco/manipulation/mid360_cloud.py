"""MID-360 geometric dummy scan of the actual compiled MuJoCo scene.

Uses the URDF livox_frame, its roll, FOV/range and first-hit scene occlusion.
Dense ROI angular rays are a geometric surrogate, not Livox's scan pattern.
No simulation stepping, object motion, or robot geometry changes occur here.
"""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT.parent/'robot_description/robocup.urdf'  # 팀 공용 URDF와 HW 메시를 직접 참조한다.

# A fixed axis adapter for the RGB-D-trained baseline, not a new viewpoint.
# Lidar X forward/Y left/Z up -> network X right/Y down/Z forward.
T_NETWORK_LIDAR = np.array([[0., -1., 0., 0.], [0., 0., -1., 0.],
                            [1., 0., 0., 0.], [0., 0., 0., 1.]])


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def scan_scene_hash(path, legacy=False):
    # 접촉 해법은 정지 장면의 광선/점군을 바꾸지 않는다. 두 설정만 해시에서 제외한다.
    # 로봇/물체 위치, 센서 관측 설정 및 나머지 설정 변경은 계속 검사한다.
    cfg=json.loads(Path(path).read_text())
    for key in ('contact_multiccd','contact_noslip_iterations'):cfg.pop(key,None)
    text=(json.dumps(cfg,indent=2)+'\n') if legacy else json.dumps(cfg,sort_keys=True,separators=(',',':'))
    return hashlib.sha256(text.encode()).hexdigest()


def apply_transform(T, points):
    return np.asarray(points)@T[:3, :3].T+T[:3, 3]


def sensor_spec():
    root = ET.parse(URDF).getroot()
    sensor = root.find('.//sensor[@name="livox_mid360s"]')
    if sensor is None or sensor.findtext('gz_frame_id') != 'livox_frame':
        raise ValueError('MID360_SENSOR_FRAME_MISSING')
    return {
        'frame': 'livox_frame', 'urdf_sensor_name': sensor.get('name'),
        'horizontal_min_rad': float(sensor.findtext('lidar/scan/horizontal/min_angle')),
        'horizontal_max_rad': float(sensor.findtext('lidar/scan/horizontal/max_angle')),
        'vertical_min_rad': float(sensor.findtext('lidar/scan/vertical/min_angle')),
        'vertical_max_rad': float(sensor.findtext('lidar/scan/vertical/max_angle')),
        'range_min_m': float(sensor.findtext('lidar/range/min')),
        'range_max_m': float(sensor.findtext('lidar/range/max')),
        'urdf_horizontal_samples': int(sensor.findtext('lidar/scan/horizontal/samples')),
        'urdf_vertical_samples': int(sensor.findtext('lidar/scan/vertical/samples')),
    }


def in_fov(points_lidar, spec):
    p = np.asarray(points_lidar)
    distance = np.linalg.norm(p, axis=1)
    azimuth = np.arctan2(p[:, 1], p[:, 0])
    elevation = np.arctan2(p[:, 2], np.linalg.norm(p[:, :2], axis=1))
    return ((distance >= spec['range_min_m']) & (distance <= spec['range_max_m'])
            & (azimuth >= spec['horizontal_min_rad']) & (azimuth <= spec['horizontal_max_rad'])
            & (elevation >= spec['vertical_min_rad']) & (elevation <= spec['vertical_max_rad']))


def scan_cube(model, data, cfg, resolution=192):
    import mujoco
    from itertools import product
    from manipulation.sim_model import body_transform
    if resolution < 8:
        raise ValueError('ROI ray resolution must be >=8')
    spec = sensor_spec()
    T_world_lidar = body_transform(model, data, spec['frame'])
    T_world_object = body_transform(model, data, 'cube')
    T_world_base = body_transform(model, data, 'base_link')
    T_lidar_object = np.linalg.inv(T_world_lidar)@T_world_object
    half = model.geom('cube_geom').size.copy()
    corners = apply_transform(T_lidar_object, np.array(list(product((-1., 1.), repeat=3)))*half)
    azimuth = np.arctan2(corners[:, 1], corners[:, 0])
    elevation = np.arctan2(corners[:, 2], np.linalg.norm(corners[:, :2], axis=1))
    if np.ptp(azimuth) > np.pi:
        raise ValueError('Cube ROI crosses azimuth wrap; use a different scene snapshot')
    az = np.linspace(azimuth.min(), azimuth.max(), resolution)
    el = np.linspace(elevation.min(), elevation.max(), resolution)
    aa, ee = np.meshgrid(az, el)
    directions = np.column_stack((np.cos(ee).ravel()*np.cos(aa).ravel(),
                                  np.cos(ee).ravel()*np.sin(aa).ravel(), np.sin(ee).ravel()))
    angular_valid = ((aa.ravel() >= spec['horizontal_min_rad']) & (aa.ravel() <= spec['horizontal_max_rad'])
                     & (ee.ravel() >= spec['vertical_min_rad']) & (ee.ravel() <= spec['vertical_max_rad']))
    # Actual collision groups: 0 scene, 3 robot; group 2 visual duplicates are excluded.
    groups = np.array([1, 0, 0, 1, 0, 0], dtype=np.uint8)
    cube_id = model.geom('cube_geom').id
    # The sensor casing is not an external occluder of its own laser.
    exclude = model.body('mid360_link').id
    points, distances, first_hits = [], [], {}
    geom = np.array([-1], dtype=np.int32)
    for direction in directions[angular_valid]:
        world_direction = T_world_lidar[:3, :3]@direction
        distance = mujoco.mj_ray(model, data, T_world_lidar[:3, 3], world_direction,
                                 groups, True, exclude, geom)
        name = model.geom(int(geom[0])).name if geom[0] >= 0 else 'no_return'
        first_hits[name] = first_hits.get(name, 0)+1
        if geom[0] == cube_id and spec['range_min_m'] <= distance <= spec['range_max_m']:
            points.append(direction*distance);distances.append(distance)
    if not points:
        raise ValueError(f'NO_MID360_CUBE_RETURN: FOV or scene occlusion; first hits={first_hits}')
    lidar = np.asarray(points, dtype=np.float64)
    world = apply_transform(T_world_lidar, lidar)
    object_points = apply_transform(np.linalg.inv(T_world_object), world)
    network = apply_transform(T_NETWORK_LIDAR, lidar)
    T_network_object = T_NETWORK_LIDAR@T_lidar_object
    meta = {
        'pipeline': 'mid360_mujoco_roi_v1', 'object': 'cube',
        'dimensions': {'dimensions_m': (2*half).tolist()}, 'source_frame': 'object', 'units': 'meters',
        'origin': 'geometric center', 'object_axes': 'MuJoCo cube local axes; initial Z up',
        'num_points': len(lidar), 'shape': [len(lidar), 3], 'dtype': 'float32',
        'sensor': spec, 'sensor_frame': 'livox_frame', 'network_frame': 'graspnet_sensor',
        'network_axes': 'X=-Y_lidar, Y=-Z_lidar, Z=+X_lidar; fixed rotation only',
        'view_policy': 'URDF-mounted MID360; FOV/range and first-hit scene occlusion',
        'sampling': 'dense angular ROI rays; geometric dummy scan, not hardware nonrepetitive pattern',
        'noise_std_m': 0.0, 'scan_time_s': None, 'hardware_scan_pattern_reproduced': False,
        'T_world_lidar': T_world_lidar.tolist(), 'T_world_object': T_world_object.tolist(),
        'T_world_base': T_world_base.tolist(), 'T_lidar_object': T_lidar_object.tolist(),
        'T_network_lidar': T_NETWORK_LIDAR.tolist(), 'T_network_object': T_network_object.tolist(),
        'roi_resolution': resolution, 'roi_rays_total': len(directions),
        'roi_rays_in_fov': int(angular_valid.sum()), 'first_hit_counts': first_hits,
        'range_bounds_m': [float(min(distances)), float(max(distances))],
        'occlusion_geometry': 'MuJoCo collision geoms groups 0 and 3; own mid360 casing excluded',
        'joint_names': [f'joint{i}' for i in range(1, 9)],
        'joint_positions_rad_or_m': [float(data.qpos[model.joint(f'joint{i}').qposadr[0]]) for i in range(1, 9)],
        'scene_config_sha256': sha256(ROOT/'config/grasp_simulation.json'),
        'scan_scene_sha256': scan_scene_hash(ROOT/'config/grasp_simulation.json'),
        'source_urdf_sha256': sha256(URDF),
        'cube_config_sha256': sha256(ROOT/'config/pointclouds.json'),
    }
    return object_points.astype(np.float32), lidar.astype(np.float32), network.astype(np.float32), meta


def validate_scene_metadata(meta):
    if meta.get('pipeline') not in ('mid360_mujoco_roi_v1', 'wrist_camera_mujoco_roi_v1'):
        return
    for key, path in [('scene_config_sha256', ROOT/'config/grasp_simulation.json'),
                      ('source_urdf_sha256', URDF), ('cube_config_sha256', ROOT/'config/pointclouds.json')]:
        if meta.get(key) != sha256(path):
            if key=='scene_config_sha256':
                # 신규 점군은 정규화 해시를, 기존 점군은 변경 전 파일 해시를 확인한다.
                if meta.get('scan_scene_sha256')==scan_scene_hash(path):continue
                if 'scan_scene_sha256' not in meta and meta.get(key)==scan_scene_hash(path,legacy=True):continue
            raise ValueError(f'MID360_SCENE_CHANGED: {key}; regenerate scan and rerun Colab')


def validate_scene_status(meta, status, tolerance_m=.005, tolerance_rad=.02):
    if meta.get('pipeline') not in ('mid360_mujoco_roi_v1', 'wrist_camera_mujoco_roi_v1'):
        return
    for key in ['T_world_base', 'T_world_object']:
        observed = np.asarray(status[key], dtype=float);expected = np.asarray(meta[key], dtype=float)
        if (np.linalg.norm(observed[:3, 3]-expected[:3, 3]) > tolerance_m
                or np.linalg.norm(observed[:3, :3]-expected[:3, :3]) > tolerance_rad):
            raise ValueError(f'MID360_SCENE_POSE_MISMATCH: {key}; restore captured scene or regenerate scan')
