"""Pinhole depth surrogate at the original URDF wrist optical frame."""
from itertools import product
import xml.etree.ElementTree as ET
import numpy as np
from manipulation.mid360_cloud import URDF, ROOT, sha256, scan_scene_hash, apply_transform
from manipulation.sim_model import body_transform


def scan_cube(model, data, cfg, resolution=192):
    import mujoco
    sensor = ET.parse(URDF).getroot().find('.//sensor[@name="wrist_d435f_depth"]')
    width = int(sensor.findtext('camera/image/width'))
    height = int(sensor.findtext('camera/image/height'))
    hfov = float(sensor.findtext('camera/horizontal_fov'))
    near = float(sensor.findtext('camera/clip/near'))
    far = float(sensor.findtext('camera/clip/far'))
    fx = width/(2*np.tan(hfov/2)); fy = fx
    cx, cy = (width-1)/2, (height-1)/2
    T = body_transform(model, data, 'wrist_camera_optical_frame')
    O = body_transform(model, data, 'cube')
    C = np.linalg.inv(T)@O
    corners = apply_transform(C, np.array(list(product((-1.,1.),repeat=3)))*model.geom('cube_geom').size)
    if np.any(corners[:,2] <= 0):
        raise ValueError('WRIST_CUBE_BEHIND_CAMERA')
    uv = corners[:,:2]/corners[:,2,None]*[fx,fy]+[cx,cy]
    lo = np.maximum(uv.min(0),[0,0]); hi = np.minimum(uv.max(0),[width-1,height-1])
    if np.any(hi <= lo):
        raise ValueError('WRIST_CUBE_OUTSIDE_FOV: use a real arm observation pose')
    if resolution < 8: raise ValueError('resolution must be >=8')
    u,v = np.meshgrid(np.linspace(lo[0],hi[0],resolution),np.linspace(lo[1],hi[1],resolution))
    directions = np.column_stack(((u.ravel()-cx)/fx,(v.ravel()-cy)/fy,np.ones(u.size)))
    directions /= np.linalg.norm(directions,axis=1)[:,None]
    groups = np.array([1,0,0,1,0,0],dtype=np.uint8)
    geom = np.array([-1],dtype=np.int32); hits = {}; points = []
    for a in directions:
        distance = mujoco.mj_ray(model,data,T[:3,3],T[:3,:3]@a,groups,True,
                                model.body('wrist_camera_cad_link').id,geom)
        name = model.geom(int(geom[0])).name if geom[0]>=0 else 'no_return'
        hits[name] = hits.get(name,0)+1
        # Depth camera clipping uses optical Z, not Euclidean range.
        if geom[0] == model.geom('cube_geom').id and near <= distance*a[2] <= far:
            points.append(a*distance)
    if not points: raise ValueError(f'NO_WRIST_CUBE_RETURN: {hits}')
    camera = np.asarray(points); obj = apply_transform(np.linalg.inv(C),camera)
    meta = dict(pipeline='wrist_camera_mujoco_roi_v1',object='cube',units='meters',source_frame='object',
        origin='geometric center', dimensions={'dimensions_m':(2*model.geom('cube_geom').size).tolist()},
        num_points=len(obj),shape=[len(obj),3],dtype='float32',sensor_frame='wrist_camera_optical_frame',
        network_frame='wrist_camera_optical_frame',network_axes='X right, Y down, Z forward; identity adapter',
        T_world_camera=T.tolist(),T_camera_object=C.tolist(),T_network_object=C.tolist(),
        T_network_camera=np.eye(4).tolist(),T_world_object=O.tolist(),
        T_world_base=body_transform(model,data,'base_link').tolist(),
        sensor={'urdf_sensor_name':'wrist_d435f_depth','width':width,'height':height,'fx':fx,'fy':fy,
                'cx':cx,'cy':cy,'horizontal_fov_rad':hfov,'near_m':near,'far_m':far},
        view_policy='URDF-mounted wrist camera; pinhole FOV/depth and first-hit scene occlusion',
        sampling='dense subpixel ROI rays; geometric depth surrogate, not hardware image',
        roi_rays_total=len(directions),first_hit_counts=hits,noise_std_m=0.,hardware_scan_pattern_reproduced=False,
        joint_positions_rad_or_m=[float(data.qpos[model.joint(f'joint{i}').qposadr[0]]) for i in range(1,9)],
        scene_config_sha256=sha256(ROOT/'config/grasp_simulation.json'),source_urdf_sha256=sha256(URDF),
        scan_scene_sha256=scan_scene_hash(ROOT/'config/grasp_simulation.json'),
        cube_config_sha256=sha256(ROOT/'config/pointclouds.json'),
        base_to_object_horizontal_distance_m=float(np.linalg.norm((O[:3,3]-body_transform(model,data,'base_link')[:3,3])[:2])))
    return obj.astype(np.float32),camera.astype(np.float32),camera.astype(np.float32),meta
