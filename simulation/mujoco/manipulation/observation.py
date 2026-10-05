"""Compute a wrist-camera top-down observation from the known object center."""
import numpy as np
from scipy.optimize import least_squares
from manipulation.sim_model import body_transform


def top_down_pose(model, data, height=.30):
    import mujoco
    if height < .22: raise ValueError('Observation height must leave depth near-clip clearance')
    original = data.qpos.copy()
    adr = [int(model.joint(f'joint{i}').qposadr[0]) for i in range(1,7)]
    bounds = np.array([model.joint(f'joint{i}').range for i in range(1,7)])
    center = body_transform(model,data,'cube')[:3,3].copy()
    target = center + [0,0,height]
    def residual(q):
        data.qpos[adr] = q; mujoco.mj_forward(model,data)
        T = body_transform(model,data,'wrist_camera_optical_frame')
        # Image roll is free. Fix camera position and optical viewing axis only.
        return np.r_[5*(T[:3,3]-target),T[:3,2]-[0,0,-1]]
    try:
        # Match the original physical seeds in the shared URDF's new joint 1 zero.
        for seed in ([-1.8,1.6,-.3,0,0,0],[-1.8,2.,-1.,0,.6,0]):
            result = least_squares(residual,np.clip(seed,bounds[:,0]+1e-6,bounds[:,1]-1e-6),
                                   bounds=(bounds[:,0],bounds[:,1]),max_nfev=600)
            residual(result.x)
            T = body_transform(model,data,'wrist_camera_optical_frame')
            if np.linalg.norm(T[:3,3]-target) > .002 or np.linalg.norm(T[:3,2]-[0,0,-1]) > .01:
                continue
            contacts = [c for c in data.contact if c.dist < -.001]
            if contacts: continue
            return {'joint_positions_rad':result.x.tolist(),'T_world_camera':T.tolist(),
                    'object_center_world_m':center.tolist(),'camera_height_above_center_m':height,
                    'position_error_m':float(np.linalg.norm(T[:3,3]-target)),
                    'static_collision_check':'no penetration deeper than 1mm',
                    'motion_path_checked':False}
        raise RuntimeError('NO_FEASIBLE_TOP_DOWN_OBSERVATION: position, orientation or static collision')
    finally:
        data.qpos[:] = original; mujoco.mj_forward(model,data)


def apply_observation(model,data,plan):
    import mujoco
    for i,q in enumerate(plan['joint_positions_rad'],1):
        data.qpos[model.joint(f'joint{i}').qposadr[0]] = q
    mujoco.mj_forward(model,data)
