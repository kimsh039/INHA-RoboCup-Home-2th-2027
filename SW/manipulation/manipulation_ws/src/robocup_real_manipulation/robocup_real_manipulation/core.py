"""ROS-independent validation, event join, association and physical grasp geometry."""
import math
import numpy as np

STAGES = ('WAIT_FOR_APPROACH', 'MOVE_TO_OBSERVATION', 'WRIST_REDETECT', 'COMPUTE_GRASP',
          'MOVE_TO_PREGRASP', 'DESCEND', 'CLOSE_GRIPPER', 'VERIFY_GRASP', 'LIFT',
          'MOVE_TO_PLACE', 'RELEASE', 'RETREAT', 'RETURN_ARM', 'COMPLETE')

class Fault(RuntimeError):
    pass

def require(condition, reason):
    if not condition:
        raise Fault(reason)

def finite(value):
    return np.isfinite(np.asarray(value, dtype=float)).all()

def fresh(stamp, now, age, future=0.05):
    require(math.isfinite(stamp) and stamp > 0 and -future <= now-stamp <= age, 'STALE_OR_FUTURE_DATA')

def rotation(q):
    q = np.asarray(q, float)
    require(q.shape == (4,) and finite(q) and abs(np.linalg.norm(q)-1) < 1e-3, 'INVALID_QUATERNION')
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def transform_point(point, xyz, quaternion):
    require(finite(point) and finite(xyz), 'INVALID_POINT')
    return rotation(quaternion) @ np.array(point) + xyz

def validate(cfg, hardware=False):
    require(cfg.get('base_completion_policy', 'hold') in ('hold', 'release'), 'BASE_COMPLETION_POLICY')
    # Initial trial setting, not a zero calibration or wider command limits.
    startup_margin=cfg.get('startup_bounds_tolerance_rad',0.)
    require(finite(startup_margin) and 0<=startup_margin<=math.radians(5),'STARTUP_BOUNDS_TOLERANCE')
    tolerance=cfg.get('axis_stability_tolerance_rad',.15)
    require(finite(tolerance) and 0<tolerance<=math.pi/4,'AXIS_STABILITY_TOLERANCE')
    for key in ('pregrasp_m','lift_m','retreat_m','place_offset_m','stage_timeout_s',
                'approach_timeout_s','observation_timeout_s','feedback_age_s','base_age_s',
                'max_position_std_m','max_observation_age_s','association_radius_m',
                'max_joint_speed_rad_s','max_joint_acceleration_rad_s2','gripper_settle_s'):
        require(finite(cfg[key]) and cfg[key]>0, 'INVALID_SETTING:'+key)
    require(0<cfg['velocity_scale']<=0.2 and 0<cfg['acceleration_scale']<=0.2, 'INVALID_SPEED_SCALE')
    require(isinstance(cfg['observation_retries'],int) and 0<=cfg['observation_retries']<=3, 'INVALID_RETRY')
    require(isinstance(cfg['observation_stable_frames'],int) and 2<=cfg['observation_stable_frames']<=10,'INVALID_STABLE_FRAMES')
    require(0.5<=cfg['gripper_command_effort']<=3 and 1<=cfg['driver_speed_percent']<=20,'DRIVER_COMMAND_RANGE')
    require(0<cfg['depth_min_fraction']<=1 and 0<cfg['depth_min_m']<cfg['depth_max_m'] and cfg['depth_min_pixels']>=3,'DEPTH_CONFIG')
    require(0<=cfg['mask_erosion_px']<=10 and 1<=cfg['depth_patch_radius_px']<=100,'DEPTH_REGION_CONFIG')
    for key in ('joint_stable_s','stable_speed_rad_s','gripper_tolerance_m','gripper_max_m','goal_tolerance_rad','path_tolerance_rad','support_margin_m'):
        require(finite(cfg[key]) and cfg[key]>0,'INVALID_SETTING:'+key)
    require(len(cfg['joint_names'])==6 and len(set(cfg['joint_names']))==6, 'JOINT_NAMES')
    require(0<cfg['min_confidence']<=1 and cfg['gripper_max_m']==0.100, 'QUALITY_OR_WIDTH')
    ids=[]
    for name,r in cfg['classes'].items():
        require(0<r['close_m']<r['open_m']<=cfg['gripper_max_m'], 'WIDTH_RANGE:'+name)
        if r['class_id'] is not None:
            require(r['class_id']>=0 and r['class_id']<len(cfg['model_names']), 'CLASS_ID:'+name)
            require(cfg['model_names'][r['class_id']]==r.get('detector_name',name), 'CLASS_NAME:'+name)
            ids.append(r['class_id'])
    require(len(ids)==len(set(ids)), 'DUPLICATE_CLASS_ID')
    if hardware:
        require(not cfg.get('fixture_only',False), 'TEST_FIXTURE_NOT_HARDWARE')
        for key in ('execution_enabled','commissioned','calibration_verified','scene_verified',
                    'model_names_verified','gravity_frame_verified'):
            require(cfg[key] is True, 'NOT_VERIFIED:'+key)
        # [TUNE:WIDTH_TRIAL] Explicit first physical trial with nominal aperture.
        # Never marks the mapping calibrated; all scene/TF/feedback gates remain.
        width_trial = cfg.get('width_mapping_mode', 'measured') == 'nominal_trial'
        require(cfg.get('width_mapping_verified') is True or width_trial,
                'NOT_VERIFIED:width_mapping_verified')
        if width_trial:
            require(cfg['velocity_scale'] <= .05 and cfg['acceleration_scale'] <= .05,
                    'WIDTH_TRIAL_SPEED')
            # Per-class apertures already obey WIDTH_RANGE above; no class whitelist.
        for key in ('joint_signs','joint_offsets','return_joints',
                    'topdown_quaternion','tcp_to_grip_center_m','table_z_m','table_bounds_xy',
                    'base_arm_transform','handeye_transform','link6_tcp_transform','obstacles',
                    'width_scale','width_bias_m','driver_width_max','grasp_min_effort','can_port','firmware',
                    'wrist_camera_model','wrist_camera_serial','camera_min_depth_m','coarse_z_error_bound_m'):
            require(cfg[key] is not None, 'UNSET:'+key)
        require(finite(cfg['joint_offsets']) and len(cfg['joint_offsets'])==6, 'JOINT_OFFSETS')
        require(len(cfg['joint_signs'])==6 and all(x in (-1,1) for x in cfg['joint_signs']), 'JOINT_SIGNS')
        limits=np.array(cfg['joint_limits'])
        require(limits.shape==(6,2) and finite(limits) and np.all(limits[:,0]<limits[:,1]), 'JOINT_LIMITS')
        for key in ('observation_joints','return_joints'):
            if cfg[key] is None: continue
            q=np.array(cfg[key]); require(q.shape==(6,) and finite(q) and np.all(q>=limits[:,0]) and np.all(q<=limits[:,1]), 'POSE_LIMIT:'+key)
        R=rotation(cfg['topdown_quaternion'])
        require(np.dot(R[:,2],[0,0,-1])>0.999, 'TCP_Z_NOT_DOWN')
        require(len(cfg['tcp_to_grip_center_m'])==3 and finite(cfg['tcp_to_grip_center_m']), 'TCP_OFFSET')
        require(0.1<=cfg['observation_surface_clearance_m']<=1.0,'OBSERVATION_CLEARANCE')
        require(finite([cfg['camera_min_depth_m'],cfg['coarse_z_error_bound_m'],cfg['observation_depth_margin_m']]) and cfg['camera_min_depth_m']>0 and cfg['coarse_z_error_bound_m']>=0 and cfg['observation_depth_margin_m']>0,'OBSERVATION_DEPTH_BOUNDS')
        require(cfg['observation_surface_clearance_m']-cfg['coarse_z_error_bound_m']>=cfg['camera_min_depth_m']+cfg['observation_depth_margin_m'],'OBSERVATION_TOO_CLOSE_FOR_DEPTH_PROFILE')
        if cfg['observation_camera_distance_m'] is not None:
            require(0.1<=cfg['observation_camera_distance_m']<=1.0,'OBSERVATION_DISTANCE')
        for key in ('base_arm_transform','handeye_transform','link6_tcp_transform'):
            tf=cfg[key]
            require(isinstance(tf,dict) and len(tf.get('xyz',[]))==3 and finite(tf['xyz']),'TRANSFORM:'+key)
            rotation(tf['quaternion_xyzw'])
        observation_geometry(np.array([0.,0.,1.]),cfg)
        require(cfg['width_scale']>0 and cfg['driver_width_max']>0 and cfg['grasp_min_effort']>0, 'WIDTH_EFFORT_CALIBRATION')
        for r in cfg['classes'].values():
            if r['enabled']:
                require(r['class_id'] is not None,'ENABLED_CLASS_WITHOUT_ID')
                for key in ('center_height_m','grip_height_m','radius_m'):
                    require(r[key] is not None and finite(r[key]) and r[key]>0, 'CLASS_UNSET:'+key)
                width_to_driver(r['open_m'],cfg); width_to_driver(r['close_m'],cfg)
                if cfg['observation_camera_distance_m'] is not None:
                    clearance=cfg['observation_camera_distance_m']-r['center_height_m']
                    require(clearance-cfg['coarse_z_error_bound_m']>=cfg['camera_min_depth_m']+cfg['observation_depth_margin_m'],'OVERRIDE_TOO_CLOSE_FOR_DEPTH_PROFILE')
                if r.get('axis_required'):
                    require(r.get('closing_axis_yaw_offset_rad') is not None and finite(r['closing_axis_yaw_offset_rad']),'BANANA_AXIS_OFFSET_UNSET')
        require(isinstance(cfg['obstacles'],list),'OBSTACLE_LIST')
        obstacle_ids=set()
        for box in cfg['obstacles']:
            require(box.get('id') and box['id'] not in obstacle_ids and box['id'] not in ('manip_table','manip_target'),'OBSTACLE_ID')
            require(len(box['center'])==3 and len(box['size'])==3 and finite(box['center']) and finite(box['size']) and min(box['size'])>0,'OBSTACLE_BOX')
            obstacle_ids.add(box['id'])
        bounds=cfg['table_bounds_xy']; require(len(bounds)==4 and finite(bounds) and bounds[0]<bounds[1] and bounds[2]<bounds[3], 'TABLE_BOUNDS')
    return cfg

def rule_for(cfg, class_id):
    rules=[r for r in cfg['classes'].values() if r['enabled'] and r['class_id']==class_id]
    require(len(rules)==1, 'UNSUPPORTED_CLASS')
    return rules[0]

# [CALIB:GRIPPER_UNITS] Do not change SDK units to tune physical closing width.
def width_to_driver(width, cfg):
    require(finite(width) and 0<=width<=cfg['gripper_max_m'], 'WIDTH_RANGE')
    raw=(width-cfg['width_bias_m'])/cfg['width_scale']
    require(finite(raw) and 0<=raw<=cfg['driver_width_max'], 'DRIVER_WIDTH_RANGE')
    return raw

class EventJoin:
    """Bounded cache. Active tasks reject new tasks; durable terminal IDs block replay."""
    def __init__(self, completed=()):
        self.targets={}; self.approaches={}; self.completed=set(completed); self.active=None
    def begin(self, task_id):
        require(bool(task_id) and len(task_id)<=128, 'INVALID_TASK_ID')
        require(self.active is None, 'BUSY')
        require(task_id not in self.completed, 'DUPLICATE_TASK')
        self.active=task_id
    def put(self, kind, task_id, value):
        require(task_id and len(task_id)<=128 and task_id not in self.completed, 'PREVIOUS_TASK_EVENT')
        require(self.active in (None,task_id), 'BUSY')
        cache=getattr(self,kind)
        require(task_id not in cache, 'DUPLICATE_EVENT')
        require(len(cache)<64, 'EVENT_CACHE_FULL')
        cache[task_id]=value
    def ready(self):
        return self.active in self.targets and self.active in self.approaches
    def finish(self):
        self.completed.add(self.active)
        self.targets.pop(self.active,None); self.approaches.pop(self.active,None); self.active=None


def robust_depth(depth, mask, K, cfg):
    require(depth.ndim==2 and mask.shape==depth.shape and finite(K), 'DEPTH_ALIGNMENT')
    require(K[0]>0 and K[4]>0, 'INVALID_INTRINSICS')
    region=mask.astype(bool).copy()
    # Erode without OpenCV dependency, avoids object boundaries and table pixels.
    for _ in range(cfg['mask_erosion_px']):
        a=np.pad(region,1,constant_values=False)
        region=a[1:-1,1:-1]&a[:-2,1:-1]&a[2:,1:-1]&a[1:-1,:-2]&a[1:-1,2:]
    ys,xs=np.nonzero(region); require(len(xs)>=cfg['depth_min_pixels'], 'MASK_TOO_SMALL')
    u,v=float(np.median(xs)),float(np.median(ys))
    region &= (np.indices(depth.shape)[1]-u)**2+(np.indices(depth.shape)[0]-v)**2<=cfg['depth_patch_radius_px']**2
    valid=region & np.isfinite(depth) & (depth>=max(cfg['depth_min_m'],cfg.get('camera_min_depth_m') or cfg['depth_min_m'])) & (depth<=cfg['depth_max_m'])
    values=depth[valid]
    require(len(values)>=cfg['depth_min_pixels'] and len(values)/max(1,region.sum())>=cfg['depth_min_fraction'], 'INVALID_DEPTH')
    z=float(np.median(values)); mad=float(np.median(abs(values-z)))
    require(mad<=cfg['depth_max_mad_m'], 'UNSTABLE_DEPTH')
    yy,xx=np.nonzero(valid & (abs(depth-z)<=max(0.002,3*mad)))
    require(len(xx)>=cfg['depth_min_pixels'], 'DEPTH_OUTLIERS')
    u,v=float(np.median(xx)),float(np.median(yy))
    return np.array([(u-K[2])*z/K[0],(v-K[5])*z/K[4],z]), max(mad*1.4826,0.001)

def associate(candidates, coarse, instance_id, cfg):
    # candidate=(position, instance_id, payload); gate identity AND location.
    ranked=sorted([(float(np.linalg.norm(p-coarse)),i,payload) for p,i,payload in candidates
                   if not instance_id or not i or i==instance_id],key=lambda x:x[0])
    require(ranked and ranked[0][0]<=cfg['association_radius_m'], 'TARGET_NOT_ASSOCIATED')
    require(len(ranked)==1 or ranked[1][0]-ranked[0][0]>=cfg['association_margin_m'], 'AMBIGUOUS_TARGET')
    return ranked[0][2],ranked[0][0]

# [TUNE:GRASP_HEIGHT] Class grip_height_m and topdown_quaternion define grasp target.
def grasp_geometry(point, kind, rule, cfg, axis=None):
    require(finite(point), 'INVALID_POINT')
    require(rule['center_height_m'] is not None and rule['grip_height_m'] is not None, 'CLASS_HEIGHT_UNSET')
    center=np.array(point,float)
    if kind=='SURFACE':
        # Initial experiment: top-facing aligned camera, tabletop objects only.
        # Mask centroid gives XY; center Z comes from measured support + center height.
        require(abs(center[2]-(cfg['table_z_m']+2*rule['center_height_m']))<=0.04, 'SURFACE_NOT_OBJECT_TOP')
        center[2]=cfg['table_z_m']+rule['center_height_m']
    else:
        require(kind=='CENTER', 'POSITION_KIND')
        require(abs(center[2]-(cfg['table_z_m']+rule['center_height_m']))<=0.025, 'CENTER_HEIGHT_MISMATCH')
    q=np.array(cfg['topdown_quaternion'],float)
    if rule.get('axis_required'):
        require(axis is not None and math.isfinite(axis) and rule.get('closing_axis_yaw_offset_rad') is not None, 'BANANA_AXIS_REQUIRED')
        yaw=axis+math.pi/2+rule['closing_axis_yaw_offset_rad']; z,w=math.sin(yaw/2),math.cos(yaw/2)
        x,y,a,b=q; q=np.array([w*x-z*y,w*y+z*x,w*a+z*b,w*b-z*a])
    grip=center.copy(); grip[2]=cfg['table_z_m']+rule['grip_height_m']
    tcp=grip-rotation(q)@np.array(cfg['tcp_to_grip_center_m'])
    return center,tcp,q

def support(point,radius,cfg):
    pad=radius+cfg['support_margin_m']
    if cfg.get('table_polygon_xy') is not None:
        corners=np.asarray(cfg['table_polygon_xy'],float); p=np.asarray(point[:2])
        edges=np.roll(corners,-1,axis=0)-corners; delta=p-corners
        signed=(edges[:,0]*delta[:,1]-edges[:,1]*delta[:,0])/np.linalg.norm(edges,axis=1)
        return bool(np.all(signed>=pad))
    x0,x1,y0,y1=cfg['table_bounds_xy']
    return x0+pad<=point[0]<=x1-pad and y0+pad<=point[1]<=y1-pad


def verify_axis_series(quaternions,tolerance):
    axes=np.array([math.atan2(rotation(q)[1,0],rotation(q)[0,0]) for q in quaternions])
    mean=np.angle(np.mean(np.exp(2j*axes)))/2
    spread=np.max(np.abs(np.angle(np.exp(2j*(axes-mean)))))/2
    require(spread<=tolerance,'UNSTABLE_BANANA_AXIS')


# [TUNE:OBSERVATION] Search observation_surface_clearance_m in config/tuning.yaml.
def observation_geometry(coarse,cfg,rule=None):
    """Top-down camera ray aimed at coarse target; TCP pose includes calibrated mount.

    The distance is optical depth, not an invented absolute joint posture.
    T_tcp_opt = inverse(T_link6_tcp) * T_link6_opt.
    """
    require(finite(coarse),'INVALID_COARSE_POINT')
    qbo=np.array(cfg['observation_camera_quaternion'],float); Rbo=rotation(qbo)
    handeye=cfg['handeye_transform']; tcp=cfg['link6_tcp_transform']
    require(handeye is not None and tcp is not None,'OBSERVATION_CALIBRATION_UNSET')
    distance=cfg['observation_camera_distance_m']
    if distance is None:
        height=rule['center_height_m'] if rule is not None else 0.
        require(height is not None,'CLASS_CENTER_HEIGHT_UNSET')
        distance=cfg['observation_surface_clearance_m']+height
    require(0.1<=distance<=1.2,'OBSERVATION_DISTANCE')
    Rlt=rotation(tcp['quaternion_xyzw']); Rlo=rotation(handeye['quaternion_xyzw'])
    tto=Rlt.T@(np.array(handeye['xyz'])-np.array(tcp['xyz']))
    qlo=np.array(handeye['quaternion_xyzw']); qlo_inv=qlo.copy();qlo_inv[:3]*=-1
    q=quaternion_product(quaternion_product(qbo,qlo_inv),tcp['quaternion_xyzw'])
    Rbt=rotation(q)
    require(np.dot(Rbo[:,2],[0,0,-1])>math.cos(math.radians(10)),'OBSERVATION_CAMERA_NOT_TOPDOWN')
    camera=np.array(coarse)-Rbo[:,2]*distance
    position=camera-Rbt@tto
    require(position[2]>coarse[2],'OBSERVATION_TCP_NOT_ABOVE_TARGET')
    return position,q


def quaternion_product(a,b):
    """Hamilton product, xyzw ordering."""
    a=np.array(a,float); b=np.array(b,float)
    return np.r_[a[3]*b[:3]+b[3]*a[:3]+np.cross(a[:3],b[:3]),a[3]*b[3]-np.dot(a[:3],b[:3])]
