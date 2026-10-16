"""Physical evidence checks. No navigation or detection implementation."""
import math
import fcntl
from pathlib import Path
from .core import require, fresh

class ProcessOwner:
    """Kernel releases the lock on crash; persistent task journal remains locked."""
    def __init__(self, path):
        path=Path(path).expanduser(); path.parent.mkdir(parents=True,exist_ok=True)
        self.file=path.open('a')
        try: fcntl.flock(self.file,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError('DUPLICATE_PROCESS:'+str(path))

def grasp_contact(width, effort, rule, cfg):
    require(math.isfinite(width) and math.isfinite(effort),'GRASP_NONFINITE')
    require(abs(effort)>=cfg['grasp_min_effort'],'GRASP_NO_CONTACT_EFFORT')
    require(cfg.get('grasp_min_width_m',.01)<width<rule['open_m']-cfg.get('grasp_open_margin_m',.005),
            'GRASP_EMPTY_OR_STILL_OPEN')
    require(width>=rule['close_m']-cfg['gripper_tolerance_m'],'GRASP_BELOW_CLOSE_TARGET')

def hold_contact(width, effort, initial, cfg):
    require(all(math.isfinite(v) for v in (width,effort,initial)),'HOLD_NONFINITE')
    require(abs(effort)>=cfg.get('hold_min_effort',cfg['grasp_min_effort']) and
            abs(width-initial)<=cfg.get('hold_width_slip_m',.004),'OBJECT_HOLD_LOST')

def lease_evidence(msg, task_id, now, cfg, requested_at):
    require(msg and msg.task_id==task_id and msg.locked and msg.stopped,'BASE_NOT_LOCKED_AND_STOPPED')
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    fresh(stamp,now,cfg['base_age_s'])
    require(stamp>=requested_at,'LEASE_BEFORE_REQUEST')

def full_view_distance(info, rule, cfg):
    """Fit full silhouette + coarse XY error on BOTH image axes; metric optical distance."""
    require(info.width>0 and info.height>0 and info.k[0]>0 and info.k[4]>0,'CAMERA_INTRINSICS')
    margin=cfg.get('observation_image_margin_fraction',.12)
    require(0<=margin<.4,'OBSERVATION_IMAGE_MARGIN')
    # min half-FOV accounts for noncentral principal point.
    tan_half=min(info.k[2]/info.k[0],(info.width-info.k[2])/info.k[0],
                 info.k[5]/info.k[4],(info.height-info.k[5])/info.k[4])*(1-2*margin)
    require(tan_half>0,'CAMERA_FOV')
    radius=rule['radius_m']+cfg.get('coarse_xy_error_bound_m',cfg['association_radius_m'])
    clearance=max(cfg['observation_surface_clearance_m'],radius/tan_half+cfg['coarse_z_error_bound_m'],
                  cfg['camera_min_depth_m']+cfg['observation_depth_margin_m']+cfg['coarse_z_error_bound_m'])
    distance=max(clearance+rule['center_height_m'],cfg.get('observation_camera_distance_m') or 0.)
    require(distance<=cfg.get('observation_max_distance_m',.65),'FULL_OBJECT_VIEW_OUT_OF_RANGE')
    return distance


def scene_at_arrival(source, xyz, quaternion):
    """Transform configured FIXED-frame scene after driving; no detector/fusion replacement."""
    import copy
    import numpy as np
    from .core import transform_point, rotation
    c=copy.deepcopy(source); R=rotation(quaternion)
    require(R[2,2]>.999,'SCENE_FRAME_NOT_LEVEL')
    x0,x1,y0,y1=c['table_bounds_xy']; z=c['table_z_m']
    polygon=np.array([transform_point([x,y,z],xyz,quaternion) for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1))])
    c['table_polygon_xy']=polygon[:,:2].tolist(); c['table_z_m']=float(polygon[:,2].mean())
    thickness=c.get('table_thickness_m',.028)
    c['table_collision_center']=transform_point([(x0+x1)/2,(y0+y1)/2,z-thickness/2],xyz,quaternion).tolist()
    c['table_collision_size']=[x1-x0,y1-y0,thickness]; c['scene_quaternion']=list(quaternion)
    for obj in c['obstacles']:
        obj['center']=transform_point(obj['center'],xyz,quaternion).tolist()
        obj['quaternion']=list(quaternion)
    return c


class TerminalIDs:
    """Exact durable replay protection without loading all historic IDs into RAM."""
    def __init__(self,path,previous=()):
        import sqlite3
        import threading
        self.mutex=threading.Lock()
        self.db=sqlite3.connect(str(path),check_same_thread=False)
        self.db.execute('CREATE TABLE IF NOT EXISTS terminal (task TEXT PRIMARY KEY)')
        self.db.executemany('INSERT OR IGNORE INTO terminal VALUES (?)',[(x,) for x in previous])
        self.db.commit()
    def __contains__(self,task):
        with self.mutex:
            return self.db.execute('SELECT 1 FROM terminal WHERE task=?',(task,)).fetchone() is not None
    def add(self,task):
        with self.mutex:
            self.db.execute('INSERT OR IGNORE INTO terminal VALUES (?)',(task,)); self.db.commit()


def validate_integration(cfg):
    import numpy as np
    require(cfg['coarse_fixed_frame'] in ('odom','map') and cfg['coarse_fixed_frame']!=cfg['gravity_frame'],'MOVING_BASE_REQUIRES_FIXED_FRAME')
    require(cfg.get('scene_frame')==cfg['coarse_fixed_frame'],'SCENE_NOT_CONFIGURED_IN_FIXED_FRAME')
    require(not cfg.get('startup_prefix_allowed_contacts'),'UNREVIEWED_STARTUP_COLLISION_EXCEPTIONS')
    for key,default in (('coarse_xy_error_bound_m',cfg['association_radius_m']),('hold_width_slip_m',.004),
                        ('hold_min_effort',cfg['grasp_min_effort']),('gripper_stall_m',.0005),
                        ('observation_max_distance_m',.65),('wrist_search_radius_m',.08),('base_unlock_timeout_s',3.)):
        value=cfg.get(key,default)
        require(math.isfinite(value) and value>0,'INVALID_INTEGRATION_SETTING:'+key)
    offsets=np.asarray(cfg.get('wrist_search_offsets_m',[[0,0],[.05,0],[-.05,0],[0,.05]]),float)
    require(offsets.ndim==2 and offsets.shape[1]==2 and 1<=len(offsets)<=4 and np.isfinite(offsets).all() and
            np.max(np.linalg.norm(offsets,axis=1))<=cfg.get('wrist_search_radius_m',.08),'WRIST_SEARCH_BOUND')
