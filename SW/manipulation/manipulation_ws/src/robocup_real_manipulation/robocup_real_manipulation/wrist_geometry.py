"""Measured mask/depth geometry; no detector or synthetic sensor feedback."""
import numpy as np
import cv2
from .core import require, robust_depth


def surface_point(depth, mask, info, cfg, error_floor):
    require(info.width == depth.shape[1] and info.height == depth.shape[0], 'CAMERA_GRID_MISMATCH')
    require(info.distortion_model in ('', 'plumb_bob', 'rational_polynomial'), 'UNSUPPORTED_DISTORTION')
    # Existing robust patch/depth checks. K projects a distorted RGB pixel initially;
    # undistort that measured pixel before converting to an optical-frame ray.
    point, quality = robust_depth(depth, mask, info.k, cfg)
    K = np.asarray(info.k, float).reshape(3, 3)
    require(np.isfinite(info.d).all(), 'INVALID_DISTORTION')
    uv = np.array([[[point[0] / point[2] * K[0, 0] + K[0, 2],
                     point[1] / point[2] * K[1, 1] + K[1, 2]]]], np.float64)
    ray = cv2.undistortPoints(uv, K, np.asarray(info.d, float) if len(info.d) else None)[0, 0]
    optical = np.array([ray[0], ray[1], 1.]) * point[2]
    # Depth dispersion + an explicit measurement floor, not a precision guarantee.
    require(np.isfinite(error_floor) and error_floor > 0, 'INVALID_SENSOR_ERROR_FLOOR')
    return optical, max(error_floor, quality)


def banana_point_axis(depth, mask, info, cfg, error_floor, R_base_camera, settings):
    """Global PCA direction translated through a measured central body point.

    PCA is in the gravity XY plane, not the distorted image. Its centroid is NOT
    the grasp point. Select a thick interior point within the central projected
    body band, then measure depth only there. No local PCA/skeleton substitution.
    """
    for key in ('component_fraction','center_fraction','center_band_fraction','clearance_fraction'):
        require(np.isfinite(settings[key]) and 0<settings[key]<=1,'BANANA_SETTING:'+key)
    require(0<=settings['end_trim_fraction']<.5 and settings['min_pca_ratio']>1
            and settings['min_clearance_px']>0 and settings['min_mask_pixels']>=20,'BANANA_SETTINGS')
    binary=(np.asarray(mask)!=0).astype(np.uint8)
    require(binary.ndim==2 and binary.shape==depth.shape,'BANANA_MASK_GRID')
    count,labels,stats,_=cv2.connectedComponentsWithStats(binary,connectivity=8)
    require(count>1,'BANANA_EMPTY_MASK')
    largest=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]))
    area=int(stats[largest,cv2.CC_STAT_AREA]); total=int(binary.sum())
    require(area>=settings['min_mask_pixels'] and area/max(1,total)>=settings['component_fraction'],'BANANA_SPLIT_MASK')
    body=(labels==largest)
    ys,xs=np.nonzero(body)
    K=np.asarray(info.k,float).reshape(3,3)
    require(info.width==depth.shape[1] and info.height==depth.shape[0],'CAMERA_GRID_MISMATCH')
    require(info.distortion_model in ('','plumb_bob','rational_polynomial'),'UNSUPPORTED_DISTORTION')
    require(K[0,0]>0 and K[1,1]>0 and np.isfinite(K).all() and np.isfinite(info.d).all(),'INVALID_INTRINSICS')
    rays=cv2.undistortPoints(np.c_[xs,ys].astype(np.float64).reshape(-1,1,2),K,
        np.asarray(info.d,float) if len(info.d) else None).reshape(-1,2)
    R=np.asarray(R_base_camera,float)
    require(R.shape==(3,3) and np.isfinite(R).all() and R[2,2]<-np.cos(np.deg2rad(10)),'BANANA_VIEW_NOT_TOPDOWN')
    plane=(np.c_[rays,np.ones(len(rays))]@R.T)[:,:2]
    centered=plane-plane.mean(axis=0)
    values,vectors=np.linalg.eigh(centered.T@centered/len(centered))
    ratio=float(values[1]/max(values[0],1e-12))
    require(ratio>=settings['min_pca_ratio'],'BANANA_PCA_AMBIGUOUS')
    axis=vectors[:,1]
    # An axis has no arrow. Canonical sign avoids frame-to-frame 180deg flips.
    if axis[0]<0 or (abs(axis[0])<1e-12 and axis[1]<0): axis=-axis
    projection=centered@axis
    lo,hi=np.quantile(projection,[settings['end_trim_fraction'],1-settings['end_trim_fraction']])
    target=lo+settings['center_fraction']*(hi-lo)
    band=np.abs(projection-target)<=settings['center_band_fraction']*(hi-lo)
    distance=cv2.distanceTransform(body.astype(np.uint8),cv2.DIST_L2,5)[ys,xs]
    require(band.any(),'BANANA_NO_CENTRAL_BODY')
    maximum=float(distance[band].max())
    require(maximum>=settings['min_clearance_px'],'BANANA_BODY_TOO_THIN')
    candidates=np.flatnonzero(band & (distance>=maximum*settings['clearance_fraction']))
    # Closest thick central body pixel to the silhouette centroid. The silhouette
    # centroid may be empty; the selected point always belongs to the body.
    score=np.sum(centered[candidates]**2,axis=1)
    selected=int(candidates[np.argmin(score)])
    u,v=int(xs[selected]),int(ys[selected])
    patch=np.zeros(body.shape,np.uint8)
    radius=cfg['depth_patch_radius_px']+cfg['mask_erosion_px']+2
    cv2.circle(patch,(u,v),radius,1,-1)
    p,std=surface_point(depth,body & patch.astype(bool),info,cfg,error_floor)
    # Bound the whole observed banana about this BODY point for support/collision.
    base_points=(np.c_[rays,np.ones(len(rays))]*p[2])@R.T
    base_grip=R@p
    extent=float(np.max(np.linalg.norm(base_points[:,:2]-base_grip[:2],axis=1)))
    rule=cfg['classes']['banana']
    require(rule['radius_m'] is not None and extent<=rule['radius_m'],'BANANA_COLLISION_RADIUS_TOO_SMALL')
    yaw=float(np.arctan2(axis[1],axis[0]))
    return p,std,yaw,{'body_pixel':[u,v],'pca_ratio':ratio,'extent_m':extent}


def mask_scene(depth,mask,info,R,translation,surface,rule,cfg):
    """Bounded measured occupancy around the target, excluding its dilated silhouette.

    Optical depth -> capture-time gravity transform. No fabricated obstacle-free scene.
    Unknown/occluded space is not certified free by this observed-surface representation.
    """
    require(mask.shape==depth.shape,'SCENE_DEPTH_GRID')
    stride=int(cfg.get('scene_depth_stride_px',6)); voxel=float(cfg.get('scene_voxel_m',.02))
    require(2<=stride<=16 and .005<=voxel<=.04,'SCENE_SAMPLING_CONFIG')
    h,w=depth.shape; yy,xx=np.mgrid[0:h:stride,0:w:stride]
    zz=depth[yy,xx]; good=np.isfinite(zz)&(zz>=cfg['camera_min_depth_m'])&(zz<=cfg['depth_max_m'])
    xx,yy,zz=xx[good],yy[good],zz[good]
    require(len(zz)>=cfg['depth_min_pixels'],'SCENE_DEPTH_MISSING')
    K=np.asarray(info.k,float).reshape(3,3)
    rays=cv2.undistortPoints(np.c_[xx,yy].astype(float).reshape(-1,1,2),K,np.asarray(info.d,float) if len(info.d) else None).reshape(-1,2)
    points=(np.c_[rays,np.ones(len(rays))]*zz[:,None])@np.asarray(R).T+translation
    selected=mask[yy,xx]!=0
    target_points=points[selected]
    require(len(target_points)>=3,'TARGET_EXTENT_UNOBSERVED')
    # Entire silhouette, including a cup handle, must fit configured collision extent.
    radial=np.linalg.norm(target_points[:,:2]-np.asarray(surface)[:2],axis=1)
    require(float(np.max(radial))<=rule['radius_m']+cfg.get('mask_extent_tolerance_m',.008),'MASK_EXCEEDS_COLLISION_EXTENT')
    dilated=cv2.dilate((mask!=0).astype(np.uint8),np.ones((5,5),np.uint8))
    outside=dilated[yy,xx]==0
    floor=cfg['table_z_m']+cfg.get('scene_table_margin_m',.008)
    occupied=points[outside & (points[:,2]>floor)]
    # Restrict measured boxes to manipulation support area + a configurable margin.
    if cfg.get('table_polygon_xy') is not None:
        polygon=np.asarray(cfg['table_polygon_xy']); lo=polygon.min(axis=0)-.15; hi=polygon.max(axis=0)+.15
    else:
        x0,x1,y0,y1=cfg['table_bounds_xy']; lo=np.array([x0,y0])-.15; hi=np.array([x1,y1])+.15
    occupied=occupied[np.all((occupied[:,:2]>=lo)&(occupied[:,:2]<=hi),axis=1)]
    cells=np.unique(np.floor(occupied/voxel).astype(int),axis=0)
    require(len(cells)<=cfg.get('scene_max_voxels',600),'SCENE_OCCUPANCY_LIMIT')
    return [dict(id='wrist_obstacle_'+str(i),center=((cell+.5)*voxel).tolist(),size=[voxel]*3) for i,cell in enumerate(cells)]
