"""Cached conservative optical cone for calibrated Brown camera projection."""
from functools import lru_cache
import cv2
import numpy as np


@lru_cache(maxsize=8)
def ray_bounds(width,height,intrinsics,distortion):
    # Include interior samples for tangential/rational distortion, not just four
    # pinhole corners. Retain a generous margin; exact image and ray-roundtrip
    # checks still follow projection. Cache per calibration/resolution.
    x,y=np.meshgrid(np.linspace(0,width,65),np.linspace(0,height,49))
    pixels=np.column_stack((x.ravel(),y.ravel())).astype(float)
    rays=cv2.undistortPoints(pixels.reshape(-1,1,2),np.array(intrinsics).reshape(3,3),
                             np.array(distortion) if distortion else None).reshape(-1,2)
    if not np.isfinite(rays).all():
        raise ValueError('Invalid camera ray envelope')
    lo,hi=rays.min(axis=0),rays.max(axis=0)
    margin=np.maximum(.05*(hi-lo),.02)
    return lo-margin,hi+margin


def in_frustum(camera,width,height,intrinsics,distortion):
    lo,hi=ray_bounds(width,height,tuple(intrinsics),tuple(distortion))
    x,y,z=camera.T
    return (np.isfinite(camera).all(axis=1) & (z>.15) & (z<5.) &
            (x>=lo[0]*z) & (x<=hi[0]*z) & (y>=lo[1]*z) & (y<=hi[1]*z))
