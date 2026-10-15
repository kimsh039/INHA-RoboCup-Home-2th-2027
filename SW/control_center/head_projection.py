"""Reject camera rays outside the calibrated sensor before lens projection.

Brown distortion polynomials can fold far off-axis rays back into the image.
An in-image distorted pixel alone therefore cannot establish camera visibility.
"""
from functools import lru_cache

import cv2
import numpy as np


@lru_cache(maxsize=8)
def _ray_bounds(k, d, width, height):
    if width < 2 or height < 2:
        raise ValueError('invalid_camera_sensor_dimensions')
    x = np.linspace(0., width - 1., 33)
    y = np.linspace(0., height - 1., 33)
    border = np.concatenate((np.column_stack((x, np.zeros_like(x))),
                             np.column_stack((x, np.full_like(x, height - 1.))),
                             np.column_stack((np.zeros_like(y), y)),
                             np.column_stack((np.full_like(y, width - 1.), y))))
    rays = cv2.undistortPoints(border.reshape(-1, 1, 2),
                               np.asarray(k).reshape(3, 3),
                               np.asarray(d) if d else None).reshape(-1, 2)
    if not np.isfinite(rays).all():
        raise ValueError('invalid_camera_sensor_rays')
    return rays.min(axis=0) - 1e-4, rays.max(axis=0) + 1e-4


def camera_ray_mask(camera, k, d, width, height):
    """A bounded ray-domain prefilter; distorted bbox testing still follows."""
    lower, upper = _ray_bounds(tuple(k), tuple(d), int(width), int(height))
    finite = np.isfinite(camera).all(axis=1) & (camera[:, 2] > 0.)
    normalized = np.full((len(camera), 2), np.nan)
    np.divide(camera[:, :2], camera[:, 2, None], out=normalized,
              where=finite[:, None])
    return finite & (normalized >= lower).all(axis=1) & (normalized <= upper).all(axis=1)
