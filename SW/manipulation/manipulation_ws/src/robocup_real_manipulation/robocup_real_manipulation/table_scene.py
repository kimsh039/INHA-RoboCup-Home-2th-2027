"""Validated, convex measured support in the locked robot's gravity frame."""
import numpy as np
from .core import require, finite


def measured_table(points, cfg):
    p = np.asarray(points, float)
    require(p.shape == (4, 3) and finite(p), 'DOCK_SURFACE_SHAPE')
    require(np.ptp(p[:, 2]) <= .015, 'DOCK_SURFACE_NOT_LEVEL')
    xy = p[:, :2]
    edges = np.roll(xy, -1, axis=0)-xy
    lengths = np.linalg.norm(edges, axis=1)
    require(np.all((lengths >= .10) & (lengths <= 5.)), 'DOCK_SURFACE_SIZE')
    cross = np.cross(edges, np.roll(edges, -1, axis=0))
    require(np.all(cross > 1e-5) or np.all(cross < -1e-5), 'DOCK_SURFACE_NOT_CONVEX')
    if cross[0] < 0:
        xy = xy[::-1]
    z = float(np.max(p[:, 2]))  # Never sink the collision top below a measured corner.
    return dict(table_z_m=z, table_polygon_xy=xy.tolist(),
                table_bounds_xy=[float(xy[:, 0].min()), float(xy[:, 0].max()),
                                 float(xy[:, 1].min()), float(xy[:, 1].max())])


def contains_disk(point, radius, polygon):
    xy = np.asarray(polygon, float)
    edges = np.roll(xy, -1, axis=0)-xy
    distances = np.cross(edges, np.asarray(point)[:2]-xy)/np.linalg.norm(edges, axis=1)
    return bool(np.all(distances >= radius))
