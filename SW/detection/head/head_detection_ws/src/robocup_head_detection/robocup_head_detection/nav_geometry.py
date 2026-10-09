"""Head-camera target placement with lidar points, and fixed Nav2 standoff goal geometry."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def valid(self):
        return (self.width > 0 and self.height > 0 and
                all(math.isfinite(v) for v in (self.fx, self.fy, self.cx, self.cy)) and
                self.fx > 0 and self.fy > 0)


def depth_array(msg):
    if msg.encoding != '32FC1':
        raise ValueError('Simulation demo requires 32FC1 depth in metres')
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width*4 or len(msg.data) != msg.height*msg.step:
        raise ValueError('Invalid depth dimensions/stride')
    dtype = '>f4' if msg.is_bigendian else '<f4'
    return np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data, strides=(msg.step, 4))


def lidar_target(points, box, color: Intrinsics, cam_from_map, surfaces=(), shrink=.1, band=(.01, .35),
                 cluster=.10, minimum_points=3):
    """Locate a head-camera bbox with lidar points (e.g. Mid-360, accumulated in map).

    points: (N, 3) in map; cam_from_map: 4x4, map -> colour optical frame (z forward, x right,
    y down). Points projecting inside the bbox (shrunk by `shrink` of its size per side) are kept.
    With support surfaces [(corners (k, 2), z)], only points `band` above a surface they lie over
    are kept, which drops the table top and the wall behind. The object is the nearest cluster:
    points within `cluster` of the 20th range percentile. Returns (map xyz median, point count)
    or (None, count). The lidar hits the side facing it, so the point is that side, not the centre.
    """
    if not color.valid() or len(points) == 0:
        return None, 0
    x, y, width, height = box
    if not all(math.isfinite(v) for v in box) or width < 2 or height < 2:
        return None, 0
    pts = np.asarray(points, float)
    cam = pts @ cam_from_map[:3, :3].T + cam_from_map[:3, 3]
    ahead = cam[:, 2] > .1
    z = np.where(ahead, cam[:, 2], 1.0)
    u, v = color.fx*cam[:, 0]/z + color.cx, color.fy*cam[:, 1]/z + color.cy
    sx, sy = shrink*width, shrink*height
    keep = ahead & (u > x+sx) & (u < x+width-sx) & (v > y+sy) & (v < y+height-sy)
    if len(surfaces):
        above = np.zeros(len(pts), bool)
        for corners, surface_z in surfaces:
            c = np.asarray(corners, float)
            inside = np.ones(len(pts), bool)
            for i in range(len(c)):                   # convex, counter-clockwise
                a, d = c[i], c[(i+1) % len(c)]-c[i]
                inside &= (d[0]*(pts[:, 1]-a[1]) - d[1]*(pts[:, 0]-a[0])) >= 0
            above |= inside & (pts[:, 2] > surface_z+band[0]) & (pts[:, 2] < surface_z+band[1])
        keep &= above
    count = int(keep.sum())
    if count < minimum_points:
        return None, count
    rng = cam[keep, 2]
    near = rng < np.percentile(rng, 20)+cluster
    return np.median(pts[keep][near], axis=0), count


def standoff_goal(target_xy, robot_xy, distance=1.0):
    """Fixed map goal before the observed object, facing it."""
    tx, ty = target_xy
    rx, ry = robot_xy
    if not all(math.isfinite(v) for v in (tx, ty, rx, ry, distance)) or distance < .5:
        raise ValueError('Invalid goal geometry')
    length = math.hypot(tx-rx, ty-ry)
    if length <= distance:
        return None
    yaw = math.atan2(ty-ry, tx-rx)
    return tx-distance*math.cos(yaw), ty-distance*math.sin(yaw), yaw


def free_goal(grid, x, y, radius=.35):
    """Require known free cells over a conservative robot footprint."""
    origin = grid.info.origin
    q = origin.orientation
    if abs(q.x)+abs(q.y)+abs(q.z) > 1e-6 or abs(abs(q.w)-1) > 1e-6:
        return False
    resolution = grid.info.resolution
    if not math.isfinite(resolution) or resolution <= 0 or not math.isfinite(x+y):
        return False
    cx, cy = math.floor((x-origin.position.x)/resolution), math.floor((y-origin.position.y)/resolution)
    reach = math.ceil(radius/resolution)
    for row in range(cy-reach, cy+reach+1):
        for col in range(cx-reach, cx+reach+1):
            if (col-cx)**2+(row-cy)**2 > reach**2:
                continue
            if not 0 <= col < grid.info.width or not 0 <= row < grid.info.height:
                return False
            index = row*grid.info.width+col
            if index >= len(grid.data) or not 0 <= grid.data[index] < 50:
                return False
    return True
