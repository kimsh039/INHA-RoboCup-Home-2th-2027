"""Gazebo RGB/depth projection and fixed Nav2 standoff goal geometry."""
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


def target_range(box, depth, color: Intrinsics, sensor: Intrinsics, minimum_points=20):
    """Project central RGB bbox rays into co-located depth optical frame.

    The current Gazebo head sensors share pose/frame but have different FOVs.
    This mapping is NOT valid for offset real RGB/depth cameras without alignment.
    """
    if not color.valid() or not sensor.valid() or depth.shape != (sensor.height, sensor.width):
        raise ValueError('Invalid CameraInfo/depth shape')
    x, y, width, height = box
    if not all(math.isfinite(v) for v in box) or width < 2 or height < 2:
        return None
    # Central 40% patch reduces background at the box boundaries.
    left, right = x+width*.3, x+width*.7
    top, bottom = y+height*.3, y+height*.7
    project_x = lambda u: (u-color.cx)*sensor.fx/color.fx + sensor.cx
    project_y = lambda v: (v-color.cy)*sensor.fy/color.fy + sensor.cy
    x0, x1 = max(0, math.floor(project_x(left))), min(sensor.width, math.ceil(project_x(right)))
    y0, y1 = max(0, math.floor(project_y(top))), min(sensor.height, math.ceil(project_y(bottom)))
    if x1 <= x0 or y1 <= y0:
        return None
    patch = depth[y0:y1, x0:x1]
    values = patch[np.isfinite(patch) & (patch >= .2) & (patch <= 5.0)]
    if values.size < minimum_points or values.size < patch.size*.5:
        return None
    # Broad/bimodal background is rejected; this is a planar billboard test.
    if np.percentile(values, 90)-np.percentile(values, 10) > .3:
        return None
    return float(np.median(values))


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
