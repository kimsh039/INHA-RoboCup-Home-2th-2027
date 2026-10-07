"""Table-edge measurement in the robot frame and the docking control law (pure numpy, no ROS).

The edge facing the robot is fitted from support-surface points in base_link: for each
lateral bin the nearest top point marks the edge; a robust line x = c0 + c1*y through those
gives the perpendicular distance and the edge's angle to the robot. Bins where the nearest
point sits at the Mid-360's blind limit are dropped: there the top continues closer to the
robot than the sensor can see, so the edge itself is hidden.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class EdgeConfig:
    sensor_xy: tuple = (-0.18, 0.0)   # Mid-360 origin in base_link
    sensor_height: float = 1.34       # above the floor
    max_depression: float = math.radians(52)  # steepest ray below horizontal
    layer_half: float = 0.02          # top points within +-this of the surface height
    lateral: float = 0.5              # use bins with |y| < lateral, m
    bin: float = 0.05                 # lateral bin width, m
    blind_margin: float = 0.03        # bins whose nearest point is this close to the blind limit are hidden
    inlier: float = 0.02              # robust fit: residual threshold, m
    min_bins: int = 6


@dataclass(frozen=True)
class Edge:
    distance: float      # perpendicular distance from base_link to the edge line, m
    angle: float         # direction of the edge normal toward the table, in base_link (rad, CCW +);
                         # the robot faces the edge squarely when it is 0
    bins: int            # inlier bins used
    residual: float      # RMS of inlier residuals, m


def blind_limit(y, height, cfg=EdgeConfig()):
    """Closest base_link x at lateral y where a surface at this height is still visible."""
    r = (cfg.sensor_height - height)/math.tan(cfg.max_depression)
    dy = y - cfg.sensor_xy[1]
    return cfg.sensor_xy[0] + math.sqrt(max(r*r - dy*dy, 0.0))


def fit_edge(points, height, floor_z=0.0, cfg=EdgeConfig()):
    """points: (N, 3) in base_link; height: surface height above the floor. Edge or None."""
    p = np.asarray(points, float).reshape(-1, 3)
    p = p[np.all(np.isfinite(p), axis=1)]
    top = p[(np.abs(p[:, 2] - floor_z - height) < cfg.layer_half) & (p[:, 0] > 0) & (np.abs(p[:, 1]) < cfg.lateral)]
    if len(top) == 0:
        return None
    ys, xs = [], []
    for lo in np.arange(-cfg.lateral, cfg.lateral, cfg.bin):
        sel = top[(top[:, 1] >= lo) & (top[:, 1] < lo + cfg.bin)]
        if len(sel) == 0:
            continue
        y = lo + cfg.bin/2
        x = float(sel[:, 0].min())
        if x < blind_limit(y, height, cfg) + cfg.blind_margin:
            continue                                  # top runs into the blind zone: edge hidden
        ys.append(y)
        xs.append(x)
    ys, xs = np.array(ys), np.array(xs)
    if len(ys) < cfg.min_bins:
        return None
    # Theil-Sen start (median of pairwise slopes) so bins hidden by objects on the top cannot
    # drag the line, then least squares on the bins within a robust residual band.
    i, j = np.triu_indices(len(ys), 1)
    c1 = float(np.median((xs[j]-xs[i])/(ys[j]-ys[i])))
    c = np.array([float(np.median(xs - c1*ys)), c1])
    for _ in range(2):
        res = xs - (c[0] + c[1]*ys)
        keep = np.abs(res) < max(cfg.inlier, 3*1.4826*np.median(np.abs(res)))
        if keep.sum() < cfg.min_bins:
            return None
        c = np.linalg.lstsq(np.c_[np.ones(keep.sum()), ys[keep]], xs[keep], rcond=None)[0]
    res = xs - (c[0] + c[1]*ys)
    rms = float(np.sqrt(np.mean(res[keep]**2)))
    # Line x = c0 + c1*y: its normal toward +x is (1, -c1), at angle atan2(-c1, 1).
    return Edge(distance=float(c[0]/math.hypot(1, c[1])), angle=float(math.atan2(-c[1], 1.0)), bins=int(keep.sum()),
                residual=rms)


@dataclass(frozen=True)
class DockConfig:
    v_max: float = 0.08          # m/s
    w_max: float = 0.3           # rad/s
    k_lateral: float = 3.0       # heading command per lateral error over the look-ahead
    look_ahead: float = 0.15     # m
    k_heading: float = 1.5       # rad/s per rad
    turn_first: float = math.radians(20)  # rotate in place first above this heading error
    slow_down: float = 0.15      # m before the goal where speed ramps down
    tol_position: float = 0.005  # m
    tol_heading: float = math.radians(1.0)


def dock_command(pose, goal, cfg=DockConfig()):
    """pose, goal: (x, y, yaw) in one frame; goal yaw is the travel direction (facing the table).
    Returns (v, w, done). Forward motion only; the robot follows the line through the goal
    along its yaw, then squares up on the spot."""
    gx, gy, gyaw = goal
    u = (math.cos(gyaw), math.sin(gyaw))
    dx, dy = pose[0]-gx, pose[1]-gy
    along = -(dx*u[0] + dy*u[1])                       # distance still to go
    lateral = -dx*u[1] + dy*u[0]                       # + : robot left of the line
    heading = math.atan2(math.sin(pose[2]-gyaw), math.cos(pose[2]-gyaw))
    if along <= cfg.tol_position:
        if abs(heading) <= cfg.tol_heading:
            return 0.0, 0.0, True
        return 0.0, max(-cfg.w_max, min(cfg.w_max, -cfg.k_heading*heading)), False
    target_heading = -math.atan2(cfg.k_lateral*lateral, cfg.look_ahead + along)
    error = math.atan2(math.sin(target_heading-heading), math.cos(target_heading-heading))
    w = max(-cfg.w_max, min(cfg.w_max, cfg.k_heading*error))
    if abs(error) > cfg.turn_first:
        return 0.0, w, False
    v = cfg.v_max*min(1.0, along/cfg.slow_down)*math.cos(error)
    return max(0.01, v), w, False
