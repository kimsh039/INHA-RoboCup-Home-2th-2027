"""Measured droop is retained; only a bounded, inward startup prefix is allowed."""
import numpy as np
from .core import require, finite


def feedback_in_bounds(q, cfg):
    q = np.asarray(q, float)
    limits = np.asarray(cfg['joint_limits'], float)
    lower_margin = np.zeros(6)
    upper_margin = np.zeros(6)
    # [TUNE:STARTUP_DROOP] Only observed joint2/joint3 zero-boundary droop.
    lower_margin[1] = cfg.get('startup_bounds_tolerance_rad', 0.)
    upper_margin[2] = cfg.get('startup_bounds_tolerance_rad', 0.)
    return bool(q.shape == (6,) and finite(q) and
                np.all(q >= limits[:, 0]-lower_margin) and np.all(q <= limits[:, 1]+upper_margin))


def check_trajectory_bounds(points, current, cfg):
    limits = np.asarray(cfg['joint_limits'], float)
    # Numerical roundoff at a zero bound is not a physical limit excursion.
    epsilon = 1e-8
    inside = np.all((points >= limits[:, 0]-epsilon) &
                    (points <= limits[:, 1]+epsilon), axis=1)
    current_inside = np.all(current >= limits[:,0]) and np.all(current <= limits[:,1])
    if not current_inside:
        home = cfg.get('observation_joints')
        require(home is not None and np.max(np.abs(points[-1]-home)) <= cfg['goal_tolerance_rad'],
                'STARTUP_PREFIX_INITIAL_POSE_ONLY')
    if np.all(inside):
        require(feedback_in_bounds(current, cfg), 'STARTUP_DROOP_TOO_LARGE')
        return
    # MoveIt may subdivide the measured-to-bounded startup prefix. Accept
    # several samples only on that same collision-checked, inward line;
    # after entering the limits, every remaining sample must stay inside.
    entered = np.flatnonzero(inside)
    require(not inside[0] and len(entered)>0, 'TRAJECTORY_LIMITS')
    first_inside = int(entered[0])
    require(np.all(inside[first_inside:]), 'TRAJECTORY_LIMITS')
    require(feedback_in_bounds(current, cfg), 'STARTUP_DROOP_TOO_LARGE')
    require(not current_inside, 'TRAJECTORY_LIMITS')
    require(np.max(np.abs(points[0]-current)) <= cfg['goal_tolerance_rad'],
            'STARTUP_PREFIX_NOT_ACTUAL')
    prefix = np.vstack([current, points[:first_inside+1]])
    require(all(feedback_in_bounds(q,cfg) for q in prefix),'STARTUP_DROOP_TOO_LARGE')
    # All joints may move toward home. Only the two permitted startup
    # excursions may remain outside, and neither may grow along the path.
    violation = np.maximum(limits[:,0]-prefix,0.)+np.maximum(prefix-limits[:,1],0.)
    require(np.all(np.diff(violation,axis=0)<=epsilon),
            'STARTUP_PREFIX_NOT_INWARD')
    home = cfg.get('observation_joints')
    require(home is not None and np.max(np.abs(points[-1]-home)) <= cfg['goal_tolerance_rad'],
            'STARTUP_PREFIX_INITIAL_POSE_ONLY')
