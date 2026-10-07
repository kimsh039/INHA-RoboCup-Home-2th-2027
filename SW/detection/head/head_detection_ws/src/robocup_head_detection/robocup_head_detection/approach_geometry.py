"""Approach poses in front of a support surface for a target on it (pure numpy, no ROS).

At the docked pose the robot faces the chosen side of the surface squarely and the target
sits at OBJECT_OFFSET in base_link: the "close approach finished" scene the manipulation
pipeline (MuJoCo / Gazebo pick-and-place tutorials) was built and validated for. If keeping
that offset would put base_link closer than min_edge to the edge, the robot stops at min_edge
and the target is farther ahead; it must still lie within the arm's top-down reach.

    p_dock  = target - offset_x * forward - offset_y * left
    p_stage = p_dock + (d_stage - edge_distance) * n_out       (Nav2 goal)
    yaw     = atan2(-n_out)                                      (facing the surface)

Along the side the robot is kept far enough from the side's ends that its body clears the
corners (table legs); the resulting change of the target's lateral offset is reported.
"""
from dataclasses import dataclass
import math
import numpy as np

# Target in base_link at the end of the close approach, from the manipulation scene
# (SW/simulation/mujoco/config/grasp_simulation.json): cube (0.5, -0.1) - base (0.1078, -0.0216).
OBJECT_OFFSET = (0.392, -0.078)
# Top-down grasp reach ahead of base_link at table height vs lateral offset (Piper IK on the
# 2026-10-06 mount, grasp point 0.11 m along the gripper axis, axis pointing straight down).
REACH = ((0.0, 0.50), (0.15, 0.475), (0.25, 0.425))


def reach_at(lateral):
    """Farthest reachable forward distance for a lateral offset; None beyond the table."""
    lateral = abs(lateral)
    if lateral > REACH[-1][0]:
        return None
    return float(np.interp(lateral, [r[0] for r in REACH], [r[1] for r in REACH]))


@dataclass(frozen=True)
class ApproachConfig:
    object_offset: tuple = OBJECT_OFFSET
    min_edge: float = 0.08        # closest base_link-to-edge distance. Front rack posts reach
                                  # x=+0.03 at table height; chassis and lower rack (<0.45 m) fit under.
    d_stage: float = 0.60         # edge -> base_link for the Nav2 staging pose
    reach_scale: float = 1.0      # scale on REACH (tests / other arms)
    min_observed: float = 0.5     # side must be at least this much a confirmed boundary
    half_width: float = 0.30      # robot half width incl. margin, m
    corner_clearance: float = 0.08  # keep the body this far from the side's ends (legs inset)
    contain_margin: float = 0.05  # target may lie this far outside the fitted outline
    max_above: float = 0.4        # target height above the surface it stands on, m


@dataclass(frozen=True)
class Approach:
    surface_id: int
    side: int
    dock: tuple                   # (x, y, yaw)
    stage: tuple                  # (x, y, yaw)
    edge_point: tuple             # (x, y) foot of the docked base_link on the side
    edge_distance: float          # base_link -> edge at the docked pose, m
    depth: float                  # target distance inside the side, m
    target_in_base: tuple         # (forward, left) of the target at the docked pose
    offset_error: float           # distance from the requested object_offset, m
    reach_margin: float           # reach at that lateral offset - forward distance
    observed: float


def _side(corners, i):
    a, b = np.asarray(corners[i], float), np.asarray(corners[(i+1) % 4], float)
    length = float(np.linalg.norm(b-a))
    d = (b-a)/length
    return a, d, length, np.array([d[1], -d[0]])     # counter-clockwise corners: outward = right


def contains(corners, xy, margin=0.0):
    """Point inside the CCW quadrilateral grown by margin."""
    return all(float((np.asarray(xy)-a) @ n) <= margin for a, _, _, n in (_side(corners, i) for i in range(4)))


def supporting_surface(surfaces, xy, height, cfg=ApproachConfig()):
    """The surface under the target: outline contains it and the target sits on top of it."""
    under = [s for s in surfaces if contains(s.corners, xy, cfg.contain_margin)
             and s.height - 0.03 <= height <= s.height + cfg.max_above]
    return max(under, key=lambda s: s.height, default=None)


def approach_candidates(surface, surface_id, xy, cfg=ApproachConfig()):
    xy = np.asarray(xy, float)
    ox, oy = cfg.object_offset
    out = []
    for i in range(4):
        a, d, length, n = _side(surface.corners, i)
        observed = surface.edge_observed[i]
        if observed < cfg.min_observed:
            continue
        lo, hi = cfg.half_width+cfg.corner_clearance, length-cfg.half_width-cfg.corner_clearance
        if lo > hi:
            continue                                  # side shorter than the robot
        depth = -float((xy-a) @ n)
        forward, left = -n, np.array([n[1], -n[0]])   # robot axes when facing the surface
        p = xy - ox*forward - oy*left
        edge_distance = max(float((p-a) @ n), cfg.min_edge)
        s = min(max(float((p-a) @ d), lo), hi)        # body clear of the corners
        p = a + s*d + edge_distance*n
        rel = xy - p
        target = (float(rel @ forward), float(rel @ left))
        reach = reach_at(target[1])
        if reach is None or target[0] > reach*cfg.reach_scale:
            continue
        yaw = math.atan2(forward[1], forward[0])
        stage = p + (cfg.d_stage - edge_distance)*n
        out.append(Approach(surface_id, i, (float(p[0]), float(p[1]), yaw), (float(stage[0]), float(stage[1]), yaw),
                            tuple(map(float, a + s*d)), edge_distance, depth, target,
                            math.hypot(target[0]-ox, target[1]-oy), reach*cfg.reach_scale - target[0], observed))
    return sorted(out, key=lambda c: (round(c.offset_error, 3), -c.reach_margin))
