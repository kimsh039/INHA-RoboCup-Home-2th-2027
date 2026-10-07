"""Piper arm kinematics from the URDF, and wrist-camera look-at poses (numpy only).

The chain is read from the URDF text (e.g. /robot_description), so the same code follows
mount changes. Poses are in base_link. A look-at pose puts the wrist camera's optical
origin at a point and its optical axis (+z) on a direction; the roll about the axis is free.
"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as E
import numpy as np


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def _origin(joint):
    T = np.eye(4)
    o = joint.find('origin')
    if o is not None:
        T[:3, :3] = _rpy(*[float(v) for v in o.get('rpy', '0 0 0').split()])
        T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
    return T


def _axis_rotation(axis, q):
    a = np.asarray(axis, float)/np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(q)*K + (1-math.cos(q))*K@K
    return T


class Chain:
    """Serial chain base -> tip with the URDF's revolute joints as variables."""

    def __init__(self, urdf_text, tip, base='base_link'):
        root = E.fromstring(urdf_text)
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        joints, link = [], tip
        while link != base:
            j = by_child[link]
            joints.append(j)
            link = j.find('parent').get('link')
        joints.reverse()
        self.static = [_origin(j) for j in joints]
        self.axes = [[float(v) for v in j.find('axis').get('xyz').split()] if j.get('type') == 'revolute' else None
                     for j in joints]
        self.links = [j.find('child').get('link') for j in joints]
        self.names = [j.get('name') for j in joints if j.get('type') == 'revolute']
        self.limits = np.array([[float(j.find('limit').get('lower')), float(j.find('limit').get('upper'))]
                                for j in joints if j.get('type') == 'revolute'])

    def frames(self, q):
        """Pose of every link along the chain (list of 4x4) for joint vector q."""
        out, T, k = [], np.eye(4), 0
        for M, axis in zip(self.static, self.axes):
            T = T @ M
            if axis is not None:
                T = T @ _axis_rotation(axis, q[k])
                k += 1
            out.append(T)
        return out

    def fk(self, q):
        return self.frames(q)[-1]


@dataclass(frozen=True)
class Table:
    """Horizontal obstacle top in base_link: CCW corners (x, y) and top z."""
    corners: tuple
    z: float

    def over(self, xy, margin):
        c = np.asarray(self.corners, float)
        for i in range(len(c)):
            a, b = c[i], c[(i+1) % len(c)]
            d = (b-a)/np.linalg.norm(b-a)
            if (np.asarray(xy)-a) @ np.array([d[1], -d[0]]) > margin:
                return False
        return True


FINGER_TIP = 0.14   # finger tips along +z of piper_gripper_base (fingers span 0.062..0.138 m)


def clearance(chain, q, table, first_link='piper_link2', margin=0.05, step=0.03):
    """Lowest height above the table top of points along the arm (from first_link to the tip,
    plus the finger tips) that lie over the table (grown by margin). +inf if none is over it."""
    frames = chain.frames(q)
    start = chain.links.index(first_link)
    pts = [f[:3, 3] for f in frames[start:]]
    segments = list(zip(pts, pts[1:]))
    if 'piper_gripper_base' in chain.links:
        g = frames[chain.links.index('piper_gripper_base')]
        segments.append((g[:3, 3], g[:3, 3] + FINGER_TIP*g[:3, 2]))
    lowest = math.inf
    for a, b in segments:
        n = max(1, int(np.linalg.norm(b-a)/step))
        for t in np.linspace(0, 1, n+1):
            p = a + (b-a)*t
            if table.over(p[:2], margin):
                lowest = min(lowest, p[2]-table.z)
    return lowest


def look_at_ik(chain, position, direction, q0, iters=150, tol=(2e-3, 1e-2)):
    """Damped least squares: camera origin -> position, optical +z -> direction (unit).
    Returns (q, position error m, direction error rad) within the joint limits."""
    direction = np.asarray(direction, float)/np.linalg.norm(direction)
    q = np.clip(np.asarray(q0, float), chain.limits[:, 0], chain.limits[:, 1])

    def residual(q):
        T = chain.fk(q)
        return np.concatenate([T[:3, 3]-position, 0.3*(T[:3, 2]-direction)])

    r = residual(q)
    for _ in range(iters):
        J = np.empty((6, len(q)))
        for k in range(len(q)):
            dq = np.zeros(len(q)); dq[k] = 1e-5
            J[:, k] = (residual(q+dq)-r)/1e-5
        step = np.linalg.solve(J.T@J + 1e-4*np.eye(len(q)), -J.T@r)
        q = np.clip(q+step, chain.limits[:, 0], chain.limits[:, 1])
        r = residual(q)
        if np.linalg.norm(r[:3]) < tol[0]*0.5 and np.linalg.norm(r[3:])/0.3 < tol[1]*0.5:
            break
    T = chain.fk(q)
    angle = math.acos(max(-1.0, min(1.0, float(T[:3, 2] @ direction))))
    return q, float(np.linalg.norm(T[:3, 3]-position)), angle


@dataclass(frozen=True)
class ObservationConfig:
    distances: tuple = (0.30, 0.35, 0.40)       # camera -> target, m (D435f depth needs >~0.28 m)
    elevations: tuple = tuple(math.radians(a) for a in (90, 75, 60, 50, 40))   # below horizontal
    min_clearance: float = 0.06                 # arm points over the table stay this high, m
    # IK starts after the current joints. joint1 ~ -pi/2 turns the arm (zero pose: toward -y)
    # to face forward; the rest spread the elbow / wrist over the reachable workspace.
    seeds: tuple = ((-1.6, 2.0, -1.3, 0.0, 1.0, 0.0), (-1.6, 1.5, -1.0, 0.0, 1.2, 0.0),
                    (-2.0, 2.0, -1.3, 0.0, 1.0, 0.0), (-1.2, 2.0, -1.3, 0.0, 1.0, 0.0))


def observation_pose(chain, target, table, q_now, cfg=ObservationConfig()):
    """Look-at pose for the wrist camera on target (base_link xyz): try the steepest view
    first (least occlusion), then lower ones. Returns (q, info) or (None, reason)."""
    target = np.asarray(target, float)
    h = target[:2]/max(np.linalg.norm(target[:2]), 1e-6)          # robot -> target, horizontal
    for elevation in cfg.elevations:
        view = np.array([h[0]*math.cos(elevation), h[1]*math.cos(elevation), -math.sin(elevation)])
        for distance in cfg.distances:
            camera = target - distance*view
            for seed in (q_now, *cfg.seeds):
                q, pos_err, ang_err = look_at_ik(chain, camera, view, seed)
                if pos_err > 2e-3 or ang_err > 1e-2:
                    continue
                low = clearance(chain, q, table)
                if low < cfg.min_clearance:
                    continue
                return q, dict(elevation=elevation, distance=distance, clearance=low,
                               position_error=pos_err, angle_error=ang_err)
    return None, 'NO_OBSERVATION_POSE'


def joint_path(chain, q_from, q_to, table, step=0.05, min_clearance=0.04):
    """Straight joint-space path sampled every `step` rad; None if it passes too low over the table."""
    q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
    n = max(1, int(np.max(np.abs(q_to-q_from))/step))
    path = [q_from + (q_to-q_from)*t for t in np.linspace(0, 1, n+1)]
    if min(clearance(chain, q, table) for q in path) < min_clearance:
        return None
    return path
