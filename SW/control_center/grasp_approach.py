"""Measured table boundaries -> staging and closest physical docking poses in odom."""
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'../robot'))
sys.path.insert(0, str(ROOT/'../detection/head/head_detection_ws/src/robocup_head_detection'))
from collision_geometry import CORNERS, RobotGeometry
from robocup_head_detection.surface_geometry import SurfaceConfig, find_surfaces
from robocup_head_detection.approach_geometry import contains
from robocup_head_detection.dock_geometry import DockConfig, EdgeConfig, blind_limit, dock_command, fit_edge


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def support_surface(clouds, target, floor_z=-.1425):
    """Require a tabletop under the object and ray evidence for its boundary."""
    if not clouds:
        return None
    point = np.asarray(target, float)
    points = np.concatenate([p for _, p, _ in clouds])
    origins = np.concatenate([np.broadcast_to(o, p.shape) for _, p, o in clouds])
    nearby = np.linalg.norm(points[:, :2]-point[:2], axis=1) < 1.8
    points, origins = points[nearby], origins[nearby]
    cfg = SurfaceConfig(floor_z=floor_z, min_height=max(.30, point[2]-floor_z-.35),
                        max_height=min(1.40, point[2]-floor_z+.025), min_layer_points=100)
    surfaces = find_surfaces(points, cfg, origins)
    surfaces = [s for s in surfaces if contains(s.corners, point[:2], .04)
                and -.025 <= point[2]-s.z <= .35 and max(s.edge_observed) >= .4]
    if not surfaces:
        return None
    chosen = max(surfaces, key=lambda s: s.z)
    return dict(z=chosen.z, corners=[list(c) for c in chosen.corners],
                edge_observed=list(chosen.edge_observed), inliers=chosen.inliers,
                residual=chosen.residual, stamp=max(s for s, _, _ in clouds))


class GraspPlanner:
    def __init__(self, settings):
        self.model = RobotGeometry()
        self.max_reach = float(settings.get('grasp_max_forward', .50))
        self.stage_edge = float(settings.get('stage_edge_distance', .65))
        self.gap_limit = float(settings.get('grasp_table_gap_limit', .03))
        self.preferred_x = float(settings.get('grasp_preferred_x', .392))
        self.preferred_y = float(settings.get('grasp_preferred_y', -.078))
        self.table_depth = float(settings.get('table_body_depth', .12))
        if not .2 <= self.max_reach <= .6 or self.stage_edge < .5:
            raise ValueError('파지 도달 거리 / Nav2 준비 거리 설정 오류')
        if not math.isfinite(self.gap_limit) or not 0 <= self.gap_limit <= .03:
            raise ValueError('파지 완료 책상 상대 간격은 0~0.03m입니다.')
        if not (.2 <= self.preferred_x <= self.max_reach and abs(self.preferred_y) <= .15
                and .04 <= self.table_depth <= .30):
            raise ValueError('파지 기준 좌표 / 책상 몸체 두께 설정 오류')
        self.edges = [(i, j) for i in range(8) for j in range(i+1, 8)
                      if np.count_nonzero(CORNERS[i] != CORNERS[j]) == 1]
        self.control = DockConfig(v_max=.06, w_max=.25, turn_first=.60,
                                  tol_position=.015, tol_heading=math.radians(2))
        self.edge_config = EdgeConfig()

    def section(self, boxes, z):
        """Exact OBB/height-slab intersections, rather than the whole XY mast envelope."""
        out = []
        lo, hi = z-self.table_depth, z+.005
        for box in boxes:
            vertices = box.vertices()
            if vertices[:, 2].max() < lo or vertices[:, 2].min() > hi:
                continue
            clipped = [p for p in vertices if lo <= p[2] <= hi]
            for i, j in self.edges:
                a, b = vertices[i], vertices[j]
                if abs(a[2]-b[2]) < 1e-10:
                    continue
                for height in (lo, hi):
                    ratio = (height-a[2])/(b[2]-a[2])
                    if 0 <= ratio <= 1:
                        clipped.append(a+ratio*(b-a))
            out.extend(clipped)
        if not out:
            raise RuntimeError('책상 높이에서 로봇의 실제 형상을 구하지 못했습니다.')
        return np.asarray(out)[:, :2]

    def plans(self, target, surface, robot, positions, clearance=0., base_z=0., include_unobserved=False):
        if not surface:
            raise RuntimeError('물체 아래 책상 평면과 실제 가장자리 LiDAR 관측 대기')
        missing = self.model.required_joints-set(positions)
        if missing:
            raise RuntimeError('파지 접근용 팔 실측 관절 대기: '+', '.join(sorted(missing)))
        boxes = self.model.boxes(positions)
        section = self.section(boxes, surface['z']-base_z)
        all_vertices = np.concatenate([box.vertices() for box in boxes])[:, :2]
        target = np.asarray(target, float)
        candidates = []
        for side, observed in enumerate(surface['edge_observed']):
            if observed < .4 and not include_unobserved:
                continue
            a, b = np.asarray(surface['corners'][side]), np.asarray(surface['corners'][(side+1) % 4])
            tangent = b-a
            length = float(np.linalg.norm(tangent))
            if length < .3:
                continue
            tangent /= length
            outward = np.array([tangent[1], -tangent[0]])
            yaw = math.atan2(-outward[1], -outward[0])
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c, -s], [s, c]])
            lateral = all_vertices @ rotation.T @ tangent
            low, high = -float(lateral.min()), length-float(lateral.max())
            if low > high:
                continue
            # Keep the base square to the face. Shift the approach line so the
            # object lies on the arm's preferred lateral coordinate instead of
            # turning the entire chassis toward a corner object.
            along = min(max(float((target[:2]-a) @ tangent)+self.preferred_y, low), high)
            edge_distance = max(0., float(np.max(section @ rotation.T @ (-outward))))+clearance
            xy = a+along*tangent+edge_distance*outward
            local = rotation.T @ (target[:2]-xy)
            distance = float(np.linalg.norm(local))
            if (distance > self.max_reach or local[0] <= 0
                    or abs(local[1]-self.preferred_y) > .06
                    or float((target[:2]-a) @ (-outward)) < -.04):
                continue
            stage = xy+max(0., self.stage_edge-edge_distance)*outward
            cost = (float(np.linalg.norm(np.asarray(robot[:2])-stage))
                    +.15*abs(wrap(yaw-robot[2]))+.5*abs(local[0]-self.preferred_x))
            candidates.append(dict(dock=[float(xy[0]), float(xy[1]), yaw],
                                   stage=[float(stage[0]), float(stage[1]), yaw],
                                   edge_distance=edge_distance, clearance=clearance,
                                   target_distance=distance, target_base=local.tolist(),
                                   side=side, observed=observed, boundary_verified=observed >= .4,
                                   edge_origin=a.tolist(), outward=outward.tolist(),
                                   surface=surface, cost=cost))
        if not candidates:
            raise RuntimeError('책상 어느 면에서도 파지 도달 범위 안의 위치를 만들 수 없습니다 · 책상 크기/물체 깊이/팔 도달 거리 확인')
        return sorted(candidates, key=lambda p: p['cost'])

    def on_approach_side(self, plan, pose):
        """Only hand over outside the chosen face, never across the tabletop."""
        signed = float((np.asarray(pose[:2])-plan['edge_origin']) @ np.asarray(plan['outward']))
        return signed >= max(0., plan['edge_distance']-.03)

    def at_staging_pose(self, plan, pose):
        stage = plan['stage']
        return (self.on_approach_side(plan, pose)
                and math.hypot(pose[0]-stage[0], pose[1]-stage[1]) <= .20)

    def in_docking_corridor(self, plan, pose):
        if not plan['boundary_verified'] or not self.on_approach_side(plan, pose):
            return False
        x, y, yaw = plan['dock']
        dx, dy = pose[0]-x, pose[1]-y
        c, s = math.cos(yaw), math.sin(yaw)
        return abs(-s*dx+c*dy) <= .12 and -.75 <= c*dx+s*dy <= .03

    def table_gap(self, pose, plan, positions, base_z=0.):
        """Gap to the selected table face at its height, independent of map XY."""
        missing = self.model.required_joints-set(positions)
        if missing:
            raise RuntimeError('책상 상대 간격용 팔 실측 관절 대기: '+', '.join(sorted(missing)))
        section = self.section(self.model.boxes(positions), plan['surface']['z']-base_z)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rotation = np.array([[c, -s], [s, c]])
        outward = np.asarray(plan['outward'])
        extent = float(np.max(section @ rotation.T @ (-outward)))
        edge = float((np.asarray(pose[:2])-plan['edge_origin']) @ outward)
        return edge-extent

    def command(self, pose, plan):
        linear, angular, done = dock_command(pose, plan['dock'], self.control)
        remaining = math.hypot(pose[0]-plan['dock'][0], pose[1]-plan['dock'][1])
        if remaining <= .15:
            # Continue into the last few centimetres with a shorter physical
            # braking trajectory, rather than repeatedly approaching at 6 cm/s.
            linear = min(linear, .02)
            angular = max(-.15, min(.15, angular))
        return linear, angular, done

    def errors(self, pose, plan):
        x, y, yaw = plan['dock']
        dx, dy = x-pose[0], y-pose[1]
        c, s = math.cos(yaw), math.sin(yaw)
        return c*dx+s*dy, -s*dx+c*dy, wrap(yaw-pose[2])

    def docked(self, pose, plan, gap):
        along, lateral, heading = self.errors(pose, plan)
        return (abs(along) <= .015 and abs(lateral) <= .02
                and abs(heading) <= self.control.tol_heading
                and gap is not None and 0 <= gap <= self.gap_limit)

    def track_edge(self, plan, target, pose, base_z, points, stamp, positions):
        """Correct only the selected face using fresh, acquisition-aligned points.

        The outline stays rigid. Fits outside the existing face's distance and
        normal gates cannot switch this request to another table or its shadow.
        """
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rotation = np.array([[c, -s], [s, c]])
        outward = np.asarray(plan['outward'])
        expected = float((np.asarray(pose[:2])-plan['edge_origin']) @ outward)
        height = plan['surface']['z']-base_z+.1425
        if expected <= blind_limit(0., height, self.edge_config)+.10:
            return None, 'blind'
        points = np.asarray(points)
        # Use points near this face, excluding other tables in the same slice.
        near = np.abs((points[:, :2]-plan['edge_origin']) @ outward) <= .12
        local = points[near].copy()
        local[:, :2] = (local[:, :2]-pose[:2]) @ rotation
        local[:, 2] -= base_z
        edge = fit_edge(local, height, -.1425, self.edge_config)
        if edge is None:
            return None, 'unseen'
        normal_yaw = pose[2]+edge.angle
        if abs(edge.distance-expected) > .08 or abs(wrap(normal_yaw-plan['dock'][2])) > math.radians(6):
            return None, 'rejected'
        # Bounded low-pass correction avoids moving the goal by a full noisy fit.
        yaw = plan['dock'][2]+.35*wrap(normal_yaw-plan['dock'][2])
        inward = np.array([math.cos(yaw), math.sin(yaw)])
        measured = np.asarray(pose[:2])+edge.distance*np.array([math.cos(normal_yaw), math.sin(normal_yaw)])
        old_origin = np.asarray(plan['edge_origin'])
        origin = old_origin+.35*float((measured-old_origin) @ inward)*inward
        # Rotate the rectangle about the selected corner, keeping its extent.
        delta = wrap(yaw-plan['dock'][2]); cd, sd = math.cos(delta), math.sin(delta)
        correction = np.array([[cd, -sd], [sd, cd]])
        surface = dict(plan['surface'])
        surface['corners'] = ((np.asarray(surface['corners'])-old_origin) @ correction.T+origin).tolist()
        surface['stamp'] = stamp
        candidates = self.plans(target, surface, pose, positions, plan['clearance'], base_z)
        updated = next((p for p in candidates if p['side'] == plan['side']), None)
        if updated is None or np.linalg.norm(np.asarray(updated['dock'][:2])-plan['dock'][:2]) > .08:
            return None, 'rejected'
        updated['edge_tracking'] = dict(state='tracked', stamp=stamp, bins=edge.bins, residual=edge.residual)
        return updated, 'tracked'
