"""Global pose hypotheses for SLAM Toolbox; no ROS, TF or motion commands.

Endpoint likelihood fields are searched over known free space and all headings.
Distinct competing hypotheses remain visible instead of accepting the first fit.
"""
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.optimize import differential_evolution
import yaml


def angular_distance(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


class MapMatcher:
    def __init__(self, cells, resolution, origin, settings):
        self.cells = np.asarray(cells)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.settings = settings
        self.height, self.width = self.cells.shape
        if self.resolution <= 0 or not np.any(self.cells >= 65) or not np.any(self.cells == 0):
            raise ValueError('Map needs occupied and known free cells')
        self.distance = distance_transform_edt(self.cells < 65) * self.resolution

    @classmethod
    def load(cls, yaml_path, settings):
        yaml_path = Path(yaml_path)
        meta = yaml.safe_load(yaml_path.read_text())
        pixels = np.asarray(Image.open(yaml_path.parent / meta['image']).convert('L'))
        probability = pixels / 255. if meta.get('negate', 0) else (255.-pixels)/255.
        cells = np.full(pixels.shape, -1, dtype=np.int16)
        cells[probability < meta['free_thresh']] = 0
        cells[probability > meta['occupied_thresh']] = 100
        return cls(np.flipud(cells), meta['resolution'], meta['origin'], settings)

    def world_to_grid(self, x, y):
        dx, dy = x-self.origin[0], y-self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return (np.floor((c*dx+s*dy)/self.resolution).astype(int),
                np.floor((-s*dx+c*dy)/self.resolution).astype(int))

    def errors(self, poses, points):
        poses = np.atleast_2d(poses)
        c, s = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])
        x = c*points[:, 0]-s*points[:, 1]+poses[:, 0, None]
        y = s*points[:, 0]+c*points[:, 1]+poses[:, 1, None]
        ix, iy = self.world_to_grid(x, y)
        inside = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        return np.where(inside, self.distance[np.clip(iy, 0, self.height-1),
                                             np.clip(ix, 0, self.width-1)], 2.)

    def score(self, poses, points):
        d = self.errors(poses, points)
        return np.mean(np.exp(-.5*(d/self.settings['sigma'])**2), axis=1)

    def metrics(self, pose, points):
        distance = self.errors(pose, points)[0]
        return dict(score=float(self.score(pose, points)[0]),
                    matched_fraction=float(np.mean(distance <= self.settings['match_distance'])))

    def search(self, points, cancelled=lambda: False):
        points = np.asarray(points, dtype=float)
        if len(points) > self.settings['max_beams']:
            points = points[np.linspace(0, len(points)-1, self.settings['max_beams']).astype(int)]
        step = self.settings['translation_step']
        gx, gy = np.meshgrid(np.arange(step/2, self.width*self.resolution, step),
                             np.arange(step/2, self.height*self.resolution, step))
        ix, iy = np.floor(gx/self.resolution).astype(int), np.floor(gy/self.resolution).astype(int)
        good = self.cells[iy, ix] == 0
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        xy = np.c_[c*gx[good]-s*gy[good]+self.origin[0],
                   s*gx[good]+c*gy[good]+self.origin[1]]
        if not len(xy):
            raise ValueError('Map has no free candidate positions at the configured search spacing')
        pool = []
        for angle in np.arange(-math.pi, math.pi, self.settings['heading_step']):
            for begin in range(0, len(xy), 1024):
                if cancelled():
                    raise RuntimeError('Global scan search cancelled')
                poses = np.c_[xy[begin:begin+1024], np.full(min(1024, len(xy)-begin), angle)]
                scores = self.score(poses, points)
                for i in np.argsort(scores)[-4:]:
                    pool.append((float(scores[i]), poses[i]))
        pool.sort(key=lambda item: -item[0])
        seeds = []
        for _, pose in pool:
            if all(np.linalg.norm(pose[:2]-p[:2]) > .5 or angular_distance(pose[2], p[2]) > .3
                   for p in seeds):
                seeds.append(pose)
            if len(seeds) >= self.settings['refine_candidates']:
                break
        fits = []
        for seed in seeds:
            if cancelled():
                raise RuntimeError('Global scan search cancelled')
            fit = differential_evolution(
                lambda p: -float(self.score(p, points)[0]),
                [(seed[0]-step, seed[0]+step), (seed[1]-step, seed[1]+step),
                 (seed[2]-self.settings['heading_step'], seed[2]+self.settings['heading_step'])],
                seed=0, popsize=8, maxiter=45, tol=.001, polish=False,
                callback=lambda *_, **__: cancelled())
            pose = fit.x
            ix, iy = self.world_to_grid(pose[0], pose[1])
            if not (0 <= ix < self.width and 0 <= iy < self.height and self.cells[iy, ix] == 0):
                continue
            pose[2] = math.atan2(math.sin(pose[2]), math.cos(pose[2]))
            fits.append(dict(pose=pose.tolist(), **self.metrics(pose, points)))
        fits.sort(key=lambda item: -item['score'])
        distinct = []
        for fit in fits:
            pose = np.asarray(fit['pose'])
            if all(np.linalg.norm(pose[:2]-np.asarray(other['pose'])[:2]) > .5 or
                   angular_distance(pose[2], other['pose'][2]) > .3 for other in distinct):
                distinct.append(fit)
        if not distinct:
            raise ValueError('No refined hypothesis lies in known free space')
        best = distinct[0]
        margin = best['score']-distinct[1]['score'] if len(distinct) > 1 else best['score']
        accepted = (best['score'] >= self.settings['minimum_score'] and
                    best['matched_fraction'] >= self.settings['minimum_matched_fraction'] and
                    margin >= self.settings['minimum_score_margin'])
        return dict(accepted=accepted, best=best, margin=float(margin), candidates=distinct)
