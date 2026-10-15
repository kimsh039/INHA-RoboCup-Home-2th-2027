"""Cached ray/URDF intersections for the rigidly mounted physical G2.

Only endpoints inside robot geometry are removed. Rays beyond a rack box
remain observable; rejected returns stay unknown, without clearing rays.
"""
import copy
import math

import numpy as np


class ScanSelfMask:
    def __init__(self, boxes, sensor_to_base, padding=.005, minimum_range=.15,
                 frame='laser_frame', paddings=None):
        self.boxes = tuple(boxes)
        self.transform = np.asarray(sensor_to_base, dtype=float)
        self.padding, self.minimum_range, self.frame = padding, minimum_range, frame
        self.paddings = tuple(float(value) for value in paddings) if paddings is not None else (padding,)*len(self.boxes)
        if len(self.paddings)!=len(self.boxes) or any(not math.isfinite(value) or not 0.<=value<=.05 for value in self.paddings):
            raise ValueError('Invalid per-part G2 self-filter allowance')
        if (self.transform.shape != (4, 4) or not np.isfinite(self.transform).all() or
                not np.allclose(self.transform[3], [0., 0., 0., 1.]) or
                not np.allclose(self.transform[:3, :3].T @ self.transform[:3, :3], np.eye(3), atol=1e-6) or
                not math.isfinite(padding) or not 0. <= padding <= .01 or
                not math.isfinite(minimum_range) or minimum_range < 0.):
            raise ValueError('Invalid G2 self-filter geometry')
        self.key, self.entries, self.exits, self.sector_indices = None, None, None, None

    def prepare(self, angle_min, angle_increment, count, sectors):
        key = (angle_min, angle_increment, count)
        if key == self.key:
            return
        if not math.isfinite(angle_min) or not math.isfinite(angle_increment):
            raise ValueError('Invalid G2 scan angles')
        angles = angle_min+np.arange(count)*angle_increment
        directions = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(count)))
        directions = directions @ self.transform[:3, :3].T
        origin = self.transform[:3, 3]
        entries, exits = [], []
        for box, padding in zip(self.boxes, self.paddings):
            local_origin = (origin-box.transform[:3, 3]) @ box.transform[:3, :3]
            local_directions = directions @ box.transform[:3, :3]
            entry, exit = np.zeros(count), np.full(count, np.inf)
            for axis in range(3):
                half = box.half[axis]+padding
                d = local_directions[:, axis]
                parallel = np.abs(d) < 1e-12
                divisor = np.where(parallel, 1., d)
                a, b = (-half-local_origin[axis])/divisor, (half-local_origin[axis])/divisor
                lower, upper = np.minimum(a, b), np.maximum(a, b)
                lower[parallel], upper[parallel] = -np.inf, np.inf
                if abs(local_origin[axis]) > half:
                    lower[parallel], upper[parallel] = np.inf, -np.inf
                entry = np.maximum(entry, lower)
                exit = np.minimum(exit, upper)
            entries.append(entry)
            exits.append(exit)
        self.entries = np.asarray(entries).reshape(len(self.boxes), count)
        self.exits = np.asarray(exits).reshape(len(self.boxes), count)
        degrees = np.degrees(angles)
        sector = np.zeros(count, dtype=bool)
        for lower, upper in sectors:
            sector |= (degrees >= lower) & (degrees <= upper)
        self.sector_indices = sector
        self.key = key

    def apply(self, message, scale, sectors):
        if message.header.frame_id != self.frame:
            raise ValueError('Unexpected G2 frame: '+message.header.frame_id)
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError('Invalid G2 range scale')
        self.prepare(message.angle_min, message.angle_increment, len(message.ranges), sectors)
        raw = np.asarray(message.ranges, dtype=float)
        valid = np.isfinite(raw) & (raw >= message.range_min) & (raw <= message.range_max)
        ranges = raw*scale
        near = valid & (ranges <= self.minimum_range)
        inside = valid & np.any((ranges >= self.entries) & (ranges <= self.exits), axis=0)
        rejected = ~valid | near | inside | self.sector_indices
        ranges[rejected] = np.nan
        # Header, angle order, timing and intensities are immutable here.
        out = copy.copy(message)
        out.range_min, out.range_max = message.range_min*scale, message.range_max*scale
        out.ranges = ranges.tolist()
        return out, dict(input_beams=len(raw), valid_input=int(valid.sum()),
                         urdf_removed=int(inside.sum()), near_sensor_removed=int(near.sum()),
                         sector_removed=int((valid & self.sector_indices).sum()),
                         kept=int(np.isfinite(ranges).sum()))
