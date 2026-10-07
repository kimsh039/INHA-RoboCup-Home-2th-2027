"""Horizontal support surfaces (table / shelf tops) from a lidar cloud in a fixed frame.

Pure numpy, no ROS. Points must already be in a gravity-aligned frame (map / odom).
A surface is a height layer whose points fill a 2D area; the scan rings that cross a
wall at the same height form thin lines and are rejected by the minimum side length.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class SurfaceConfig:
    floor_z: float = 0.0          # frame z of the floor (true height = z - floor_z)
    min_height: float = 0.3       # m above the floor
    max_height: float = 1.3
    bin_size: float = 0.01        # height histogram bin, m
    layer_half: float = 0.015     # points within +-this of a peak belong to the layer
    min_layer_points: int = 150
    link_cell: float = 0.15       # XY grid joining the scan lines that cross one surface, m
    vertical_cell: float = 0.05   # XY grid for spotting vertical structures (walls, posts)
    vertical_span: float = 0.15   # a cell with returns spread over more height than this is vertical
    min_side: float = 0.25        # shorter side of a real surface, m (a ring on a wall is a thin line)
    min_fill: float = 0.5         # occupied link cells / link cells inside the fitted rectangle
    edge_band: float = 0.06       # surface points this close inside a side observe it, m
    edge_bin: float = 0.1         # side length resolution of edge_observed, m
    drop_band: tuple = (0.02, 0.4)  # look for lower returns this far outside a side, m
    drop_depth: float = 0.15      # a return this much below the surface shows the surface ends there


@dataclass(frozen=True)
class Surface:
    z: float                      # mean surface z in the input frame
    height: float                 # above the floor
    centre: tuple                 # (x, y)
    yaw: float                    # direction of the long side, rad in (-pi/2, pi/2]
    length: float                 # along yaw
    width: float
    corners: tuple                # 4 x (x, y), counter-clockwise; side i runs corners[i] -> corners[i+1]
    edge_observed: tuple          # per side: min(share reached by the top, share with lower returns outside)
    inliers: int
    residual: float               # z standard deviation, m

    def edge(self, i):
        """Side i as (start, end, midpoint, outward unit normal)."""
        a, b = np.asarray(self.corners[i]), np.asarray(self.corners[(i+1) % 4])
        d = (b-a)/np.linalg.norm(b-a)
        return a, b, (a+b)/2, np.array([d[1], -d[0]])   # CCW polygon: right-hand normal points out


def _height_peaks(h, cfg):
    bins = np.arange(cfg.min_height, cfg.max_height+cfg.bin_size, cfg.bin_size)
    hist, edges = np.histogram(h, bins=bins)
    peaks = []
    for i in np.argsort(hist)[::-1]:
        if hist[i] < cfg.min_layer_points // 3:
            break
        centre = (edges[i]+edges[i+1])/2
        if all(abs(centre-p) > 2*cfg.layer_half for p in peaks):
            peaks.append(centre)
    return peaks


def _components(cells):
    """8-connected component label for each row of an (M, 2) integer cell array."""
    index = {tuple(c): k for k, c in enumerate(cells)}
    labels = np.full(len(cells), -1)
    for start in range(len(cells)):
        if labels[start] >= 0:
            continue
        labels[start] = start
        stack = [start]
        while stack:
            i, j = cells[stack.pop()]
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    k = index.get((i+di, j+dj))
                    if k is not None and labels[k] < 0:
                        labels[k] = start
                        stack.append(k)
    return labels


def _convex_hull(p):
    """Monotone chain hull of (N, 2) points, counter-clockwise."""
    p = sorted(set(map(tuple, p)))
    if len(p) < 3:
        return np.array(p)
    def half(points):
        out = []
        for x, y in points:
            while len(out) >= 2 and ((out[-1][0]-out[-2][0])*(y-out[-2][1]) -
                                     (out[-1][1]-out[-2][1])*(x-out[-2][0])) <= 0:
                out.pop()
            out.append((x, y))
        return out
    return np.array(half(p)[:-1] + half(p[::-1])[:-1])


def min_area_rect(xy):
    """Smallest enclosing rectangle: (centre, yaw of long side, length, width, CCW corners)."""
    hull = _convex_hull(xy)
    best = None
    for k in range(len(hull)):
        e = hull[(k+1) % len(hull)]-hull[k]
        if not np.any(e):
            continue
        a = math.atan2(e[1], e[0])
        R = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
        q = hull @ R.T
        lo, hi = q.min(0), q.max(0)
        area = np.prod(hi-lo)
        if best is None or area < best[0]:
            best = (area, a, lo, hi, R)
    _, a, lo, hi, R = best
    size = hi-lo
    if size[1] > size[0]:                      # make yaw follow the long side
        a += math.pi/2
        R = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
        q = hull @ R.T
        lo, hi = q.min(0), q.max(0)
        size = hi-lo
    a = (a+math.pi/2) % math.pi - math.pi/2    # long-side direction is symmetric: keep (-pi/2, pi/2]
    if a <= -math.pi/2 + 1e-9:
        a += math.pi
    R = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
    q = hull @ R.T
    lo, hi = q.min(0), q.max(0)
    local = np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])
    corners = local @ R                         # back to the input frame, counter-clockwise
    centre = ((lo+hi)/2) @ R
    return centre, a, float(hi[0]-lo[0]), float(hi[1]-lo[1]), corners


def _edge_coverage(surface_xy, drop_xy, a, b, cfg):
    """How much side a->b is a confirmed boundary: the smaller of the share of its length the
    top reaches and the share where rays went past the surface just outside it (drop_xy: where
    rays that hit something lower crossed the surface height). Where the scan simply ends
    (range, occlusion, the surface's own shadow) no ray passes there, so that side stays
    unconfirmed. Both shares are taken separately because the robot's own arm can hide part
    of the floor in front of an edge whose top is fully seen."""
    d = b-a
    length = float(np.linalg.norm(d))
    d = d/length
    outward = np.array([d[1], -d[0]])               # corners are counter-clockwise
    bins = max(1, int(round(length/cfg.edge_bin)))

    def hit_bins(xy, lo, hi):
        rel = xy-a
        along, out = rel @ d, rel @ outward
        sel = along[(out > lo) & (out < hi) & (along >= 0) & (along <= length)]
        return set(np.clip((sel/length*bins).astype(int), 0, bins-1).tolist())

    top = hit_bins(surface_xy, -cfg.edge_band, 0.01)
    drop = hit_bins(drop_xy, *cfg.drop_band)
    return min(len(top), len(drop))/bins


def find_surfaces(points, cfg=SurfaceConfig(), origins=None):
    """points: (N, 3) array in a gravity-aligned frame; origins: optional (N, 3) sensor position
    for each point. With origins, an edge is confirmed by rays passing just outside it at the
    surface height (works close up, where the floor beyond the edge is out of view); without,
    by lower returns just outside it. Returns surfaces, largest first."""
    p = np.asarray(points, float).reshape(-1, 3)
    finite = np.all(np.isfinite(p), axis=1)
    p = p[finite]
    o = None if origins is None else np.asarray(origins, float).reshape(-1, 3)[finite]
    everything, everything_origins = p, o
    h = p[:, 2]-cfg.floor_z
    # Cells where returns stack up over a height span are walls / posts: a scan ring crossing a
    # wall at table height must not join (or be) a surface. The floor is left out of the span.
    above_floor = p[h > 0.05]
    vij, vinv = np.unique(np.floor(above_floor[:, :2]/cfg.vertical_cell).astype(int), axis=0, return_inverse=True)
    vinv = vinv.ravel()
    zmin, zmax = np.full(len(vij), np.inf), np.full(len(vij), -np.inf)
    np.minimum.at(zmin, vinv, above_floor[:, 2])
    np.maximum.at(zmax, vinv, above_floor[:, 2])
    vertical = {tuple(c) for c in vij[zmax-zmin > cfg.vertical_span]}
    keep = (h > cfg.min_height) & (h < cfg.max_height)
    p, h = p[keep], h[keep]
    surfaces = []
    for peak in _height_peaks(h, cfg):
        layer = p[np.abs(h-peak) < cfg.layer_half]
        if vertical:
            layer = layer[[tuple(c) not in vertical for c in np.floor(layer[:, :2]/cfg.vertical_cell).astype(int)]]
        if len(layer) < cfg.min_layer_points:
            continue
        cells, inverse = np.unique(np.floor(layer[:, :2]/cfg.link_cell).astype(int), axis=0, return_inverse=True)
        cell_label = _components(cells)
        point_label = cell_label[inverse.ravel()]
        for label in np.unique(cell_label):
            pts = layer[point_label == label]
            if len(pts) < cfg.min_layer_points:
                continue
            spread = np.linalg.eigvalsh(np.cov(pts[:, :2].T))
            if 2*np.sqrt(3*max(spread[0], 0)) < cfg.min_side:
                continue                         # thin line (uniform-width estimate); skip the hull
            centre, yaw, length, width, corners = min_area_rect(pts[:, :2])
            if width < cfg.min_side:
                continue                         # a ring line on a wall, not a surface
            occupied = np.count_nonzero(cell_label == label)*cfg.link_cell**2
            fill = occupied/max((length+cfg.link_cell)*(width+cfg.link_cell), 1e-6)
            if fill < cfg.min_fill:
                continue
            z = pts[:, 2].mean()
            low = everything[:, 2] < z-cfg.drop_depth
            if everything_origins is None:
                drop = everything[low, :2]
            else:                                # where each ray to a lower return crossed height z
                q, so = everything[low], everything_origins[low]
                above = so[:, 2] > z
                q, so = q[above], so[above]
                f = (so[:, 2]-z)/(so[:, 2]-q[:, 2])
                drop = so[:, :2] + (q[:, :2]-so[:, :2])*f[:, None]
            observed = tuple(_edge_coverage(pts[:, :2], drop, corners[i], corners[(i+1) % 4], cfg)
                             for i in range(4))
            surfaces.append(Surface(
                z=float(pts[:, 2].mean()), height=float(pts[:, 2].mean()-cfg.floor_z),
                centre=(float(centre[0]), float(centre[1])), yaw=float(yaw), length=length, width=width,
                corners=tuple((float(x), float(y)) for x, y in corners), edge_observed=observed,
                inliers=len(pts), residual=float(pts[:, 2].std())))
    return sorted(surfaces, key=lambda s: s.length*s.width, reverse=True)
