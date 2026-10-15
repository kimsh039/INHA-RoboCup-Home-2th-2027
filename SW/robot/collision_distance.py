"""Batched exact OBB distances with the existing KD-tree broad phase."""
import math
import time

import numpy as np
from scipy.spatial import cKDTree


def build_scene(points):
    # Rebuilt at sensor rate: favor cheap midpoint construction. Exact queries
    # still visit every required branch (eps=0); no occupied points are removed.
    return cKDTree(points, leafsize=32, compact_nodes=False, balanced_tree=False)


class ComputationExpired(Exception):
    pass


class PreparedBoxes:
    def __init__(self, boxes):
        self.boxes = tuple(boxes)
        self.centers = np.array([b.transform[:3, 3] for b in self.boxes]).reshape(-1, 3)
        self.rotations = np.array([b.transform[:3, :3] for b in self.boxes]).reshape(-1, 3, 3)
        self.half = np.array([b.half for b in self.boxes]).reshape(-1, 3)
        self.radii = np.linalg.norm(self.half, axis=1)
        self.extent = max((float(np.linalg.norm(b.vertices()[:, :2], axis=1).max())
                           for b in self.boxes), default=0.)

    def __iter__(self):
        return iter(self.boxes)

    def nearest(self, scene, pose, radius, deadline, from_pose=None, sources=None):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        centers = self.centers @ rotation.T+[pose[0], pose[1], 0.]
        rotations = rotation @ self.rotations
        if from_pose is not None:
            c0, s0 = math.cos(from_pose[2]), math.sin(from_pose[2])
            from_rotation = np.array([[c0, -s0, 0.], [s0, c0, 0.], [0., 0., 1.]])
            from_centers = self.centers @ from_rotation.T+[from_pose[0], from_pose[1], 0.]
            from_rotations = from_rotation @ self.rotations
        best, best_point, best_box = float('inf'), None, None
        # Bound temporary query lists and preserve deadline checks between batches.
        for start in range(0, len(self.boxes), 8):
            if time.monotonic() > deadline:
                raise ComputationExpired()
            end = min(start+8, len(self.boxes))
            matches = scene.query_ball_point(centers[start:end], self.radii[start:end]+radius,
                                             eps=0., workers=1, return_sorted=False)
            if time.monotonic() > deadline:
                raise ComputationExpired()
            lengths = np.fromiter((len(ids) for ids in matches), dtype=np.intp, count=end-start)
            if not lengths.any():
                continue
            indices = np.concatenate([np.asarray(ids, dtype=np.intp) for ids in matches if len(ids)])
            owners = np.repeat(np.arange(start, end), lengths)
            for offset in range(0, len(indices), 4096):
                if time.monotonic() > deadline:
                    raise ComputationExpired()
                chosen, owner = indices[offset:offset+4096], owners[offset:offset+4096]
                local = np.einsum('ni,nij->nj', scene.data[chosen]-centers[owner], rotations[owner])
                distances = np.linalg.norm(np.maximum(np.abs(local)-self.half[owner], 0.), axis=1)
                if from_pose is not None:
                    source = scene.data[chosen] if sources is None else sources[chosen]
                    before = np.abs(np.einsum('ni,nij->nj', source-from_centers[owner], from_rotations[owner]))-self.half[owner]
                    after = np.abs(np.einsum('ni,nij->nj', source-centers[owner], rotations[owner]))-self.half[owner]
                    before = np.linalg.norm(np.maximum(before, 0.), axis=1)+np.minimum(before.max(axis=1), 0.)
                    after = np.linalg.norm(np.maximum(after, 0.), axis=1)+np.minimum(after.max(axis=1), 0.)
                    distances = np.where(after < before-1e-9, distances, np.inf)
                index = int(np.argmin(distances))
                if distances[index] < best:
                    best, best_point, best_box = float(distances[index]), int(chosen[index]), int(owner[index])
                if best == 0.:
                    # Distances are nonnegative; later boxes cannot improve an
                    # exact contact result. Deadline still applies to this batch.
                    if time.monotonic() > deadline:
                        raise ComputationExpired()
                    return best, best_point, best_box
        if time.monotonic() > deadline:
            raise ComputationExpired()
        return best, best_point, best_box
