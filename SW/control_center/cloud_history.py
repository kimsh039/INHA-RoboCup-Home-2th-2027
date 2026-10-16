"""Bounded, voxel-filtered point-cloud history in a stable ROS frame."""
from collections import deque
import math
import time

import numpy as np


class CloudHistory:
    def __init__(self, config):
        self.window = max(0.0, min(5.0, float(config.get('cloud_decay_sec', 3.0))))
        self.voxel = max(0.0, float(config.get('cloud_voxel_size', 0.04)))
        self.maximum = max(1, int(config.get('max_accumulated_cloud_points', 120000)))
        # Rebuilding the accumulated cloud and its browser payload costs ~0.1 s; doing it for every
        # frame (5 Hz) saturated the bridge loop and starved every other channel.
        self.period = max(0.0, float(config.get('cloud_accumulated_period_sec', 0.5)))
        self.frames = {}
        self.built = {}

    def add(self, key, points, frame, source_frame, stamp, topic, total, stable):
        now = time.monotonic()
        accumulate = key == 'cloud' and stable and stamp is not None and stamp > 0 and self.window > 0
        identity = (frame, source_frame, topic)
        history = self.frames.setdefault(key, deque())
        if (not accumulate or (history and (history[-1]['identity'] != identity
                                            or stamp <= history[-1]['stamp']))):
            history.clear()
        while history and now - history[0]['received'] > self.window:
            history.popleft()
        history.append(dict(identity=identity, points=points, total=total, stamp=stamp, received=now))
        if accumulate and len(history) > 1 and now - self.built.get(key, 0.0) < self.period:
            return None, None  # stored; the next rebuild includes it
        self.built[key] = now
        # Bound raw storage as well as the final browser payload.
        stored = sum(len(item['points']) for item in history)
        while len(history) > 1 and (len(history) > 32 or stored > self.maximum * 3):
            stored -= len(history.popleft()['points'])
        combined = np.concatenate([item['points'] for item in reversed(history)], axis=0)
        if accumulate and self.voxel > 0 and len(combined):
            # One int64 key per voxel: same result as np.unique(axis=0) on the cells, about 9x faster
            # (255 -> 29 ms for 120k points on the Orin), which kept the bridge loop saturated.
            cells = np.floor(combined / self.voxel).astype(np.int64)
            cells -= cells.min(axis=0)
            span = cells.max(axis=0) + 1
            keys = (cells[:, 0] * span[1] + cells[:, 1]) * span[2] + cells[:, 2]
            _, indices = np.unique(keys, return_index=True)
            combined = combined[np.sort(indices)]
        if len(combined) > self.maximum:
            combined = combined[::math.ceil(len(combined) / self.maximum)]
        metadata = dict(total_points=sum(item['total'] for item in history), source_points=total,
                        accumulated_frames=len(history), decay_sec=self.window if accumulate else 0.0,
                        accumulation_sec=now - history[0]['received'],
                        accumulation_warning='TF 누락 · 현재 프레임만 표시' if key == 'cloud' and not stable else '')
        return combined, metadata
