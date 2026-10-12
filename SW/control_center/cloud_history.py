"""Bounded, voxel-filtered point-cloud history in a stable ROS frame."""
from collections import deque
import math
import time

import numpy as np


class CloudHistory:
    def __init__(self, config):
        self.window = max(0.0, min(5.0, float(config.get('cloud_decay_sec', 2.0))))
        self.voxel = max(0.0, float(config.get('cloud_voxel_size', 0.04)))
        self.maximum = max(1, int(config.get('max_accumulated_cloud_points', 80000)))
        self.frames = {}

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
        # Bound raw storage as well as the final browser payload.
        stored = sum(len(item['points']) for item in history)
        while len(history) > 1 and (len(history) > 32 or stored > self.maximum * 3):
            stored -= len(history.popleft()['points'])
        combined = np.concatenate([item['points'] for item in reversed(history)], axis=0)
        if accumulate and self.voxel > 0 and len(combined):
            cells = np.floor(combined / self.voxel)
            _, indices = np.unique(cells, axis=0, return_index=True)
            combined = combined[np.sort(indices)]
        if len(combined) > self.maximum:
            combined = combined[::math.ceil(len(combined) / self.maximum)]
        metadata = dict(total_points=sum(item['total'] for item in history), source_points=total,
                        accumulated_frames=len(history), decay_sec=self.window if accumulate else 0.0,
                        accumulation_sec=now - history[0]['received'],
                        accumulation_warning='TF 누락 · 현재 프레임만 표시' if key == 'cloud' and not stable else '')
        return combined, metadata
