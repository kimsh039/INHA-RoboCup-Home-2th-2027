"""Pure control for the isolated Tracer object-follow experiment (no ROS)."""
from dataclasses import dataclass
import math

import numpy as np
from robocup_head_detection.core import iou


@dataclass(frozen=True)
class Settings:
    target: str = 'banana'
    distance: float = 1.0
    min_distance: float = 0.45
    max_distance: float = 3.0
    linear_max: float = 0.08
    angular_max: float = 0.25
    linear_gain: float = 0.25
    angular_gain: float = 0.6
    distance_deadband: float = 0.12
    center_deadband: float = 0.08
    turn_first: float = 0.35
    confidence: float = 0.5
    confirm_frames: int = 3
    allow_reverse: bool = False

    def __post_init__(self):
        numbers = [v for v in vars(self).values() if isinstance(v, (float, int)) and not isinstance(v, bool)]
        if not all(math.isfinite(v) and v > 0 for v in numbers):
            raise ValueError('Numeric settings must be finite and positive')
        if not self.min_distance < self.distance < self.max_distance:
            raise ValueError('Require min_distance < distance < max_distance')
        if not 0 < self.confidence <= 1 or not 0 < self.center_deadband < self.turn_first < 1:
            raise ValueError('Invalid confidence or steering deadbands')
        if (self.linear_max > 0.2 or self.angular_max > 0.6 or
                isinstance(self.confirm_frames, bool) or not isinstance(self.confirm_frames, int)):
            raise ValueError('Test limits: linear <= 0.2 m/s, angular <= 0.6 rad/s; integer confirmations')


@dataclass(frozen=True)
class Decision:
    linear: float = 0.0
    angular: float = 0.0
    reason: str = 'NO_TARGET'
    box: object = None
    distance: object = None


def decode_depth(msg):
    formats = {'16UC1': ('u2', 2, 0.001), '32FC1': ('f4', 4, 1.0)}
    if msg.encoding not in formats:
        raise ValueError('Aligned depth must be 16UC1 (mm) or 32FC1 (m)')
    kind, size, scale = formats[msg.encoding]
    if (msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * size or
            len(msg.data) != msg.height * msg.step):
        raise ValueError('Invalid aligned-depth dimensions or stride')
    dtype = ('>' if msg.is_bigendian else '<') + kind
    array = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.step, size))
    return array.astype(np.float32) * scale


def object_depth(depth, box):
    # The inner 40% reduces bbox-edge background; still an estimate, not segmentation.
    x0, y0 = math.floor(box.x + box.width * 0.3), math.floor(box.y + box.height * 0.3)
    x1, y1 = math.ceil(box.x + box.width * 0.7), math.ceil(box.y + box.height * 0.7)
    patch = depth[max(0, y0):min(depth.shape[0], y1), max(0, x0):min(depth.shape[1], x1)]
    values = patch[np.isfinite(patch) & (patch >= 0.15) & (patch <= 8.0)]
    if len(values) < 9 or len(values) < patch.size * 0.5:
        return None
    if np.percentile(values, 90) - np.percentile(values, 10) > 0.35:
        return None  # Mixed target/background depth cannot authorize motion.
    return float(np.median(values))


class Controller:
    def __init__(self, settings=Settings()):
        self.cfg = settings
        self.reset()

    def reset(self):
        self.previous = None
        self.streak = 0

    def update(self, boxes, depth, width, height):
        candidates = [b.clipped(width, height) for b in boxes
                      if b.class_name == self.cfg.target and b.confidence >= self.cfg.confidence]
        candidates = [b for b in candidates if b is not None]
        if not candidates:
            self.reset()
            return Decision()
        matches = [b for b in candidates if self.previous and iou(b, self.previous) >= 0.1]
        if matches:
            box = max(matches, key=lambda b: iou(b, self.previous))
            self.streak += 1
        else:
            box = max(candidates, key=lambda b: b.confidence)
            self.streak = 1
        self.previous = box
        if depth is None or depth.shape != (height, width):
            self.reset()
            return Decision(reason='NO_ALIGNED_DEPTH', box=box)
        distance = object_depth(depth, box)
        if distance is None:
            self.reset()
            return Decision(reason='INVALID_DEPTH', box=box)
        if not self.cfg.min_distance <= distance <= self.cfg.max_distance:
            self.reset()
            return Decision(reason='DISTANCE_OUT_OF_RANGE', box=box, distance=distance)
        if self.streak < self.cfg.confirm_frames:
            return Decision(reason='CONFIRMING', box=box, distance=distance)
        # ROS angular.z positive turns LEFT; image x positive is RIGHT.
        error = (box.x + box.width / 2 - width / 2) / (width / 2)
        angular = 0.0 if abs(error) < self.cfg.center_deadband else max(
            -self.cfg.angular_max, min(self.cfg.angular_max, -self.cfg.angular_gain * error))
        delta = distance - self.cfg.distance
        linear = 0.0 if abs(delta) < self.cfg.distance_deadband else max(
            -self.cfg.linear_max * 0.5, min(self.cfg.linear_max, self.cfg.linear_gain * delta))
        if not self.cfg.allow_reverse:
            linear = max(0.0, linear)
        if abs(error) >= self.cfg.turn_first:
            linear = 0.0
        return Decision(linear, angular, 'FOLLOW', box, distance)


def fresh_command(decision, observed_at, now, timeout=0.3, stopped=False):
    """Called independently of inference; old camera evidence must stop the base."""
    age = now - observed_at
    if (stopped or not math.isfinite(age) or not 0 <= age <= timeout or
            not math.isfinite(decision.linear) or not math.isfinite(decision.angular)):
        return Decision(reason='STOPPED' if stopped else 'STALE_TARGET')
    return decision
