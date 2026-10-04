"""ROS/YOLO-independent detect-then-track state machine.

Inspired by the earlier ARMS ACQUIRE/TRACK/LOST architecture (before
45af414). No ARMS code or red-balloon HSV assumptions are copied here.
All boxes use original image pixels. Time is the source image timestamp.
"""
from dataclasses import dataclass, replace
import math
from typing import Any, Callable, Optional


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    width: float
    height: float
    class_id: int
    class_name: str
    confidence: float
    mask: Any = None

    def clipped(self, width: int, height: int):
        values = (self.x, self.y, self.width, self.height, self.confidence)
        if not all(math.isfinite(v) for v in values):
            return None
        if self.width <= 0 or self.height <= 0:
            return None
        x0, y0 = max(0.0, self.x), max(0.0, self.y)
        x1, y1 = min(float(width), self.x + self.width), min(float(height), self.y + self.height)
        if x1 - x0 < 2 or y1 - y0 < 2:
            return None
        return replace(self, x=x0, y=y0, width=x1-x0, height=y1-y0)


def iou(a: Box, b: Box) -> float:
    overlap = max(0.0, min(a.x+a.width, b.x+b.width)-max(a.x, b.x)) * max(
        0.0, min(a.y+a.height, b.y+b.height)-max(a.y, b.y))
    union = a.width*a.height + b.width*b.height - overlap
    return overlap / union if union > 0 else 0.0


def roi_bounds(box: Box, width: int, height: int, margin: float):
    cx, cy = box.x + box.width/2, box.y + box.height/2
    return (
        max(0, math.floor(cx-box.width*margin/2)),
        max(0, math.floor(cy-box.height*margin/2)),
        min(width, math.ceil(cx+box.width*margin/2)),
        min(height, math.ceil(cy+box.height*margin/2)),
    )


@dataclass(frozen=True)
class Config:
    target_class: str = ""
    confirm_frames: int = 3
    redetect_interval: int = 5
    roi_margin: float = 2.0
    match_iou: float = 0.25
    max_unconfirmed: int = 2
    max_verification_age: float = 0.5
    max_frame_gap: float = 0.5

    def __post_init__(self):
        for name in ("confirm_frames", "redetect_interval", "max_unconfirmed"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if not math.isfinite(self.roi_margin) or self.roi_margin < 1:
            raise ValueError("roi_margin must be finite and >= 1")
        if not math.isfinite(self.match_iou) or not 0 < self.match_iou <= 1:
            raise ValueError("match_iou must be in (0, 1]")
        for name in ("max_verification_age", "max_frame_gap"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and > 0")


@dataclass(frozen=True)
class Observation:
    state: str
    reason: str = ""
    box: Optional[Box] = None
    target_id: str = ""
    valid: bool = False
    measured: bool = False
    last_verified: float = 0.0


class HeadPipeline:
    """detector.detect(frame, roi=None) returns full-image-coordinate Boxes.

    tracker_factory(frame, box) returns an object with update(frame)->Box|None.
    IDs are process-local; the ROS adapter adds a session UUID.
    """

    def __init__(self, detector, tracker_factory: Callable, config: Config):
        self.detector, self.tracker_factory, self.cfg = detector, tracker_factory, config
        self.counter = 0
        self.reset()

    def reset(self):
        self.state = "SEARCH"
        self.box = None
        self.tracker = None
        self.target_id = ""
        self.streak = self.frames = self.unconfirmed = 0
        self.last_verified = 0.0
        self.previous_stamp = None
        self.shape = None

    def _matches(self, candidate, reference):
        return candidate.class_id == reference.class_id and iou(candidate, reference) >= self.cfg.match_iou

    def _detect(self, frame, roi=None):
        height, width = frame.shape[:2]
        boxes = [b.clipped(width, height) for b in self.detector.detect(frame, roi)]
        return [b for b in boxes if b is not None and (
            not self.cfg.target_class or b.class_name == self.cfg.target_class)]

    def _lost(self, reason):
        result = Observation("LOST", reason, target_id=self.target_id,
                             last_verified=self.last_verified)
        self.reset()
        return result

    def step(self, frame, stamp: float):
        if not math.isfinite(stamp) or stamp <= 0:
            return self._lost("INVALID_TIMESTAMP")
        if self.previous_stamp is not None and stamp <= self.previous_stamp:
            return self._lost("NON_MONOTONIC_TIMESTAMP")
        if self.previous_stamp is not None and (
            stamp-self.previous_stamp > self.cfg.max_frame_gap or frame.shape[:2] != self.shape
        ):
            self.reset()  # Camera gap/resolution change invalidates the CV tracker.
        self.previous_stamp, self.shape = stamp, frame.shape[:2]
        if not self.detector.ready:
            self.reset()
            return Observation("WAITING_MODEL", "MODEL_NOT_READY")
        try:
            return self._track(frame, stamp) if self.tracker else self._search(frame, stamp)
        except Exception as exc:
            return self._lost(f"PROCESSING_ERROR:{type(exc).__name__}:{exc}")

    def _search(self, frame, stamp):
        candidates = self._detect(frame)
        if not candidates:
            self.box = None
            self.streak = 0
            return Observation("SEARCH", "TARGET_NOT_FOUND")
        matched = [b for b in candidates if self.box and self._matches(b, self.box)]
        if matched:
            chosen = max(matched, key=lambda b: iou(b, self.box))
            self.streak += 1
        else:
            chosen = max(candidates, key=lambda b: b.confidence)
            self.streak = 1
            self.counter += 1
            self.target_id = str(self.counter)
        self.box, self.last_verified = chosen, stamp
        if self.streak < self.cfg.confirm_frames:
            return Observation("CONFIRMING", "", chosen, self.target_id, False, True, stamp)
        self.tracker = self.tracker_factory(frame, chosen)
        if self.tracker is None:
            return self._lost("TRACKER_INIT_FAILED")
        self.state = "TRACK"
        self.frames = self.unconfirmed = 0
        return Observation("TRACK", "", chosen, self.target_id, True, True, stamp)

    def _track(self, frame, stamp):
        tracked = self.tracker.update(frame)
        height, width = frame.shape[:2]
        tracked = tracked.clipped(width, height) if tracked else None
        if tracked is None:
            return self._lost("TRACKER_FAILED")
        # No old segmentation mask is attached to a tracker-only observation.
        tracked = replace(tracked, mask=None)
        self.frames += 1
        age = stamp-self.last_verified
        due = self.frames >= self.cfg.redetect_interval or age >= self.cfg.max_verification_age
        if due:
            self.frames = 0
            roi = roi_bounds(tracked, width, height, self.cfg.roi_margin)
            matched = [b for b in self._detect(frame, roi) if self._matches(b, tracked)]
            if matched:
                # Geometry first: a higher-confidence neighbour must not steal the target.
                chosen = max(matched, key=lambda b: (iou(b, tracked), b.confidence))
                tracker = self.tracker_factory(frame, chosen)
                if tracker is None:
                    return self._lost("TRACKER_REINIT_FAILED")
                self.tracker, self.box = tracker, chosen
                self.last_verified, self.unconfirmed = stamp, 0
                return Observation("TRACK", "", chosen, self.target_id, True, True, stamp)
            self.unconfirmed += 1
            if self.unconfirmed >= self.cfg.max_unconfirmed or age >= self.cfg.max_verification_age:
                return self._lost("YOLO_VERIFICATION_FAILED")
        self.box = tracked
        reason = "ROI_UNCONFIRMED" if self.unconfirmed else ""
        return Observation("TRACK", reason, tracked, self.target_id, True, False, self.last_verified)
