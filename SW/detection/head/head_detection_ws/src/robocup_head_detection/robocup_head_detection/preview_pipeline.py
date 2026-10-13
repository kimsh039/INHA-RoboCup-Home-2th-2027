"""Optional low-rate full-frame YOLO with short-lived OpenCV KCF boxes.

This is a display pipeline: tracker results never become robot motion commands.
Every YOLO pass replaces all boxes. Failed or expired tracks disappear.
"""
import math


class PreviewPipeline:
    def __init__(self, detector, tracker_factory=None, yolo_hz=0.0, max_age=0.5):
        if not math.isfinite(yolo_hz) or yolo_hz < 0:
            raise ValueError('yolo_hz must be finite and >= 0')
        if yolo_hz and (tracker_factory is None or 1 / yolo_hz > max_age):
            raise ValueError('Tracking requires a factory and a YOLO period <= max_age')
        self.detector, self.tracker_factory = detector, tracker_factory
        self.period = 1 / yolo_hz if yolo_hz else 0
        self.max_age = max_age
        self.last_yolo = None
        self.verified_stamp = None
        self.previous_stamp = None
        self.shape = None
        self.tracks = []

    def step(self, frame, stamp, now):
        discontinuity = (self.previous_stamp is not None and
                         (stamp <= self.previous_stamp or stamp - self.previous_stamp > self.max_age
                          or self.shape != frame.shape[:2]))
        if discontinuity:
            self.tracks = []
            self.last_yolo = None
        self.previous_stamp, self.shape = stamp, frame.shape[:2]
        due = self.last_yolo is None or not self.period or now - self.last_yolo >= self.period
        if due:
            self.last_yolo = now
            boxes = [box.clipped(frame.shape[1], frame.shape[0]) for box in self.detector.detect(frame)]
            boxes = [box for box in boxes if box is not None]
            self.verified_stamp = stamp
            self.tracks = []
            if self.period:
                for box in boxes:
                    tracker = self.tracker_factory(frame, box)
                    if tracker is not None:
                        self.tracks.append(tracker)
            return boxes, True
        if self.verified_stamp is None or stamp - self.verified_stamp > self.max_age:
            self.tracks = []
            return [], False
        boxes, kept = [], []
        for tracker in self.tracks:
            box = tracker.update(frame)
            box = box.clipped(frame.shape[1], frame.shape[0]) if box is not None else None
            if box is not None:
                boxes.append(box)
                kept.append(tracker)
        self.tracks = kept
        return boxes, False
