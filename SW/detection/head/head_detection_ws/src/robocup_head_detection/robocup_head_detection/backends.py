"""Lazy YOLO loader and OpenCV CSRT/KCF adapter."""
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

from .core import Box


class YoloDetector:
    def __init__(self, model_path, confidence=0.35, nms_iou=0.45,
                 imgsz=640, device="cpu", task="detect"):
        self.model = None
        self.error = "Model path is empty; supply a trained local .pt file and restart."
        self.confidence, self.nms_iou = confidence, nms_iou
        self.imgsz, self.device = imgsz, device
        if not model_path:
            return
        path = Path(model_path).expanduser()
        if not path.is_file() or path.suffix.lower() != ".pt":
            self.error = f"Expected an existing local .pt model: {path}"
            return  # Never pass a nonexistent model name to YOLO (implicit download).
        try:
            from ultralytics import YOLO
            model = YOLO(str(path), task=task)
            if model.task != task:
                raise ValueError(f"Checkpoint task {model.task!r} != configured task {task!r}")
            model.predict(np.zeros((imgsz, imgsz, 3), np.uint8),
                          conf=confidence, iou=nms_iou, imgsz=imgsz,
                          device=device, verbose=False)
            self.model, self.error = model, ""
        except Exception as exc:
            self.error = f"Model load/warmup failed: {exc}"

    @property
    def ready(self):
        return self.model is not None

    def detect(self, frame, roi=None):
        if not self.ready:
            return []
        height, width = frame.shape[:2]
        x0, y0, x1, y1 = roi or (0, 0, width, height)
        crop = np.ascontiguousarray(frame[y0:y1, x0:x1])
        if crop.size == 0:
            return []
        result = self.model.predict(crop, conf=self.confidence, iou=self.nms_iou,
                                    imgsz=self.imgsz, device=self.device, verbose=False)[0]
        if result.boxes is None:
            return []
        # masks.xy is scaled by Ultralytics to the input crop coordinates.
        # rasterize there, then embed into the ORIGINAL RGB grid.
        polygons = result.masks.xy if result.masks is not None else None
        output = []
        for index, item in enumerate(result.boxes):
            left, top, right, bottom = item.xyxy[0].tolist()
            class_id = int(item.cls[0])
            mask = None
            if polygons is not None and index < len(polygons):
                points = np.asarray(polygons[index])
                if len(points) >= 3:
                    local_mask = np.zeros(crop.shape[:2], np.uint8)
                    cv2.fillPoly(local_mask, [np.rint(points).astype(np.int32)], 255)
                    mask = np.zeros((height, width), np.uint8)
                    mask[y0:y1, x0:x1] = local_mask
            output.append(Box(left+x0, top+y0, right-left, bottom-top,
                              class_id, str(result.names[class_id]), float(item.conf[0]), mask))
        return output


def tracker_constructor(kind):
    kind = kind.upper()
    if kind not in ("CSRT", "KCF"):
        raise ValueError("tracker_type must be CSRT or KCF")
    name = f"Tracker{kind}_create"
    for module in (cv2, getattr(cv2, "legacy", None)):
        if module is not None and hasattr(module, name):
            return getattr(module, name)
    raise RuntimeError(f"OpenCV {kind} is unavailable; install opencv-contrib-headless")


class CvTracker:
    def __init__(self, frame, box, constructor):
        self.template = replace(box, mask=None)
        self.tracker = constructor()
        rect = tuple(int(round(v)) for v in (box.x, box.y, box.width, box.height))
        if self.tracker.init(frame, rect) is False:
            raise RuntimeError("OpenCV tracker initialization failed")

    def update(self, frame):
        ok, rect = self.tracker.update(frame)
        if not ok:
            return None
        return replace(self.template, x=float(rect[0]), y=float(rect[1]),
                       width=float(rect[2]), height=float(rect[3]))


def make_tracker_factory(kind):
    constructor = tracker_constructor(kind)  # Fail at startup, not silently per frame.
    return lambda frame, box: CvTracker(frame, box, constructor)
