#!/usr/bin/env python3
"""Expose the current Head detector's full-frame objects and FPS to the control center."""
import array
import json
import math
from pathlib import Path
import sys
import threading
import time

# Prefer the pulled source over any old colcon installation.
sys.path.insert(0, str(Path(__file__).resolve().parent / '../detection/head/head_detection_ws/src/robocup_head_detection'))

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robocup_head_detection.backends import YoloDetector
from robocup_head_detection.class_schema import CLASSES, LEGACY_CLASSES
from robocup_head_detection.images import decode_raw
from robocup_head_detection.model_paths import current_model_path


class HeadObjects(Node):
    def __init__(self):
        super().__init__('head_detection_node')
        defaults = dict(input_topic='/head/calibrated/color/image_raw',
                        objects_topic='/detection/head/objects', debug_topic='/detection/head/debug_image',
                        model_path=str(current_model_path()), device='0', confidence=0.25,
                        cup_confidence=0.50, banana_confidence=0.50, nms_iou=0.7, imgsz=640,
                        input_timeout=1.0, max_input_age=1.0, debug_hz=6.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        get = lambda key: self.get_parameter(key).value
        if get('input_timeout') <= 0 or get('max_input_age') <= 0:
            raise ValueError('input_timeout/max_input_age must be positive')
        if not 0 < get('confidence') <= 1 or not 0 < get('nms_iou') <= 1:
            raise ValueError('confidence/nms_iou must be in (0, 1]')
        if get('imgsz') < 32 or get('imgsz') % 32:
            raise ValueError('imgsz must be a positive multiple of 32')
        self.timeout, self.max_age = get('input_timeout'), get('max_input_age')
        # Objects go out for every frame; the annotated image only for viewers (control center shows ~6 Hz).
        self.debug_period = 1.0 / get('debug_hz') if get('debug_hz') > 0 else 0.0
        self.last_debug = 0.0
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.objects_pub = self.create_publisher(String, get('objects_topic'), 10)
        self.debug_pub = self.create_publisher(Image, get('debug_topic'), image_qos)
        self.incoming = threading.Condition()
        self.latest = None
        self.closed = threading.Event()
        self.last_received = None
        self.received = self.processed = 0
        self.previous_received = self.previous_processed = 0
        self.camera_fps = self.detection_fps = 0.0
        self.last_report = time.monotonic()
        self.error = ''
        self.detector = YoloDetector(get('model_path'), get('confidence'), get('nms_iou'),
                                     get('imgsz'), get('device'), 'detect',
                                     cup_confidence=get('cup_confidence'), banana_confidence=get('banana_confidence'))
        if self.detector.ready:
            names = tuple(self.detector.model.names[i] for i in range(len(self.detector.model.names)))
            if names not in (CLASSES, LEGACY_CLASSES):
                raise ValueError(f'Expected Head six-class or legacy checkpoint, got {names}')
        else:
            self.get_logger().error(self.detector.error)
        self.create_subscription(Image, get('input_topic'), self.receive, image_qos)
        self.create_timer(1.0, self.report)
        self.worker = threading.Thread(target=self.process, name='head-yolo', daemon=True)
        self.worker.start()
        self.get_logger().info(f'Head 전체 객체 검출: {get("input_topic")} -> {get("objects_topic")} · {", ".join(CLASSES)}')
        self.get_logger().info(f'Model: {get("model_path")} | full-frame YOLO; no ROI/tracking')

    def receive(self, msg):
        with self.incoming:
            self.latest = msg
            self.last_received = time.monotonic()
            self.received += 1
            self.incoming.notify()

    def publish(self, state, reason='', msg=None, objects=None, processing_ms=None, processing_total_ms=None):
        stamp = None if msg is None else msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        output = dict(mode='all_objects', classes=list(CLASSES), state=state, reason=reason,
                      valid=state == 'DETECTED', measured=state in ('DETECTED', 'SEARCH'),
                      objects=objects or [], stamp=stamp, frame=msg.header.frame_id if msg else '',
                      width=msg.width if msg else None, height=msg.height if msg else None,
                      processing_ms=processing_ms, processing_total_ms=processing_total_ms,
                      camera_fps=self.camera_fps, detection_fps=self.detection_fps)
        self.objects_pub.publish(String(data=json.dumps(output, allow_nan=False)))

    def report(self):
        now = time.monotonic()
        duration = max(0.001, now - self.last_report)
        with self.incoming:
            self.camera_fps = (self.received - self.previous_received) / duration
            self.detection_fps = (self.processed - self.previous_processed) / duration
            self.previous_received, self.previous_processed = self.received, self.processed
            last_received = self.last_received
        self.last_report = now
        if not self.detector.ready:
            self.publish('WAITING_MODEL', self.detector.error)
        elif last_received is None or now - last_received > self.timeout:
            self.publish('WAITING_IMAGE', 'INPUT_TIMEOUT')
        self.get_logger().info(f'Camera FPS={self.camera_fps:.1f} | Detection FPS={self.detection_fps:.1f}')

    def process(self):
        while not self.closed.is_set():
            with self.incoming:
                self.incoming.wait_for(lambda: self.latest is not None or self.closed.is_set(), timeout=0.2)
                msg, self.latest = self.latest, None
            if msg is None or self.closed.is_set() or not self.detector.ready:
                continue
            start = time.perf_counter()
            try:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                age = self.get_clock().now().nanoseconds / 1e9 - stamp
                if stamp <= 0 or not -self.max_age <= age <= self.max_age:
                    self.publish('STALE', 'STALE_OR_FUTURE_IMAGE', msg)
                    continue
                frame = decode_raw(msg).copy()
                before = time.perf_counter()
                boxes = self.detector.detect(frame)
                processing_ms = (time.perf_counter() - before) * 1000
                with self.incoming:
                    self.processed += 1
                objects = []
                for box in boxes:
                    if not all(math.isfinite(v) for v in (box.x, box.y, box.width, box.height, box.confidence)):
                        continue
                    objects.append(dict(label=box.class_name, class_id=box.class_id, confidence=box.confidence,
                                        x=box.x, y=box.y, width=box.width, height=box.height))
                    x, y, right, bottom = [round(v) for v in
                                          (box.x, box.y, box.x + box.width, box.y + box.height)]
                    cv2.rectangle(frame, (x, y), (right, bottom), (40, 220, 90), 2)
                    cv2.putText(frame, f'{box.class_name} {box.confidence:.2f}', (max(0, x), max(18, y - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 220, 90), 1)
                age = self.get_clock().now().nanoseconds / 1e9 - stamp
                if age > self.max_age:
                    self.publish('STALE', 'RESULT_TOO_OLD', msg, processing_ms=processing_ms)
                    continue
                if self.closed.is_set():
                    return
                self.publish('DETECTED' if objects else 'SEARCH', msg=msg, objects=objects,
                             processing_ms=processing_ms, processing_total_ms=(time.perf_counter() - start) * 1000)
                if self.debug_pub.get_subscription_count() and time.monotonic() - self.last_debug >= self.debug_period:
                    self.last_debug = time.monotonic()
                    cv2.putText(frame, f'Camera {self.camera_fps:.1f} FPS | YOLO {self.detection_fps:.1f} FPS',
                                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                    image = Image()
                    image.header, image.encoding = msg.header, 'bgr8'
                    image.height, image.width = frame.shape[:2]
                    # array.array skips rclpy's per-element uint8 check on bytes (about 175 ms per 640x480 frame on the Orin).
                    image.step, image.is_bigendian, image.data = image.width * 3, 0, array.array('B', frame.tobytes())
                    self.debug_pub.publish(image)
                self.error = ''
            except Exception as exc:
                if self.closed.is_set() or not rclpy.ok():
                    return
                reason = str(exc)
                if reason != self.error:
                    self.get_logger().error(reason)
                    self.error = reason
                self.publish('ERROR', reason, msg)

    def close(self):
        self.closed.set()
        with self.incoming:
            self.incoming.notify_all()
        self.worker.join(timeout=10)


def main():
    rclpy.init()
    node = None
    try:
        node = HeadObjects()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
