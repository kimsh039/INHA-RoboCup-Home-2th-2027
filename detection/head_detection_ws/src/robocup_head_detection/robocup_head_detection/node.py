"""ROS 2 adapter: source timestamps, verified masks and input-loss watchdog."""
import os
import time
import uuid

import cv2
import rclpy
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
from robocup_detection_msgs.msg import HeadTarget

from .backends import YoloDetector, make_tracker_factory
from .core import Config, HeadPipeline, Observation, roi_bounds
from .images import decode_compressed, decode_raw


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class HeadDetectionNode(Node):
    def __init__(self):
        super().__init__("head_detection_node")
        defaults = {
            "model_path": os.environ.get("ROBOCUP_MODEL", ""),
            "device": os.environ.get("ROBOCUP_DEVICE", "cpu"),
            "task": "detect", "confidence": 0.35, "nms_iou": 0.45, "imgsz": 640,
            "target_class": "", "tracker_type": "CSRT", "confirm_frames": 3,
            "redetect_interval": 5, "roi_margin": 2.0, "match_iou": 0.25,
            "max_unconfirmed": 2, "max_verification_age": 0.5, "max_frame_gap": 0.5,
            "input_topic": "/head_camera/color/image_raw", "compressed_input": False,
            "target_topic": "/detection/head/target",
            "mask_topic": "/detection/head/target_mask",
            "debug_topic": "/detection/head/debug_image",
            "input_timeout": 1.0, "max_input_age": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        get = lambda name: self.get_parameter(name).value
        if get("task") not in ("detect", "segment"):
            raise ValueError("task must be detect or segment")
        if not 0 < get("confidence") <= 1 or not 0 < get("nms_iou") <= 1:
            raise ValueError("confidence/nms_iou must be in (0,1]")
        if get("imgsz") < 32 or get("imgsz") % 32:
            raise ValueError("imgsz must be a positive multiple of 32")
        if get("input_timeout") <= 0 or get("max_input_age") <= 0:
            raise ValueError("input_timeout/max_input_age must be > 0")
        config = Config(**{name: get(name) for name in Config.__dataclass_fields__})
        detector = YoloDetector(get("model_path"), get("confidence"), get("nms_iou"),
                                get("imgsz"), get("device"), get("task"))
        self.pipeline = HeadPipeline(detector, make_tracker_factory(get("tracker_type")), config)
        if not detector.ready:
            self.get_logger().warning(detector.error)
        self.session = uuid.uuid4().hex[:12]
        self.timeout, self.max_input_age = get("input_timeout"), get("max_input_age")
        self.last_received = time.monotonic()
        self.last_header, self.timeout_reported = Header(), False
        self.verified_stamp = Time()
        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.target_pub = self.create_publisher(HeadTarget, get("target_topic"), 10)
        self.mask_pub = self.create_publisher(Image, get("mask_topic"), 10)
        self.debug_pub = self.create_publisher(Image, get("debug_topic"), sensor_qos)
        self.compressed = get("compressed_input")
        message_type = CompressedImage if self.compressed else Image
        self.create_subscription(message_type, get("input_topic"), self.on_image, sensor_qos)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(f"Head detection ready: {get('input_topic')} -> {get('target_topic')}")

    def watchdog(self):
        if time.monotonic()-self.last_received > self.timeout and not self.timeout_reported:
            self.pipeline.reset()
            self.publish(Observation("WAITING_IMAGE", "INPUT_TIMEOUT"), self.last_header)
            self.timeout_reported = True

    def on_image(self, msg):
        start = time.perf_counter()
        self.last_received, self.last_header = time.monotonic(), msg.header
        self.timeout_reported = False
        age = self.get_clock().now().nanoseconds/1e9 - stamp_seconds(msg.header.stamp)
        if age > self.max_input_age or age < -self.max_input_age:
            self.pipeline.reset()
            self.publish(Observation("LOST", "STALE_OR_FUTURE_IMAGE"), msg.header)
            return
        try:
            frame = decode_compressed(msg) if self.compressed else decode_raw(msg)
        except (ValueError, cv2.error) as exc:
            self.pipeline.reset()
            self.publish(Observation("LOST", f"INVALID_IMAGE:{exc}"), msg.header)
            return
        observation = self.pipeline.step(frame, stamp_seconds(msg.header.stamp))
        elapsed = (time.perf_counter()-start)*1000
        # A slow inference must not publish a currently-valid control observation.
        completion_age = self.get_clock().now().nanoseconds/1e9-stamp_seconds(msg.header.stamp)
        if completion_age > self.max_input_age:
            self.pipeline.reset()
            observation = Observation("LOST", "RESULT_TOO_OLD")
        self.publish(observation, msg.header, elapsed)
        if observation.reason.startswith("PROCESSING_ERROR"):
            self.get_logger().error(observation.reason, throttle_duration_sec=2.0)
        if self.debug_pub.get_subscription_count():
            debug = frame.copy()
            box = observation.box
            if box:
                x, y, w, h = (round(v) for v in (box.x, box.y, box.width, box.height))
                cv2.rectangle(debug, (x, y), (x+w, y+h), (0, 255, 0), 2)
                x0, y0, x1, y1 = roi_bounds(box, frame.shape[1], frame.shape[0], self.pipeline.cfg.roi_margin)
                cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 180, 0), 1)
            text = f"{observation.state} YOLO={observation.measured} {elapsed:.1f}ms"
            cv2.putText(debug, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            self.debug_pub.publish(self.image_message(debug, msg.header, "bgr8"))

    def publish(self, observation, header, processing_ms=0.0):
        message = HeadTarget()
        message.header = header
        message.state, message.reason = observation.state, observation.reason
        message.target_id = f"{self.session}:{observation.target_id}" if observation.target_id else ""
        message.valid, message.measured = observation.valid, observation.measured
        # Preserve integer sec/nanosec exactly: epoch-sized float seconds lose ns.
        if observation.measured:
            self.verified_stamp = header.stamp
        elif observation.last_verified <= 0:
            self.verified_stamp = Time()
        message.last_verified_stamp = self.verified_stamp
        age = ((header.stamp.sec-self.verified_stamp.sec) +
               (header.stamp.nanosec-self.verified_stamp.nanosec)*1e-9)
        message.verification_age_sec = max(0.0, age) if observation.last_verified > 0 else 0.0
        message.processing_ms, message.class_id = float(processing_ms), -1
        box = observation.box
        if box:
            for name in ("x", "y", "width", "height", "confidence"):
                setattr(message, name, float(getattr(box, name)))
            message.class_id, message.class_name = box.class_id, box.class_name
        message.mask_available = bool(observation.valid and observation.measured
                                      and box is not None and box.mask is not None)
        self.target_pub.publish(message)
        if message.mask_available:
            self.mask_pub.publish(self.image_message(box.mask, header, "mono8"))

    @staticmethod
    def image_message(image, header, encoding):
        msg = Image()
        msg.header, msg.encoding = header, encoding
        msg.height, msg.width = image.shape[:2]
        msg.step = msg.width * (3 if encoding == "bgr8" else 1)
        msg.is_bigendian, msg.data = 0, image.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeadDetectionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
