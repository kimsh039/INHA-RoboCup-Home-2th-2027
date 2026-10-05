"""8-bit ROS image conversion respecting row padding; no cv_bridge ABI dependency."""
import cv2
import numpy as np


def decode_raw(msg):
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1, "bgra8": 4, "rgba8": 4}
    if msg.encoding not in channels:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    count = channels[msg.encoding]
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width*count:
        raise ValueError("Invalid dimensions/step")
    raw = np.frombuffer(msg.data, np.uint8)
    if raw.size != msg.height*msg.step:
        raise ValueError("Image data length does not match height*step")
    image = raw.reshape(msg.height, msg.step)[:, :msg.width*count]
    image = np.ascontiguousarray(image.reshape(msg.height, msg.width, count))
    conversions = {"rgb8": cv2.COLOR_RGB2BGR, "mono8": cv2.COLOR_GRAY2BGR,
                   "bgra8": cv2.COLOR_BGRA2BGR, "rgba8": cv2.COLOR_RGBA2BGR}
    if msg.encoding == "mono8":
        image = image[:, :, 0]
    return cv2.cvtColor(image, conversions[msg.encoding]) if msg.encoding != "bgr8" else image


def decode_compressed(msg):
    image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("Compressed image decode failed")
    return image
