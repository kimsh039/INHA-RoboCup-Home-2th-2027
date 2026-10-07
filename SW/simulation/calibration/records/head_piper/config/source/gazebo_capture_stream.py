"""Gazebo-only capture worker; never import OpenCV in this process.

The parent requests one fresh, timestamp-matched snapshot with each stdin line.
JSON snapshots cross the pipe; no continuous sensor log is written to disk.
"""
import argparse
import base64
import json
import os
import sys
import threading
import time
from collections import deque


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--partition', required=True)
    parser.add_argument('--timeout', type=float, required=True)
    args = parser.parse_args()
    os.environ['GZ_PARTITION'] = args.partition
    os.environ['GZ_IP'] = '127.0.0.1'

    from google.protobuf.json_format import MessageToDict
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.camera_info_pb2 import CameraInfo
    from gz.msgs10.model_pb2 import Model

    topics = [
        (Image, args.prefix + '/color/image', 'image'),
        (CameraInfo, args.prefix + '/color/camera_info', 'info'),
        (Model, '/robocup/joint_states', 'joints'),
    ]
    node = Node()
    lock = threading.Lock()
    buffers = {key: deque(maxlen=2000 if key == 'joints' else 4)
               for _, _, key in topics}
    callbacks = []
    subscribed = []

    def receive(key):
        def callback(msg):
            with lock:
                buffers[key].append(msg)
        return callback

    end = time.monotonic() + args.timeout
    last_image_time = None
    try:
        for kind, topic, key in topics:
            callback = receive(key)
            callbacks.append(callback)
            if not node.subscribe(kind, topic, callback):
                raise RuntimeError('Subscription failed: ' + topic)
            subscribed.append(topic)

        for request in sys.stdin:
            last_reason = 'Waiting for image / CameraInfo / measured joints'
            while time.monotonic() < end:
                with lock:
                    copies = {key: list(values) for key, values in buffers.items()}
                if not all(copies.values()):
                    time.sleep(.02)
                    continue
                image = copies['image'][-1]
                ts = stamp(image)
                if ts == last_image_time:
                    time.sleep(.02)
                    continue
                info = min(copies['info'], key=lambda msg: abs(stamp(msg) - ts))
                joints = min(copies['joints'], key=lambda msg: abs(stamp(msg) - ts))
                if abs(stamp(info) - ts) > .005 or abs(stamp(joints) - ts) > .005:
                    last_reason = 'Image / CameraInfo / measured joint time mismatch'
                    time.sleep(.02)
                    continue

                packet = {
                    'image_sim_time': ts,
                    'info_sim_time': stamp(info),
                    'joint_sim_time': stamp(joints),
                    'image': {
                        'width': image.width, 'height': image.height,
                        'step': image.step, 'pixel_format_type': image.pixel_format_type,
                        'frame_ids': [value for item in image.header.data
                                      if item.key == 'frame_id' for value in item.value],
                        'data': base64.b64encode(image.data).decode('ascii'),
                    },
                    'info': {
                        'width': info.width, 'height': info.height,
                        'k': list(info.intrinsics.k),
                        'distortion': list(info.distortion.k),
                        'proto_json': MessageToDict(info, preserving_proto_field_name=True),
                    },
                    'joints': [{'name': joint.name, 'position': joint.axis1.position,
                                'velocity': joint.axis1.velocity} for joint in joints.joint],
                }
                print(json.dumps(packet, separators=(',', ':')), flush=True)
                last_image_time = ts
                break
            else:
                print(json.dumps({'error': 'Capture timed out: ' + last_reason}), flush=True)
                return
    finally:
        for topic in subscribed:
            node.unsubscribe(topic)


if __name__ == '__main__':
    main()
