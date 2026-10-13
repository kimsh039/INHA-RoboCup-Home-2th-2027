#!/usr/bin/env python3
"""Show the control center's existing detections without a second YOLO process."""
import argparse
import json
import threading
import time

from live_ros_viewer import Preview, make_server
from robocup_head_detection.images import decode_raw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview-fps', type=float, default=15)
    args = parser.parse_args()
    if not 0 < args.preview_fps <= 60:
        parser.error('preview-fps must be in (0, 60]')
    import cv2
    import rclpy
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('head_existing_detection_preview')
    preview = Preview()
    server = make_server(preview, '127.0.0.1', 8787)
    incoming = threading.Condition()
    latest = None
    status = {}

    def receive(msg):
        nonlocal latest
        with incoming:
            latest = msg
            incoming.notify()

    def receive_status(msg):
        nonlocal status
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                status = value
        except (ValueError, TypeError):
            pass

    node.create_subscription(Image, '/detection/head/debug_image', receive,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(String, '/detection/head/objects', receive_status, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    threading.Thread(target=spin, daemon=True).start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print('Preview: http://127.0.0.1:8787/ | reusing control-center YOLO; no second inference', flush=True)
    next_preview = 0
    report_at = time.monotonic()
    count = 0
    try:
        while rclpy.ok():
            with incoming:
                incoming.wait_for(lambda: latest is not None, timeout=0.2)
                msg, latest = latest, None
            now = time.monotonic()
            if msg is not None and now >= next_preview:
                next_preview = now + 1 / args.preview_fps
                ok, encoded = cv2.imencode('.jpg', decode_raw(msg), [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    preview.publish(encoded.tobytes())
                    count += 1
            if now - report_at >= 1:
                snapshot = status
                print(f'Camera FPS={snapshot.get("camera_fps", "-")} | '
                      f'Detection FPS={snapshot.get("detection_fps", "-")} | '
                      f'Preview FPS={count/(now-report_at):.1f}', flush=True)
                count, report_at = 0, now
    except KeyboardInterrupt:
        pass
    finally:
        preview.close()
        server.shutdown()
        server.server_close()
        executor.shutdown(timeout_sec=2)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
