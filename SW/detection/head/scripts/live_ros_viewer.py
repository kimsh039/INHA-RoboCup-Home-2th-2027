#!/usr/bin/env python3
"""Detect all six Head objects from ROS RGB, serve a preview and print live FPS."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'head_detection_ws' /
                       'src' / 'robocup_head_detection'))


class Preview:
    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None
        self.sequence = 0
        self.closed = False

    def publish(self, jpeg):
        with self.condition:
            self.jpeg = jpeg
            self.sequence += 1
            self.condition.notify_all()

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


def make_server(preview, host, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == '/':
                page = (b'<!doctype html><meta charset="utf-8"><title>Head detection</title>'
                        b'<style>body{background:#101418;color:#eee;font:16px sans-serif;'
                        b'margin:20px}img{width:100%;max-width:1280px}p{line-height:1.6}</style>'
                        b'<h2>Head detection</h2><p>green_apple / peach / tomato / fanta_can / '
                        b'cup / banana<br>Live FPS is printed in the SSH terminal. '
                        b'Waiting for ROS camera frames if the image is blank.</p>'
                        b'<img src="/stream.mjpg" alt="Live camera detection">')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(page)))
                self.end_headers()
                self.wfile.write(page)
                return
            if self.path != '/stream.mjpg':
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            sequence = -1
            try:
                while True:
                    with preview.condition:
                        preview.condition.wait_for(
                            lambda: preview.closed or preview.sequence != sequence, timeout=2)
                        if preview.closed:
                            return
                        if preview.jpeg is None or sequence == preview.sequence:
                            sequence = preview.sequence
                            continue
                        jpeg, sequence = preview.jpeg, preview.sequence
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                     str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=Path(__file__).resolve().parents[1] /
                        'models/head_yolo11n_20261013.pt')
    parser.add_argument('--topic', default='/head/calibrated/color/image_raw')
    parser.add_argument('--device', default='0')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--cup-conf', type=float, default=0.70,
                        help='Additional minimum confidence only for cup')
    parser.add_argument('--iou', type=float, default=0.7)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8787)
    args = parser.parse_args()
    if not 0 < args.conf <= 1 or not 0 <= args.cup_conf <= 1 or not 0 < args.iou <= 1:
        parser.error('conf/iou must be in (0,1] and cup-conf in [0,1]')
    import cv2
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from robocup_head_detection.backends import YoloDetector
    from robocup_head_detection.class_schema import CLASSES, LEGACY_CLASSES
    from robocup_head_detection.images import decode_raw

    detector = YoloDetector(str(args.model), confidence=args.conf, nms_iou=args.iou,
                            imgsz=args.imgsz, device=args.device, cup_confidence=args.cup_conf)
    if not detector.ready:
        raise SystemExit(detector.error)
    names = tuple(detector.model.names[i] for i in range(len(detector.model.names)))
    if names not in (CLASSES, LEGACY_CLASSES):
        raise SystemExit(f'Expected this project\'s Head checkpoint, got {names}')

    preview = Preview()
    server = make_server(preview, args.host, args.port)
    rclpy.init()
    node = rclpy.create_node('head_live_detection_preview')
    incoming = threading.Condition()
    latest = None
    received = 0

    def receive(msg):
        nonlocal latest, received
        with incoming:
            latest = (msg, time.perf_counter())  # Replace stale input, never grow a frame queue.
            received += 1
            incoming.notify()

    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT)
    subscription = node.create_subscription(Image, args.topic, receive, qos)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'Preview: http://{args.host}:{args.port}/ | topic={args.topic}', flush=True)
    print(f'Classes: {", ".join(CLASSES)} | device={args.device} | imgsz={args.imgsz}', flush=True)
    print(f'Confidence: other={args.conf:.2f} | cup={max(args.conf, args.cup_conf):.2f} | NMS IoU={args.iou:.2f}', flush=True)
    print(('Existing 7-output checkpoint adapted to six classes; no new training. '
           if names == LEGACY_CLASSES else 'Native six-class checkpoint. ') +
          'Ctrl+C to stop.', flush=True)
    started = time.perf_counter()
    processed = previous_received = 0
    detection_ms = age_ms = 0.0
    fps = 0.0
    try:
        while rclpy.ok():
            with incoming:
                incoming.wait_for(lambda: latest is not None, timeout=0.2)
                item, latest = latest, None
            if item is not None:
                msg, arrival = item
                frame = decode_raw(msg).copy()
                before = time.perf_counter()
                boxes = detector.detect(frame)
                detection_ms += (time.perf_counter() - before) * 1000
                for box in boxes:
                    x, y, right, bottom = [round(v) for v in
                        (box.x, box.y, box.x + box.width, box.y + box.height)]
                    color = (40, 220, 90)
                    cv2.rectangle(frame, (x, y), (right, bottom), color, 2)
                    cv2.putText(frame, f'{box.class_name} {box.confidence:.2f}',
                                (max(0, x), max(18, y-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                cv2.putText(frame, f'Detection FPS: {fps:.1f}', (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ok:
                    raise RuntimeError('Preview JPEG encoding failed')
                preview.publish(encoded.tobytes())
                age_ms += (time.perf_counter() - arrival) * 1000
                processed += 1
            now = time.perf_counter()
            elapsed = now - started
            if elapsed >= 1:
                with incoming:
                    count = received
                fps = processed / elapsed
                camera_fps = (count - previous_received) / elapsed
                average = f'{detection_ms/processed:.1f}' if processed else '-'
                age = f'{age_ms/processed:.1f}' if processed else '-'
                print(f'Camera FPS={camera_fps:.1f} | Detection FPS={fps:.1f} | '
                      f'detect={average} ms | callback-to-preview={age} ms' +
                      (' | waiting for camera input' if not processed else ''), flush=True)
                started, previous_received = now, count
                processed = 0
                detection_ms = age_ms = 0.0
    except KeyboardInterrupt:
        pass
    finally:
        preview.close()
        server.shutdown()
        server.server_close()
        executor.shutdown(timeout_sec=2)
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
