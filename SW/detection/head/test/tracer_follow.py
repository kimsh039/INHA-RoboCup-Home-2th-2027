#!/usr/bin/env python3
"""Standalone Head RGB-D follower. Default: preview and candidate commands only."""
import argparse
import os
from collections import deque
from pathlib import Path
import signal
import sys
import threading
import time

HEAD = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HEAD / 'head_detection_ws/src/robocup_head_detection'))
sys.path.insert(0, str(HEAD / 'scripts'))
from follow_control import Controller, Decision, Settings, decode_depth, fresh_command
from live_ros_viewer import Preview, make_server
from camera_topics import input_status, select_topics
from robocup_head_detection.model_paths import current_model_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', choices=['green_apple', 'peach', 'tomato', 'fanta_can', 'cup', 'banana'], default='banana')
    parser.add_argument('--model', type=Path, default=current_model_path(HEAD))
    parser.add_argument('--rgb', help='Explicit RGB topic; also supply --depth')
    parser.add_argument('--depth', help='Explicit aligned depth topic; also supply --rgb')
    parser.add_argument('--device', default='0')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--distance', type=float, default=1.0)
    parser.add_argument('--max-linear', type=float, default=0.08)
    parser.add_argument('--max-angular', type=float, default=0.25)
    parser.add_argument('--conf', type=float, default=0.5)
    parser.add_argument('--allow-reverse', action='store_true')
    parser.add_argument('--move', action='store_true', help='Actually publish to /cmd_vel; default is dry run')
    parser.add_argument('--port', type=int, default=8787)
    args = parser.parse_args()
    if bool(args.rgb) != bool(args.depth):
        parser.error('--rgb and --depth must be provided together')
    cfg = Settings(target=args.target, distance=args.distance, linear_max=args.max_linear,
                   angular_max=args.max_angular, confidence=args.conf, allow_reverse=args.allow_reverse)
    import cv2
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Empty
    from robocup_head_detection.backends import YoloDetector
    from robocup_head_detection.class_schema import CLASSES, LEGACY_CLASSES
    from robocup_head_detection.images import decode_raw

    detector = YoloDetector(str(args.model), confidence=args.conf, imgsz=args.imgsz, device=args.device)
    if not detector.ready:
        raise SystemExit(detector.error)
    if tuple(detector.model.names.values()) not in (CLASSES, LEGACY_CLASSES):
        raise SystemExit('Expected the Head six-class or original seven-class checkpoint')
    controller = Controller(cfg)
    preview = Preview()
    server = make_server(preview, '127.0.0.1', args.port)
    quit_event = threading.Event()
    for name in ('SIGINT', 'SIGTERM', 'SIGHUP'):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), lambda *_: quit_event.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('tracer_object_follow_test')
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    incoming = threading.Condition()
    state_lock = threading.Lock()
    latest_rgb = None
    depths = deque(maxlen=12)
    camera_count = 0
    desired = Decision()
    command = Decision()
    observed_at = float('-inf')
    latched_stop = False
    candidate_pub = node.create_publisher(Twist, '/object_follow/candidate_cmd_vel', 1)
    drive_pub = node.create_publisher(Twist, '/cmd_vel', 1) if args.move else None
    drive_started = False
    server_started = False

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def rgb_callback(msg):
        nonlocal latest_rgb, camera_count
        with incoming:
            latest_rgb = (msg, time.monotonic())
            camera_count += 1
            incoming.notify()

    def depth_callback(msg):
        with incoming:
            depths.append(msg)

    def stop_callback(_):
        nonlocal latched_stop
        with state_lock:
            latched_stop = True

    def other_publishers():
        # One publisher is ours. Include duplicate nodes with the SAME node name.
        return node.get_publishers_info_by_topic('/cmd_vel')[1:]

    def publish_command():
        nonlocal command, latched_stop
        conflict = bool(drive_pub and other_publishers())
        with state_lock:
            latched_stop |= conflict
            command = fresh_command(desired, observed_at, time.monotonic(),
                                    stopped=latched_stop or quit_event.is_set())
            twist = Twist()
            twist.linear.x, twist.angular.z = command.linear, command.angular
        candidate_pub.publish(twist)
        if drive_pub:
            drive_pub.publish(twist)
        if conflict:
            node.get_logger().error('Another /cmd_vel publisher detected: stopping this test.')
            quit_event.set()

    subscriptions = [node.create_subscription(Empty, '/object_follow/stop', stop_callback, 1)]
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        # Discover existing teleop/Nav2 publishers before sending any drive command.
        discovery_end = time.monotonic() + 2
        while time.monotonic() < discovery_end and not quit_event.is_set():
            executor.spin_once(timeout_sec=0.05)
        published = {name for name, types in node.get_topic_names_and_types()
                     if 'sensor_msgs/msg/Image' in types and node.count_publishers(name) > 0}
        args.rgb, args.depth = select_topics(published, args.rgb, args.depth)
        subscriptions.extend([node.create_subscription(Image, args.rgb, rgb_callback, qos),
                              node.create_subscription(Image, args.depth, depth_callback, qos)])
        if drive_pub and other_publishers():
            raise RuntimeError('Another /cmd_vel publisher exists; stop that controller first')
        if quit_event.is_set():
            return
        timer = node.create_timer(0.05, publish_command)
        drive_started = bool(drive_pub)
        threading.Thread(target=executor.spin, daemon=True).start()
        threading.Thread(target=server.serve_forever, daemon=True).start()
        server_started = True
        print(f'{"MOVE" if args.move else "DRY RUN"} | target={args.target} | distance={args.distance:.2f} m', flush=True)
        print(f'Preview http://127.0.0.1:{args.port}/ | RGB={args.rgb} | depth={args.depth}', flush=True)
        print(f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")} | '
              f'RGB publishers={node.count_publishers(args.rgb)} | depth publishers={node.count_publishers(args.depth)}', flush=True)
        print('Dry run never publishes /cmd_vel. Stop: Ctrl+C or /object_follow/stop (latched).', flush=True)
        frames = previous_camera = 0
        previous_input_at = float('-inf')
        started = time.monotonic()
        fps = 0.0
        while rclpy.ok() and not quit_event.is_set():
            with incoming:
                incoming.wait_for(lambda: latest_rgb is not None or quit_event.is_set(), timeout=0.1)
                item, latest_rgb = latest_rgb, None
                if item:
                    msg, arrival = item
                    depth_msg = min(depths, key=lambda d: abs(stamp(d)-stamp(msg))) if depths else None
                count = camera_count
            if item:
                try:
                    if arrival - previous_input_at > 0.3:
                        controller.reset()
                    previous_input_at = arrival
                    age = node.get_clock().now().nanoseconds * 1e-9 - stamp(msg)
                    frame = decode_raw(msg).copy()
                    if not -0.05 <= age <= 0.3 or time.monotonic() - arrival > 0.3:
                        controller.reset()
                        result = Decision(reason='STALE_RGB')
                    elif depth_msg is None or abs(stamp(depth_msg)-stamp(msg)) > 0.1:
                        controller.reset()
                        result = Decision(reason='UNSYNCED_DEPTH')
                    else:
                        depth = decode_depth(depth_msg)
                        boxes = detector.detect(frame)
                        frames += 1
                        result = controller.update(boxes, depth, frame.shape[1], frame.shape[0])
                    with state_lock:
                        # Preserve the source-image age even when inference takes a long time.
                        source_age = node.get_clock().now().nanoseconds * 1e-9 - stamp(msg)
                        desired = result
                        observed_at = time.monotonic() - max(0.0, source_age)
                    if result.box:
                        b = result.box
                        cv2.rectangle(frame, (round(b.x), round(b.y)),
                                      (round(b.x+b.width), round(b.y+b.height)), (40, 220, 90), 2)
                        cv2.putText(frame, f'{b.class_name} {b.confidence:.2f}',
                                    (max(0, round(b.x)), max(50, round(b.y)-5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 220, 90), 1)
                    with state_lock:
                        shown = command
                    distance = '-' if result.distance is None else f'{result.distance:.2f}m'
                    cv2.putText(frame, f'{"MOVE" if args.move else "DRY"} {args.target} {distance} FPS={fps:.1f}',
                                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.putText(frame, f'{shown.reason} v={shown.linear:.2f} w={shown.angular:.2f}',
                                (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    ok, jpeg = cv2.imencode('.jpg', frame)
                    if ok:
                        preview.publish(jpeg.tobytes())
                except Exception:
                    controller.reset()
                    with state_lock:
                        desired = Decision(reason='PROCESSING_ERROR')
                    raise
            elif time.monotonic() - previous_input_at > 0.3:
                controller.reset()
            elapsed = time.monotonic() - started
            if elapsed >= 1:
                fps = frames / elapsed
                with state_lock:
                    shown = command
                diagnostic = input_status(node.count_publishers(args.rgb), node.count_publishers(args.depth), count)
                print(f'Camera FPS={(count-previous_camera)/elapsed:.1f} | Detection FPS={fps:.1f} | '
                      f'{diagnostic or shown.reason} | v={shown.linear:.3f} m/s w={shown.angular:.3f} rad/s', flush=True)
                started, frames, previous_camera = time.monotonic(), 0, count
        node.destroy_timer(timer)
    finally:
        quit_event.set()
        if drive_started and rclpy.ok():
            # Normal Ctrl+C / SIGTERM / SSH hangup all send repeated zero commands.
            for _ in range(3):
                drive_pub.publish(Twist())
                time.sleep(0.05)
        preview.close()
        # server.shutdown requires serve_forever to have started.
        if server_started:
            server.shutdown()
        server.server_close()
        executor.shutdown(timeout_sec=2)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
