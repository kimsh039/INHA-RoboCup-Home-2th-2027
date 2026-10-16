#!/usr/bin/env python3
"""Jetson ROS 2 web bridge with separately managed camera and LiDAR drivers."""
import argparse
import base64
from collections import deque
import json
import math
import os
from pathlib import Path
import struct
import threading
import time

from server import State, finite_json, load_config, make_server, png_gray
from sensor_manager import SensorManager, SENSORS
from can_monitor import CanMonitor, OdometryInputs

TYPES = {
    'map': ['nav_msgs/msg/OccupancyGrid'],
    'scan': ['sensor_msgs/msg/LaserScan'],
    'odom': ['nav_msgs/msg/Odometry'],
    'path': ['nav_msgs/msg/Path'],
    'cloud': ['sensor_msgs/msg/PointCloud2'],
    'object_cloud': ['sensor_msgs/msg/PointCloud2'],
    'detections': ['std_msgs/msg/String', 'robocup_detection_msgs/msg/HeadTarget', 'yolo_msgs/msg/DetectionArray',
                   'vision_msgs/msg/Detection2DArray'],
    'wrist_status': ['std_msgs/msg/String'],
    'battery': ['sensor_msgs/msg/BatteryState'],
    'joints': ['sensor_msgs/msg/JointState'],
    'rosout': ['rcl_interfaces/msg/Log'],
}
for key in ('head_image', 'head_debug', 'wrist_image', 'wrist_debug'):
    TYPES[key] = ['sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage']


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def run(args):
    try:
        import cv2
        import numpy as np
        from cloud_history import CloudHistory
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.time import Time
        from rosidl_runtime_py.utilities import get_message
        from rclpy.serialization import deserialize_message
        from tf2_ros import Buffer, TransformListener
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String
    except ImportError as exc:
        raise SystemExit(f'ROS 의존성 없음: {exc}\nsource /opt/ros/humble/setup.bash 후 /usr/bin/python3로 실행하세요.\n'
                         '영상 변환에는 ros-humble-cv-bridge와 python3-opencv가 필요합니다.')

    config = load_config(args.config)
    use_sim_time = args.sim_time or config.get('use_sim_time', False)
    if args.with_robot and use_sim_time:
        raise SystemExit('--with-robot은 실물 센서 실행입니다. 시뮬레이션은 --observe --sim-time으로 실행하세요.')
    if use_sim_time:
        os.environ['ROBOCUP_USE_SIM_TIME'] = 'true'
    state = State('live', config)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=' + str(use_sim_time).lower()])

    class Bridge(Node):
        def __init__(self):
            super().__init__('inha_control_center')
            self.cv = CvBridge()
            self.tf = Buffer()
            # TF stays on the bridge executor: a separate spin thread starved every other channel (~2 Hz).
            # Lookups at sensor stamps may lag; display_transform falls back to the latest TF in a steady frame.
            self.listener = TransformListener(self.tf, self)
            self.bound, self.overrides, self.last_processed = {}, {}, {}
            self.cloud_history = CloudHistory(config)
            self.trail = deque(maxlen=1200)
            self.trail_frame = None
            self.odom_message = None
            self.fixed = config.get('fixed_frame', 'map')
            self.base = config.get('base_frame', 'base_link')
            self.last_cpu = None
            self.odom_inputs = OdometryInputs(state, use_sim_time)
            self.create_timer(2.0, self.discover)
            self.create_timer(0.2, self.update_pose)
            self.create_timer(3.0, self.health)
            state.log('ROS 수신 모드 · ROS_DOMAIN_ID=' + os.environ.get('ROS_DOMAIN_ID', '0'))
            self.create_timer(1.0, lambda: state.sensor_manager.refresh() if state.sensor_manager else None)
            state.log('카메라·LiDAR 드라이버는 센서 제어 버튼에서 실행/중지합니다.')
            self.create_subscription(String, '/localization/status', self.localization_status,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(String, '/collision/status', self.collision_status, 10)
            self.create_subscription(String, '/collision/actuator_status', self.collision_actuator_status, 10)
            self.create_subscription(String, '/piper/feedback_status', self.piper_feedback_status, 10)
            from control_process import ControllerProcess
            self.control = ControllerProcess(state, config, use_sim_time)
            self.create_timer(.1, self.control.dispatch_services)
            from manipulation_control import ManipulationControl
            self.manipulation = ManipulationControl(self, state)
            from task_control import TaskControl
            self.task_control = TaskControl(self, state, self.manipulation)
            self.discover()

        def discover(self):
            with state.condition:
                pending, state.pending = state.pending, {}
            for key, topic in pending.items():
                if topic:
                    self.overrides[key] = topic
                else:
                    self.overrides.pop(key, None)
                if key in self.bound:
                    owner = self.odom_inputs.node if key == 'odom' else self
                    owner.destroy_subscription(self.bound.pop(key)[3])
                self.last_processed.pop(key, None)
                self.cloud_history.frames.pop(key, None)
                state.clear(key)
                with state.condition:
                    state.channels[key].update(topic='', type='', publishers=0)
            graph = dict(self.get_topic_names_and_types())
            topic_list = []
            publishers = {}
            for topic, types in graph.items():
                info = self.get_publishers_info_by_topic(topic)
                publishers[topic] = info
                if info:
                    topic_list.append(dict(name=topic, types=types, publishers=len(info)))
            state.put('topics', sorted(topic_list, key=lambda t: t['name']))
            if state.sensor_manager:
                state.sensor_manager.observe(publishers)
            for key, choices in config['channels'].items():
                candidates = [self.overrides[key]] if key in self.overrides else choices
                selected = next(((topic, typ) for topic in candidates for typ in graph.get(topic, [])
                                 if publishers.get(topic) and typ in TYPES.get(key, [])), None)
                if not selected:
                    with state.condition:
                        state.channels[key]['publishers'] = 0
                        state.channels[key]['manual'] = key in self.overrides
                        if key in self.overrides:
                            state.channels[key]['topic'] = self.overrides[key]
                    continue
                topic, typ = selected
                infos = publishers[topic]
                transient = all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos)
                signature = (topic, typ, transient)
                with state.condition:
                    state.channels[key].update(topic=topic, type=typ, publishers=len(infos),
                                               manual=key in self.overrides,
                                               qos='best_effort / ' + ('transient_local' if transient else 'volatile'))
                if key in self.bound and self.bound[key][:3] == signature:
                    continue
                if key in self.bound:
                    self.destroy_subscription(self.bound.pop(key)[3])
                    state.clear(key)
                try:
                    message_type = get_message(typ)
                    qos = QoSProfile(depth=100 if key == 'odom' else 20 if key == 'rosout' else 1,
                                     reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
                    # Cameras publish ~1 MB frames at 20-30 Hz but the web shows a few per
                    # second. Deserializing every frame starved odometry/TF on this executor.
                    heavy = key in ('cloud', 'object_cloud', 'joints') or key.endswith('_image') or key.endswith('_debug')
                    callback = ((lambda data, key=key, kind=message_type: self.receive_serialized(key, data, kind))
                                if heavy else (lambda msg, key=key: self.receive(key, msg)))
                    owner = self.odom_inputs.node if key == 'odom' else self
                    subscription = owner.create_subscription(message_type, topic, callback, qos, raw=heavy)
                    self.bound[key] = signature + (subscription,)
                    state.log(f'{key} ← {topic}')
                except Exception as exc:
                    state.error(key, f'{typ}: {exc} · 커스텀 메시지 workspace의 install/setup.bash를 source하세요.')

        def age(self, msg):
            stamp = stamp_seconds(msg.header.stamp) if hasattr(msg, 'header') else None
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp if stamp else None
            return stamp, age

        def transform(self, source, stamp=None, target=None):
            target = target or self.fixed
            if source == target:
                return np.eye(3), np.zeros(3)
            when = Time.from_msg(stamp) if stamp and (stamp.sec or stamp.nanosec) else Time()
            tf = self.tf.lookup_transform(target, source, when)
            q, p = tf.transform.rotation, tf.transform.translation
            x, y, z, w = q.x, q.y, q.z, q.w
            matrix = np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                               [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
                               [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])
            return matrix, np.array([p.x, p.y, p.z])

        def display_transform(self, source, stamp):
            """Fixed-frame pose of a sensor message for display. SLAM publishes map -> odom only at scan
            rate, so an exact-stamp lookup intermittently extrapolates; the latest map -> odom composed
            with odom -> sensor at the stamp keeps the fixed frame. Falls back to odom, then native."""
            try:
                rotation, offset = self.transform(source, stamp)
                return rotation, offset, self.fixed
            except Exception:
                pass
            try:
                rotation, offset = self.transform(source, stamp, target='odom')
            except Exception:
                for target in (self.fixed, 'odom'):  # stamp outside the buffer: latest TF in a steady frame
                    try:
                        rotation, offset = self.transform(source, target=target)
                        return rotation, offset, target
                    except Exception:
                        continue
                return None, None, source
            if self.fixed != 'odom':
                try:
                    tf = self.tf.lookup_transform(self.fixed, 'odom', Time()).transform
                    q, t = tf.rotation, tf.translation
                    x, y, z, w = q.x, q.y, q.z, q.w
                    r = np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                                  [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
                                  [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])
                    return r @ rotation, r @ offset + np.array([t.x, t.y, t.z]), self.fixed
                except Exception:
                    pass
            return rotation, offset, 'odom'

        def collision_status(self, msg):
            try:
                value = json.loads(msg.data)
                if not isinstance(value, dict):
                    raise ValueError('invalid collision status')
                closest = value.get('nearest_point')
                if closest is not None:
                    try:
                        matrix, offset = self.transform(value.get('frame', 'odom'))
                        value['display_point'] = (matrix @ np.asarray(closest, dtype=float) + offset).tolist()
                        value['display_frame'] = self.fixed
                    except Exception:
                        pass
                stamp = value.get('stamp')
                age = self.get_clock().now().nanoseconds/1e9-stamp if isinstance(stamp, (int, float)) else None
                state.put('collision_safety', value, stamp, age)
                with state.condition:
                    state.channels['collision_safety'].update(topic='/collision/status', type='std_msgs/msg/String', publishers=1)
            except (ValueError, TypeError, KeyError):
                state.put('collision_safety', dict(ready=False, reason='invalid_status'))
                state.error('collision_safety', '충돌 보호 상태 형식 오류')

        def piper_feedback_status(self, msg):
            try:
                value = json.loads(msg.data)
                if not isinstance(value, dict) or value.get('state') not in ('live', 'waiting', 'stale'):
                    raise ValueError('invalid PiPER feedback status')
                state.put('piper_feedback', value)
                with state.condition:
                    state.channels['piper_feedback'].update(topic='/piper/feedback_status',
                                                           type='std_msgs/msg/String', publishers=1)
            except (ValueError, TypeError):
                state.put('piper_feedback', dict(state='stale', reason='invalid_status'))
                state.error('piper_feedback', 'PiPER 피드백 상태 형식 오류')

        def collision_actuator_status(self, msg):
            try:
                value = json.loads(msg.data)
                if not isinstance(value, dict):
                    raise ValueError('invalid actuator status')
                state.put('collision_actuator', value)
                with state.condition:
                    state.channels['collision_actuator'].update(topic='/collision/actuator_status', type='std_msgs/msg/String', publishers=1)
            except (ValueError, TypeError):
                state.put('collision_actuator', dict(ready=False, reason='invalid_status'))
                state.error('collision_actuator', '차체 보호 출력 상태 형식 오류')

        def localization_status(self, message):
            try:
                value = json.loads(message.data)
                if isinstance(value, dict):
                    state.put('localization', value)
            except (ValueError, TypeError):
                pass

        def receive_serialized(self, key, data, kind):
            limit = (config.get('image_hz', 6) if key in ('head_image', 'head_debug', 'wrist_image', 'wrist_debug')
                     else config.get('joint_hz', 15) if key == 'joints' else config.get('cloud_hz', 5))
            if time.monotonic() - self.last_processed.get(key, 0) < 1 / max(float(limit), 0.1):
                return
            try:
                msg = deserialize_message(data, kind)
            except Exception as exc:
                state.error(key, f'deserialize: {exc}')
                return
            self.receive(key, msg)

        def receive(self, key, msg):
            limit = config.get('image_hz', 6) if key in ('head_image', 'head_debug', 'wrist_image', 'wrist_debug') else (
                    config.get('cloud_hz', 5) if key in ('cloud', 'object_cloud') else 8 if key == 'scan' else 0)
            now = time.monotonic()
            if limit and now - self.last_processed.get(key, 0) < 1 / max(float(limit), 0.1):
                return
            self.last_processed[key] = now
            try:
                stamp, age = self.age(msg)
                if key.endswith('_image') or key.endswith('_debug'):
                    if self.bound[key][1] == 'sensor_msgs/msg/CompressedImage':
                        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                        if image is None:
                            raise ValueError('JPEG/PNG 입력을 디코딩할 수 없습니다.')
                    else:
                        image = self.cv.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    max_width = int(config.get('image_width', 960))
                    if image.shape[1] > max_width:
                        image = cv2.resize(image, (max_width, max(1, round(image.shape[0] * max_width / image.shape[1]))))
                    ok, data = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, int(config.get('jpeg_quality', 75))])
                    if not ok:
                        raise ValueError('JPEG encoding failed')
                    state.put_frame(key, data.tobytes(), 'image/jpeg', image.shape[1], image.shape[0], stamp, age)
                elif key == 'map':
                    self.fixed = msg.header.frame_id or config.get('fixed_frame', 'map')
                    width, height = msg.info.width, msg.info.height
                    if not width or not height or len(msg.data) != width * height or msg.info.resolution <= 0:
                        raise ValueError('Invalid OccupancyGrid dimensions/resolution')
                    grid = np.asarray(msg.data, dtype=np.int16).reshape(height, width)
                    factor = max(1, math.ceil(max(width, height) / 2048))
                    reduced = grid[::factor, ::factor]
                    gray = np.where(reduced < 0, 95, 235 - np.clip(reduced, 0, 100) * 2.15).astype(np.uint8)
                    png = png_gray(gray.shape[1], gray.shape[0], gray.tobytes())
                    with state.condition:
                        state.map_png = png_gray(gray.shape[1], gray.shape[0], gray[::-1].tobytes())
                    origin = msg.info.origin
                    state.put(key, dict(image='data:image/png;base64,' + base64.b64encode(png).decode(),
                                        width=gray.shape[1], height=gray.shape[0], resolution=msg.info.resolution * factor,
                                        source_width=width, source_height=height, frame=self.fixed,
                                        origin=[origin.position.x, origin.position.y, yaw(origin.orientation)],
                                        known_area=float(np.count_nonzero(grid >= 0) * msg.info.resolution**2),
                                        occupied_area=float(np.count_nonzero(grid >= 65) * msg.info.resolution**2)), stamp, age)
                elif key == 'odom':
                    self.odom_message = msg
                    state.put(key, dict(linear=msg.twist.twist.linear.x, angular=msg.twist.twist.angular.z,
                                        frame=msg.header.frame_id), stamp, age)
                elif key == 'scan':
                    points = []
                    for i in range(0, len(msg.ranges), max(1, len(msg.ranges) // 600)):
                        r = msg.ranges[i]
                        if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                            a = msg.angle_min + i * msg.angle_increment
                            points.append([r * math.cos(a), r * math.sin(a), 0])
                    rotation, offset, frame = self.display_transform(msg.header.frame_id, msg.header.stamp)
                    transformed = np.asarray(points).reshape(-1, 3)
                    if rotation is not None:
                        transformed = transformed @ rotation.T + offset
                    state.put(key, dict(frame=frame, points=transformed[:, :2].round(3).tolist()), stamp, age)
                elif key in ('cloud', 'object_cloud'):
                    points = self.cloud_points(msg)
                    rotation, offset, frame = self.display_transform(msg.header.frame_id, msg.header.stamp)
                    stable = rotation is not None  # native frames are shown without accumulating moving observations
                    if stable:
                        points = points @ rotation.T + offset
                    points, metadata = self.cloud_history.add(
                        key, points, frame, msg.header.frame_id, stamp, self.bound[key][0],
                        msg.width * msg.height, stable)
                    if points is None:
                        return  # accumulated cloud is rebuilt at most every cloud_accumulated_period_sec
                    value = finite_json(dict(frame=frame, source_frame=msg.header.frame_id,
                                             point_size=max(1.0, min(6.0, float(config.get('cloud_point_size', 3.0)))),
                                             **metadata))
                    points = points[np.isfinite(points).all(axis=1)]
                    value['points'] = points.round(3).tolist()
                    state.put(key, value, stamp, age, finite=True)
                elif key == 'path':
                    rotation, offset = self.transform(msg.header.frame_id, msg.header.stamp)
                    step = max(1, len(msg.poses) // 1000)
                    points = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                                       for p in msg.poses[::step]]).reshape(-1, 3)
                    points = points @ rotation.T + offset
                    state.put(key, dict(points=points[:, :2].round(3).tolist(), frame=self.fixed), stamp, age)
                elif key == 'detections':
                    result = self.detections(msg)
                    if result.get('stamp') is not None:
                        stamp = float(result['stamp'])
                        age = self.get_clock().now().nanoseconds / 1e9 - stamp
                    state.put(key, result, stamp, age)
                elif key == 'wrist_status':
                    state.put(key, dict(text=msg.data), stamp, age)
                elif key == 'battery':
                    percentage = msg.percentage * 100 if math.isfinite(msg.percentage) and 0 <= msg.percentage <= 1 else None
                    state.put(key, dict(voltage=msg.voltage, percentage=percentage), stamp, age)
                elif key == 'joints':
                    state.put(key, dict(names=list(msg.name), positions=list(msg.position)), stamp, age)
                elif key == 'rosout':
                    if msg.level >= 30 and msg.name != self.get_name():
                        state.log(msg.msg, 'error' if msg.level >= 40 else 'warn', msg.name)
                    state.put(key, dict(last_node=msg.name), stamp, age)
            except Exception as exc:
                state.error(key, str(exc))

        def cloud_points(self, msg):
            fields = {f.name: f for f in msg.fields}
            if not all(name in fields for name in ('x', 'y', 'z')):
                raise ValueError('PointCloud2에 x/y/z 필드가 없습니다. Livox xfer_format=0이 필요합니다.')
            total = msg.width * msg.height
            if not total:
                return np.empty((0, 3))
            endian = '>' if msg.is_bigendian else '<'
            names, formats, offsets = [], [], []
            for name in ('x', 'y', 'z'):
                field = fields[name]
                if field.datatype not in (7, 8):
                    raise ValueError('XYZ fields must be FLOAT32 or FLOAT64')
                names.append(name)
                formats.append(endian + ('f4' if field.datatype == 7 else 'f8'))
                offsets.append(field.offset)
            dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets, itemsize=msg.point_step))
            maximum = max(1, int(config.get('max_cloud_points', 20000)))
            step = max(1, math.ceil(total / maximum))
            records = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                                 strides=(msg.row_step, msg.point_step))
            indices = np.arange(0, total, step)
            rows, columns = np.divmod(indices, msg.width)
            points = np.column_stack([records[name][rows, columns] for name in names])
            return points[np.isfinite(points).all(axis=1)]

        def detections(self, msg):
            if hasattr(msg, 'data'):
                value = json.loads(msg.data)
                if not isinstance(value, dict) or value.get('mode') != 'all_objects' or not isinstance(value.get('objects'), list):
                    raise ValueError('Head 전체 검출 JSON 형식이 아닙니다.')
                return value
            if hasattr(msg, 'target_id') and hasattr(msg, 'valid'):
                objects = []
                if msg.width > 0 and msg.height > 0:
                    objects.append(dict(label=msg.class_name, confidence=msg.confidence, id=msg.target_id,
                                        x=msg.x, y=msg.y, width=msg.width, height=msg.height))
                return dict(state=msg.state, reason=msg.reason, valid=msg.valid, measured=msg.measured,
                            mask_available=msg.mask_available, verification_age_sec=msg.verification_age_sec,
                            processing_ms=msg.processing_ms, objects=objects)
            objects = []
            for detection in getattr(msg, 'detections', []):
                label = getattr(detection, 'class_name', str(getattr(detection, 'class_id', 'object')))
                score = float(getattr(detection, 'score', 0.0))
                if getattr(detection, 'results', []):
                    result = max(detection.results, key=lambda item: item.hypothesis.score)
                    label, score = str(result.hypothesis.class_id), float(result.hypothesis.score)
                bbox = getattr(detection, 'bbox', None)
                if bbox is None:
                    continue
                center = getattr(bbox.center, 'position', bbox.center)
                w, h = float(bbox.size_x), float(bbox.size_y)
                objects.append(dict(label=label, confidence=score, x=float(center.x) - w/2,
                                    y=float(center.y) - h/2, width=w, height=h))
            return dict(state='DETECTED' if objects else 'SEARCH', valid=bool(objects), measured=True, objects=objects)

        def update_pose(self):
            try:
                tf = self.tf.lookup_transform(self.fixed, self.base, Time())
                p, q = tf.transform.translation, tf.transform.rotation
                pose = dict(x=p.x, y=p.y, yaw=yaw(q), frame=self.fixed)
                stamp = stamp_seconds(tf.header.stamp)
                age = self.get_clock().now().nanoseconds * 1e-9 - stamp if stamp else None
            except Exception:
                if self.odom_message is None:
                    return
                msg = self.odom_message
                if msg.child_frame_id and msg.child_frame_id != self.base:
                    return
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                pose = dict(x=p.x, y=p.y, yaw=yaw(q), frame=msg.header.frame_id)
                stamp, age = self.age(msg)
            if pose['frame'] != self.trail_frame:
                self.trail.clear()
                self.trail_frame = pose['frame']
            if not self.trail or math.hypot(pose['x']-self.trail[-1][0], pose['y']-self.trail[-1][1]) > 0.04:
                self.trail.append([pose['x'], pose['y']])
            pose['trail'] = list(self.trail)
            state.put('pose', pose, stamp, age)
            with state.condition:
                state.channels['pose'].update(topic='TF ' + pose['frame'] + ' ← ' + self.base,
                                              type='tf2', publishers=1, qos='TF')

        def health(self):
            health = dict(domain_id=os.environ.get('ROS_DOMAIN_ID', '0'), ros_time=self.get_clock().now().nanoseconds / 1e9,
                          sim_time=use_sim_time, cpu=None, memory_percent=None, temperature=None)
            try:
                cpu = [int(x) for x in Path('/proc/stat').read_text().splitlines()[0].split()[1:9]]
                total, idle = sum(cpu), cpu[3] + cpu[4]
                if self.last_cpu and total > self.last_cpu[0]:
                    health['cpu'] = 100 * (1 - (idle-self.last_cpu[1])/(total-self.last_cpu[0]))
                self.last_cpu = (total, idle)
                mem = {line.split(':')[0]: int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()}
                health['memory_percent'] = 100 * (1 - mem['MemAvailable'] / mem['MemTotal'])
                temperatures = []
                for zone in Path('/sys/class/thermal').glob('thermal_zone*'):
                    kind = (zone / 'type').read_text().lower()
                    if 'cpu' in kind or 'soc' in kind:
                        temperatures.append(float((zone / 'temp').read_text()) / 1000)
                if temperatures:
                    health['temperature'] = max(temperatures)
            except (OSError, ValueError, KeyError):
                pass
            state.put('health', health)

    node = None
    server = None
    server_thread = None
    can_monitor = None
    try:
        node = Bridge()
        server = make_server(state, args.host, args.port)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        state.sensor_manager = SensorManager(state)
        if args.with_robot:
            state.sensor_manager.start_auxiliary('sensor_core', [
                'ros2', 'launch', str(Path(__file__).resolve().parent / '../robot/physical.launch.py'),
                'tracer:=false', 'arm:=false', 'calibration:=true',
                'head_camera:=false', 'wrist_camera:=false', 'g2_lidar:=false', 'mid360_lidar:=false'])
        can_monitor = CanMonitor(state, state.sensor_manager, args.with_robot)
        node.discover()
        if args.with_robot:
            for key in SENSORS:
                try:
                    state.sensor_manager.request(key, 'start')
                except (ValueError, RuntimeError) as exc:
                    state.log(f'{key}: {exc}', 'warn', 'sensor-control')
        if args.mapping:
            try:
                state.sensor_manager.service_request('mapping', 'start', resume_params=getattr(args, 'resume_mapping_params', None))
            except (ValueError, RuntimeError) as exc:
                state.log(f'mapping: {exc}', 'warn', 'sensor-control')
        if args.detection:
            try:
                state.sensor_manager.service_request('detection', 'start')
            except (ValueError, RuntimeError) as exc:
                state.log(f'detection: {exc}', 'warn', 'sensor-control')
        print(f'INHA_CONTROL_CENTER_READY http://{args.host}:{args.port}', flush=True)
        print('Jetson web server. Ctrl+C stops processes started by this control center.', flush=True)
        try:
            import bridge_profiler
            bridge_profiler.start()
        except Exception as exc:
            print(f'bridge profiler unavailable: {exc}', flush=True)
        while rclpy.ok() and not state.shutdown_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if node and hasattr(node, 'control'):
            node.control.close()
        if node and hasattr(node, 'odom_inputs'):
            node.odom_inputs.close()
        state.closed.set()
        with state.condition:
            state.condition.notify_all()
        if can_monitor:
            can_monitor.close()
        if state.sensor_manager:
            state.sensor_manager.close()
        if server:
            if server_thread:
                server.shutdown()
            server.server_close()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--config')
    parser.add_argument('--sim-time', action='store_true')
    parser.add_argument('--with-robot', action='store_true', help='Start physical TF/odom core and four owned sensor drivers')
    parser.add_argument('--mapping', action='store_true', help='Start SLAM Toolbox on the Jetson')
    parser.add_argument('--resume-mapping-params', help='Initial SLAM profile with a serialized map and map_start_pose; later resets use the normal profile')
    parser.add_argument('--detection', action=argparse.BooleanOptionalAction, default=True, help='Start full-frame Head TensorRT by default; --no-detection disables it')
    run(parser.parse_args())
