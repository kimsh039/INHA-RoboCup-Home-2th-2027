"""Associate YOLO boxes with measured MID-360 returns in odom.

The camera identifies the object; only LiDAR supplies its metric position. The
captured returns can then be retained by the approach controller outside RGB FOV.
"""
from collections import deque
import json
import math
import threading
import time

import cv2
import numpy as np


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class HeadLidar:
    def __init__(self, node, settings):
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import CameraInfo, PointCloud2
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from tf2_ros import Buffer, TransformListener
        # A separate node/executor is essential: spinning the web bridge from a
        # second thread would contend for its image/cloud callbacks again.
        self.node = Node('inha_detection_inputs', parameter_overrides=[Parameter(
            'use_sim_time', value=bool(node.get_parameter('use_sim_time').value))])
        self.lock = threading.RLock()
        self.epoch = 0
        self.clouds, self.pending, self.infos = deque(maxlen=24), deque(maxlen=24), deque(maxlen=90)
        self.surface_clouds = deque(maxlen=24)
        self.surface_cache, self.surface_cache_point, self.surface_cache_at = None, None, 0.
        self.latest_detection = None
        self.base = node.base
        self.latest_odometry, self.odometry_meta = {}, {}
        self.stats = dict(obstacle_received=0, surface_received=0, info_received=0, yolo_received=0,
                          obstacle_transformed=0, surface_transformed=0, tf_retries=0, tf_expired=0,
                          last_error='', input_error='')
        self.stats['odom_received'] = 0
        self.topic = settings.get('cloud_topic', '/mid360/points_obstacles')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions = [
            self.node.create_subscription(PointCloud2, self.topic, self.receive, qos),
            self.node.create_subscription(PointCloud2, settings.get('surface_topic', '/mid360/points_filtered'),
                                          lambda msg: self.receive(msg, True), qos),
            self.node.create_subscription(CameraInfo, settings.get('camera_info_topic', '/head/calibrated/color/camera_info'),
                                          self.camera_info, qos),
            self.node.create_subscription(String, settings.get('objects_topic', '/detection/head/objects'),
                                          self.detections, qos),
            self.node.create_subscription(Odometry, settings.get('odom_topic', '/odom_ekf'), self.odometry_input, qos),
        ]
        self.timer = self.node.create_timer(.05, self.flush)
        self.executor = SingleThreadedExecutor(context=self.node.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.spin, name='detection-inputs', daemon=True)
        self.thread.start()

    def spin(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:
            with self.lock:
                self.stats['input_error'] = str(exc)[:250]

    def receive(self, msg, surface=False):
        with self.lock:
            self.pending.append((msg, surface))
            self.stats['surface_received' if surface else 'obstacle_received'] += 1

    def camera_info(self, msg):
        with self.lock:
            self.infos.append(msg)
            self.stats['info_received'] += 1

    def detections(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict) or value.get('mode') != 'all_objects' or not isinstance(value.get('objects'), list):
                raise ValueError('invalid_head_objects_json')
            with self.lock:
                self.latest_detection = value
                self.stats['yolo_received'] += 1
        except (ValueError, TypeError) as exc:
            with self.lock:
                self.stats['last_error'] = str(exc)[:200]

    def detection(self):
        with self.lock:
            return self.latest_detection

    def odometry_input(self, msg):
        linear, angular = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if (msg.header.frame_id != 'odom' or msg.child_frame_id != self.base
                or not math.isfinite(linear+angular)):
            return
        with self.lock:
            self.latest_odometry = dict(linear=float(linear), angular=float(angular))
            self.odometry_meta = dict(stamp=seconds(msg.header.stamp), received=time.monotonic())
            self.stats['odom_received'] += 1

    def odometry(self):
        with self.lock:
            return dict(self.latest_odometry), dict(self.odometry_meta)

    def transform(self, source, stamp=None, target='odom'):
        from rclpy.time import Time
        if source == target:
            return np.eye(3), np.zeros(3)
        when = Time.from_msg(stamp) if stamp and (stamp.sec or stamp.nanosec) else Time()
        tf = self.tf.lookup_transform(target, source, when).transform
        q, p = tf.rotation, tf.translation
        x, y, z, w = np.asarray([q.x, q.y, q.z, q.w], float)
        norm = math.sqrt(x*x+y*y+z*z+w*w)
        if not math.isfinite(norm) or norm < 1e-9:
            raise ValueError('invalid_detection_tf_rotation')
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        return rotation, np.array([p.x, p.y, p.z])

    def reset(self):
        with self.lock:
            self.epoch += 1
            self.clouds.clear()
            self.surface_clouds.clear()
            self.pending.clear()
            self.infos.clear()
            self.latest_detection = None
            self.latest_odometry, self.odometry_meta = {}, {}
            self.surface_cache = self.surface_cache_point = None
            self.surface_cache_at = 0.

    def flush(self):
        now = self.node.get_clock().now().nanoseconds/1e9
        with self.lock:
            batch, epoch = tuple(self.pending), self.epoch
            self.pending.clear()
        for msg, surface in batch:
            stamp = seconds(msg.header.stamp)
            if not 0 < stamp or not -.1 <= now-stamp <= 1.2:
                continue
            try:
                rotation, offset = self.transform(msg.header.frame_id, msg.header.stamp)
                if surface:
                    _, sensor_origin = self.transform('livox_frame', msg.header.stamp)
            except Exception as exc:
                with self.lock:
                    if epoch != self.epoch:
                        continue
                    self.stats['last_error'] = str(exc)[:250]
                    if now-stamp < .4:
                        self.stats['tf_retries'] += 1
                        self.pending.append((msg, surface))
                    else:
                        self.stats['tf_expired'] += 1
                continue  # never let a pending surface frame stall object frames
            try:
                fields = {f.name: f for f in msg.fields}
                if not all(k in fields and fields[k].datatype in (7, 8) for k in ('x', 'y', 'z')):
                    raise ValueError('invalid_lidar_xyz_fields')
                endian = '>' if msg.is_bigendian else '<'
                dtype = np.dtype(dict(names=['x', 'y', 'z'],
                                      formats=[endian+('f4' if fields[k].datatype == 7 else 'f8') for k in ('x', 'y', 'z')],
                                      offsets=[fields[k].offset for k in ('x', 'y', 'z')], itemsize=msg.point_step))
                records = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                                     strides=(msg.row_step, msg.point_step))
                points = np.column_stack([records[k].ravel() for k in ('x', 'y', 'z')])
                points = points[np.isfinite(points).all(axis=1)]
                world = points @ rotation.T+offset
            except (ValueError, TypeError) as exc:
                with self.lock:
                    self.stats['last_error'] = str(exc)[:200]
                continue
            with self.lock:
                if epoch != self.epoch:
                    continue
                destination = self.surface_clouds if surface else self.clouds
                if destination and stamp < destination[-1][0]:
                    continue
                if len(world):
                    destination.append((stamp, world, sensor_origin) if surface else (stamp, world))
                self.stats['surface_transformed' if surface else 'obstacle_transformed'] += 1
                self.stats['last_error'] = ''
        with self.lock:
            for cache in (self.clouds, self.surface_clouds):
                while cache and now-cache[0][0] > 2.0:
                    cache.popleft()

    def table(self, point, stamp):
        from grasp_approach import support_surface
        with self.lock:
            epoch = self.epoch
            if (stamp >= self.surface_cache_at and stamp-self.surface_cache_at < .8
                    and self.surface_cache_point is not None
                    and np.linalg.norm(np.asarray(point)-self.surface_cache_point) <= .25):
                return self.surface_cache
            clouds = tuple(c for c in self.surface_clouds if stamp-2.0 <= c[0] <= stamp+.15)
        surface = support_surface(clouds, point)
        with self.lock:
            if epoch != self.epoch:
                return None
            self.surface_cache = surface
            self.surface_cache_point, self.surface_cache_at = np.asarray(point), stamp
        return surface

    def diagnostics(self):
        now = self.node.get_clock().now().nanoseconds/1e9
        with self.lock:
            info = dict(self.stats, obstacle_frames=len(self.clouds), surface_frames=len(self.surface_clouds),
                        pending_frames=len(self.pending), camera_info_frames=len(self.infos))
            info['cloud_source_age'] = now-self.clouds[-1][0] if self.clouds else None
            info['camera_info_source_age'] = now-seconds(self.infos[-1].header.stamp) if self.infos else None
            info['yolo_source_age'] = now-self.latest_detection['stamp'] if self.latest_detection and self.latest_detection.get('stamp') else None
            info['odom_source_age'] = now-self.odometry_meta['stamp'] if self.odometry_meta else None
        return info

    def close(self):
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.)
        if not self.thread.is_alive():
            self.executor.remove_node(self.node)
            self.node.destroy_node()

    def locate(self, detection):
        from rclpy.time import Time
        with self.lock:
            infos, clouds = tuple(self.infos), tuple(self.clouds)
        stamp, frame = detection.get('stamp'), detection.get('frame')
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or not frame:
            return None, 'YOLO 촬영 시각·좌표계 대기'
        if not infos:
            return None, 'Head CameraInfo 수신 대기'
        if not clouds:
            return None, '물체 위치용 MID-360 점군 변환 대기'
        info = min(infos, key=lambda m: abs(seconds(m.header.stamp)-stamp))
        if (abs(seconds(info.header.stamp)-stamp) > .2 or info.header.frame_id != frame
                or (info.width, info.height) != (detection.get('width'), detection.get('height'))):
            return None, 'YOLO 영상과 CameraInfo 시각·해상도·좌표계 불일치'
        if (info.distortion_model not in ('plumb_bob', 'rational_polynomial', '')
                or not all(math.isfinite(v) for v in (*info.k, *info.d)) or min(info.k[0], info.k[4]) <= 0):
            return None, 'Head 내부 보정 형식 오류'
        recent = [(s, p) for s, p in clouds if stamp-1.0 <= s <= stamp+.1]
        if not recent or max(s for s, _ in recent) < stamp-.25:
            return None, 'YOLO 촬영 시각의 LiDAR 관측 대기'
        capture = Time(nanoseconds=round(stamp*1e9)).to_msg()
        try:
            rotation, offset = self.transform('odom', capture, target=frame)
        except Exception:
            return None, 'YOLO 촬영 시각의 Head ↔ odom TF 대기'
        points = np.concatenate([p for _, p in recent])
        point_stamps = np.concatenate([np.full(len(p), s) for s, p in recent])
        camera = points @ rotation.T + offset
        valid = np.isfinite(camera).all(axis=1) & (camera[:, 2] > .15) & (camera[:, 2] < 5.)
        points, camera, point_stamps = points[valid], camera[valid], point_stamps[valid]
        if not len(points):
            return None, 'Head 시야 안 LiDAR 점 없음'
        pixels, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), np.array(info.k).reshape(3, 3),
                                      np.array(info.d) if len(info.d) else None)
        pixels = pixels.reshape(-1, 2)
        objects = []
        for source in detection.get('objects', []):
            item = dict(source, position_valid=False, position_reason='물체 박스 안 LiDAR 점 부족')
            x, y, w, h = [source.get(k, 0.) for k in ('x', 'y', 'width', 'height')]
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y, w, h)) or min(w, h) < 2:
                objects.append(item)
                continue
            inside = ((pixels[:, 0] > x+.22*w) & (pixels[:, 0] < x+.78*w)
                      & (pixels[:, 1] > y+.22*h) & (pixels[:, 1] < y+.78*h))
            found, ranges, stamps = points[inside], camera[inside, 2], point_stamps[inside]
            # Count distinct spatial returns, keeping the newest hit per cell.
            if len(found):
                _, unique = np.unique(np.floor(found[::-1]/.01).astype(np.int32), axis=0, return_index=True)
                found, ranges, stamps = found[::-1][unique], ranges[::-1][unique], stamps[::-1][unique]
            if len(found) >= 3:
                # Nearest compact depth cluster suppresses the background behind
                # the object. Bbox association is an estimate, not segmentation.
                nearest = float(np.percentile(ranges, 20))
                cluster = np.abs(ranges-nearest) <= .08
                found, stamps = found[cluster], stamps[cluster]
                if len(found) >= 3:
                    center = np.median(found, axis=0)
                    cluster = np.linalg.norm(found-center, axis=1) <= .18
                    found, stamps = found[cluster], stamps[cluster]
                    if len(found) >= 3:
                        point = found[np.argmin(np.linalg.norm(found-np.median(found, axis=0), axis=1))]
                        sample = found[::max(1, math.ceil(len(found)/64))]
                        item.update(position_valid=True, position_reason='', point_odom=point.tolist(),
                                    lidar_points_odom=sample.tolist(), lidar_count=len(found),
                                    lidar_stamp=float(stamps.max()), lidar_oldest_stamp=float(stamps.min()),
                                    capture_stamp=stamp, point_method='yolo_mid360_bbox_cluster', source_topic=self.topic)
                        # Object coordinates must be usable before the tabletop
                        # is understood. Surface fitting runs independently in
                        # the approach owner and cannot gate this observation.
                        item['support_surface'] = None
            objects.append(item)
        return objects, ''
