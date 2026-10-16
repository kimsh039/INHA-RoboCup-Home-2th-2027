"""Associate YOLO boxes with measured MID-360 returns in odom.

The camera identifies the object; only LiDAR supplies its metric position. The
captured returns can then be retained by the approach controller outside RGB FOV.
"""
from collections import deque
from bisect import bisect_left
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
        self.fixed = getattr(node, 'fixed', 'map')
        self.poses = deque(maxlen=200)
        self.latest_odometry, self.odometry_meta = {}, {}
        self.odometry_epoch = 0
        self.odometry_reference = None
        self.protection_values, self.protection_received = {}, {}
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
            self.node.create_subscription(String, '/collision/status',
                                          lambda msg: self.protection_input('guard', msg), 1),
            self.node.create_subscription(String, '/collision/actuator_status',
                                          lambda msg: self.protection_input('actuator', msg), 1),
        ]
        self.timer = self.node.create_timer(.05, self.flush)
        self.executor = SingleThreadedExecutor(context=self.node.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.spin, name='detection-inputs', daemon=True)
        self.thread.start()
        # EKF poses on their own node/executor: queued behind TF (~100 Hz) and cloud callbacks they
        # arrived >0.4 s late and the approach aborted with "EKF 오도메트리 수신 대기".
        self.odom_node = Node('inha_detection_odometry', parameter_overrides=[Parameter(
            'use_sim_time', value=bool(node.get_parameter('use_sim_time').value))])
        self.odom_subscription = self.odom_node.create_subscription(
            Odometry, settings.get('odom_topic', '/odom_ekf'), self.odometry_input,
            QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.odom_executor = SingleThreadedExecutor(context=self.odom_node.context)
        self.odom_executor.add_node(self.odom_node)
        self.odom_thread = threading.Thread(target=self.spin_odometry, name='detection-odometry', daemon=True)
        self.odom_thread.start()

    def spin(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    def spin_odometry(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self.odom_executor.spin()
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

    def visual_target(self, label, previous=None, since=None):
        """Measured image bearing only; does not require or infer object depth."""
        from rclpy.time import Time
        with self.lock:
            detection, infos = self.latest_detection, tuple(self.infos)
        now = self.node.get_clock().now().nanoseconds/1e9
        if (not detection or not detection.get('valid') or not detection.get('measured')
                or not -.1 <= now-detection.get('stamp', 0.) <= .5
                or (since is not None and detection.get('stamp', 0.) < since)):
            return None
        objects = [o for o in detection.get('objects', []) if o.get('label') == label
                   and all(isinstance(o.get(k), (int, float)) and math.isfinite(o[k])
                           for k in ('x', 'y', 'width', 'height', 'confidence'))
                   and min(o['width'], o['height']) >= 2.]
        if not objects or not infos:
            return None
        stamp, frame = detection['stamp'], detection.get('frame')
        info = min(infos, key=lambda m: abs(seconds(m.header.stamp)-stamp))
        if (abs(seconds(info.header.stamp)-stamp) > .2 or info.header.frame_id != frame
                or (info.width, info.height) != (detection.get('width'), detection.get('height'))
                or info.distortion_model not in ('plumb_bob', 'rational_polynomial', '')
                or not all(math.isfinite(v) for v in (*info.k, *info.d))
                or min(info.k[0], info.k[4]) <= 0):
            return None
        if previous is not None:
            x, y, w, h = previous['bbox']
            candidates = [o for o in objects if math.hypot(
                (o['x']+o['width']/2-x-w/2)/info.width,
                (o['y']+o['height']/2-y-h/2)/info.height) <= .18]
            if not candidates:
                return None
            chosen = min(candidates, key=lambda o: math.hypot(
                (o['x']+o['width']/2-x-w/2)/info.width,
                (o['y']+o['height']/2-y-h/2)/info.height))
        else:
            chosen = max(objects, key=lambda o: o['confidence'])
        pixel = np.array([[[chosen['x']+chosen['width']/2,
                            chosen['y']+chosen['height']/2]]], dtype=float)
        ray = cv2.undistortPoints(pixel, np.array(info.k).reshape(3, 3),
                                 np.array(info.d) if len(info.d) else None)[0, 0]
        try:
            rotation, _ = self.transform(frame, Time(nanoseconds=round(stamp*1e9)).to_msg(), target=self.base)
        except Exception:
            return None
        direction = rotation @ np.array([ray[0], ray[1], 1.])
        if not np.isfinite(direction).all() or np.linalg.norm(direction[:2]) < 1e-6:
            return None
        return dict(label=label, stamp=stamp, bearing=float(math.atan2(direction[1], direction[0])),
                    bbox=[chosen[k] for k in ('x', 'y', 'width', 'height')],
                    confidence=float(chosen['confidence']), frame=frame, width=info.width, height=info.height)

    def odometry_input(self, msg):
        linear, angular = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = seconds(msg.header.stamp)
        if (msg.header.frame_id != 'odom' or msg.child_frame_id != self.base
                or not all(math.isfinite(v) for v in (linear, angular, stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                or stamp <= 0 or q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < 1e-12):
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            previous = self.odometry_reference
            if previous is not None:
                old_stamp, x, y, old_yaw = previous
                elapsed = stamp-old_stamp
                if -.1 <= elapsed <= 0:
                    return
                angle = math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw))
                if (elapsed < -.1 or math.hypot(p.x-x, p.y-y) > .08+.4*max(0., elapsed)
                        or abs(angle) > .12+.8*max(0., elapsed)):
                    self.odometry_epoch += 1
                    self.poses.clear()
            self.odometry_reference = stamp, p.x, p.y, yaw
            self.poses.append((stamp, p.x, p.y, p.z, yaw))
            self.latest_odometry = dict(linear=float(linear), angular=float(angular))
            self.odometry_meta = dict(stamp=stamp, received=time.monotonic(), epoch=self.odometry_epoch)
            self.stats['odom_received'] += 1

    def odometry(self):
        with self.lock:
            return dict(self.latest_odometry), dict(self.odometry_meta)

    def protection_input(self, key, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            with self.lock:
                self.protection_values[key] = value
                self.protection_received[key] = time.monotonic()
        except (ValueError, TypeError):
            pass

    def protection(self):
        with self.lock:
            return (dict(self.protection_values.get('guard') or {}),
                    dict(self.protection_values.get('actuator') or {}), dict(self.protection_received))

    def transform(self, source, stamp=None, target='odom'):
        from rclpy.time import Time
        if source == target:
            return np.eye(3), np.zeros(3)
        # Both sources describe the same EKF transform. Its Odometry callback
        # can already contain the measured pose while a separate /tf backlog
        # makes Buffer report future extrapolation and discard valid clouds.
        # Compose sensor extrinsics with the pose at the ORIGINAL stamp.
        if target == 'odom' and source != self.fixed:
            rotation, offset = self.odometry_transform(stamp)
            sensor_rotation, sensor_offset = self.transform(source, stamp, target=self.base)
            return rotation @ sensor_rotation, rotation @ sensor_offset+offset
        if source == 'odom' and target != self.fixed:
            rotation, offset = self.odometry_transform(stamp)
            sensor_rotation, sensor_offset = self.transform(self.base, stamp, target=target)
            inverse = sensor_rotation @ rotation.T
            return inverse, sensor_offset-inverse @ offset
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

    def odometry_transform(self, stamp=None):
        """Interpolate the planar EKF pose; never extrapolate across missing data."""
        with self.lock:
            poses = tuple(self.poses)
            received = self.odometry_meta.get('received')
        now = self.node.get_clock().now().nanoseconds/1e9
        # 1.0 s, not 0.4 s: the shared Python bridge can delay this thread by a few hundred ms.
        if (not poses or received is None or time.monotonic()-received > 1.
                or not -.1 <= now-poses[-1][0] <= 1.):
            raise RuntimeError('점군 변환용 현재 EKF 오도메트리 수신 대기')
        requested = seconds(stamp) if stamp and (stamp.sec or stamp.nanosec) else poses[-1][0]
        index = bisect_left([p[0] for p in poses], requested)
        if index < len(poses) and abs(poses[index][0]-requested) <= 1e-6:
            _, x, y, z, yaw = poses[index]
        else:
            if index == 0 or index == len(poses):
                raise RuntimeError('점군 측정 시각의 EKF 자세 이력 대기 · 외삽하지 않음')
            a, b = poses[index-1], poses[index]
            interval = b[0]-a[0]
            if interval <= 0 or interval > .3:
                raise RuntimeError('점군 측정 시각의 EKF 자세 간격 초과')
            weight = (requested-a[0])/interval
            x, y, z = (a[j]+weight*(b[j]-a[j]) for j in (1, 2, 3))
            angle = math.atan2(math.sin(b[4]-a[4]), math.cos(b[4]-a[4]))
            yaw = a[4]+weight*angle
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]), np.array([x, y, z])

    def reset(self):
        with self.lock:
            self.epoch += 1
            self.clouds.clear()
            self.surface_clouds.clear()
            self.pending.clear()
            self.infos.clear()
            self.latest_detection = None
            self.latest_odometry, self.odometry_meta = {}, {}
            self.odometry_reference = None
            self.poses.clear()
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

    def edge_observation(self):
        """Recent surface returns and the EKF pose at their newest source stamp."""
        from rclpy.time import Time
        now = self.node.get_clock().now().nanoseconds/1e9
        with self.lock:
            clouds = tuple(self.surface_clouds)
        if not clouds or not -.05 <= now-clouds[-1][0] <= .35:
            return None
        stamp = clouds[-1][0]
        points = np.concatenate([p for s, p, _ in clouds if stamp-.25 <= s <= stamp])
        rotation, offset = self.odometry_transform(Time(nanoseconds=round(stamp*1e9)).to_msg())
        pose = (float(offset[0]), float(offset[1]), math.atan2(rotation[1, 0], rotation[0, 0]))
        return stamp, points, pose, float(offset[2])

    def diagnostics(self):
        now = self.node.get_clock().now().nanoseconds/1e9
        with self.lock:
            info = dict(self.stats, obstacle_frames=len(self.clouds), surface_frames=len(self.surface_clouds),
                        pending_frames=len(self.pending), camera_info_frames=len(self.infos))
            info['cloud_source_age'] = now-self.clouds[-1][0] if self.clouds else None
            info['camera_info_source_age'] = now-seconds(self.infos[-1].header.stamp) if self.infos else None
            info['yolo_source_age'] = now-self.latest_detection['stamp'] if self.latest_detection and self.latest_detection.get('stamp') else None
            info['odom_source_age'] = now-self.odometry_meta['stamp'] if self.odometry_meta else None
            info['pose_history_frames'] = len(self.poses)
            info['pose_transform_source'] = 'timestamped_ekf_odometry'
        return info

    def close(self):
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.)
        if not self.thread.is_alive():
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        self.odom_executor.shutdown(timeout_sec=2.)
        self.odom_thread.join(timeout=2.)
        if not self.odom_thread.is_alive():
            self.odom_executor.remove_node(self.odom_node)
            self.odom_node.destroy_node()

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
        recent = [(s, p) for s, p in clouds if stamp-2.0 <= s <= stamp+.1]
        if not recent or max(s for s, _ in recent) < stamp-.25:
            return None, 'YOLO 촬영 시각의 LiDAR 관측 대기'
        capture = Time(nanoseconds=round(stamp*1e9)).to_msg()
        try:
            rotation, offset = self.transform('odom', capture, target=frame)
        except Exception:
            return None, 'YOLO 촬영 시각의 Head ↔ odom TF 대기'
        points = np.concatenate([p for _, p in recent])
        point_stamps = np.concatenate([np.full(len(p), s) for s, p in recent])
        projection_started=time.monotonic()
        from camera_frustum import in_frustum
        camera = points @ rotation.T + offset
        valid = in_frustum(camera,info.width,info.height,info.k,info.d)
        with self.lock:
            self.stats['projection_input_points']=len(camera)
            self.stats['projection_frustum_points']=int(np.count_nonzero(valid))
        points, camera, point_stamps = points[valid], camera[valid], point_stamps[valid]
        if not len(points):
            return None, 'Head 시야 안 LiDAR 점 없음'
        pixels, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), np.array(info.k).reshape(3, 3),
                                      np.array(info.d) if len(info.d) else None)
        pixels = pixels.reshape(-1, 2)
        # Brown distortion is not one-to-one far outside the physical image
        # cone: a side-wall return can fold back onto a central YOLO box.
        # Only keep points whose projected pixel reconstructs its original
        # optical ray. Being in front of the camera (z > 0) is insufficient.
        in_image = (np.isfinite(pixels).all(axis=1) & (pixels[:, 0] >= 0.)
                    & (pixels[:, 0] < info.width) & (pixels[:, 1] >= 0.)
                    & (pixels[:, 1] < info.height))
        indices = np.flatnonzero(in_image)
        consistent = np.zeros(len(points), dtype=bool)
        if len(indices):
            rays = cv2.undistortPoints(pixels[indices].reshape(-1, 1, 2),
                np.array(info.k).reshape(3, 3), np.array(info.d) if len(info.d) else None).reshape(-1, 2)
            optical = camera[indices, :2]/camera[indices, 2:3]
            error = np.linalg.norm((rays-optical)*np.array([info.k[0], info.k[4]]), axis=1)
            consistent[indices] = np.isfinite(error) & (error <= 2.)
        with self.lock:
            self.stats['projection_ms']=(time.monotonic()-projection_started)*1000.
            self.stats['projection_image_candidates'] = len(indices)
            self.stats['projection_off_ray_rejected'] = int(len(indices)-np.count_nonzero(consistent))
        points, camera, point_stamps, pixels = (a[consistent] for a in (points, camera, point_stamps, pixels))
        if not len(points):
            return None, 'YOLO 영상의 실제 카메라 광선에 대응하는 LiDAR 점 관측 중'
        objects = []
        for source in detection.get('objects', []):
            item = dict(source, position_valid=False, position_reason='물체 박스 안 LiDAR 점 부족', lidar_count=0)
            x, y, w, h = [source.get(k, 0.) for k in ('x', 'y', 'width', 'height')]
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y, w, h)) or min(w, h) < 2:
                objects.append(item)
                continue
            # Start at the box centre, then use more of a small box if sparse
            # returns cannot support a coordinate. Never include outside pixels.
            for margin in (.22, .08, .02):
                inside = ((pixels[:, 0] > x+margin*w) & (pixels[:, 0] < x+(1-margin)*w)
                          & (pixels[:, 1] > y+margin*h) & (pixels[:, 1] < y+(1-margin)*h))
                found, ranges, stamps = points[inside], camera[inside, 2], point_stamps[inside]
                # Count distinct spatial returns, keeping the newest hit per cell.
                if len(found):
                    order = np.argsort(stamps)[::-1]
                    _, unique = np.unique(np.floor(found[order]/.01).astype(np.int32), axis=0, return_index=True)
                    selected = order[unique]
                    found, ranges, stamps = found[selected], ranges[selected], stamps[selected]
                if len(found) < 3:
                    item['lidar_count'] = max(item['lidar_count'], len(found))
                    continue
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
                                    capture_stamp=stamp, point_method='yolo_mid360_bbox_cluster', source_topic=self.topic,
                                    bbox_margin=margin, projection_verified=True, camera_frame=frame)
                        # Object coordinates must be usable before the tabletop
                        # is understood. Surface fitting runs independently in
                        # the approach owner and cannot gate this observation.
                        item['support_surface'] = None
                        break
                item['lidar_count'] = max(item['lidar_count'], len(found))
            objects.append(item)
        return objects, ''
