#!/usr/bin/env python3
"""Sanitize, deskew and select SLAM scans without changing obstacle scans or TF.

G2 uses the driver's acquisition-order PointCloud/stamps, paired by header with
the calibrated LaserScan. Angular array indices are never used as timestamps.
MID-360 uses per-point timestamps from its PointCloud2 before height projection.
Each cloud is moved to its temporal midpoint using interpolated EKF poses and the
measured sensor extrinsic. See slam-scan-processing.md for sources and limitations.
"""
from collections import OrderedDict, deque
import copy
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, PointCloud, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

REPO = Path(__file__).resolve().parents[2]
NS = 1_000_000_000


def stamp_ns(header):
    return header.stamp.sec * NS + header.stamp.nanosec


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rotation(q):
    v = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(v)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid sensor quaternion')
    x, y, z, w = v / norm
    return np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])


class AwaitOdometry(Exception):
    """The required pose interval has not arrived yet."""


class Trajectory:
    def __init__(self, history_sec, max_gap_sec):
        self.samples = deque()
        self.history_ns = int(history_sec * NS)
        self.max_gap_ns = int(max_gap_sec * NS)

    def add(self, timestamp, pose):
        if self.samples:
            previous_time, previous = self.samples[-1]
            if timestamp <= previous_time:
                return False
            # Keep yaw continuous across +/-pi for temporal interpolation.
            pose[2] = previous[2] + math.atan2(math.sin(pose[2] - previous[2]),
                                             math.cos(pose[2] - previous[2]))
        self.samples.append((timestamp, pose))
        while len(self.samples) > 2 and self.samples[1][0] < timestamp - self.history_ns:
            self.samples.popleft()
        return True

    def at(self, timestamps):
        ts = np.asarray(timestamps, dtype=np.int64)
        if len(self.samples) < 2 or ts.max() > self.samples[-1][0]:
            raise AwaitOdometry('Waiting for timestamped EKF poses')
        if ts.min() < self.samples[0][0]:
            raise ValueError('Scan is older than EKF pose history')
        times = np.array([s[0] for s in self.samples], dtype=np.int64)
        poses = np.array([s[1] for s in self.samples])
        right = np.clip(np.searchsorted(times, ts, side='right'), 1, len(times) - 1)
        left = right - 1
        gap = times[right] - times[left]
        # Also reject a missing odometry interval between queried endpoints.
        begin = max(0, int(np.searchsorted(times, ts.min(), side='right')) - 1)
        end = min(len(times) - 1, int(np.searchsorted(times, ts.max(), side='left')))
        if np.any(gap > self.max_gap_ns) or np.any(np.diff(times[begin:end+1]) > self.max_gap_ns):
            raise ValueError('EKF pose interval contains a gap; refusing extrapolation')
        weight = ((ts - times[left]) / gap)[:, None]
        return poses[left] + weight * (poses[right] - poses[left])


class SlamScan(Node):
    def __init__(self):
        super().__init__('g2_slam_scan')
        defaults = dict(source='g2', scan_topic='/scan', timed_cloud_topic='/g2/points_raw',
                        output_topic='/scan_slam', odom_topic='/odom_ekf', odom_frame='odom',
                        base_frame='base_link', odom_history_sec=3.0, max_odom_gap_sec=0.08,
                        max_pending_sec=0.4, minimum_time_interval=0.2,
                        minimum_travel_distance=0.04, minimum_travel_heading=0.02,
                        stationary_refresh_interval=0.0, minimum_valid_beams=40,
                        subscriber_warmup_sec=0.5, slam_pose_topic='/pose', slam_node_name='slam_toolbox',
                        initial_pose_topic='/initialpose', map_frame='map',
                        initial_slam_scans=3,
                        deskew_enabled=True, max_scan_duration=0.25,
                        min_range=0.35, max_range=8.0, min_height=0.1075, max_height=1.0575,
                        angle_min=-3.14159, angle_max=3.14159, angle_increment=0.017453,
                        point_time_field='timestamp', point_time_format='absolute_ns')
        self.settings = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.source = self.settings['source']
        if self.source not in ('g2', 'mid360'):
            raise ValueError('source must be g2 or mid360')
        if not 0 < self.settings['min_range'] < self.settings['max_range']:
            raise ValueError('Invalid SLAM range limits')
        if self.settings['angle_increment'] <= 0 or self.settings['angle_min'] >= self.settings['angle_max']:
            raise ValueError('Invalid scan angular limits')
        self.trajectory = Trajectory(self.settings['odom_history_sec'], self.settings['max_odom_gap_sec'])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pending = OrderedDict()
        self.last_pose = None
        self.last_stamp = None
        self.last_frame = None
        self.subscribers = set()
        self.subscriber_ready_at = 0.0
        self.awaiting_first_pose = False
        self.first_slam_stamp = None
        self.initial_pending = set()
        self.initial_confirmed = 0
        self.last_initial_pose_stamp = None
        self.last_input_stamp = None
        self.counts = dict(received=0, published=0, motion_filtered=0, rejected=0)
        self.reason = 'waiting for scans'
        self.pub = self.create_publisher(LaserScan, self.settings['output_topic'], qos_profile_sensor_data)
        # Fresh common-time observations for global initialization do not become
        # SLAM constraints merely because the initializer needs another scan.
        self.continuous_pub = self.create_publisher(
            LaserScan, self.settings['output_topic'] + '_continuous', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, self.settings['output_topic'] + '/status', 10)
        self.create_subscription(Odometry, self.settings['odom_topic'], self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, self.settings['slam_pose_topic'],
                                 self.on_slam_pose, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, self.settings['initial_pose_topic'],
                                 self.on_initial_pose, 10)
        if self.source == 'g2':
            self.scale = json.loads((REPO / 'HW/calibration/physical/base_2dlidar/vs_mid360/calibration.json').read_text())['g2_range_scale']
            self.create_subscription(LaserScan, self.settings['scan_topic'], self.on_scan, qos_profile_sensor_data)
            self.create_subscription(PointCloud, self.settings['timed_cloud_topic'], self.on_g2_cloud, qos_profile_sensor_data)
        else:
            self.create_subscription(PointCloud2, self.settings['timed_cloud_topic'], self.on_mid_cloud, qos_profile_sensor_data)
        self.create_timer(0.02, self.drain)
        self.create_timer(1.0, self.status)
        self.get_logger().info(f'{self.source}: timed points + {self.settings["odom_topic"]} -> '
                               f'{self.settings["output_topic"]}; deskew={self.settings["deskew_enabled"]}, '
                               'movement OR rotation gate; no TF publisher')

    def on_odom(self, msg):
        if msg.header.frame_id != self.settings['odom_frame'] or msg.child_frame_id != self.settings['base_frame']:
            self.get_logger().warning('Ignoring EKF odometry with unexpected frames', throttle_duration_sec=5.)
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        if q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12:
            return
        timestamp = stamp_ns(msg.header)
        if timestamp <= 0:
            return
        if self.trajectory.samples and timestamp < self.trajectory.samples[-1][0]:
            # Clear on clock rewind (e.g. a new rosbag replay); never interpolate across it.
            self.trajectory.samples.clear()
            self.pending.clear()
            self.last_pose = self.last_stamp = self.last_frame = None
            self.get_logger().warning('EKF time moved backwards; cleared scan/pose history', throttle_duration_sec=5.)
        self.trajectory.add(timestamp, np.array([p.x, p.y, yaw(q)]))

    def queue(self, header, key, msg):
        timestamp = stamp_ns(header)
        if timestamp <= 0 or not header.frame_id:
            return
        if key in ('scan', 'mid'):
            self.counts['received'] += 1
            if self.last_input_stamp is not None and timestamp < self.last_input_stamp:
                self.pending.clear()
                self.last_pose = self.last_stamp = self.last_frame = None
            self.last_input_stamp = timestamp
        entry = self.pending.setdefault(timestamp, dict(received=time.monotonic()))
        entry[key] = msg
        while len(self.pending) > 8:
            self.pending.popitem(last=False)
            self.counts['rejected'] += 1

    def on_scan(self, msg):
        self.queue(msg.header, 'scan', msg)

    def on_slam_pose(self, msg):
        timestamp = stamp_ns(msg.header)
        if timestamp in self.initial_pending:
            self.initial_pending.remove(timestamp)
            self.initial_confirmed += 1
            if self.initial_confirmed >= self.settings['initial_slam_scans']:
                self.awaiting_first_pose = False
                self.initial_pending.clear()

    def on_initial_pose(self, msg):
        # Localization consumes a requested pose on its next scan. A stationary
        # robot otherwise never sends that scan after the initial seed finishes.
        if msg.header.frame_id != self.settings['map_frame']:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        if q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12:
            return
        backend = self.settings['slam_node_name']
        if not any(info.node_name == backend for info in
                   self.get_subscriptions_info_by_topic(self.settings['output_topic'])):
            return
        # Mapping has no backend subscription to initialpose. Do not add static
        # mapping constraints merely because someone publishes an initial pose.
        if not any(info.node_name == backend for info in
                   self.get_subscriptions_info_by_topic(self.settings['initial_pose_topic'])):
            return
        self.pending.clear()
        self.last_pose = self.last_stamp = self.last_frame = None
        self.subscriber_ready_at = time.monotonic() + self.settings['subscriber_warmup_sec']
        self.awaiting_first_pose = True
        self.first_slam_stamp = None
        self.initial_pending.clear()
        self.initial_confirmed = 0
        self.last_initial_pose_stamp = stamp_ns(msg.header)
        self.reason = 'initial pose requested; restarting bounded localization observations'
        self.get_logger().info(self.reason)

    def on_g2_cloud(self, msg):
        if self.settings['deskew_enabled']:
            self.queue(msg.header, 'cloud', msg)

    def on_mid_cloud(self, msg):
        self.queue(msg.header, 'mid', msg)

    def extrinsic(self, frame, timestamp):
        if frame == self.settings['base_frame']:
            return np.eye(3), np.zeros(3)
        tr = self.tf.lookup_transform(self.settings['base_frame'], frame, Time(nanoseconds=timestamp)).transform
        xyz = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        if not np.all(np.isfinite(xyz)):
            raise ValueError('Invalid sensor translation')
        return rotation(tr.rotation), xyz

    def common_time_points(self, points, timestamps, reference, r, offset, sensor_output):
        # T_laser(ref)^-1 * T_base(t_i) * T_base_laser * p_laser(t_i).
        poses = self.trajectory.at(np.r_[timestamps, reference])
        pose = poses[-1]
        body = points @ r.T + offset
        c, s = np.cos(poses[:-1, 2]), np.sin(poses[:-1, 2])
        world_x = c * body[:, 0] - s * body[:, 1] + poses[:-1, 0]
        world_y = s * body[:, 0] + c * body[:, 1] + poses[:-1, 1]
        dx, dy = world_x - pose[0], world_y - pose[1]
        cr, sr = math.cos(pose[2]), math.sin(pose[2])
        common_body = np.c_[cr * dx + sr * dy, -sr * dx + cr * dy, body[:, 2]]
        return ((common_body - offset) @ r if sensor_output else common_body), pose

    def g2(self, scan, cloud):
        if len(scan.ranges) < 2 or scan.angle_increment <= 0:
            raise ValueError('Invalid G2 angular geometry')
        out = copy.deepcopy(scan)
        out.range_min = max(scan.range_min, self.settings['min_range'])
        out.range_max = min(scan.range_max, self.settings['max_range'])
        if out.range_min >= out.range_max:
            raise ValueError('G2 and SLAM range limits do not overlap')
        timestamp = stamp_ns(scan.header)
        if not self.settings['deskew_enabled']:
            ranges = np.asarray(scan.ranges, dtype=float)
            good = np.isfinite(ranges) & (ranges >= out.range_min) & (ranges <= out.range_max)
            out.ranges = np.where(good, ranges, np.nan).tolist()
            return out, self.trajectory.at([timestamp])[0]
        if scan.header.frame_id != cloud.header.frame_id:
            raise ValueError('Paired G2 scan/cloud frames differ')
        if not math.isfinite(scan.scan_time) or not 0 < scan.scan_time <= self.settings['max_scan_duration']:
            raise ValueError('G2 has no usable acquisition duration')
        channels = {c.name: c.values for c in cloud.channels}
        offsets = np.asarray(channels.get('stamps', []), dtype=float)
        if not len(cloud.points) or len(offsets) != len(cloud.points):
            raise ValueError('G2 cloud lacks per-point stamps')
        points = np.array([[p.x, p.y, p.z] for p in cloud.points], dtype=float) * self.scale
        good = np.all(np.isfinite(points), axis=1) & np.isfinite(offsets)
        # SDK fixed_resolution can pad repeated final points beyond the acquisition.
        good &= (offsets >= 0) & (offsets < scan.scan_time)
        points, offsets = points[good], offsets[good]
        if not len(points):
            raise ValueError('G2 has no timed returns in this revolution')
        angle = np.arctan2(points[:, 1], points[:, 0])
        raw_index = np.ceil((angle - scan.angle_min) / scan.angle_increment).astype(int)
        inside = (raw_index >= 0) & (raw_index < len(scan.ranges))
        raw_index = np.clip(raw_index, 0, len(scan.ranges) - 1)
        masked_ranges = np.asarray(scan.ranges)[raw_index]
        radius = np.hypot(points[:, 0], points[:, 1])
        inside &= np.isfinite(masked_ranges) & (radius >= out.range_min) & (radius <= out.range_max)
        points, offsets = points[inside], offsets[inside]
        if not len(points):
            raise ValueError('G2 has no returns after masking and range limits')
        reference = timestamp + round(scan.scan_time * NS / 2)
        beam_times = timestamp + np.rint(offsets * NS).astype(np.int64)
        r, offset = self.extrinsic(scan.header.frame_id, reference)
        points, pose = self.common_time_points(points, beam_times, reference, r, offset, True)
        out.header.stamp = Time(nanoseconds=reference).to_msg()
        out.time_increment = 0.0
        self.bin_points(out, points)
        return out, pose

    def mid360(self, cloud):
        field = self.settings['point_time_field']
        names = {f.name for f in cloud.fields}
        if not {'x', 'y', 'z', field}.issubset(names):
            raise ValueError('MID-360 cloud lacks XYZ or configured point time field')
        data = point_cloud2.read_points(cloud, field_names=['x', 'y', 'z', field], skip_nans=False)
        points = np.c_[data['x'], data['y'], data['z']].astype(float)
        times = np.asarray(data[field], dtype=float)
        good = np.all(np.isfinite(points), axis=1) & np.isfinite(times)
        points, times = points[good], times[good]
        if not len(points):
            raise ValueError('MID-360 cloud has no finite timed points')
        header_time = stamp_ns(cloud.header)
        fmt = self.settings['point_time_format']
        if fmt == 'absolute_ns':
            # Installed driver uses absolute nanoseconds, despite the SDK name offset_time.
            offsets = times - header_time
        elif fmt == 'relative_ns':
            offsets = times
        elif fmt == 'relative_sec':
            offsets = times * NS
        else:
            raise ValueError('Unknown MID-360 point_time_format')
        if np.any(offsets < -1000) or np.any(offsets > self.settings['max_scan_duration'] * NS):
            raise ValueError('MID-360 point timestamps do not match the configured clock format')
        beam_times = header_time + np.rint(np.maximum(offsets, 0)).astype(np.int64)
        reference = int(beam_times.min()) + int(beam_times.max() - beam_times.min()) // 2
        r, offset = self.extrinsic(cloud.header.frame_id, reference)
        if self.settings['deskew_enabled']:
            points, pose = self.common_time_points(points, beam_times, reference, r, offset, False)
        else:
            points = points @ r.T + offset
            reference = header_time
            pose = self.trajectory.at([reference])[0]
        good = (points[:, 2] >= self.settings['min_height']) & (points[:, 2] <= self.settings['max_height'])
        out = LaserScan()
        out.header.frame_id = self.settings['base_frame']
        out.header.stamp = Time(nanoseconds=reference).to_msg()
        out.angle_min, out.angle_max = self.settings['angle_min'], self.settings['angle_max']
        out.angle_increment = self.settings['angle_increment']
        out.range_min, out.range_max = self.settings['min_range'], self.settings['max_range']
        out.scan_time = max(float(beam_times.max() - beam_times.min()) / NS, 0.001)
        out.time_increment = 0.0
        out.ranges = [float('nan')] * (int(round((out.angle_max - out.angle_min) / out.angle_increment)) + 1)
        self.bin_points(out, points[good])
        return out, pose

    @staticmethod
    def bin_points(out, points):
        count = len(out.ranges)
        ranges = np.full(count, np.inf)
        radius = np.hypot(points[:, 0], points[:, 1])
        angle = np.arctan2(points[:, 1], points[:, 0])
        index = np.rint((angle - out.angle_min) / out.angle_increment).astype(int)
        good = np.isfinite(radius) & (radius >= out.range_min) & (radius <= out.range_max)
        good &= (index >= 0) & (index < count)
        np.minimum.at(ranges, index[good], radius[good])
        out.ranges = np.where(np.isfinite(ranges), ranges, np.nan).tolist()
        # Intensities from the old bins would no longer correspond to these points.
        out.intensities = []

    def select(self, scan, pose):
        if sum(math.isfinite(r) for r in scan.ranges) < self.settings['minimum_valid_beams']:
            raise ValueError('Too few valid beams for a SLAM constraint')
        timestamp = stamp_ns(scan.header)
        infos = self.get_subscriptions_info_by_topic(self.settings['output_topic'])
        subscribers = {bytes(info.endpoint_gid) for info in infos}
        added = subscribers - self.subscribers
        slam_subscribers = {bytes(info.endpoint_gid) for info in infos
                            if getattr(info, 'node_name', '') == self.settings['slam_node_name']}
        fresh_slam = bool(added & slam_subscribers)
        # A diagnostic subscriber must not insert an extra stationary constraint
        # into a backend that has already completed its initial scan seed.
        fresh_subscriber = bool(added) and (fresh_slam or not slam_subscribers)
        self.subscribers = subscribers
        if not subscribers:
            self.last_pose = self.last_stamp = self.last_frame = None
            self.awaiting_first_pose = False
            self.first_slam_stamp = None
            self.initial_pending.clear()
            self.initial_confirmed = 0
            self.reason = 'waiting for a SLAM scan subscriber'
            return False
        if fresh_subscriber:
            # Mapping can start/reset while the sensor core stays running and the
            # robot stays still. Give each newly attached consumer a fresh first scan.
            self.last_pose = self.last_stamp = self.last_frame = None
            self.subscriber_ready_at = time.monotonic() + self.settings['subscriber_warmup_sec']
            if fresh_slam:
                self.awaiting_first_pose = True
                self.first_slam_stamp = None
                self.initial_pending.clear()
                self.initial_confirmed = 0
        if not slam_subscribers:
            self.awaiting_first_pose = False
            self.initial_pending.clear()
        if time.monotonic() < self.subscriber_ready_at:
            self.reason = 'waiting for new subscriber DDS matching and TF history'
            return False
        if self.last_stamp is not None:
            if scan.header.frame_id != self.last_frame:
                raise ValueError('SLAM sensor frame changed; restart mapping before switching sensors')
            elapsed = (timestamp - self.last_stamp) / NS
            if elapsed < self.settings['minimum_time_interval']:
                self.reason = 'minimum scan interval'
                return False
            delta = pose - self.last_pose
            heading = abs(math.atan2(math.sin(delta[2]), math.cos(delta[2])))
            moved = math.hypot(delta[0], delta[1]) >= self.settings['minimum_travel_distance']
            turned = heading >= self.settings['minimum_travel_heading']
            refresh = self.settings['stationary_refresh_interval']
            if not (self.awaiting_first_pose or moved or turned or (refresh > 0 and elapsed >= refresh)):
                self.reason = 'motion gate: obstacle scans remain at full rate'
                return False
            scan.scan_time = elapsed
        self.pub.publish(scan)
        if self.awaiting_first_pose and self.first_slam_stamp is None:
            self.first_slam_stamp = timestamp
        if self.awaiting_first_pose:
            self.initial_pending.add(timestamp)
            # Bound memory if a paused/misconfigured backend never acknowledges.
            if len(self.initial_pending) > 50:
                self.initial_pending.remove(min(self.initial_pending))
        self.last_pose, self.last_stamp, self.last_frame = pose.copy(), timestamp, scan.header.frame_id
        self.counts['published'] += 1
        self.reason = ('seeding initial SLAM scans; waiting for pose acknowledgements' if self.awaiting_first_pose else
                       'publishing common-time SLAM scans')
        return True

    def drain(self):
        for timestamp in sorted(self.pending):
            entry = self.pending[timestamp]
            age = time.monotonic() - entry['received']
            paired = ('mid' in entry if self.source == 'mid360' else
                      'scan' in entry and ('cloud' in entry or not self.settings['deskew_enabled']))
            if not paired:
                self.reason = 'waiting for scan and timed cloud with the same header'
                if age <= self.settings['max_pending_sec']:
                    break
                self.get_logger().warning(self.reason + '; expired input discarded', throttle_duration_sec=5.)
                self.counts['rejected'] += 1
                del self.pending[timestamp]
                continue
            try:
                scan, pose = (self.mid360(entry['mid']) if self.source == 'mid360' else
                              self.g2(entry['scan'], entry.get('cloud')))
                if self.continuous_pub.get_subscription_count():
                    self.continuous_pub.publish(scan)
                if not self.select(scan, pose) and self.subscribers:
                    self.counts['motion_filtered'] += 1
            except (AwaitOdometry, TransformException) as exc:
                self.reason = str(exc)
                if age <= self.settings['max_pending_sec']:
                    break
                self.counts['rejected'] += 1
                self.get_logger().warning('SLAM input timed out: ' + self.reason, throttle_duration_sec=5.)
            except ValueError as exc:
                self.reason = str(exc)
                self.counts['rejected'] += 1
                self.get_logger().warning('SLAM input rejected: ' + self.reason, throttle_duration_sec=5.)
            del self.pending[timestamp]

    def status(self):
        msg = String()
        msg.data = json.dumps(dict(source=self.source, output=self.settings['output_topic'],
                                   deskew=self.settings['deskew_enabled'], pending=len(self.pending),
                                   awaiting_first_slam_pose=self.awaiting_first_pose,
                                   initial_scans_confirmed=self.initial_confirmed,
                                   last_initial_pose_stamp_ns=self.last_initial_pose_stamp,
                                   reason=self.reason, last_published_stamp_ns=self.last_stamp,
                                   **self.counts))
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = SlamScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
