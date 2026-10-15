#!/usr/bin/env python3
"""Find a global scan hypothesis, then let SLAM Toolbox confirm and track it.

Publishes no TF or movement commands. Initial poses are computed automatically;
neither a map-origin prior nor user clicks determine the search candidates.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from collections import deque
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from global_scan_matcher import MapMatcher


def pose_matrix(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0., 0., 1.]])


def matrix_pose(matrix):
    return np.array([matrix[0, 2], matrix[1, 2], math.atan2(matrix[1, 0], matrix[0, 0])])


def quaternion_rotation(q):
    v = np.array([q.x, q.y, q.z, q.w]); v /= np.linalg.norm(v)
    x, y, z, w = v
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class GlobalInitializer(Node):
    def __init__(self, args):
        super().__init__('slam_global_initializer')
        profile = yaml.safe_load(Path(args.backend).read_text())
        backend = dict(profile.get('/**', {}).get('ros__parameters', {}))
        backend.update(profile.get('/slam_toolbox', profile.get('slam_toolbox', {})).get('ros__parameters', {}))
        self.base = backend.get('base_frame', 'base_link')
        self.odom = backend.get('odom_frame', 'odom')
        self.map_frame = backend.get('map_frame', 'map')
        self.map_name = Path(args.map).name
        defaults = dict(translation_step=.25, heading_step=math.radians(5), sigma=.12,
                        max_beams=120, minimum_valid_beams=40, refine_candidates=8,
                        match_distance=.2, minimum_score=.6, minimum_matched_fraction=.65,
                        minimum_score_margin=.08, retry_interval=3., confirmation_timeout=8.,
                        scan_max_age=1.)
        self.settings = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.matcher = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.scan = None
        self.scan_received = 0.
        self.anchor = None
        self.future = None
        self.stop_event = threading.Event()
        self.executor_pool = ThreadPoolExecutor(max_workers=1)
        self.pose_queue = deque(maxlen=10)
        self.confirmed = set()
        self.sent_at = None
        self.next_attempt = 0.
        self.confirm_deadline = 0.
        self.details = dict(state='waiting', map=self.map_name, reason='센서·지도 입력 준비 중')
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.status_pub = self.create_publisher(String, '/localization/status', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            self.matcher = MapMatcher.load(str(args.map)+'.yaml', self.settings)
        except (ValueError, OSError, KeyError) as exc:
            self.details.update(state='error', reason='전역 탐색에 필요한 지도 자료를 불러올 수 없습니다')
            self.get_logger().error(str(exc))
        self.create_subscription(LaserScan, backend['scan_topic']+'_continuous', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.on_pose, qos_profile_sensor_data)
        self.create_timer(.1, self.tick)
        self.create_timer(1., self.status)

    def on_scan(self, msg):
        self.scan, self.scan_received = msg, time.monotonic()

    def on_pose(self, msg):
        timestamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        if (msg.header.frame_id == self.map_frame and self.details['state'] == 'confirming' and
                self.sent_at is not None and timestamp >= self.sent_at):
            self.pose_queue.append(msg)

    def odom_pose(self, when):
        tr = self.tf.lookup_transform(self.odom, self.base, when)
        p, q = tr.transform.translation, tr.transform.rotation
        angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return pose_matrix(p.x, p.y, angle), tr.header.stamp

    def observation(self):
        if self.scan is None or time.monotonic()-self.scan_received > self.settings['scan_max_age']:
            raise ValueError('새 운동 보상 스캔 대기 중')
        scan = self.scan
        stamp = Time.from_msg(scan.header.stamp)
        tr = self.tf.lookup_transform(self.base, scan.header.frame_id, stamp).transform
        ranges = np.asarray(scan.ranges)
        angle = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        if valid.sum() < self.settings['minimum_valid_beams']:
            raise ValueError('전역 탐색에 필요한 유효 측정 대기 중')
        points = np.c_[ranges[valid]*np.cos(angle[valid]), ranges[valid]*np.sin(angle[valid]),
                       np.zeros(valid.sum())]
        offset = tr.translation
        points = points @ quaternion_rotation(tr.rotation).T + [offset.x, offset.y, offset.z]
        odom, _ = self.odom_pose(stamp)
        return dict(points=points[:, :2], odom=odom, stamp_ns=stamp.nanoseconds)

    def tick(self):
        try:
            self.advance()
        except (ValueError, TransformException) as exc:
            self.details.update(reason=str(exc))

    def advance(self):
        if self.matcher is None:
            return
        if self.details['state'] == 'localized':
            return
        if self.future is not None:
            if not self.future.done():
                return
            future, self.future = self.future, None
            try:
                result = future.result()
            except Exception as exc:
                self.details.update(state='waiting', reason=str(exc))
                self.next_attempt = time.monotonic()+self.settings['retry_interval']
                return
            self.details.update(score=result['best']['score'], margin=result['margin'],
                                matched_fraction=result['best']['matched_fraction'],
                                candidate=result['best']['pose'])
            if not result['accepted']:
                self.details.update(state='ambiguous', reason='위치 후보를 더 비교하고 있습니다')
                self.next_attempt = time.monotonic()+self.settings['retry_interval']
                return
            current_odom, stamp = self.odom_pose(Time())
            age = self.get_clock().now().nanoseconds-Time.from_msg(stamp).nanoseconds
            if age > 500_000_000 or age < -100_000_000:
                raise ValueError('신선한 오도메트리 대기 중')
            pose = result['best']['pose']
            propagated = pose_matrix(*pose) @ np.linalg.inv(self.anchor['odom']) @ current_odom
            pose = matrix_pose(propagated)
            request = PoseWithCovarianceStamped()
            request.header.frame_id = self.map_frame
            request.header.stamp = self.get_clock().now().to_msg()
            request.pose.pose.position.x, request.pose.pose.position.y = float(pose[0]), float(pose[1])
            request.pose.pose.orientation.z, request.pose.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
            request.pose.covariance[0] = request.pose.covariance[7] = .05**2
            request.pose.covariance[35] = math.radians(3)**2
            self.sent_at = Time.from_msg(request.header.stamp).nanoseconds
            self.confirmed.clear(); self.pose_queue.clear()
            self.initial_pub.publish(request)
            self.confirm_deadline = time.monotonic()+self.settings['confirmation_timeout']
            self.details.update(state='confirming', reason='SLAM Toolbox 정합 응답 확인 중',
                                automatic_pose=pose.tolist())
            self.get_logger().info('Computed global hypothesis; waiting for SLAM Toolbox confirmation')
            return
        if self.details['state'] == 'confirming':
            while self.pose_queue:
                msg = self.pose_queue[0]
                odom, _ = self.odom_pose(Time.from_msg(msg.header.stamp))
                self.pose_queue.popleft()
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                at_anchor = pose_matrix(p.x, p.y, angle) @ np.linalg.inv(odom) @ self.anchor['odom']
                metrics = self.matcher.metrics(matrix_pose(at_anchor), self.anchor['points'])
                if (metrics['score'] >= self.settings['minimum_score'] and
                        metrics['matched_fraction'] >= self.settings['minimum_matched_fraction']):
                    self.confirmed.add(Time.from_msg(msg.header.stamp).nanoseconds)
                    self.details.update(confirmed=len(self.confirmed), **metrics,
                                        pose=[p.x, p.y, angle])
                if len(self.confirmed) >= 2:
                    self.details.update(state='localized', reason='자동 위치 확인 완료')
                    self.get_logger().info('SLAM Toolbox confirmed the automatically computed global pose')
                    return
            if time.monotonic() >= self.confirm_deadline:
                self.details.update(state='waiting', reason='정합 후보를 다시 탐색하고 있습니다')
                self.next_attempt = time.monotonic()+self.settings['retry_interval']
            return
        if time.monotonic() < self.next_attempt:
            return
        subscribers = self.get_subscriptions_info_by_topic('/initialpose')
        if not any(info.node_name == 'slam_toolbox' for info in subscribers):
            self.details.update(state='waiting', reason='SLAM Toolbox 지도 로딩 대기 중')
            return
        self.anchor = self.observation()
        self.details.update(state='searching', reason='지도 전체에서 위치와 방향 탐색 중')
        self.future = self.executor_pool.submit(self.matcher.search, self.anchor['points'], self.stop_event.is_set)

    def status(self):
        self.status_pub.publish(String(data=json.dumps(self.details, ensure_ascii=False)))

    def close(self):
        self.stop_event.set()
        self.executor_pool.shutdown(wait=True, cancel_futures=True)
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', required=True)
    parser.add_argument('--map', required=True)
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = GlobalInitializer(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
