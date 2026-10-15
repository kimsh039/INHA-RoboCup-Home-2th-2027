#!/usr/bin/env python3
"""Synthetic status/odometry -> actual adapter topics, isolated from robot DDS."""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

assert os.environ.get('ROS_DOMAIN_ID') == '187', 'Use isolated domain 187'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use localhost only'

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from robocup_real_manipulation_msgs.msg import Target, ApproachResult
from robocup_real_manipulation.control_center_input import CLASSES


def run():
    rclpy.init()
    node = rclpy.create_node('isolated_control_center_contract_test')
    targets, results = [], []
    node.create_subscription(Target, '/manipulation/target', targets.append, 10)
    node.create_subscription(ApproachResult, '/manipulation/approach_result', results.append, 10)
    statuses = node.create_publisher(String, '/detection/closed_approach/status', 1)
    odoms = node.create_publisher(Odometry, '/odom_ekf', 10)
    captured = node.get_clock().now().nanoseconds / 1e9
    value = dict(phase='succeeded', active=False, label='banana', classes=list(CLASSES),
        target=dict(label='banana', point_odom=[.3, .1, .6], point_map=[9., 8., .6],
            frame='map', capture_stamp=captured, lidar_stamp=captured-.02,
            lidar_oldest_stamp=captured-.8, lidar_count=9,
            source_topic='/mid360/points_obstacles', point_method='yolo_mid360_bbox_cluster'))

    def pump(seconds, phase=None, active=False, moving=False, odometry=True):
        if phase:
            value.update(phase=phase, active=active)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            statuses.publish(String(data=json.dumps(value)))
            if odometry:
                msg = Odometry()
                msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.twist.twist.linear.x = .1 if moving else 0.
                odoms.publish(msg)
            rclpy.spin_once(node, timeout_sec=.05)
            time.sleep(.02)

    with tempfile.TemporaryDirectory(prefix='control_center_contract_') as td:
        proc = subprocess.Popen([sys.executable, '-c',
            'from robocup_real_manipulation.control_center_input import main; main()',
            '--ros-args', '-p', 'owner_path:='+td+'/owner'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            deadline = time.monotonic()+8
            while not statuses.get_subscription_count() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            assert proc.poll() is None and statuses.get_subscription_count(), 'Adapter startup failed'
            pump(1.0)
            assert not targets and not results, 'Startup succeeded snapshot was replayed'
            pump(.3, 'ready')
            pump(.4, 'navigating', True)
            pump(.9, 'succeeded', moving=True)
            assert not targets and not results, 'Moving base was accepted'
            pump(.5, odometry=False)
            assert not targets and not results, 'Missing odometry was accepted'
            pump(1.1)
            assert len(targets) == len(results) == 1, (len(targets), len(results))
            target, result = targets[0], results[0]
            assert target.task_id == result.task_id == target.instance_id
            assert target.class_id == 5 and target.header.frame_id == 'odom'
            assert [target.coarse_position.x, target.coarse_position.y, target.coarse_position.z] == [.3, .1, .6]
            stamp = target.header.stamp.sec+target.header.stamp.nanosec/1e9
            assert abs(stamp-captured) < 1e-6, 'Original capture stamp lost'
            assert result.outcome == 'SUCCESS' and result.base_stopped
            pump(1.)
            assert len(targets) == len(results) == 1, 'Duplicate handoff'
            pump(.3, 'ready')
            pump(.3, 'navigating', True)
            pump(.8, 'canceled')
            assert len(targets) == len(results) == 1, 'Canceled approach handed off'
            assert not node.get_publishers_info_by_topic('/manipulation/base_lease')
            assert not node.get_publishers_info_by_topic('/joint_ctrl_single')
            print('PASS actual ROS adapter: startup replay, moving/stale odom rejection, '
                  'odom coordinates, capture stamp, paired IDs, single handoff, cancellation; no arm/lease publisher')
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill(); proc.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run()
