#!/usr/bin/env python3
"""Domain-187 real ROS contract test; no driver or velocity publisher."""
from pathlib import Path
import json
import os
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from robocup_real_manipulation_msgs.action import Manipulate

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'control_center'))
from server import State
from manipulation_control import ManipulationControl

assert os.environ.get('ROS_DOMAIN_ID') == '187', 'Run only in isolated ROS_DOMAIN_ID=187'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
rclpy.init()
node = Node('inha_control_center')
state = State(mode='live'); control = ManipulationControl(node, state)
goals = []
def reject(goal):
    goals.append(goal)
    return GoalResponse.REJECT
server = ActionServer(node, Manipulate, '/manipulation/run',
                      execute_callback=lambda _: None, goal_callback=reject)
status = node.create_publisher(String, '/detection/closed_approach/status', 1)
odom = node.create_publisher(Odometry, '/odom_ekf', 1)
base = node.create_publisher(String, '/manipulation/base_gate/status', 1)
readiness = node.create_publisher(String, '/manipulation/readiness', 1)
wrist = node.create_publisher(String, '/manipulation/wrist/status', 1)
hardware_ready = False
moving = False
capture = None
classes = ['green_apple', 'peach', 'tomato', 'fanta_can', 'cup', 'banana']


def pump(seconds, phase='idle', active=False, measured=False):
    global capture
    end = time.monotonic()+seconds; last = 0.
    while time.monotonic() < end:
        stamp = node.get_clock().now().to_msg()
        msg = Odometry(); msg.header.stamp = stamp; msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'; msg.pose.pose.orientation.w = 1.
        msg.twist.twist.linear.x = .1 if moving else 0.; odom.publish(msg)
        if time.monotonic()-last >= .1:
            now = node.get_clock().now().nanoseconds/1e9
            capture = capture or now
            target = dict(label='cup', point_odom=[.3, .1, .6], capture_stamp=capture,
                lidar_stamp=capture-.02, lidar_oldest_stamp=capture-.3, lidar_count=9,
                source_topic='/mid360/points_obstacles', point_method='yolo_mid360_bbox_cluster') if measured else None
            status.publish(String(data=json.dumps(dict(phase=phase, active=active,
                label='cup', classes=classes, target=target))))
            base.publish(String(data=json.dumps(dict(ready=True, stopped=not moving, locked=False))))
            readiness.publish(String(data=json.dumps(dict(ready=hardware_ready,
                errors=[] if hardware_ready else ['READ_ONLY_MODE']))))
            wrist.publish(String(data=json.dumps(dict(valid=True))))
            last = time.monotonic()
        rclpy.spin_once(node, timeout_sec=.015)


with tempfile.TemporaryDirectory(prefix='docking-handoff-') as directory:
    child = subprocess.Popen([sys.executable, '-c',
        'from robocup_real_manipulation.control_center_input import main; main()',
        '--ros-args', '-p', 'owner_path:='+str(Path(directory)/'owner')],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    late = None
    try:
        pump(2.)
        assert child.poll() is None
        for phase in ('visual_waiting', 'visual_aligning', 'visual_approaching'):
            pump(.3, phase, True)
        for phase in ('visual_handoff', 'navigating_target', 'point_handoff',
                      'docking_handoff', 'final_align'):
            pump(.3, phase, True, True)
        pump(1.5, 'succeeded', False, True)
        task = control.candidate()
        assert task, control.monitor.values
        assert control.targets[task][0].class_id == 4
        assert control.targets[task][0].header.frame_id == 'odom'
        assert control.approaches[task][0].base_stopped
        assert control.monitor.values['execution']['handoff']['ready']
        assert control.blockers() == ['실행기 준비 미완료'], control.blockers()
        hardware_ready = True; pump(.7, 'succeeded', False, True)
        assert control.blockers() == [], control.blockers()
        # A late subscriber receives the retained pair but cannot replay the old run.
        late = Node('late_control_center')
        late_control = ManipulationControl(late, State(mode='live'))
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            pump(.04, 'succeeded', False, True)
            rclpy.spin_once(late, timeout_sec=.01)
        assert task in late_control.targets and task in late_control.approaches, (late_control.targets, late_control.approaches)
        assert late_control.candidate() is None
        moving = True; pump(.7, 'succeeded', False, True)
        assert control.candidate() is None
        assert control.monitor.values['adapter']['reason'] == 'CONTROL_CENTER_BASE_MOVED_AFTER_COMPLETION'
        assert not goals
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        assert not node.get_publishers_info_by_topic('/tracer/cmd_vel_safe')
        print('PASS: real docking phases → measured stop → retained same-ID handoff → web readiness gating; movement revokes; no goals or velocity publishers')
    finally:
        child.terminate()
        try: output, _ = child.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill(); output, _ = child.communicate()
        if child.returncode not in (0, -15, -2): print(output)
        if late: late.destroy_node()
        server.destroy(); node.destroy_node(); rclpy.shutdown()
