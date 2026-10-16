#!/usr/bin/env python3
"""Isolated ROS event delivery; no driver, hardware, motion publisher or goals."""
from pathlib import Path
import json
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'control_center'))
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from robocup_real_manipulation_msgs.msg import TaskStatus
from server import State
from manipulation_control import ManipulationControl

rclpy.init()
node=Node('control_center_events_test')
state=State(mode='live')
control=ManipulationControl(node,state)
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
status=node.create_publisher(TaskStatus,'/manipulation/status',qos)
wrist=node.create_publisher(String,'/manipulation/wrist/status',1)
try:
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    for topic in ('/manipulation/target','/manipulation/approach_result'):
        endpoints=node.get_subscriptions_info_by_topic(topic)
        own=[e for e in endpoints if e.node_name=='control_center_events_test']
        assert len(own)==1 and own[0].qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL,(topic,own)
    msg=TaskStatus(task_id='isolated_event_task',stage='OBSERVE',outcome='FAILED',reason='CAMERA_SYNC')
    for _ in range(5):
        msg.header.stamp=node.get_clock().now().to_msg();status.publish(msg)
        wrist.publish(String(data=json.dumps(dict(valid=False,reason='CAMERA_FRAME'))))
        deadline=time.monotonic()+.15
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    events=list(state.logs)
    assert sum('CAMERA_SYNC' in e['message'] for e in events)==1,events
    assert sum('CAMERA_FRAME' in e['message'] for e in events)==1,events
    assert not control.monitor.active
    assert not control.candidate()
    assert control.blockers()
    print('PASS: real ROS TaskStatus/wrist → shared event list, duplicate suppression, disabled unsafe start')
finally:
    node.destroy_node();rclpy.shutdown()
