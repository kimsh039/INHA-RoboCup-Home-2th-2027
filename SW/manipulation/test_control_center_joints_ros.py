#!/usr/bin/env python3
"""Real ROS subscription with synthetic model joints; no robot command topics."""
import os
from pathlib import Path
import sys
import time

assert os.environ.get('ROS_DOMAIN_ID') == '187'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'control_center'))
import rclpy
from sensor_msgs.msg import JointState
from joint_contract import ModelJointFeedback, MODEL_JOINTS

rclpy.init()
node = rclpy.create_node('isolated_model_joint_contract_test')
topic = '/test/model_joint_states'
feedback = ModelJointFeedback(node, dict(model_joint_topic=topic))
pub = node.create_publisher(JointState, topic, 1)

def send(names, values, duration):
    end = time.monotonic()+duration
    while time.monotonic() < end:
        msg = JointState(name=list(names), position=list(values))
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=.02)
        time.sleep(.02)

try:
    send([f'joint{i}' for i in range(1, 7)], [1.55, .2, -.3, .1, .6, -.04], .5)
    assert feedback.positions() == {}, 'Raw coordinates were accepted as model coordinates'
    values = [-.05, .2, -.3, .1, .6, -.04]
    send(MODEL_JOINTS[::-1], values[::-1], .5)
    assert feedback.positions() == dict(zip(MODEL_JOINTS, values)), 'Name order or double offset error'
    end = time.monotonic()+.7
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
    assert feedback.positions() == {}, 'Stale joints accepted'
    assert not node.get_publishers_info_by_topic('/joint_ctrl_single')
    print('PASS ROS model-joint input: raw rejected, canonical names/order preserved, no double offset, stale rejected')
finally:
    node.destroy_node()
    rclpy.shutdown()
