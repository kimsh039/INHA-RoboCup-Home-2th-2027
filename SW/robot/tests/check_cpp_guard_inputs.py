#!/usr/bin/env python3
"""Required input-failure checks for the native velocity guard, without hardware.

Run with the Humble environment sourced. Domain 179 and /guard_cpp_test topics
are mandatory here; no Tracer driver or actuator watchdog is started.
"""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '179'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import yaml


PREFIX = '/guard_cpp_test/'


class Inputs(Node):
    def __init__(self, names):
        super().__init__('guard_cpp_test_inputs')
        self.mode = 'good'
        self.names = names
        self.generation = 1
        self.last_cloud_ns = 0
        self.last_cloud_time = 0.
        self.status = {}
        self.outputs = []
        self.ticks = 0
        self.command = self.create_publisher(Twist, PREFIX + 'command', 1)
        self.odom = self.create_publisher(Odometry, PREFIX + 'odom', 1)
        self.joints = self.create_publisher(JointState, PREFIX + 'joints', 1)
        self.cloud = self.create_publisher(PointCloud2, PREFIX + 'scene', 1)
        self.health = self.create_publisher(String, PREFIX + 'health', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(String, PREFIX + 'status', self.receive_status, 10)
        self.create_subscription(TwistStamped, PREFIX + 'approved', self.receive_output, 10)
        self.create_timer(.02, self.publish_inputs)

    def receive_status(self, message):
        self.status = json.loads(message.data)

    def receive_output(self, message):
        self.outputs.append((time.monotonic(), message.twist.linear.x, message.twist.angular.z))
        self.outputs = self.outputs[-100:]

    def publish_inputs(self):
        self.ticks += 1
        now = self.get_clock().now()
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = now.to_msg()
        odom.pose.pose.orientation.w = 1.
        if self.mode != 'no_odom':
            self.odom.publish(odom)
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_link'
        tf.transform.rotation.w = 1.
        tf.transform.translation.x = .1 if self.mode == 'bad_tf' else 0.
        self.tf.sendTransform(tf)
        joint = JointState()
        joint.header.stamp = now.to_msg()
        joint.name = self.names
        joint.position = [0.] * len(self.names)
        if self.mode == 'moving_arm':
            joint.position[0] = .004 if self.ticks % 2 else 0.
        if self.mode != 'no_joints':
            self.joints.publish(joint)
        command = Twist()
        command.linear.x = float('nan') if self.mode == 'bad_command' else .1
        if self.mode != 'no_command':
            self.command.publish(command)
        # A scene and its status have exactly the same original observation time.
        if time.monotonic() - self.last_cloud_time > .09 and self.mode != 'no_scene':
            self.last_cloud_time = time.monotonic()
            self.last_cloud_ns = now.nanoseconds
            cloud = PointCloud2()
            cloud.header.frame_id = 'odom'
            cloud.header.stamp = now.to_msg()
            cloud.height, cloud.width = 1, 1
            cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z', 'source_x', 'source_y', 'source_z'))]
            cloud.point_step = cloud.row_step = 24
            cloud.is_dense = True
            cloud.data = struct.pack('<6f', float('nan') if self.mode == 'bad_scene' else 3., 0., 1., 3., 0., 1.)
            self.cloud.publish(cloud)
        health = dict(session_id='isolated-native-guard-check', generation=self.generation,
                      ready=self.mode != 'unready_health', reason='injected_cloud_fault',
                      reference_stamp_ns=self.last_cloud_ns + (1 if self.mode == 'unpaired_health' else 0))
        self.health.publish(String(data=json.dumps(health)))


def main():
    binary, model = sys.argv[1:]
    names = yaml.safe_load(Path(model).read_text())['required_joints']
    rclpy.init()
    inputs = Inputs(names)
    # These are endpoint declarations only. They never forward a velocity.
    watchdog = Node('tracer_safety_watchdog')
    protected = watchdog.create_publisher(Twist, PREFIX + 'protected', 1)
    tracer = Node('tracer_base_node')
    subscription = tracer.create_subscription(Twist, PREFIX + 'protected', lambda _: None, 1)
    params = dict(geometry_file=model, command_topic=PREFIX+'command', odom_topic=PREFIX+'odom',
                  joint_topic=PREFIX+'joints', safety_topic=PREFIX+'scene',
                  cloud_status_topic=PREFIX+'health', approved_topic=PREFIX+'approved',
                  protected_topic=PREFIX+'protected', status_topic=PREFIX+'status')
    args = [binary, '--ros-args']
    for key, value in params.items():
        args += ['-p', key + ':=' + value]
    log_path = '/tmp/robocup-cpp-guard-input-check.log'
    log = open(log_path, 'w')
    process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
    cases = []

    def spin(duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            if process.poll() is not None:
                raise AssertionError(f'guard exited: {log_path}')
            rclpy.spin_once(inputs, timeout_sec=.005)

    def expect(mode, reasons=None, moving=False, timeout=3.):
        inputs.mode = mode
        began = time.monotonic()
        while time.monotonic() - began < timeout:
            spin(.05)
            recent = [v for v in inputs.outputs if v[0] > time.monotonic() - .15]
            status = inputs.status
            output_ok = bool(recent) and all((abs(x)+abs(w) > .01) == moving for _, x, w in recent)
            reason_ok = status.get('reason') in reasons if reasons else status.get('ready') is True
            if output_ok and reason_ok:
                cases.append(dict(mode=mode, reason=status.get('reason'), moving=moving))
                print(json.dumps(cases[-1]), flush=True)
                return
        raise AssertionError(f'{mode}: status={inputs.status}, recent outputs={inputs.outputs[-4:]}; {log_path}')

    try:
        expect('good', moving=True, timeout=10.)
        expect('no_scene', {'stale_mid360'})
        expect('good', moving=True)
        expect('unpaired_health', {'stale_mid360', 'missing_cloud_health'})
        expect('good', moving=True)
        inputs.generation += 1
        expect('no_scene', {'stale_mid360', 'waiting_for_mid360'})
        expect('good', moving=True)
        expect('no_odom', {'stale_odometry'})
        expect('good', moving=True)
        expect('no_joints', {'stale_arm_feedback'})
        expect('good', moving=True)
        expect('bad_tf', {'stale_or_inconsistent_tf'})
        expect('good', moving=True)
        expect('moving_arm', {'arm_is_moving'})
        expect('good', moving=True)
        expect('bad_command', {'invalid_velocity_command'})
        expect('good', moving=True)
        expect('no_command', {'command_timeout'})
        expect('good', moving=True)
        expect('unready_health', {'injected_cloud_fault'})
        expect('good', moving=True)
        expect('bad_scene', {'invalid_retained_scene', 'stale_mid360', 'missing_cloud_health'})
        expect('good', moving=True)
        print(json.dumps(dict(cases=len(cases), isolated_domain=179, physical_commands=0)), flush=True)
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        tracer.destroy_node()
        watchdog.destroy_node()
        inputs.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
