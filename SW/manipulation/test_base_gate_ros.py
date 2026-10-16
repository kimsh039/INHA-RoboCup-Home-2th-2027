#!/usr/bin/env python3
"""Only run with ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1. Synthetic final velocity bus."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='187' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import sys,time,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'robot'))
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import Twist,TwistStamped
from nav_msgs.msg import Odometry
from robocup_real_manipulation_msgs.msg import BaseLease
from tracer_safety_watchdog import SafetyWatchdog

with tempfile.TemporaryDirectory(prefix='base-gate-test-') as directory:
    rclpy.init(args=['--ros-args','-p','manipulation_lock_file:='+directory+'/lock.json'])
    watchdog=SafetyWatchdog();guard=Node('collision_guard');coordinator=Node('real_manipulation')
    executor=SingleThreadedExecutor()
    for node in (watchdog,guard,coordinator):executor.add_node(node)
    pub=guard.create_publisher(TwistStamped,'/collision/cmd_vel_approved',1)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    lock=coordinator.create_publisher(BaseLease,'/manipulation/base_lock_request',qos)
    odom=coordinator.create_publisher(Odometry,'/odom_ekf',10)
    outputs=[];leases=[]
    guard.create_subscription(Twist,'/tracer/cmd_vel_safe',lambda m:outputs.append(m.linear.x),10)
    guard.create_subscription(BaseLease,'/manipulation/base_lease',leases.append,qos)
    def drive(seconds,locked=None):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            stamp=guard.get_clock().now().to_msg()
            msg=TwistStamped();msg.header.stamp=stamp;msg.header.frame_id='base_link';msg.twist.linear.x=.1;pub.publish(msg)
            observation=Odometry();observation.header.stamp=stamp;observation.header.frame_id='odom';observation.child_frame_id='base_link';odom.publish(observation)
            if locked is not None:
                request=BaseLease(task_id='isolated-lock',locked=locked);request.header.stamp=stamp;lock.publish(request)
            executor.spin_once(timeout_sec=.01)
    try:
        drive(2.)
        assert any(x>.09 for x in outputs[-20:]),outputs[-20:]
        drive(1.,True)
        assert all(x==0 for x in outputs[-15:]),outputs[-15:]
        assert leases and leases[-1].locked and leases[-1].stopped
        assert Path(directory+'/lock.json').exists()
        drive(.5) # No request stream: retained gate must stay closed.
        assert all(x==0 for x in outputs[-15:])
        drive(.5,False)
        assert not leases[-1].locked
        assert any(x>.09 for x in outputs[-15:])
        print('PASS: actual final watchdog forwards unlocked, gates all velocity while locked, measured stop lease, retained lock, explicit release')
    finally:
        executor.shutdown()
        for node in (watchdog,guard,coordinator):node.destroy_node()
        rclpy.shutdown()
