#!/usr/bin/env python3
"""Passive PiPER feedback with reconnect and actual per-frame arrival times.

The installed PiPER SDK 0.6.2 protocol decodes 0x2A5/6/7 as two signed big
endian int32 joint angles each, in 0.001 degrees. No CAN frames are transmitted.
Missing responses never cause a ROS shutdown or fabricated joint observations.
"""
import errno
import json
import math
import socket
import struct
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINT_IDS = (0x2A5, 0x2A6, 0x2A7)
FRAME = struct.Struct('=IB3x8s')
TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)  # Linux arm64 SO_TIMESTAMPNS_OLD


class PiperFeedback(Node):
    def __init__(self):
        super().__init__('piper_feedback')
        self.interface = self.declare_parameter('can_port', 'can_arm').value
        self.timeout = self.declare_parameter('feedback_timeout', 0.20).value
        self.span = self.declare_parameter('maximum_sample_span', 0.05).value
        self.pub = self.create_publisher(JointState, '/piper/joint_states_feedback', 20)
        self.status_pub = self.create_publisher(String, '/piper/feedback_status', 10)
        self.bus, self.next_connect = None, 0.
        self.parts, self.updated, self.counts = {}, set(), {key: 0 for key in JOINT_IDS}
        self.last_published, self.error = None, 'waiting_for_joint_frames'
        self.previous_status = None
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.005, self.receive, clock=self.wall_clock)
        self.create_timer(0.5, self.status, clock=self.wall_clock)

    def disconnect(self, reason):
        if self.bus is not None:
            self.bus.close()
        self.bus = None
        self.parts.clear()
        self.updated.clear()
        self.last_published = None
        self.next_connect = time.monotonic()+1.
        self.error = reason

    def connect(self):
        if time.monotonic() < self.next_connect:
            return False
        bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            # Standard 11-bit ID plus EFF/RTR flags (Linux SocketCAN ABI).
            mask = 0x7FF | 0x80000000 | 0x40000000
            bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                           b''.join(struct.pack('=II', cid, mask) for cid in JOINT_IDS))
            bus.setsockopt(socket.SOL_SOCKET, TIMESTAMPNS, 1)
            bus.setblocking(False)
            bus.bind((self.interface,))
        except OSError as exc:
            bus.close()
            self.disconnect('can_unavailable: '+str(exc))
            return False
        self.bus, self.error = bus, 'waiting_for_joint_frames'
        return True

    def receive(self):
        if self.bus is None and not self.connect():
            return
        try:
            for _ in range(256):
                try:
                    frame, ancillary, flags, _ = self.bus.recvmsg(FRAME.size, socket.CMSG_SPACE(16))
                except BlockingIOError:
                    break
                if len(frame) != FRAME.size or flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
                    continue
                can_id, length, data = FRAME.unpack(frame)
                if can_id not in JOINT_IDS or length != 8:
                    continue
                stamp = None
                for level, kind, payload in ancillary:
                    if level == socket.SOL_SOCKET and kind == TIMESTAMPNS and len(payload) >= 16:
                        sec, nanosec = struct.unpack('=qq', payload[:16])
                        stamp = sec*1_000_000_000+nanosec
                # A kernel receipt stamp is required: old queued frames must not
                # become fresh merely because the ROS callback runs later.
                if stamp is None or not -0.05 <= (self.get_clock().now().nanoseconds-stamp)/1e9 <= self.timeout:
                    continue
                pair = tuple(value*math.pi/180_000 for value in struct.unpack('>ii', data))
                if not all(math.isfinite(value) and abs(value) <= 2*math.pi for value in pair):
                    self.error = 'invalid_joint_angles'
                    continue
                previous = self.parts.get(can_id)
                if previous is not None and stamp <= previous[0]:
                    continue
                self.parts[can_id] = (stamp, time.monotonic(), pair)
                self.updated.add(can_id)
                self.counts[can_id] += 1
        except OSError as exc:
            if exc.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                self.disconnect('can_disconnected: '+str(exc))
            return
        if not all(cid in self.updated for cid in JOINT_IDS):
            return
        stamps = [self.parts[cid][0] for cid in JOINT_IDS]
        if max(stamps)-min(stamps) > self.span*1e9:
            self.error = 'incomplete_joint_sample'
            return
        oldest = min(stamps)
        if not -0.05 <= (self.get_clock().now().nanoseconds-oldest)/1e9 <= self.timeout:
            return
        out = JointState()
        out.header.frame_id = 'base_link'
        out.header.stamp = Time(nanoseconds=oldest).to_msg()
        out.name = ['joint'+str(i) for i in range(1, 7)]
        out.position = [angle for cid in JOINT_IDS for angle in self.parts[cid][2]]
        self.pub.publish(out)
        self.updated.clear()
        self.last_published, self.error = (oldest, time.monotonic()), ''

    def status(self):
        now = time.monotonic()
        live = bool(self.last_published and now-self.last_published[1] <= self.timeout and
                    -0.05 <= (self.get_clock().now().nanoseconds-self.last_published[0])/1e9 <= self.timeout)
        missing = [hex(cid) for cid in JOINT_IDS if cid not in self.parts or now-self.parts[cid][1] > self.timeout]
        state = 'live' if live else 'waiting' if not self.parts else 'stale'
        reason = 'live' if live else self.error or 'missing_joint_frames'
        self.status_pub.publish(String(data=json.dumps(dict(interface=self.interface,
                                    state=state, reason=reason,
                                    missing_frames=missing,
                                    frames={hex(cid): count for cid, count in self.counts.items()}))))
        identity = (state, reason)
        if identity != self.previous_status:
            self.get_logger().info(f'{self.interface}: {state} ({reason})')
            self.previous_status = identity


def main():
    rclpy.init()
    node = PiperFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.bus is not None:
            node.bus.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
