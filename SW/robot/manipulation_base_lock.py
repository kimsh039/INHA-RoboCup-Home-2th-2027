"""Real base lease at the sole final Tracer velocity publisher. No arm control."""
import json
import math
import os
from pathlib import Path
import tempfile
import time


class BaseLockAdapter:
    def __init__(self, node):
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from robocup_real_manipulation_msgs.msg import BaseLease
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
        self.node, self.Lease, self.String = node, BaseLease, String
        self.path = Path(node.declare_parameter('manipulation_lock_file',
            '~/.ros/robocup_real_manipulation/base_watchdog_lock.json').value).expanduser()
        self.task, self.release_task, self.stop_since, self.odom_stamp = '', '', None, None
        self.error = ''; self.last_ack = 0.
        self.pose = None
        if self.path.exists():
            try:
                self.task = json.loads(self.path.read_text())['task_id']
                if not isinstance(self.task, str): raise ValueError('invalid lock')
            except Exception:
                self.task, self.error = 'INVALID_PERSISTED_LOCK', 'BASE_LOCK_JOURNAL_INVALID'
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = node.create_publisher(BaseLease, '/manipulation/base_lease', qos)
        self.status_pub = node.create_publisher(String, '/manipulation/base_gate/status', 1)
        self.req = node.create_subscription(BaseLease, '/manipulation/base_lock_request', self.request, qos)
        self.odom = node.create_subscription(Odometry, '/odom_ekf', self.odometry, qos_profile_sensor_data)

    def now(self): return self.node.get_clock().now().nanoseconds/1e9

    def persist(self, task):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix='.base-lock-', dir=self.path.parent)
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(dict(task_id=task), f); f.flush(); os.fsync(f.fileno())
            os.replace(temporary, self.path)
        finally:
            if Path(temporary).exists(): Path(temporary).unlink()

    def owners_ok(self):
        sources = self.node.get_publishers_info_by_topic('/manipulation/base_lock_request')
        return (len(sources) == 1 and sources[0].node_name == 'real_manipulation'
                and self.node.owner_valid)

    def request(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not msg.task_id or len(msg.task_id) > 128 or not -.05 <= self.now()-stamp <= .5:
            return
        if msg.locked:
            if self.task and self.task != msg.task_id:
                self.error = 'OTHER_BASE_LOCK_OWNER'; return
            if not self.task:
                self.task = msg.task_id  # Gate closes before persistence/ACK.
                try: self.persist(self.task)
                except OSError: self.error = 'BASE_LOCK_PERSIST_FAILED'; return
            self.release_task = ''
        elif self.task == msg.task_id and self.owners_ok() and self.stopped():
            try: self.persist('')
            except OSError: self.error = 'BASE_UNLOCK_PERSIST_FAILED'; return
            self.release_task, self.task = self.task, ''

    def odometry(self, msg):
        now, stamp = self.now(), msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        previous = self.odom_stamp
        if (msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link'
                or not -.05 <= now-stamp <= .3 or (previous is not None and stamp <= previous)):
            self.stop_since = None; return
        self.odom_stamp = stamp
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if all(math.isfinite(x) for x in (p.x,p.y,q.x,q.y,q.z,q.w)):
            self.pose = (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        else:
            self.pose = None; self.stop_since = None; return
        v = msg.twist.twist
        values = (v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z)
        if (not all(math.isfinite(x) for x in values) or
                math.sqrt(sum(x*x for x in values[:3])) > .02 or
                math.sqrt(sum(x*x for x in values[3:])) > .04):
            self.stop_since = None
        elif previous is None or stamp-previous > .3 or self.stop_since is None:
            self.stop_since = time.monotonic()

    def stopped(self):
        return (self.odom_stamp is not None and -.05 <= self.now()-self.odom_stamp <= .3
                and self.stop_since is not None and time.monotonic()-self.stop_since >= .6)

    def tick(self, zero_output):
        if time.monotonic()-self.last_ack < .1: return
        self.last_ack = time.monotonic()
        stopped = bool(self.stopped())
        owned = self.owners_ok()
        # During manipulation the independent latch itself enforces final zero.
        # An arm-is-moving rejection upstream must not revoke that real lock.
        valid = not self.error and owned and zero_output and (bool(self.task) or self.node.fresh())
        if self.task and valid:
            msg = self.Lease(task_id=self.task, locked=True, stopped=stopped)
            msg.header.stamp = self.node.get_clock().now().to_msg(); self.pub.publish(msg)
        elif self.release_task and owned and stopped:
            msg = self.Lease(task_id=self.release_task, locked=False, stopped=True)
            msg.header.stamp = self.node.get_clock().now().to_msg(); self.pub.publish(msg)
        self.status_pub.publish(self.String(data=json.dumps(dict(
            task_id=self.task or self.release_task, locked=bool(self.task), stopped=stopped,
            ready=bool(valid), reason=self.error or ('BASE_LOCK_RETAINED' if self.task else 'UNLOCKED'),
            owner_verified=bool(owned), final_topic=self.node.output))))
