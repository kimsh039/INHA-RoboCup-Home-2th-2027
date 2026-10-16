"""Persistent final-output interlock for a completed docking session."""
import json
import math
import os
from pathlib import Path
import time


class DockMotionGate:
    NEXT = {'hold': {'prepare'}, 'prepare': {'reverse'}, 'reverse': {'stop_reverse'},
            'stop_reverse': {'turn'}, 'turn': {'stop_turn'}, 'stop_turn': {'return'},
            'return': {'done'}, 'done': set(), 'blocked': set()}

    def __init__(self, node):
        from std_msgs.msg import String
        self.node, self.String = node, String
        self.path = Path(node.declare_parameter('dock_lock_file', '~/.ros/inha_control_center/dock_gate.json').value).expanduser()
        self.dock_id = ''; self.task_id = ''; self.phase = 'done'; self.error = ''
        self.control = {}; self.received = 0.; self.drive = {}; self.drive_received = 0.
        self.phase_at = 0.; self.origin = None; self.distance = 0.; self.reverse_required = 0.; self.target_yaw = None
        try:
            if self.path.exists():
                saved = json.loads(self.path.read_text())
                self.dock_id, self.task_id = saved.get('dock_id', ''), saved.get('task_id', '')
                self.phase = 'blocked' if saved.get('phase') != 'done' else 'done'
        except (OSError, ValueError):
            self.phase, self.error = 'blocked', 'DOCK_JOURNAL_INVALID'
        node.create_subscription(String, '/detection/closed_approach/status', self.docking, 1)
        node.create_subscription(String, '/navigation/departure/control', self.command, 1)
        node.create_subscription(String, '/manipulation/drive_ready', self.arm, 1)
        self.pub = node.create_publisher(String, '/navigation/departure/gate', 1)

    def now(self): return self.node.get_clock().now().nanoseconds/1e9

    def persist(self):
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix('.tmp')
            with tmp.open('w') as f:
                json.dump(dict(dock_id=self.dock_id, task_id=self.task_id, phase=self.phase), f)
                f.flush(); os.fsync(f.fileno())
            tmp.replace(self.path)
        except OSError:
            self.phase, self.error = 'blocked', 'DOCK_PERSIST_FAILED'

    def owner(self, topic, name):
        sources = self.node.get_publishers_info_by_topic(topic)
        return len(sources) == 1 and sources[0].node_name == name

    def docking(self, msg):
        try:
            value = json.loads(msg.data)
            dock = value.get('task_id')
            if (value.get('phase') == 'succeeded' and dock and dock != self.dock_id
                    and self.owner('/detection/closed_approach/status', 'inha_docking_control')):
                self.dock_id, self.phase, self.task_id = dock, 'hold', ''
                self.control = {}; self.origin = None; self.distance = 0.
                self.persist()
        except (ValueError, TypeError): pass

    def arm(self, msg):
        try:
            value = json.loads(msg.data)
            if (self.owner('/manipulation/drive_ready', 'real_manipulation') and
                    -.05 <= self.now()-value['stamp'] <= .4):
                self.drive, self.drive_received = value, time.monotonic()
        except (ValueError, TypeError, KeyError): pass

    def command(self, msg):
        try:
            value = json.loads(msg.data)
            if not self.owner('/navigation/departure/control', 'inha_docking_control'): return
            if not -.05 <= self.now()-value['stamp'] <= .3 or value['dock_id'] != self.dock_id: return
            if self.task_id and value['task_id'] != self.task_id: return
            phase = value['phase']
            if phase == 'blocked':
                self.phase = 'blocked'; self.persist(); return
            if phase != self.phase:
                if phase not in self.NEXT.get(self.phase, set()): return
                base = self.node.manipulation_lock
                if phase in ('prepare', 'reverse', 'turn', 'return', 'done') and not base.stopped(): return
                if phase == 'prepare':
                    self.task_id = value['task_id']
                    required = value['reverse_distance']
                    if not isinstance(required, (float, int)) or not .05 <= required <= 1.: return
                    self.reverse_required = required
                if phase == 'reverse':
                    if not base.pose: return
                    self.origin = base.pose
                if phase == 'turn' and self.distance < self.reverse_required-.005: return
                if phase == 'turn': self.target_yaw = value['target_yaw']
                if phase == 'return':
                    if not base.pose or self.target_yaw is None: return
                    if abs(math.atan2(math.sin(base.pose[2]-self.target_yaw), math.cos(base.pose[2]-self.target_yaw))) > math.radians(2): return
                self.phase = phase; self.phase_at = self.now(); self.persist()
            self.control, self.received = value, time.monotonic()
        except (ValueError, TypeError, KeyError): pass

    def allows(self, velocity, approval_stamp):
        if self.phase == 'done': return not self.error
        if self.phase not in ('reverse', 'turn', 'return'): return False
        now = time.monotonic()
        if (self.error or now-self.received > .25 or not -.05 <= self.now()-self.control.get('stamp', 0.) <= .3
                or now-self.drive_received > .3 or not self.drive.get('ready')
                or self.drive.get('task_id') != self.task_id or not self.drive.get('released')
                or approval_stamp is None or approval_stamp/1e9 < self.phase_at): return False
        base = self.node.manipulation_lock
        if base.pose and self.origin:
            dx, dy = base.pose[0]-self.origin[0], base.pose[1]-self.origin[1]
            self.distance = -(dx*math.cos(self.origin[2])+dy*math.sin(self.origin[2]))
        v, w = velocity.linear.x, velocity.angular.z
        if self.phase == 'reverse':
            intended = self.control.get('linear', 0.)
            return (abs(w) <= 1e-6 and -.05 <= v <= 0 and intended <= v <= 0
                    and self.distance <= self.reverse_required+.015)
        if self.phase == 'turn':
            intended = self.control.get('angular', 0.)
            return abs(v) <= 1e-6 and abs(w) <= min(.15, abs(intended))+1e-6 and w*intended >= 0
        return True

    def publish(self):
        self.pub.publish(self.String(data=json.dumps(dict(dock_id=self.dock_id, task_id=self.task_id,
            phase=self.phase, stamp=self.now(), reverse_distance=self.distance, reason=self.error))))
