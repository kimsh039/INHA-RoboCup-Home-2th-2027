"""Measured tabletop plate placement and explicit stopped-arm departure handoff."""
from collections import deque
import json
import math
import time
import uuid
import numpy as np
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld
from std_msgs.msg import String
from robocup_real_manipulation_msgs.msg import ObservationRequest
from .core import Fault, fresh, require, support
from .ros_util import seconds


class PlateTaskMixin:
    def orchestrator_status(self, msg):
        try:
            value = json.loads(msg.data)
            sources = self.get_publishers_info_by_topic('/task/orchestrator')
            if len(sources) == 1 and sources[0].node_name == 'inha_control_center':
                self.orchestrator = value
        except (ValueError, TypeError): pass

    def require_orchestrator(self):
        value = self.orchestrator
        require(value and value.get('active') and value.get('task_id') == self.locked_task,
                'TABLE_TASK_CANCELED_OR_OWNER_LOST')
        fresh(value['stamp'], self.now(), 1.)

    def plate_config(self):
        require(self.c.get('plate_placement_verified') is True, 'PLATE_PLACEMENT_NOT_COMMISSIONED')
        limits = np.asarray(self.c['joint_limits'])
        for name in ('rest_joints', 'plate_observation_joints'):
            q = np.asarray(self.c.get(name), float)
            require(q.shape == (6,) and np.isfinite(q).all() and
                    np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]), 'UNSET_OR_INVALID:'+name)
        for q in self.c.get('transit_waypoints', []):
            q = np.asarray(q, float)
            require(q.shape == (6,) and np.isfinite(q).all() and
                    np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]), 'TRANSIT_POSE_LIMIT')
        require(self.c.get('base_completion_policy', 'hold') == 'hold', 'PLATE_REQUIRES_EXPLICIT_DEPARTURE')

    def arm_stowed(self):
        try:
            fresh(self.joint_stamp, self.now(), self.c['feedback_age_s'])
            require(self.joint is not None and self.still_since is not None and
                time.monotonic()-self.still_since >= self.c['joint_stable_s'], 'ARM_NOT_STILL')
            measured = dict(zip(self.joint.name, self.joint.position))
            expected = self.c.get('rest_joints')
            require(expected is not None and len(expected) == 6, 'REST_POSE_UNSET')
            return all(math.isfinite(measured[n]) and abs(measured[n]-q) <= self.c['goal_tolerance_rad']
                       for n, q in zip(self.c['joint_names'], expected))
        except (Fault, KeyError, TypeError):
            return False

    def observe_plate(self):
        self.motion.joint_move(self.c['plate_observation_joints'], self.guard)
        self.until(lambda: self.still_since is not None and
                   time.monotonic()-self.still_since >= self.c['joint_stable_s'], self.c['stage_timeout_s'])
        self.request_id = uuid.uuid4().hex
        self.observations = deque(maxlen=64)
        acquired = self.now()
        req = ObservationRequest(task_id=self.goal_handle.request.task_id, observation_id=self.request_id,
                                 class_id=-1, purpose='plate', support_z=self.c['table_z_m'])
        req.header.stamp = self.get_clock().now().to_msg()
        req.coarse_position = self.point(self.coarse)
        self.request_pub.publish(req)
        deadline = time.monotonic()+self.c['observation_timeout_s']
        stable = []
        while time.monotonic() < deadline:
            self.guard()
            with self.mutex:
                batch = list(self.observations); self.observations.clear()
            for obs in batch:
                if obs.purpose != 'plate': continue
                fresh(seconds(obs.header.stamp), self.now(), self.c['max_observation_age_s'])
                require(seconds(obs.header.stamp) >= acquired, 'PLATE_CAPTURE_BEFORE_REQUEST')
                require(obs.candidate_count == 1 and obs.candidate_index == 0, 'PLATE_AMBIGUOUS')
                require(obs.has_position and obs.confidence >= .85 and
                        0 < obs.position_std_m <= self.c['max_position_std_m'], 'PLATE_DEPTH_QUALITY')
                require(obs.free_radius_m >= self.rule['radius_m']+.01, 'OBJECT_DOES_NOT_FIT_PLATE')
                point = self.to_gravity([obs.position.x, obs.position.y, obs.position.z], obs.header)
                require(support(point, self.rule['radius_m'], self.c), 'PLATE_OUTSIDE_MEASURED_TABLE')
                require(self.still_since is not None and seconds(obs.header.stamp) <=
                        self.joint_stamp+self.c['future_tolerance_s'], 'PLATE_ARM_NOT_STILL')
                stamp = seconds(obs.header.stamp)
                if stable and stamp <= stable[-1][0]: continue
                stable.append((stamp, point, float(obs.free_radius_m)))
                stable = stable[-self.c['observation_stable_frames']:]
                if len(stable) < self.c['observation_stable_frames']: continue
                points = np.array([s[1] for s in stable])
                if np.max(np.std(points, axis=0)) > self.c['max_position_std_m']:
                    stable = []; continue
                self.plate_point = np.mean(points, axis=0)
                self.plate_radius = min(s[2] for s in stable)
                self.plate_stamp = stamp
                return
            time.sleep(.01)
        raise Fault('PLATE_NOT_FOUND_OR_DEPTH_UNCONFIRMED')

    def place_on_plate(self):
        require(hasattr(self, 'plate_point'), 'MEASURED_PLATE_REQUIRED')
        # The whole base remains locked between acquisition and placement.
        fresh(self.plate_stamp, self.now(), self.c.get('plate_plan_max_age_s', 20.))
        up = np.array([0., 0., 1.])
        center = self.plate_point+up*(self.rule['center_height_m']+.005)
        candidate = center+(self.tcp-self.center)
        radius = self.plate_radius
        plate = self.motion.box('manip_plate', self.plate_point-up*.003,
                                [2*radius, 2*radius, .006])
        self.motion.apply(PlanningScene(is_diff=True, world=PlanningSceneWorld(
            collision_objects=[plate])), self.guard)
        self.motion.contacts(self.c['touch_links'], self.guard)
        hover = self.motion.plan_to(candidate+up*self.c['lift_m'], self.q, guard=self.guard)
        self.motion.contacts(self.c['touch_links']+['manip_plate'], self.guard)
        self.motion.cart_plan(candidate, self.q, self.guard, start=hover)
        self.motion.contacts(self.c['touch_links'], self.guard)
        self.motion.execute(hover, self.guard)
        fresh(self.plate_stamp, self.now(), self.c.get('plate_plan_max_age_s', 20.))
        self.motion.contacts(self.c['touch_links']+['manip_plate'], self.guard)
        self.motion.cart_move(candidate, self.q, self.guard)
        self.place_tcp, self.place_center = candidate, center

    def departure_request(self, msg):
        try:
            value = json.loads(msg.data)
            fresh(value['stamp'], self.now(), .5)
            sources = self.get_publishers_info_by_topic('/manipulation/departure/request')
            require(len(sources) == 1 and sources[0].node_name == 'inha_docking_control', 'DEPARTURE_OWNER')
            with self.mutex:
                require(value['task_id'] == self.locked_task and self.goal_handle is None, 'DEPARTURE_TASK')
                terminal = self.last_status
                require(terminal is not None and terminal.task_id == self.locked_task and
                    terminal.outcome == 'SUCCESS' and terminal.place_success and terminal.return_success and
                    terminal.stop_confirmed and terminal.holding == 'EMPTY', 'DEPARTURE_REQUIRES_SUCCESS')
                require(self.arm_stowed(), 'DEPARTURE_ARM_NOT_STOWED')
                gate = self.departure_gate
                require(gate and gate.get('task_id') == self.locked_task and gate.get('phase') == 'prepare'
                    and time.monotonic()-self.departure_gate_received < .5, 'DEPARTURE_GATE_NOT_PREPARED')
                if not self.departure_release:
                    self.departure_release = dict(task_id=self.locked_task, started=self.now())
                    self.lock_requested = False; self.lease = None; self.publish_lock()
        except Exception as exc:
            self.get_logger().warning('departure rejected: '+str(exc), throttle_duration_sec=2.)

    def departure_gate_status(self, msg):
        try:
            self.departure_gate = json.loads(msg.data)
            self.departure_gate_received = time.monotonic()
        except ValueError:
            self.departure_gate = None

    def departure_tick(self):
        with self.mutex:
            if self.departure_release:
                attempt = self.departure_release
                gate = self.departure_gate or {}
                valid = (self.arm_stowed() and time.monotonic()-self.departure_gate_received < .5
                         and gate.get('task_id') == attempt['task_id'] and gate.get('phase') == 'prepare')
                if not valid or self.now()-attempt['started'] > self.c.get('base_unlock_timeout_s', 3.):
                    self.lock_requested = True; self.departure_release = None; self.publish_lock()
                elif (self.lease and self.lease.task_id == attempt['task_id'] and not self.lease.locked
                      and self.lease.stopped and attempt['started'] <= seconds(self.lease.header.stamp)
                      and -.05 <= self.now()-seconds(self.lease.header.stamp) <= self.c['base_age_s']):
                    self.release_task = self.locked_task; self.locked_task = ''
                    self.departure_release = None; self.persist(); self.publish_lock()
            terminal = self.last_status
            completed = bool(terminal and terminal.outcome == 'SUCCESS' and terminal.place_success and
                             terminal.return_success and terminal.stop_confirmed and terminal.holding == 'EMPTY')
            self.drive_pub.publish(String(data=json.dumps(dict(stamp=self.now(),
                task_id=terminal.task_id if terminal else '', ready=completed and self.arm_stowed(),
                released=completed and not self.locked_task and bool(self.release_task),
                joints=dict(zip(self.joint.name,self.joint.position)) if self.joint else {}))))
