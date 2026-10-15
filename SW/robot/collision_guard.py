#!/usr/bin/env python3
"""Reference Python guard and compatibility entry point for the C++ guard.

Nav2 chooses the route. This gate limits its velocity using measured URDF body
geometry and a 3D occupied scene, including the body's swept turning corners.
CollisionGuard remains importable for offline parity checks. Running this file
starts the native guard; the independent watchdog owns the protected output.
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Point32, Polygon, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy

from collision_geometry import RobotGeometry, rectangle
from collision_distance import ComputationExpired, PreparedBoxes, build_scene
from slam_scan import NS, stamp_ns, yaw


class CollisionGuard(Node):
    def __init__(self):
        super().__init__('collision_guard')
        defaults = dict(base_frame='base_link', odom_frame='odom', odom_topic='/odom_ekf',
                        joint_topic='/robot/joint_states', safety_topic='/collision/points',
                        cloud_status_topic='/collision/cloud_status', command_topic='/cmd_vel',
                        approved_topic='/collision/cmd_vel_approved',
                        protected_topic='/tracer/cmd_vel_safe', status_topic='/collision/status',
                        clearance=0.1, uncertainty_margin=0.05, safety_voxel=0.04,
                        cloud_timeout=0.4, odom_timeout=0.2, joint_timeout=0.5,
                        require_arm_feedback=True, arm_motion_limit=0.08, arm_settle_sec=0.5,
                        command_timeout=0.25, control_frequency=20., reaction_time=0.3,
                        linear_deceleration=0.5, angular_deceleration=1.,
                        max_linear_velocity=0.25, max_reverse_velocity=0.15,
                        max_angular_velocity=0.6, prediction_step=0.04, minimum_speed_scale=0.1,
                        computation_budget=0.04, costmap_resolution=0.05)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for key in ('clearance', 'cloud_timeout', 'odom_timeout', 'joint_timeout',
                    'command_timeout', 'control_frequency', 'linear_deceleration',
                    'angular_deceleration', 'prediction_step', 'computation_budget'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError('Invalid collision guard parameter: '+key)
        if len({self.cfg['command_topic'], self.cfg['approved_topic'], self.cfg['protected_topic']}) != 3:
            raise ValueError('Raw and protected velocity topics must differ')
        self.model = RobotGeometry()
        self._geometry_cache = {}
        self.pose, self.measured = None, np.zeros(2)
        self.odom_stamp, self.odom_received, self.odom_fault_until = None, None, 0.
        self.joints, self.joint_received, self.joint_stamp, self.arm_moved = {}, None, None, 0.
        self.scene, self.scene_received, self.scene_stamp = None, None, None
        self.scene_sources, self.nearest_source = None, None
        self.pending_scenes, self.pending_health, self.scene_generation = {}, {}, None
        self.cloud_health, self.health_received = {}, None
        self.command, self.command_received, self.command_error = Twist(), None, ''
        self.route_error = 'waiting_for_protected_tracer'
        self.result = dict(state='blocked', reason='waiting_for_mid360', ready=False,
                           minimum_distance=None, nearest_point=None, nearest_link=None, speed_scale=0.)
        self.pub = self.create_publisher(TwistStamped, self.cfg['approved_topic'], 1)
        self.status_pub = self.create_publisher(String, self.cfg['status_topic'], 10)
        self.footprint_pubs = [self.create_publisher(Polygon, topic, 1) for topic in
                              ('/local_costmap/footprint', '/global_costmap/footprint')]
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Twist, self.cfg['command_topic'], self.on_command, 1)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, self.cfg['odom_topic'], self.on_odom, latest)
        self.create_subscription(JointState, self.cfg['joint_topic'], self.on_joints, latest)
        self.create_subscription(PointCloud2, self.cfg['safety_topic'], self.on_scene, latest)
        self.create_subscription(String, self.cfg['cloud_status_topic'], self.on_cloud_health, 10)
        # A paused /clock must not pause the command watchdog.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1./self.cfg['control_frequency'], self.control, clock=self.wall_clock)
        self.create_timer(0.2, self.status, clock=self.wall_clock)
        self.create_timer(0.5, self.routes, clock=self.wall_clock)

    def on_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        self.command_received = time.monotonic()
        if not all(math.isfinite(v) for v in values) or any(abs(v) > 1e-6 for v in (values[1], values[2], values[3], values[4])):
            self.command, self.command_error = Twist(), 'invalid_velocity_command'
            return
        self.command_error = ''
        self.command = Twist()
        self.command.linear.x = float(np.clip(msg.linear.x, -self.cfg['max_reverse_velocity'], self.cfg['max_linear_velocity']))
        self.command.angular.z = float(np.clip(msg.angular.z, -self.cfg['max_angular_velocity'], self.cfg['max_angular_velocity']))

    def on_odom(self, msg):
        stamp = stamp_ns(msg.header)
        p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        if (msg.header.frame_id != self.cfg['odom_frame'] or msg.child_frame_id != self.cfg['base_frame'] or
                stamp <= 0 or not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w, twist.linear.x, twist.angular.z)) or
                q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < 1e-12):
            self.odom_fault_until = time.monotonic()+1.
            return
        pose = np.array([p.x, p.y, yaw(q)])
        if self.odom_stamp is not None:
            dt = (stamp-self.odom_stamp)/NS
            heading = abs(math.atan2(math.sin(pose[2]-self.pose[2]), math.cos(pose[2]-self.pose[2])))
            if dt <= 0 or np.linalg.norm(pose[:2]-self.pose[:2]) > 0.15 or heading > 0.15:
                self.odom_fault_until = time.monotonic()+1.
        self.pose, self.odom_stamp, self.odom_received = pose, stamp, time.monotonic()
        self.measured = np.array([twist.linear.x, twist.angular.z])

    def on_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not self.model.required_joints.issubset(positions) or not all(math.isfinite(v) for v in positions.values()):
            return
        now = time.monotonic()
        stamp = stamp_ns(msg.header)
        if stamp <= 0 or (self.joint_stamp is not None and stamp <= self.joint_stamp):
            return
        if self.joint_stamp is not None:
            dt = (stamp-self.joint_stamp)/NS
            if any(abs(positions[k]-self.joints[k])/dt > self.cfg['arm_motion_limit'] for k in self.model.required_joints):
                self.arm_moved = now
        self.joints, self.joint_received, self.joint_stamp = positions, now, stamp

    def on_scene(self, msg):
        if msg.header.frame_id != self.cfg['odom_frame'] or stamp_ns(msg.header) <= 0:
            self.scene_received = None
            return
        try:
            began = time.monotonic()
            fields = {field.name for field in msg.fields}
            names = ['x', 'y', 'z'] + (['source_x', 'source_y', 'source_z'] if {'source_x', 'source_y', 'source_z'}.issubset(fields) else [])
            data = pc2.read_points(msg, field_names=names, skip_nans=False)
            points = np.empty((data.size, 3), dtype=float)
            for axis, name in enumerate(('x', 'y', 'z')):
                points[:, axis] = data[name].ravel()
            if not np.isfinite(points).all():
                raise ValueError('invalid retained scene')
            sources = np.column_stack([data[name].ravel() for name in names[3:]]) if len(names) == 6 else None
            stamp = stamp_ns(msg.header)
            tree_began = time.monotonic()
            scene = build_scene(points)
            completed = time.monotonic()
            self.scene_parse_ms, self.scene_tree_build_ms = (tree_began-began)*1000, (completed-tree_began)*1000
            self.pending_scenes[stamp] = (scene, sources, completed)
            while len(self.pending_scenes) > 4:
                del self.pending_scenes[next(iter(self.pending_scenes))]
            self.pair_scene(stamp)
        except (ValueError, KeyError, TypeError):
            self.scene_received = None

    def on_cloud_health(self, msg):
        try:
            health = json.loads(msg.data)
            if not isinstance(health, dict):
                raise ValueError('invalid cloud health')
            generation = (health.get('session_id'), health.get('generation'))
            if generation != self.scene_generation:
                self.scene_generation = generation
                self.scene_received = self.scene_stamp = None
                self.pending_scenes.clear()
                self.pending_health.clear()
            stamp = health.get('reference_stamp_ns')
            if not health.get('ready') or not isinstance(stamp, int) or stamp <= 0:
                self.cloud_health, self.health_received = dict(health, ready=False), time.monotonic()
                return
            self.pending_health[stamp] = (health, time.monotonic())
            while len(self.pending_health) > 4:
                del self.pending_health[next(iter(self.pending_health))]
            self.pair_scene(stamp)
        except (ValueError, TypeError):
            self.cloud_health, self.health_received = dict(ready=False, reason='invalid_cloud_health'), time.monotonic()

    def pair_scene(self, stamp):
        if stamp not in self.pending_health:
            return
        health, received = self.pending_health[stamp]
        if self.scene_stamp == stamp:
            self.cloud_health, self.health_received = health, received
            return
        if stamp not in self.pending_scenes or (self.scene_stamp is not None and stamp < self.scene_stamp):
            return
        scene, sources, arrived = self.pending_scenes[stamp]
        self.scene, self.scene_sources = scene, sources
        self.scene_stamp, self.scene_received = stamp, arrived
        self.cloud_health, self.health_received = health, received
        self.pending_scenes = {key: value for key, value in self.pending_scenes.items() if key >= stamp}
        self.pending_health = {key: value for key, value in self.pending_health.items() if key >= stamp}

    def routes(self):
        try:
            approved = self.get_publishers_info_by_topic(self.cfg['approved_topic'])
            protected = self.get_publishers_info_by_topic(self.cfg['protected_topic'])
            drivers = self.get_subscriptions_info_by_topic(self.cfg['protected_topic'])
            bypass = self.get_subscriptions_info_by_topic(self.cfg['command_topic'])
            if len(approved) != 1 or approved[0].node_name != self.get_name():
                self.route_error = 'duplicate_velocity_guard'
            elif len(protected) != 1 or protected[0].node_name != 'tracer_safety_watchdog':
                self.route_error = 'missing_actuator_watchdog'
            elif any('tracer_base' in info.node_name for info in bypass):
                self.route_error = 'tracer_requires_protected_remap'
            elif not any('tracer_base' in info.node_name for info in drivers):
                self.route_error = 'waiting_for_protected_tracer'
            else:
                self.route_error = ''
        except Exception:
            self.route_error = 'velocity_route_unavailable'

    def healthy(self):
        now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds
        if self.route_error:
            return self.route_error
        if self.health_received is None or now-self.health_received > self.cfg['cloud_timeout']:
            return 'missing_cloud_health'
        if not self.cloud_health.get('ready'):
            return self.cloud_health.get('reason', 'invalid_mid360')
        if (self.scene_received is None or now-self.scene_received > self.cfg['cloud_timeout'] or
                not -0.05 <= (ros_now-self.scene_stamp)/NS <= self.cfg['cloud_timeout']):
            return 'stale_mid360'
        if (self.odom_received is None or now-self.odom_received > self.cfg['odom_timeout'] or
                not -0.05 <= (ros_now-self.odom_stamp)/NS <= self.cfg['odom_timeout']):
            return 'stale_odometry'
        if now < self.odom_fault_until:
            return 'odometry_discontinuity'
        if self.cfg['require_arm_feedback']:
            if (self.joint_received is None or now-self.joint_received > self.cfg['joint_timeout'] or
                    not -0.05 <= (ros_now-self.joint_stamp)/NS <= self.cfg['joint_timeout']):
                return 'stale_arm_feedback'
            if now-self.arm_moved < self.cfg['arm_settle_sec']:
                return 'arm_is_moving'
        try:
            transform = self.tf.lookup_transform(self.cfg['odom_frame'], self.cfg['base_frame'], Time())
            age = (ros_now-stamp_ns(transform.header))/NS
            p, q = transform.transform.translation, transform.transform.rotation
            delta_yaw = math.atan2(math.sin(yaw(q)-self.pose[2]), math.cos(yaw(q)-self.pose[2]))
            if not -0.05 <= age <= self.cfg['odom_timeout'] or math.hypot(p.x-self.pose[0], p.y-self.pose[1]) > 0.05 or abs(delta_yaw) > 0.08:
                return 'stale_or_inconsistent_tf'
        except Exception:
            return 'missing_odometry_tf'
        return ''

    def geometry(self, include_arm):
        key = tuple(self.joints[name] for name in sorted(self.model.required_joints)) if include_arm else ()
        cache = getattr(self, '_geometry_cache', {})
        cached = cache.get(include_arm)
        if cached is None or cached[0] != key:
            boxes = self.model.boxes(self.joints) if include_arm else self.model.fixed
            cached = (key, PreparedBoxes(boxes))
            cache[include_arm] = cached
            self._geometry_cache = cache
        return cached[1]

    def distance(self, boxes, pose, radius, deadline):
        measured, index, owner = boxes.nearest(self.scene, pose, radius, deadline)
        if index is None:
            return measured, None, None
        if self.scene_sources is not None and np.array_equal(pose, self.pose):
            self.nearest_source = self.scene_sources[index].tolist()
        return measured, self.scene.data[index].tolist(), boxes.boxes[owner].name

    def predict(self, pose, phases, boxes, boundary, extent, deadline):
        pose = pose.copy()
        step = self.cfg['prediction_step']
        deceleration = np.array([self.cfg['linear_deceleration'], self.cfg['angular_deceleration']])
        for velocity, duration, braking in phases:
            if not np.any(np.abs(velocity) > 1e-6):
                continue
            elapsed = 0.
            while elapsed < duration-1e-9:
                dt = min(step, duration-elapsed)
                following = np.sign(velocity)*np.maximum(np.abs(velocity)-deceleration*dt, 0.) if braking else velocity
                v, w = (velocity+following)/2
                theta = pose[2]+w*dt/2
                previous = pose.copy()
                pose += np.array([v*math.cos(theta)*dt, v*math.sin(theta)*dt, w*dt])
                # Enclose motion between discrete pose samples, including corners.
                sampling_margin = (abs(v)+abs(w)*extent)*dt
                distance, _, _ = boxes.nearest(self.scene, pose, boundary+sampling_margin, deadline,
                                                from_pose=previous, sources=self.scene_sources)
                if distance < boundary+sampling_margin:
                    return None
                velocity, elapsed = following, elapsed+dt
        return pose

    def admissible(self, command, reaction_pose, latency, boxes, boundary, extent, deadline):
        phases = [(command.copy(), latency, False),
                  (command.copy(), max(abs(command[0])/self.cfg['linear_deceleration'],
                                       abs(command[1])/self.cfg['angular_deceleration']), True)]
        return self.predict(reaction_pose, phases, boxes, boundary, extent, deadline) is not None

    def control(self):
        began = time.monotonic()
        output = Twist()
        self.nearest_source = None
        state, reason, scale, distance, point, link, ready = 'blocked', '', 0., None, None, None, False
        try:
            reason = self.healthy()
            ready = not reason
            if ready:
                deadline = time.monotonic()+self.cfg['computation_budget']
                boxes = self.geometry(self.cfg['require_arm_feedback'])
                boundary = self.cfg['clearance']+self.cfg['uncertainty_margin']+math.sqrt(3)*self.cfg['safety_voxel']/2
                measured, point, link = self.distance(boxes, self.pose, boundary+0.1, deadline)
                distance = max(0., measured-math.sqrt(3)*self.cfg['safety_voxel']/2) if math.isfinite(measured) else None
                if self.command_error:
                    reason = self.command_error
                elif self.command_received is None or time.monotonic()-self.command_received > self.cfg['command_timeout']:
                    state, reason = 'idle', 'command_timeout'
                elif not (abs(self.command.linear.x) > 1e-5 or abs(self.command.angular.z) > 1e-5):
                    state, reason = 'idle', 'zero_command'
                else:
                    command = np.array([self.command.linear.x, self.command.angular.z])
                    sensor_age = max(0., (self.get_clock().now().nanoseconds-self.scene_stamp)/NS)
                    latency = self.cfg['reaction_time']+sensor_age
                    extent = boxes.extent
                    reaction_pose = self.predict(self.pose, [(self.measured.copy(), latency, False)],
                                                 boxes, boundary, extent, deadline)
                    if reaction_pose is not None:
                        for candidate in (1., 0.75, 0.5, 0.25, self.cfg['minimum_speed_scale']):
                            if self.admissible(command*candidate, reaction_pose, latency, boxes, boundary, extent, deadline):
                                scale = candidate
                                output.linear.x, output.angular.z = map(float, command*scale)
                                state, reason = ('clear', 'clear_path') if scale == 1. else ('slowed', 'braking_clearance')
                                break
                    if scale == 0.:
                        reason = 'predicted_clearance_violation'
        except ComputationExpired:
            ready, reason = False, 'computation_timeout'
        except Exception as exc:
            ready, reason = False, 'guard_error'
            self.get_logger().error('3D velocity gate stopped: '+str(exc), throttle_duration_sec=2.)
        # A fresh approval timestamp cannot make an aged scene valid again.
        if ready:
            expired = self.healthy()
            if time.monotonic() > deadline:
                expired = 'computation_timeout'
            if expired:
                output, state, ready, reason, scale = Twist(), 'blocked', False, expired, 0.
        # Stopping bypasses smoothing so a downstream smoother cannot prolong it.
        approved = TwistStamped()
        approved.header.frame_id = self.cfg['base_frame']
        approved.header.stamp = self.get_clock().now().to_msg()
        approved.twist = output
        self.pub.publish(approved)
        self.result = dict(state=state, reason=reason, ready=ready, minimum_distance=distance,
                           nearest_source_point=self.nearest_source,
                           nearest_point=point, nearest_link=link, speed_scale=scale,
                           command=[self.command.linear.x, self.command.angular.z],
                           output=[output.linear.x, output.angular.z],
                           guard_processing_ms=(time.monotonic()-began)*1000,
                           guard_budget_ms=self.cfg['computation_budget']*1000)
        if (state == 'blocked' and self.command_received is not None and
                time.monotonic()-self.command_received <= self.cfg['command_timeout'] and
                (abs(self.command.linear.x) > 1e-5 or abs(self.command.angular.z) > 1e-5)):
            self.get_logger().warning(
                f'3D command blocked: {reason}; requested=({self.command.linear.x:.3f}, '
                f'{self.command.angular.z:.3f}); guard={self.result["guard_processing_ms"]:.1f} ms',
                throttle_duration_sec=2.)

    def status(self):
        try:
            boxes = self.geometry(self.model.required_joints.issubset(self.joints))
            padding = 0.  # Direction-dependent clearance is enforced by predict().
            footprint = rectangle(boxes, padding)
            if self.cfg['require_arm_feedback'] and not self.model.required_joints.issubset(self.joints):
                # Nav2 is blocked by readiness while feedback is missing.
                footprint = rectangle(self.geometry(False), padding)
            polygon = Polygon(points=[Point32(x=x, y=y, z=0.) for x, y in footprint])
            for publisher in self.footprint_pubs:
                publisher.publish(polygon)
            result = dict(self.result, clearance=self.cfg['clearance'], uncertainty=self.cfg['uncertainty_margin'],
                          frame=self.cfg['odom_frame'], base_frame=self.cfg['base_frame'],
                          tracer_bounds=[self.model.tracer_low.tolist(), self.model.tracer_high.tolist()],
                          body_footprint=rectangle(self.model.tracer),
                          clearance_footprint=rectangle(self.model.tracer, self.cfg['clearance']),
                          planning_footprint=footprint, protected_topic=self.cfg['protected_topic'],
                          scene_parse_ms=getattr(self, 'scene_parse_ms', 0.),
                          scene_tree_build_ms=getattr(self, 'scene_tree_build_ms', 0.),
                          cloud=self.cloud_health, stamp=self.get_clock().now().nanoseconds/NS)
            self.status_pub.publish(String(data=json.dumps(result, allow_nan=False)))
        except Exception as exc:
            self.get_logger().error('Collision status unavailable: '+str(exc), throttle_duration_sec=2.)


def main():
    import os
    from pathlib import Path
    import sys
    launcher = str(Path(__file__).resolve().with_name('run-collision-guard.sh'))
    os.execv('/bin/bash', ['bash', launcher, *sys.argv[1:]])


if __name__ == '__main__':
    main()
