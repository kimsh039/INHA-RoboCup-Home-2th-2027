"""Real-robot YOLO/LiDAR approach owned by the control-center ROS executor.

Nav2 targets a reachable grasp side when a table is known, or the observed 3D
point while acquiring that model. Final feedback uses the stored LiDAR target
relative to current odometry, so leaving RGB FOV does not cancel the approach.
All velocities enter /cmd_vel and the existing 3D guard / actuator watchdog.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import math
import time

CLASSES = ('green_apple', 'peach', 'tomato', 'fanta_can', 'cup', 'banana')
ALIASES = {'사과': 'green_apple', '청사과': 'green_apple', 'apple': 'green_apple', 'green apple': 'green_apple',
           '복숭아': 'peach', '토마토': 'tomato', '환타': 'fanta_can', '환타캔': 'fanta_can', '환타 캔': 'fanta_can',
           'fanta': 'fanta_can', 'fanta can': 'fanta_can', '컵': 'cup', '머그': 'cup', 'mug': 'cup', '바나나': 'banana'}


def target_label(value):
    if not isinstance(value, str) or len(value) > 48:
        raise ValueError('찾을 물체 이름을 입력하세요.')
    name = value.strip().lower()
    name = ALIASES.get(name, name)
    if name not in CLASSES:
        raise ValueError('현재 YOLO 지원 물체: 사과, 복숭아, 토마토, 환타캔, 컵, 바나나')
    return name


class DetectionApproach:
    def __init__(self, node, state, settings):
        import numpy as np
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import PointStamped, Twist
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from head_lidar import HeadLidar
        from grasp_approach import GraspPlanner
        self.np, self.PointStamped, self.Twist, self.String = np, PointStamped, Twist, String
        self.node, self.state = node, state
        self.lidar = HeadLidar(node, settings)
        self.planner = GraspPlanner(settings)
        from joint_contract import ModelJointFeedback
        self.arm_feedback = ModelJointFeedback(node, settings)
        self.requests = deque()
        state.detection_request = self.requests.append
        self.label = ''
        self.phase, self.message = 'idle', '물체 이름을 입력하세요.'
        self.phase_started = time.monotonic()
        self.active = False
        self.observing = True
        self.target = self.anchor = self.stage_goal = None
        self.samples = deque(maxlen=3)
        self.fusion_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='head-lidar')
        self.fusion_future = None
        self.generation = 0
        self.last_image = self.last_lidar = self.last_fusion = 0.
        self.last_seen = self.started = self.settle_until = self.arrived_since = None
        self.last_pose = self.last_command = None
        self.previous_reference = None
        self.previous_ros_time = None
        self.progress_best, self.progress_at = math.inf, 0.
        self.edge_clearance = float(settings.get('table_clearance', 0.))
        if not 0 <= self.edge_clearance <= .1:
            raise ValueError('detection.table_clearance는 0~0.1m입니다.')
        self.distance = None
        self.plan, self.candidates, self.surface_future = None, [], None
        self.surface_requested = 0.
        self.surface_note = ''
        self.grasp_surface_attempted = None
        self.surface_after = 0.
        self.point_blocked_since = None
        self.recovery, self.recovery_attempts = None, 0
        self.finish_request = None
        self.heading_tolerance = .02
        self.nav_busy = False
        self.cmd = node.create_publisher(Twist, '/cmd_vel', 1)
        self.status_pub = node.create_publisher(String, '/detection/closed_approach/status', 1)
        self.point_pub = node.create_publisher(PointStamped, '/detection/closed_approach/target', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.nav_subscription = node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                                        self.navigation_status, qos)

    def navigation_status(self, msg):
        self.nav_busy = any(s.status in (1, 2, 3) for s in msg.status_list)

    def report(self, phase, message):
        if phase != self.phase:
            self.phase_started = time.monotonic()
        if (phase, message) != (self.phase, self.message):
            self.state.log('Detection · '+message, 'warn' if phase == 'error' else 'info', 'detection')
        self.phase, self.message = phase, message

    def command(self, linear=0., angular=0.):
        msg = self.Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd.publish(msg)
        self.last_command = (linear, angular)

    def cancel(self, message='접근 취소됨', phase='canceled'):
        if self.active:
            self.command()
            if self.stage_goal is not None and self.node.nav_status.get('goal') is self.stage_goal:
                self.node.cancel_goal()
        self.active = False
        self.stage_goal = None
        self.surface_future = None
        self.recovery = None
        self.finish_request = None
        self.last_command = None
        self.report(phase, message)

    def clear(self):
        self.generation += 1
        self.target = self.anchor = None
        self.plan, self.candidates = None, []
        self.samples.clear()
        self.last_image = self.last_lidar = 0.
        self.last_seen = None
        self.previous_reference = None
        self.surface_note, self.grasp_surface_attempted = '', None
        self.surface_after = 0.
        self.point_blocked_since = None
        self.recovery, self.recovery_attempts = None, 0
        self.finish_request = None

    def check_reference(self, now):
        ros_now = self.node.get_clock().now().nanoseconds / 1e9
        reset = self.previous_ros_time is not None and ros_now < self.previous_ros_time-.1
        self.previous_ros_time = ros_now
        if self.target is not None or self.samples:
            rotation, position = self.transform(self.node.base, target='odom')
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            if self.previous_reference:
                then, previous, previous_yaw = self.previous_reference
                elapsed = max(.0, now-then)
                reset |= (self.np.linalg.norm(position[:2]-previous[:2]) > .15+.5*elapsed
                          or abs(math.atan2(math.sin(yaw-previous_yaw), math.cos(yaw-previous_yaw))) > .15+.8*elapsed)
            self.previous_reference = now, position, yaw
        if reset:
            self.cancel('오도메트리 / 시각 초기화 · LiDAR 목표를 다시 관측합니다.', 'error')
            self.clear()
            self.lidar.reset()

    def request(self, request):
        action = request['action']
        if action == 'select':
            self.cancel('목표 변경 · 접근 취소') if self.active else None
            self.label = target_label(request['label'])
            self.clear()
            self.report('searching', self.label+' · YOLO와 LiDAR 위치 관측 대기')
        elif action == 'find_and_approach':
            label = target_label(request['label'])
            if self.active and label == self.label:
                return
            if self.active:
                self.cancel('목표 변경 · 이전 접근 취소')
            if self.label != label:
                self.label = label
                self.clear()
            self.observing = True
            with self.state.condition:
                service = (self.state.values.get('service_control') or {}).get('detection') or {}
            if not (service.get('managed') or service.get('publishers') or service.get('busy')):
                self.state.sensor_manager.service_request('detection', 'start')
            self.start(float(request.get('edge_clearance', 0.)))
        elif action in ('start', 'stop'):
            if action == 'stop':
                self.observing = False
                self.cancel('검출 중지 · 접근 취소')
                self.clear()
            else:
                self.observing = True
            self.state.sensor_manager.service_request('detection', action)
        elif action == 'cancel':
            self.cancel()
        elif action == 'clear':
            self.cancel('저장된 물체 위치 지움')
            self.clear()
            self.report('searching' if self.label else 'idle', '저장 좌표 삭제 · 새 관측 대기')
        elif action == 'approach':
            if not self.active:
                self.start(float(request.get('edge_clearance', 0.)))

    def observe(self, now):
        if not self.label:
            return
        ros_now = self.node.get_clock().now().nanoseconds / 1e9
        if self.fusion_future is None:
            if now-self.last_fusion < .25:
                return
            detection = self.lidar.detection() or {}
            stamp = detection.get('stamp')
            if (not isinstance(stamp, (int, float)) or not math.isfinite(stamp)
                    or stamp == self.last_image or not -.1 <= ros_now-stamp <= 1.0):
                return
            if not detection.get('valid') or not detection.get('measured'):
                return
            matching = [o for o in detection.get('objects', []) if o.get('label') == self.label]
            if not matching:
                self.last_image = stamp
                return
            self.last_fusion = now
            self.fusion_future = (self.generation, stamp, self.fusion_pool.submit(
                self.lidar.locate, dict(detection, objects=matching)))
            return  # keep TF, action callbacks and odometry off the projection worker
        generation, stamp, future = self.fusion_future
        if not future.done():
            return
        self.fusion_future = None
        if generation != self.generation or not -.1 <= ros_now-stamp <= 1.0:
            return
        objects, reason = future.result()
        if objects is None:
            if self.target is None and self.phase in ('searching', 'confirming', 'waiting_target'):
                self.report('waiting_target' if self.active else 'searching', reason)
            return  # retry this frame when its TF/cloud arrives
        self.last_image = stamp
        candidates = [o for o in objects if o.get('position_valid')]
        if not candidates:
            if self.target is None and self.phase in ('searching', 'confirming', 'waiting_target'):
                self.report('waiting_target' if self.active else 'searching', 'YOLO 검출됨 · 물체 박스의 LiDAR 점 3개 이상 대기')
            return
        np = self.np
        reference = self.anchor if self.active else self.target
        if reference is not None:
            point = np.array(reference['point_odom'])
            candidates = [o for o in candidates if np.linalg.norm(np.array(o['point_odom'])-point) <= .25]
            if not candidates:
                return  # never switch to a different object of the same class
            item = min(candidates, key=lambda o: np.linalg.norm(np.array(o['point_odom'])-point))
        else:
            _, robot = self.transform(self.node.base, target='odom')
            item = min(candidates, key=lambda o: np.linalg.norm(np.array(o['point_odom'])[:2]-robot[:2]))
        if item['lidar_stamp'] == self.last_lidar:
            return
        self.last_lidar = item['lidar_stamp']
        if self.samples and (stamp-self.samples[-1]['capture_stamp'] > 1.5 or
                             np.linalg.norm(np.array(item['point_odom'])-np.array(self.samples[-1]['point_odom'])) > .12):
            self.samples.clear()
        self.samples.append(item)
        if len(self.samples) < 3:
            if self.target is None:
                self.report('waiting_target' if self.active else 'confirming', f'{self.label} · LiDAR 위치 확인 {len(self.samples)}/3')
            return
        points = np.array([o['point_odom'] for o in self.samples])
        if np.max(np.linalg.norm(points-np.median(points, axis=0), axis=1)) > .12:
            return
        self.target = dict(item)
        self.last_seen = now-max(0., ros_now-stamp)
        if not self.active and self.phase in ('idle', 'searching', 'confirming', 'ready'):
            self.report('ready', self.label+' · LiDAR 좌표 저장됨')

    def transform(self, source, stamp=None, target=None):
        return self.lidar.transform(source, stamp, target=target or self.node.fixed)

    def relative(self, target):
        rotation, offset = self.transform('odom', target=self.node.base)
        point = rotation @ self.np.array(target['point_odom']) + offset
        if not self.np.isfinite(point).all():
            raise RuntimeError('물체 상대 좌표 오류')
        return point

    def robot(self):
        rotation, position = self.transform(self.node.base, target='odom')
        return (float(position[0]), float(position[1]), math.atan2(rotation[1, 0], rotation[0, 0])), float(position[2])

    def start(self, clearance):
        if self.active:
            raise RuntimeError('이미 접근 중입니다. 취소 후 다시 시작하세요.')
        if not self.label:
            raise RuntimeError('찾을 물체 이름을 먼저 적용하세요.')
        if not math.isfinite(clearance) or not 0 <= clearance <= .1:
            raise ValueError('책상 추가 이격은 0~0.1m입니다.')
        now = time.monotonic()
        self.check_reference(now)
        if self.nav_busy or self.node.nav_status['state'] in ('active', 'sending', 'canceling'):
            raise RuntimeError('진행 중인 Nav2 이동을 먼저 취소하세요.')
        self.state.sensor_manager.require_collision_ready()
        if not self.node.nav_client:
            raise RuntimeError('Nav2 메시지 패키지가 없습니다.')
        self.edge_clearance = clearance
        self.anchor = None
        self.plan, self.candidates, self.stage_goal, self.surface_future = None, [], None, None
        self.surface_note, self.grasp_surface_attempted = '', None
        self.surface_after = 0.
        self.surface_requested, self.point_blocked_since = 0., None
        self.distance = None
        self.active, self.started = True, now
        self.last_pose = self.last_command = None
        self.progress_best, self.progress_at = math.inf, now
        self.arrived_since = None
        self.recovery, self.recovery_attempts = None, 0
        self.finish_request = None
        self.report('waiting_target', self.label+' · YOLO 3D 좌표가 확인되면 바로 이동')

    def connect_navigation(self):
        if not self.state.sensor_manager.navigation_available():
            with self.state.condition:
                service = (self.state.values.get('service_control') or {}).get('navigation') or {}
            if not service.get('busy'):
                self.state.sensor_manager.service_request('navigation', 'start')
        self.report('waiting_nav', 'YOLO 3D 좌표 저장됨 · Nav2 활성화 후 해당 좌표로 이동')

    def begin_object_navigation(self, now, force_navigation=False):
        if self.try_grasp_plan(now):
            return
        point = self.relative(self.anchor)
        if not force_navigation and math.hypot(point[0], point[1]) <= self.planner.max_reach:
            self.settle_until = now+1.2
            self.report('target_settling', '이미 3D 목표 가까이 있음 · 물체 방향 정렬 후 계속 접근')
            return
        rotation, offset = self.transform('odom')
        target = rotation @ self.np.array(self.anchor['point_odom'])+offset
        _, robot = self.transform(self.node.base)
        yaw = math.atan2(target[1]-robot[1], target[0]-robot[0])
        # Start without waiting for a table model. Once one arrives, replace
        # this tolerant object goal with a goal outside a reachable table face.
        self.stage_goal = self.node.send_goal(dict(x=float(target[0]), y=float(target[1]),
                                                   yaw=yaw, owner='detection_target'))
        self.report('navigating_target', self.label+' · 표시된 YOLO 3D 좌표로 Nav2 이동')

    def refresh_surface(self, now):
        if self.anchor is None or (self.plan is not None and self.phase != 'observing_table'):
            return
        if self.surface_future is not None:
            if not self.surface_future.done():
                return
            try:
                surface = self.surface_future.result()
                if surface is not None:
                    self.anchor['support_surface'] = surface
                    self.surface_note = ''
            except Exception as exc:
                self.surface_note = '책상 정밀 계획 대기 · '+str(exc)[:150]
            self.surface_future = None
        if now-self.surface_requested > 1.:
            self.surface_requested = now
            stamp = self.node.get_clock().now().nanoseconds/1e9
            self.surface_future = self.fusion_pool.submit(self.lidar.table, self.anchor['point_odom'], stamp)

    def try_grasp_plan(self, now):
        surface = self.anchor.get('support_surface')
        if not surface:
            return False
        self.command()
        # Wait for the old action's terminal result before changing its goal.
        # Publishing zero alone would leave Nav2 publishing competing commands.
        if self.stage_goal is not None:
            nav = self.node.nav_status
            if nav.get('goal') is not self.stage_goal:
                self.cancel('Nav2 목표가 다른 요청으로 변경됨', 'error')
                return True
            if nav['state'] in ('active', 'sending', 'canceling'):
                if nav['state'] == 'active':
                    self.node.cancel_goal()
                self.report('rerouting_table', '책상 접근 면 확인 · 기존 물체 이동을 종료하고 우회 경로로 전환')
                return True
            self.stage_goal = None
        if self.nav_busy:
            self.report('rerouting_table', '책상 우회 경로 전환 · Nav2 정지 확인 중')
            return True
        if surface['stamp'] == self.grasp_surface_attempted:
            self.report('planning_table', self.surface_note or '책상 도달 가능한 접근 면 재관측 중')
            return True
        self.grasp_surface_attempted = surface['stamp']
        positions = self.arm_feedback.positions()
        try:
            pose, z = self.robot()
            candidates = self.planner.plans(self.anchor['point_odom'], surface, pose, positions,
                                             self.edge_clearance, z, include_unobserved=True)
        except (ValueError, RuntimeError) as exc:
            self.surface_note = '책상 접근 면 계획 대기 · '+str(exc)[:150]
            self.report('planning_table', self.surface_note)
            return True
        self.surface_note = ''
        self.candidates = candidates
        self.begin_plan(self.candidates.pop(0), now)
        return True

    def observe_table_side(self, now):
        """An inferred far edge is a staging hint, never a precision boundary."""
        self.command()
        if now < self.settle_until:
            return
        surface = self.anchor.get('support_surface')
        if (surface and surface['stamp'] > self.surface_after
                and surface['stamp'] != self.grasp_surface_attempted):
            self.grasp_surface_attempted = surface['stamp']
            positions = self.arm_feedback.positions()
            try:
                pose, z = self.robot()
                candidates = self.planner.plans(self.anchor['point_odom'], surface, pose, positions,
                                                 self.edge_clearance, z)
                outward = self.np.asarray(self.plan['outward'])
                matching = [p for p in candidates if float(self.np.asarray(p['outward']) @ outward) >= .85]
                if matching:
                    self.surface_note = ''
                    self.candidates = [p for p in candidates if p is not matching[0]]
                    self.begin_plan(matching[0], now)
                    return
                self.surface_note = '돌아온 책상 면의 실제 가장자리 LiDAR 관측 대기'
            except (ValueError, RuntimeError) as exc:
                self.surface_note = str(exc)[:150]
        if now-self.phase_started > 15.:
            self.cancel('책상 우회 위치 도착 · 이쪽 가장자리 확인 불가 · '+self.surface_note, 'error')

    def advance_point(self, now, pose, odom):
        if self.phase == 'target_settling':
            if now < self.settle_until:
                return
            if self.nav_busy or abs(odom.get('linear', 1.)) > .02 or abs(odom.get('angular', 1.)) > .04:
                if now-self.settle_until > 5.:
                    self.cancel('Nav2 정지 확인 시간 초과', 'error')
                return
            self.progress_best, self.progress_at = math.inf, now
            self.report('point_aligning', '저장된 3D 물체 좌표 쪽으로 방향 정렬')
        if self.nav_busy:
            return self.cancel('다른 Nav2 이동 감지 · 3D 목표 접근 중지', 'error')
        if self.try_grasp_plan(now):
            return
        point = self.relative(self.anchor)
        distance = math.hypot(point[0], point[1])
        angle = math.atan2(point[1], point[0])
        if self.phase == 'point_aligning' and abs(angle) <= .035:
            self.progress_best, self.progress_at = math.inf, now
            self.report('point_approaching', '책상 인식과 무관하게 저장한 3D 물체 좌표로 접근')
        if distance <= .02:
            self.begin_final_alignment(now, pose, 'point', 'at_target',
                                       self.label+' · 3D 목표 위치·방향 도착 (파지 위치 별도)')
            return
        self.arrived_since = None
        if self.phase == 'point_aligning' or abs(angle) > .12:
            self.command(0., max(-.25, min(.25, 1.2*angle)))
        else:
            old_v, old_w = self.last_command or (0., 0.)
            linear = min(.06, max(.01, .3*(distance-.02)))
            angular = max(-.25, min(.25, 1.2*angle))
            self.command(max(old_v-.04, min(old_v+.015, linear)),
                         max(old_w-.06, min(old_w+.06, angular)))
        with self.state.condition:
            collision = self.state.values.get('collision_safety') or {}
            receipt = (self.state.channels.get('collision_safety') or {}).get('received')
        command, output = collision.get('command') or [], collision.get('output') or []
        physical_limit = (receipt is not None and now-receipt <= .3 and collision.get('ready')
                          and collision.get('state') == 'blocked'
                          and collision.get('reason') == 'predicted_clearance_violation'
                          and len(command) == 2 and command[0] > 0
                          and len(output) == 2 and abs(output[0]) < 1e-6)
        self.point_blocked_since = (self.point_blocked_since or now) if physical_limit else None
        if self.point_blocked_since is not None and now-self.point_blocked_since >= .6 and distance <= self.planner.max_reach:
            self.begin_final_alignment(now, pose, 'body', 'at_target',
                                       self.label+' · 몸체 한계까지 접근·물체 방향 정렬 완료 · 정밀 파지 위치 미확정')
            return
        metric = distance+.15*abs(angle)
        if metric < self.progress_best-.005:
            self.progress_best, self.progress_at = metric, now
        elif now-self.progress_at > 3.:
            detail = collision.get('blocking_link') or collision.get('reason') or '진행 없음'
            self.begin_recovery(now, '3D 목표 접근 진행 불가 · '+detail)

    def begin_final_alignment(self, now, pose, kind, phase, message):
        self.command()
        self.finish_request = dict(kind=kind, phase=phase, message=message, position=list(pose[:2]))
        self.arrived_since = None
        self.progress_best, self.progress_at = math.inf, now
        self.report('final_align', '목표 접근 위치 도착 · 물체 방향 정렬·실제 정지 확인 후 완료')

    def advance_heading(self, now, pose, odom, reapproach=False):
        if self.nav_busy:
            return self.cancel('다른 Nav2 이동 감지 · 최종 방향 정렬 중지', 'error')
        point = self.relative(self.anchor)
        distance, angle = math.hypot(point[0], point[1]), math.atan2(point[1], point[0])
        if not reapproach:
            request = self.finish_request
            if request['kind'] == 'dock':
                valid = math.hypot(pose[0]-self.plan['dock'][0], pose[1]-self.plan['dock'][1]) <= .008
            elif request['kind'] == 'point':
                valid = distance <= .02
            else:
                valid = (distance <= self.planner.max_reach and
                         math.hypot(pose[0]-request['position'][0], pose[1]-request['position'][1]) <= .02)
            if not valid:
                self.command()
                self.finish_request = None
                self.arrived_since = None
                self.progress_best, self.progress_at = math.inf, now
                self.report('aligning' if self.plan is not None else 'point_aligning', '방향 정렬 중 위치 변화 · 접근 위치 다시 보정')
                return
        if abs(angle) <= self.heading_tolerance:
            self.command()
            stopped = abs(odom.get('linear', 1.)) <= .01 and abs(odom.get('angular', 1.)) <= .02
            self.arrived_since = (self.arrived_since or now) if stopped else None
            if self.arrived_since is not None and now-self.arrived_since >= .6:
                if reapproach:
                    self.resume_navigation(now)
                else:
                    self.active = False
                    self.report(self.finish_request['phase'], self.finish_request['message'])
            return
        self.arrived_since = None
        self.command(0., math.copysign(min(.15, max(.025, abs(angle))), angle))
        if abs(angle) < self.progress_best-.005:
            self.progress_best, self.progress_at = abs(angle), now
        elif now-self.progress_at > 3.:
            self.begin_recovery(now, '물체 최종 방향 정렬 경로 막힘', align_first=True)

    def resume_navigation(self, now):
        self.finish_request = None
        self.last_pose = None
        self.arrived_since = None
        self.progress_best, self.progress_at = math.inf, now
        if self.plan is not None:
            self.begin_plan(self.plan, now, force_navigation=True)
        else:
            self.begin_object_navigation(now, force_navigation=True)

    def begin_recovery(self, now, reason, align_first=False):
        """Escape a blocked fine approach through the same 3D velocity guard."""
        self.command()
        if self.recovery_attempts >= 2:
            self.cancel('회전·후진 복구 후에도 접근 불가 · '+reason, 'error')
            return
        if self.nav_busy:
            self.cancel('다른 Nav2 이동 감지 · 접근 복구 중지', 'error')
            return
        self.recovery_attempts += 1
        point = self.relative(self.anchor)
        direction = 1. if math.atan2(point[1], point[0]) >= 0. else -1.
        self.recovery = dict(index=0, direction=direction, origin=None, wait_until=now+.4,
                             complete=False, reason=reason, align_first=align_first,
                             reverse_distance=.05 if align_first else .20)
        self.finish_request = None
        self.arrived_since = self.point_blocked_since = None
        turn = '왼쪽' if direction > 0 else '오른쪽'
        self.report('recovering', f'전진/정렬 경로 막힘 · {turn} 45° 제자리 회전 우선 · 복구 {self.recovery_attempts}/2')

    def advance_recovery(self, now, pose, odom):
        if self.nav_busy:
            return self.cancel('다른 Nav2 이동 감지 · 회전·후진 복구 중지', 'error')
        recovery = self.recovery
        if recovery['complete'] or recovery['origin'] is None:
            self.command()
            if (now < recovery['wait_until'] or abs(odom.get('linear', 1.)) > .02
                    or abs(odom.get('angular', 1.)) > .04):
                if now-recovery['wait_until'] > 5.:
                    self.cancel('접근 복구 동작 사이 정지 확인 시간 초과', 'error')
                return
            if recovery['complete']:
                self.recovery = None
                self.last_pose = None
                self.progress_best, self.progress_at = math.inf, now
                if recovery['align_first']:
                    self.arrived_since = None
                    self.report('realigning', '확보한 회전 공간에서 물체 쪽 선정렬 · 정렬 후 Nav2 재접근')
                else:
                    self.resume_navigation(now)
                return
            recovery.update(origin=pose, started=now, best=0., progress_at=now)
        origin = recovery['origin']
        reverse = recovery['index'] == 2
        direction = recovery['direction'] * (1. if recovery['index'] == 0 else -1.)
        if reverse:
            progress = -((pose[0]-origin[0])*math.cos(origin[2])+(pose[1]-origin[1])*math.sin(origin[2]))
            goal, tolerance = recovery['reverse_distance'], .005
        else:
            progress = direction*math.atan2(math.sin(pose[2]-origin[2]), math.cos(pose[2]-origin[2]))
            goal, tolerance = math.pi/4, .015
        stalled = now-recovery['progress_at'] > 1.2
        timed_out = now-recovery['started'] > 6.
        useful = progress >= (.05 if reverse else .15)
        if progress >= goal-tolerance or ((stalled or timed_out) and useful):
            self.command()
            recovery.update(complete=True, wait_until=now+.4)
            self.report('recovering', '복구 동작 완료 · 정지 후 Nav2 경로 재계획')
            return
        if progress > recovery['best']+(.003 if reverse else .01):
            recovery.update(best=progress, progress_at=now)
        if stalled or timed_out:
            self.command()
            recovery['index'] += 1
            if recovery['index'] >= 3:
                return self.cancel('좌우 제자리 회전과 후진 모두 진행 불가 · '+recovery['reason'], 'error')
            recovery.update(origin=None, wait_until=now+.4)
            if recovery['index'] == 2:
                distance = round(recovery['reverse_distance']*100)
                self.report('recovering', f'좌우 제자리 회전 모두 진행 불가 · 마지막으로 {distance}cm 후진 시도')
            else:
                direction = -recovery['direction']
                self.report('recovering', '제자리 회전 경로 막힘 · '+('왼쪽' if direction > 0 else '오른쪽')+' 45° 회전 시도')
            return
        remaining = max(0., goal-progress)
        if reverse:
            self.command(-min(.05, max(.015, .8*remaining)), 0.)
        else:
            self.command(0., direction*min(.20, max(.05, .8*remaining)))

    def begin_plan(self, plan, now, force_navigation=False):
        self.plan = plan
        self.distance = plan['target_distance']
        self.last_pose = None
        self.arrived_since = self.point_blocked_since = None
        self.progress_best, self.progress_at = math.inf, now
        pose, _ = self.robot()
        stage = plan['stage']
        # A nearby pose on the opposite side must still use Nav2 around the
        # desk. Distance and lateral alignment alone do not establish a lane.
        if not force_navigation and self.planner.in_docking_corridor(plan, pose):
            self.stage_goal = None
            self.settle_until = now+1.2
            self.report('settling', 'Nav2 준비 영역 안 · 정지 후 물체 방향 정렬')
            return
        rotation, offset = self.transform('odom')
        point = rotation @ self.np.array([stage[0], stage[1], 0.])+offset
        yaw = stage[2]+math.atan2(rotation[1, 0], rotation[0, 0])
        self.stage_goal = self.node.send_goal(dict(x=float(point[0]), y=float(point[1]), yaw=yaw, owner='detection'))
        route = '책상 반대편·옆면으로 우회' if not self.planner.on_approach_side(plan, pose) else '책상 접근 위치로 이동'
        boundary = ' · 도착 후 가장자리 확인' if not plan['boundary_verified'] else ''
        self.report('navigating', f'{self.label} · Nav2 {route} · {plan["side"]+1}번 면{boundary}')

    def stage_arrived(self, now):
        pose, _ = self.robot()
        stage = self.plan['stage']
        if (not self.planner.on_approach_side(self.plan, pose)
                or math.hypot(pose[0]-stage[0], pose[1]-stage[1]) > .20):
            if self.candidates:
                self.begin_plan(self.candidates.pop(0), now)
            else:
                self.cancel('Nav2가 선택한 책상 면의 준비 위치에 도달하지 못했습니다.', 'error')
            return
        self.stage_goal = None
        self.last_pose = None
        self.settle_until = now+1.2
        if self.plan['boundary_verified']:
            self.report('settling', 'Nav2 책상 접근 면 도착 · 물체 방향 정렬로 전환')
        else:
            self.surface_after = self.node.get_clock().now().nanoseconds/1e9
            self.surface_requested = 0.
            self.surface_note = '돌아온 책상 면에서 실제 가장자리 관측 중'
            self.report('observing_table', 'Nav2 책상 우회 위치 도착 · 이쪽 가장자리 LiDAR 확인 후 정밀 접근')

    def advance(self, now):
        if not self.active:
            return
        if now-self.started > 120.:
            return self.cancel('접근 제한 시간 120초 초과', 'error')
        if self.phase == 'waiting_target':
            if self.target is None or self.last_seen is None or now-self.last_seen > 30.:
                if now-self.phase_started > 30.:
                    return self.cancel('YOLO와 LiDAR의 3D 목표 관측 시간 초과', 'error')
                return
            self.anchor = dict(self.target)
            self.connect_navigation()
        self.refresh_surface(now)
        if self.phase == 'waiting_nav':
            if not self.node.nav_ready():
                if now-self.phase_started > 30.:
                    return self.cancel('Nav2 활성화 시간 초과 · NAV 실행 상태 확인', 'error')
                return
            self.begin_object_navigation(now)
            return
        if self.phase in ('rerouting_table', 'planning_table'):
            timeout = 10. if self.phase == 'rerouting_table' else 15.
            if now-self.phase_started > timeout:
                return self.cancel('책상 우회 전환 시간 초과 · '+(self.surface_note or 'Nav2 정지 확인 실패'), 'error')
            self.try_grasp_plan(now)
            return
        if self.phase == 'navigating_target':
            if self.try_grasp_plan(now):
                return
            nav = self.node.nav_status
            if nav.get('goal') is not self.stage_goal:
                return self.cancel('3D 물체 이동 목표가 다른 요청으로 변경됨', 'error')
            if nav['state'] == 'succeeded':
                self.stage_goal = None
                self.settle_until = now+1.2
                self.report('target_settling', '3D 목표 쪽 Nav2 경로 도착 · 저장 좌표로 계속 접근')
            elif nav['state'] not in ('active', 'sending'):
                point = self.relative(self.anchor)
                if nav['state'] == 'aborted' and math.hypot(point[0], point[1]) <= 1.0:
                    self.stage_goal = None
                    self.settle_until = now+1.2
                    self.report('target_settling', 'Nav2 2D 경로 끝에서 3D 보호를 받는 정밀 접근으로 전환')
                else:
                    return self.cancel('3D 물체 좌표로 Nav2 이동 실패 · '+nav.get('message', ''), 'error')
            return
        if self.phase == 'navigating':
            nav = self.node.nav_status
            if nav.get('goal') is not self.stage_goal:
                return self.cancel('Nav2 목표가 다른 요청으로 변경됨', 'error')
            if nav['state'] == 'succeeded':
                self.stage_arrived(now)
            elif nav['state'] not in ('active', 'sending'):
                if nav['state'] in ('aborted', 'rejected', 'error') and self.candidates:
                    self.begin_plan(self.candidates.pop(0), now)
                else:
                    return self.cancel('Nav2 책상 준비 위치 이동 실패 · '+nav.get('message', ''), 'error')
            return
        self.state.sensor_manager.require_collision_ready()
        pose, z = self.robot()
        position = self.np.array(pose[:2])
        if self.last_pose:
            then, previous, old_yaw = self.last_pose
            if (now-then > .5 or self.np.linalg.norm(position-previous) > .2
                    or abs(math.atan2(math.sin(pose[2]-old_yaw), math.cos(pose[2]-old_yaw))) > .35):
                self.cancel('오도메트리 연속성 손실 · 저장 좌표 재관측 필요', 'error')
                self.clear()
                return
        self.last_pose = now, position, pose[2]
        odom, meta = self.lidar.odometry()
        ros_now = self.node.get_clock().now().nanoseconds/1e9
        received, source_stamp = meta.get('received'), meta.get('stamp')
        if (received is None or now-received > .4 or source_stamp is None
                or not -.1 <= ros_now-source_stamp <= .4):
            return self.cancel('접근용 현재 오도메트리 수신 지연', 'error')
        if self.phase in ('target_settling', 'point_aligning', 'point_approaching'):
            self.advance_point(now, pose, odom)
            return
        if self.phase == 'recovering':
            self.advance_recovery(now, pose, odom)
            return
        if self.phase in ('final_align', 'realigning'):
            self.advance_heading(now, pose, odom, reapproach=self.phase == 'realigning')
            return
        if self.phase == 'observing_table':
            if self.nav_busy or abs(odom.get('linear', 1.)) > .02 or abs(odom.get('angular', 1.)) > .04:
                self.command()
                if now-self.settle_until > 5.:
                    self.cancel('책상 우회 후 Nav2 정지 확인 시간 초과', 'error')
                return
            self.observe_table_side(now)
            return
        if self.phase == 'settling':
            if now < self.settle_until:
                return
            if self.nav_busy or abs(odom.get('linear', 1.)) > .02 or abs(odom.get('angular', 1.)) > .04:
                if now-self.settle_until > 5.:
                    return self.cancel('Nav2 정지 확인 시간 초과', 'error')
                return
            self.progress_at, self.progress_best = now, math.inf
            self.report('aligning', '물체 쪽으로 방향 정렬 · 정렬 전 전진 없음')
        if self.nav_busy:
            return self.cancel('다른 Nav2 이동 감지 · 정밀 접근 중지', 'error')
        point = self.relative(self.anchor)
        object_angle = math.atan2(point[1], point[0])
        dock = self.plan['dock']
        heading = math.atan2(math.sin(dock[2]-pose[2]), math.cos(dock[2]-pose[2]))
        xy_error = math.hypot(dock[0]-pose[0], dock[1]-pose[1])
        if self.phase == 'aligning':
            if abs(heading) > .035:
                self.command(0., max(-.25, min(.25, 1.2*heading)))
                metric = xy_error+.15*abs(heading)
                if metric < self.progress_best-.005:
                    self.progress_best, self.progress_at = metric, now
                elif now-self.progress_at > 3.:
                    self.begin_recovery(now, '물체 방향 회전 불가 · 현재 3D 충돌 보호')
                return
            self.progress_best, self.progress_at = math.inf, now
            self.report('approaching', '책상 끝까지 몸체 기준 정밀 접근 · 저장 LiDAR 좌표 사용')
        if xy_error <= .008:
            self.begin_final_alignment(now, pose, 'dock', 'succeeded', self.label+' · 책상 최대 접근·물체 방향 정렬 완료')
            return
        else:
            self.arrived_since = None
            linear, angular, done = self.planner.command(pose, self.plan)
            if done:
                self.begin_recovery(now, '파지 위치의 옆 방향 오차 · 후진 후 Nav2 재접근 필요')
                return
            if abs(heading) > .12:
                linear = 0.
            old_v, old_w = self.last_command or (0., 0.)
            self.command(max(old_v-.04, min(old_v+.015, linear)), max(old_w-.06, min(old_w+.06, angular)))
        metric = xy_error+.15*abs(object_angle)
        if metric < self.progress_best-.005:
            self.progress_best, self.progress_at = metric, now
        elif now-self.progress_at > 3.:
            with self.state.condition:
                collision = self.state.values.get('collision_safety') or {}
            detail = collision.get('blocking_link') or collision.get('reason') or '진행 없음'
            self.begin_recovery(now, '책상 정밀 접근 진행 불가 · '+detail)

    def publish(self, now):
        target = self.anchor if self.anchor is not None and (self.active or self.finish_request is not None) else self.target or self.anchor
        info = None
        if target is not None:
            info = dict(label=self.label, point_odom=target['point_odom'], capture_stamp=target['capture_stamp'],
                        lidar_stamp=target['lidar_stamp'], lidar_count=target['lidar_count'],
                        lidar_oldest_stamp=target['lidar_oldest_stamp'],
                        source_topic=target['source_topic'], point_method=target['point_method'],
                        table=target.get('support_surface'),
                        capture_age=max(0., self.node.get_clock().now().nanoseconds/1e9-target['capture_stamp']),
                        visible=self.last_seen is not None and now-self.last_seen < 1., frame='odom')
            try:
                rotation, offset = self.transform('odom')
                info.update(point_map=(rotation @ self.np.array(target['point_odom'])+offset).tolist(), frame=self.node.fixed,
                            lidar_points_map=(self.np.array(target['lidar_points_odom']) @ rotation.T+offset).tolist())
                relative = self.relative(target)
                heading_error = float(math.atan2(relative[1], relative[0]))
                info.update(point_base=relative.tolist(), range=float(math.hypot(relative[0], relative[1])),
                            heading_error=heading_error, heading_aligned=abs(heading_error) <= self.heading_tolerance)
            except Exception:
                pass
            msg = self.PointStamped()
            from rclpy.time import Time
            msg.header.frame_id = 'odom'
            msg.header.stamp = Time(nanoseconds=round(target['capture_stamp']*1e9)).to_msg()
            msg.point.x, msg.point.y, msg.point.z = map(float, target['point_odom'])
            self.point_pub.publish(msg)
        plan = None
        if self.plan is not None:
            plan = dict(self.plan)
            plan.pop('cost', None)
            try:
                pose, _ = self.robot()
                plan['position_error'] = math.hypot(pose[0]-self.plan['dock'][0], pose[1]-self.plan['dock'][1])
                rotation, offset = self.transform('odom')
                map_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
                plan['frame'] = self.node.fixed
                for kind in ('dock', 'stage'):
                    xy = rotation @ self.np.array([*self.plan[kind][:2], 0.])+offset
                    plan[kind+'_map'] = [float(xy[0]), float(xy[1]), self.plan[kind][2]+map_yaw]
                corners = self.np.array([[*c, self.plan['surface']['z']] for c in self.plan['surface']['corners']])
                plan['table_corners_map'] = (corners @ rotation.T+offset).tolist()
            except Exception:
                pass
        value = dict(phase=self.phase, message=self.message, label=self.label, classes=list(CLASSES),
                     active=self.active, distance=self.distance, edge_clearance=self.edge_clearance,
                     target=info, plan=plan, nav_ready=self.node.nav_ready(), surface_note=self.surface_note,
                     inputs=self.lidar.diagnostics(), move_requested=self.active,
                     recovery_attempts=self.recovery_attempts,
                     heading_tolerance=self.heading_tolerance, orientation_required=True,
                     can_approach=not self.active and bool(self.label))
        self.state.put('detection_control', value)
        import json
        self.status_pub.publish(self.String(data=json.dumps(value, allow_nan=False)))

    def tick(self):
        now = time.monotonic()
        while self.requests:
            try:
                self.request(self.requests.popleft())
            except Exception as exc:
                self.cancel(str(exc)[:300], 'error')
        try:
            self.check_reference(now)
            if self.label and self.observing:
                self.observe(now)
            self.advance(now)
        except Exception as exc:
            if self.active:
                self.cancel(str(exc)[:300], 'error')
            elif self.target is None and self.label:
                self.report('searching', str(exc)[:200])
        self.publish(now)

    def close(self):
        if self.active:
            self.cancel('관제센터 종료 · 접근 취소')
        self.fusion_pool.shutdown(wait=True, cancel_futures=True)
        self.lidar.close()
