"""Real-robot YOLO/LiDAR approach owned by the independent control process.

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
        from geometry_msgs.msg import PointStamped, PolygonStamped, Point32, Twist
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from head_lidar import HeadLidar
        from grasp_approach import GraspPlanner
        self.np, self.PointStamped, self.Twist, self.String = np, PointStamped, Twist, String
        self.PolygonStamped, self.Point32 = PolygonStamped, Point32
        self.node, self.state = node, state
        self.lidar = HeadLidar(node, settings)
        self.planner = GraspPlanner(settings)
        from joint_contract import ModelJointFeedback
        self.arm_feedback = ModelJointFeedback(self.lidar.node, settings)
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
        self.surface_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='table-surface')
        self.edge_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='table-edge')
        self.edge_future = None
        self.edge_epoch = 0
        self.edge_stamp = self.edge_seen = None
        self.edge_requested = 0.
        self.edge_wait_since = None
        self.edge_frozen = False
        self.fusion_future = None
        self.generation = 0
        self.last_image = self.last_lidar = self.last_fusion = 0.
        self.last_seen = self.started = self.settle_until = self.arrived_since = None
        self.last_pose = self.last_command = None
        self.previous_reference = None
        self.previous_ros_time = None
        self.previous_odometry_epoch = None
        self.progress_best, self.progress_at = math.inf, 0.
        self.edge_clearance = float(settings.get('table_clearance', 0.))
        if not 0 <= self.edge_clearance <= .1:
            raise ValueError('detection.table_clearance는 0~0.1m입니다.')
        self.distance = None
        self.plan, self.candidates, self.surface_future = None, [], None
        self.handoff_from_corridor = True
        self.failed_approach_directions = []
        self.surface_requested = 0.
        self.surface_note = ''
        self.position_note = ''
        self.visual = None
        self.selection_started_ros = None
        self.visual_missing_since = self.visual_input_wait = None
        self.visual_stopped_since = None
        self.pending_table_candidates = None
        self.grasp_surface_attempted = None
        self.surface_after = 0.
        self.point_blocked_since = None
        self.table_blocked_since = None
        self.table_gap = None
        self.recovery, self.recovery_attempts = None, 0
        self.pending_reverse_failure = self.reverse_assessment = None
        self.final_reverse_used = False
        self.deadline = None
        self.finish_request = None
        self.heading_tolerance = math.radians(2)
        self.position_tolerance = .015
        self.nav_busy = False
        self.last_status_at = 0.
        self.cmd = node.create_publisher(Twist, '/cmd_vel', 1)
        self.status_pub = node.create_publisher(String, '/detection/closed_approach/status', 1)
        self.point_pub = node.create_publisher(PointStamped, '/detection/closed_approach/target', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.surface_pub=node.create_publisher(PolygonStamped,'/detection/dock/surface',qos)
        self.nav_subscription = node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                                        self.navigation_status, qos)

    def navigation_status(self, msg):
        self.nav_busy = any(s.status in (1, 2, 3) for s in msg.status_list)

    def report(self, phase, message):
        if phase != self.phase:
            if phase!='succeeded' and hasattr(self,'surface_pub'):
                self.surface_pub.publish(self.PolygonStamped())
            self.phase_started = time.monotonic()
        if (phase, message) != (self.phase, self.message):
            self.state.log('Detection · '+message, 'warn' if phase == 'error' else 'info', 'detection')
        self.phase, self.message = phase, message

    def command(self, linear=0., angular=0.):
        msg = self.Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd.publish(msg)
        self.last_command = (linear, angular)

    def require_protection_ready(self):
        guard, actuator, received = self.lidar.protection()
        now = time.monotonic()
        ros_now = self.node.get_clock().now().nanoseconds/1e9
        if (not guard.get('ready') or now-received.get('guard', 0.) > .5
                or not -.1 <= ros_now-guard.get('stamp', 0.) <= .5):
            raise RuntimeError('3D 보호 입력 대기 · '+str(guard.get('reason', 'status_missing')))
        if not actuator.get('ready') or now-received.get('actuator', 0.) > .5:
            raise RuntimeError('차체 보호 출력 대기 · '+str(actuator.get('reason', 'status_missing')))

    def cancel(self, message='접근 취소됨', phase='canceled'):
        retreat = getattr(self.node, 'front_retreat', None)
        if retreat is not None and retreat.detection_owned:
            retreat.cancel(message)
        if phase == 'error' and self.active:
            assessment = self.reverse_assessment or {}
            if assessment.get('state') in (None, 'waiting', 'checking'):
                self.assess_reverse('unavailable', '후진 판단 불가 · '+message)
        if self.active:
            self.command()
            if self.stage_goal is not None and self.node.nav_status.get('goal') is self.stage_goal:
                self.node.cancel_goal()
        self.active = False
        self.stage_goal = None
        self.surface_future = None
        self.pending_table_candidates = None
        self.recovery = None
        self.pending_reverse_failure = None
        self.finish_request = None
        self.last_command = None
        self.report(phase, message)

    def clear(self):
        self.generation += 1
        self.reset_edge_tracking()
        self.target = self.anchor = None
        self.plan, self.candidates = None, []
        self.failed_approach_directions = []
        self.samples.clear()
        self.last_image = self.last_lidar = 0.
        self.last_seen = None
        self.previous_reference = None
        self.surface_note, self.grasp_surface_attempted = '', None
        self.position_note, self.pending_table_candidates = '', None
        self.visual = None
        self.visual_missing_since = self.visual_input_wait = self.visual_stopped_since = None
        self.surface_after = 0.
        self.point_blocked_since = None
        self.table_blocked_since = self.table_gap = None
        self.recovery, self.recovery_attempts = None, 0
        self.pending_reverse_failure = self.reverse_assessment = None
        self.final_reverse_used = False
        self.finish_request = None

    def check_reference(self, now):
        ros_now = self.node.get_clock().now().nanoseconds / 1e9
        reason = None
        if self.previous_ros_time is not None and ros_now < self.previous_ros_time-.1:
            reason = f'ROS 시각 역행 {self.previous_ros_time-ros_now:.2f}s'
        self.previous_ros_time = ros_now
        _, meta = self.lidar.odometry()
        epoch = meta.get('epoch')
        if epoch is not None:
            if self.previous_odometry_epoch is not None and epoch != self.previous_odometry_epoch:
                reason = reason or 'EKF 오도메트리 메시지 점프'
            self.previous_odometry_epoch = epoch
        # The odom->base_link TF comes from the same EKF whose message continuity (by message stamps) is checked
        # above. Comparing the latest TF against this executor's tick time reported false resets whenever the
        # busy Python executor delayed TF while the base turned, so that tick-based check is removed.
        if reason:
            self.cancel(reason+' · LiDAR 목표를 다시 관측합니다.', 'error')
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
        if self.active and self.anchor is None and self.visual is None:
            return  # Select the current camera object before associating depth.
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
            if self.active and self.anchor is None:
                if stamp < self.selection_started_ros or stamp < self.visual['stamp']-.5:
                    return
                x, y, w, h = self.visual['bbox']
                def image_distance(o):
                    return math.hypot((o['x']+o['width']/2-x-w/2)/self.visual['width'],
                                      (o['y']+o['height']/2-y-h/2)/self.visual['height'])
                selected = min(matching, key=image_distance)
                if image_distance(selected) > .18:
                    return
                matching = [selected]  # Never let LiDAR range choose another box.
            elif not self.active:
                matching = [max(matching, key=lambda o: float(o.get('confidence', 0.)))]
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
            self.position_note = reason
            if self.target is None and self.phase in ('searching', 'confirming', 'waiting_target'):
                self.report('waiting_target' if self.active else 'searching', reason)
            return  # retry this frame when its TF/cloud arrives
        self.last_image = stamp
        candidates = [o for o in objects if o.get('position_valid')]
        if not candidates:
            count = max((o.get('lidar_count', 0) for o in objects), default=0)
            self.position_note = f'YOLO {self.label} 검출됨 · LiDAR 군집 {count}/3점 · 위치 관측 중'
            if self.target is None and self.phase in ('searching', 'confirming', 'waiting_target'):
                self.report('waiting_target' if self.active else 'searching', self.position_note)
            return
        np = self.np
        if self.active and self.anchor is None and self.visual is not None:
            x, y, w, h = self.visual['bbox']
            candidates = [o for o in candidates if math.hypot(
                (o['x']+o['width']/2-x-w/2)/self.visual['width'],
                (o['y']+o['height']/2-y-h/2)/self.visual['height']) <= .18]
            if not candidates:
                return
        reference = self.anchor if self.active else None
        if reference is not None:
            point = np.array(reference['point_odom'])
            candidates = [o for o in candidates if np.linalg.norm(np.array(o['point_odom'])-point) <= .25]
            if not candidates:
                return  # never switch to a different object of the same class
            item = min(candidates, key=lambda o: np.linalg.norm(np.array(o['point_odom'])-point))
        else:
            item = max(candidates, key=lambda o: float(o.get('confidence', 0.)))
        if item['lidar_stamp'] == self.last_lidar:
            return
        self.last_lidar = item['lidar_stamp']
        if self.samples and (stamp-self.samples[-1]['capture_stamp'] > 1.5 or
                             np.linalg.norm(np.array(item['point_odom'])-np.array(self.samples[-1]['point_odom'])) > .12):
            self.samples.clear()
        self.samples.append(item)
        # A compact cluster already contains three distinct measured returns.
        # A second three-frame gate delayed valid targets until request timeout.
        self.target = dict(item)
        if self.active:
            self.target['selection_generation'] = self.generation
        self.position_note = f'YOLO {self.label} · LiDAR {item["lidar_count"]}점 좌표 확보'
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
        retreat = getattr(self.node, 'front_retreat', None)
        if (self.nav_busy or self.node.nav_status['state'] in ('active', 'sending', 'canceling')
                or (retreat is not None and retreat.active)):
            raise RuntimeError('진행 중인 Nav2 이동을 먼저 취소하세요.')
        # A new request must select a current Head YOLO object. Discard the
        # idle/previous request's point and reject its pending fusion result.
        self.clear()
        self.selection_started_ros = self.node.get_clock().now().nanoseconds/1e9
        self.edge_clearance = clearance
        self.anchor = None
        self.plan, self.candidates, self.stage_goal, self.surface_future = None, [], None, None
        self.surface_note, self.grasp_surface_attempted = '', None
        self.pending_table_candidates = None
        self.visual = None
        self.visual_missing_since = self.visual_input_wait = self.visual_stopped_since = None
        self.surface_after = 0.
        self.surface_requested, self.point_blocked_since = 0., None
        self.distance = None
        self.active, self.started = True, now
        self.failed_approach_directions = []
        self.deadline = now+120.
        self.last_pose = self.last_command = None
        self.progress_best, self.progress_at = math.inf, now
        self.arrived_since = None
        self.table_blocked_since = self.table_gap = None
        self.recovery, self.recovery_attempts = None, 0
        self.pending_reverse_failure = self.reverse_assessment = None
        self.final_reverse_used = False
        self.finish_request = None
        self.report('waiting_target', self.label+' · YOLO 발견 즉시 영상 방향으로 접근 · LiDAR 좌표는 이동 중 확보')

    def advance_visual(self, now):
        """Follow a calibrated image bearing before metric target acquisition."""
        if self.nav_busy or self.node.nav_status['state'] in ('active', 'sending', 'canceling'):
            return self.cancel('다른 Nav2 이동 감지 · YOLO 접근 중지', 'error')
        try:
            self.require_protection_ready()
            odom, meta = self.lidar.odometry()
            ros_now = self.node.get_clock().now().nanoseconds/1e9
            if (now-meta.get('received', -math.inf) > 1.
                    or not -.1 <= ros_now-meta.get('stamp', 0.) <= 1.):
                raise RuntimeError('YOLO 접근용 현재 오도메트리 수신 대기')
        except RuntimeError as exc:
            self.command()
            self.visual_input_wait = self.visual_input_wait or now
            self.report('visual_waiting', 'YOLO 접근 일시 정지 · '+str(exc))
            if now-self.visual_input_wait > 3.:
                self.cancel('YOLO 접근 입력 복구 시간 초과 · '+str(exc), 'error')
            return
        self.visual_input_wait = None
        current = self.lidar.visual_target(self.label, self.visual, since=self.selection_started_ros)
        if current is not None:
            self.visual, self.visual_missing_since = current, None
        measured_target = (self.visual is not None and self.target is not None
                           and self.target.get('projection_verified') is True
                           and self.target.get('selection_generation') == self.generation
                           and self.target['capture_stamp'] >= self.selection_started_ros
                           and self.last_seen is not None and now-self.last_seen <= 30.)
        if measured_target:
            # Stop visual commands before transferring to the metric owner.
            self.command()
            stopped = abs(odom.get('linear', 1.)) <= .01 and abs(odom.get('angular', 1.)) <= .02
            self.visual_stopped_since = (self.visual_stopped_since or now) if stopped else None
            self.report('visual_handoff', '3D LiDAR 좌표 저장됨 · 정지 후 저장 좌표 기반 접근으로 전환')
            if self.visual_stopped_since is not None and now-self.visual_stopped_since >= .2:
                self.anchor = dict(self.target)
                self.last_pose = None
                point = self.relative(self.anchor)
                if math.hypot(point[0], point[1]) <= .9:
                    self.begin_object_navigation(now)
                else:
                    self.connect_navigation()
            return
        if current is None:
            self.command()
            self.visual_missing_since = self.visual_missing_since or now
            self.report('visual_waiting', self.label+' · YOLO 영상 재관측 중 · LiDAR 좌표 없이 시야 이탈 시 정지')
            allowance = 3. if self.visual is not None else 30.
            if now-self.visual_missing_since > allowance:
                self.cancel('YOLO 물체 영상 관측 시간 초과', 'error')
            return
        self.visual, self.visual_missing_since = current, None
        angle = current['bearing']
        angular = max(-.18, min(.18, 1.2*angle))
        # The image supplies direction, never a fabricated target distance.
        # Translation starts only after bearing alignment and stays protected
        # by the existing full 3D body guard and actuator watchdog.
        linear = .04*max(0., 1.-abs(angle)/.16) if abs(angle) < .16 else 0.
        old_v, old_w = self.last_command or (0., 0.)
        self.command(min(linear, max(0., old_v+.01)), max(old_w-.04, min(old_w+.04, angular)))
        if linear > 0.:
            self.report('visual_approaching', self.label+' · YOLO 영상 방향으로 저속 접근 · LiDAR 좌표 수집 중')
        else:
            self.report('visual_aligning', self.label+' · YOLO 영상의 물체 쪽으로 방향 정렬')

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
        if not force_navigation and math.hypot(point[0], point[1]) <= .9:
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
            self.surface_future = self.surface_pool.submit(self.lidar.table, self.anchor['point_odom'], stamp)

    def reset_edge_tracking(self):
        self.edge_epoch += 1
        if self.edge_future is not None:
            self.edge_future[1].cancel()
        self.edge_future = None
        self.edge_stamp = self.edge_seen = None
        self.edge_requested = 0.
        self.edge_wait_since = None
        self.edge_frozen = False

    def refresh_edge(self, now):
        """Track the selected face until the MID-360 blind range, then use odom."""
        ros_now = self.node.get_clock().now().nanoseconds/1e9
        if self.edge_future is not None and self.edge_future[1].done():
            epoch, future, stamp = self.edge_future
            self.edge_future = None
            if epoch == self.edge_epoch and -.05 <= ros_now-stamp <= .5:
                updated, state = future.result()
                if updated is not None:
                    self.plan = updated
                    self.anchor['support_surface'] = updated['surface']
                    self.edge_seen = stamp
                    self.surface_note = '실시간 책상 가장자리 추적'
                elif state == 'blind':
                    self.edge_frozen = self.edge_seen is not None
                    self.surface_note = 'LiDAR 사각 구간 · 마지막 실측 경계와 오도메트리 사용'
                else:
                    self.surface_note = '현재 책상 가장자리 재관측 · '+state
        if self.edge_frozen:
            return True
        if self.edge_future is None and now-self.edge_requested >= .1:
            self.edge_requested = now
            observation = self.lidar.edge_observation()
            if observation is not None and observation[0] != self.edge_stamp:
                stamp, points, pose, z = observation
                self.edge_stamp = stamp
                future = self.edge_pool.submit(self.planner.track_edge, self.plan,
                    self.anchor['point_odom'], pose, z, points, stamp, self.arm_feedback.positions())
                self.edge_future = self.edge_epoch, future, stamp
        fresh = self.edge_seen is not None and -.05 <= ros_now-self.edge_seen <= .5
        if fresh:
            self.edge_wait_since = None
            return True
        self.command()
        self.edge_wait_since = self.edge_wait_since or now
        if now-self.edge_wait_since > 3.:
            self.begin_recovery(now, '정밀 도킹용 현재 책상 가장자리 관측 불가')
        return False

    def try_grasp_plan(self, now):
        surface = self.anchor.get('support_surface')
        if self.pending_table_candidates is None:
            if not surface or surface['stamp'] == self.grasp_surface_attempted:
                return False
            self.grasp_surface_attempted = surface['stamp']
            try:
                pose, z = self.robot()
                candidates = self.planner.plans(self.anchor['point_odom'], surface, pose,
                    self.arm_feedback.positions(), self.edge_clearance, z, include_unobserved=True)
                candidates = [p for p in candidates if self.approach_side_available(p)]
                if not candidates:
                    raise RuntimeError('실패한 책상 면을 제외한 도달 가능한 접근 면이 없습니다.')
            except (ValueError, RuntimeError) as exc:
                self.surface_note = '책상 계획 재관측 · 3D 목표 접근 계속 · '+str(exc)[:150]
                return False
            self.pending_table_candidates = candidates
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
        self.surface_note = ''
        self.candidates, self.pending_table_candidates = self.pending_table_candidates, None
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
            self.fail_motion(now, '책상 우회 위치 도착 · 이쪽 가장자리 확인 불가 · '+self.surface_note)

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
                                       self.label+' · 3D 목표 앞 도착 · 정밀 파지 위치 미확정')
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
        collision, _, received = self.lidar.protection()
        receipt = received.get('guard')
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
        self.report('final_align', '목표 접근 위치 도착 · 도킹 방향·실제 정지 확인 후 완료')

    def heading_only(self):
        return False

    def grasp_gap_satisfied(self):
        return (self.plan is not None and self.plan.get('boundary_verified')
                and self.table_gap is not None and 0 <= self.table_gap <= self.planner.gap_limit)

    def advance_heading(self, now, pose, odom, reapproach=False):
        if self.nav_busy:
            return self.cancel('다른 Nav2 이동 감지 · 최종 방향 정렬 중지', 'error')
        point = self.relative(self.anchor)
        distance, angle = math.hypot(point[0], point[1]), math.atan2(point[1], point[0])
        if self.plan is not None:
            angle = math.atan2(math.sin(self.plan['dock'][2]-pose[2]), math.cos(self.plan['dock'][2]-pose[2]))
        if not reapproach:
            request = self.finish_request
            if request['kind'] in ('dock', 'table_body'):
                along, lateral, _ = self.planner.errors(pose, self.plan)
                valid = abs(along) <= .015 and abs(lateral) <= .02 and self.grasp_gap_satisfied()
            elif request['kind'] == 'point':
                valid = distance <= .02
            else:
                valid = (distance <= self.planner.max_reach and
                         math.hypot(pose[0]-request['position'][0], pose[1]-request['position'][1]) <= .04)
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
                    if self.plan is not None and self.finish_request['phase']=='succeeded':
                        surface=self.plan['surface']
                        msg=self.PolygonStamped()
                        msg.header.frame_id='odom'; msg.header.stamp=self.node.get_clock().now().to_msg()
                        msg.polygon.points=[self.Point32(x=float(x),y=float(y),z=float(surface['z']))
                                            for x,y in surface['corners']]
                        self.surface_pub.publish(msg)
                    self.active = False
                    self.report(self.finish_request['phase'], self.finish_request['message'])
            return
        self.arrived_since = None
        self.command(0., math.copysign(min(.15, max(.025, abs(angle))), angle))
        if abs(angle) < self.progress_best-.005:
            self.progress_best, self.progress_at = abs(angle), now
        elif now-self.progress_at > 3.:
            self.begin_recovery(now, '최종 도킹 방향 정렬 경로 막힘', align_first=True)

    def resume_navigation(self, now):
        self.finish_request = None
        self.last_pose = None
        self.arrived_since = None
        self.progress_best, self.progress_at = math.inf, now
        if self.anchor is None:
            self.visual_stopped_since = None
            self.report('visual_approaching', '후진 정지 완료 · YOLO 영상 방향으로 접근 재개')
            return
        if self.plan is not None:
            self.begin_plan(self.plan, now, force_navigation=True)
        else:
            self.begin_object_navigation(now, force_navigation=True)

    def approach_side_available(self, plan):
        outward = self.np.asarray(plan['outward'])
        return all(float(outward @ self.np.asarray(direction)) < .85
                   for direction in self.failed_approach_directions)

    def reroute_after_retreat(self, now):
        """Prefer another reachable face instead of replaying a failed dock lane."""
        self.finish_request = None
        if self.anchor is None:
            if self.target is not None and self.last_seen is not None and now-self.last_seen <= 30.:
                self.anchor = dict(self.target)
                self.connect_navigation()
            else:
                self.cancel('YOLO 접근 경로의 실제 3D 차단 지속 · 우회용 물체 좌표 재관측 필요', 'error')
            return
        if self.plan is not None:
            if self.approach_side_available(self.plan):
                self.failed_approach_directions.append(list(self.plan['outward']))
            candidates = [p for p in self.candidates if self.approach_side_available(p)]
            if not candidates:
                pose, z = self.robot()
                try:
                    candidates = self.planner.plans(self.anchor['point_odom'], self.plan['surface'], pose,
                        self.arm_feedback.positions(), self.edge_clearance, z, include_unobserved=True)
                    candidates = [p for p in candidates if self.approach_side_available(p)]
                except (ValueError, RuntimeError) as exc:
                    self.surface_note = '다른 접근 면 계획 · '+str(exc)[:150]
            self.candidates = candidates
            if candidates:
                self.begin_plan(self.candidates.pop(0), now, force_navigation=True)
                self.report('navigating', self.label+' · 후진 효과 없음 · 다른 책상 면으로 Nav2 우회')
                return
            self.state.log('다른 파지 가능 면 없음 · 실패한 진입 구간을 제외하고 현재 면의 다른 경로 탐색',
                           'info', 'detection')
        self.resume_navigation(now)

    def assess_reverse(self, status, reason, **details):
        value = dict(state=status, reason=reason, source='detection_3d_guard', **details)
        if value != self.reverse_assessment:
            self.reverse_assessment = value
            self.state.log('후진 판단 · '+reason, 'warn' if status in ('blocked', 'unavailable', 'no_progress') else 'info', 'detection')

    def fail_motion(self, now, reason):
        """A motion failure must pass one final reverse assessment before ending."""
        self.command()
        if self.heading_only():
            return self.cancel(reason+' · 현재 위치 유지·추가 위치 보정 없음', 'error')
        if self.final_reverse_used:
            outcome = (self.reverse_assessment or {}).get('reason', '최종 후진 판단 완료')
            self.cancel(reason+' · '+outcome, 'error')
            return
        if self.anchor is None:
            self.assess_reverse('unavailable', '후진 판단 불가 · 확정된 이동 목표 없음')
            self.cancel(reason, 'error')
            return
        self.final_reverse_used = True
        self.pending_reverse_failure = dict(reason=reason, began=now,
                                             align_first=(self.phase in ('final_align', 'realigning') or
                                                          bool(self.recovery and self.recovery.get('align_first'))))
        self.recovery = None
        self.last_pose = None
        self.assess_reverse('waiting', '실패 확정 전 후진 판단 예약 · 기존 Nav2 정지 확인 중')
        self.report('checking_reverse', '실패 전 최종 후진 판단 · '+reason)

    def advance_reverse_check(self, now, pose, odom):
        pending = self.pending_reverse_failure
        self.command()
        nav = self.node.nav_status
        if nav['state'] in ('active', 'sending', 'canceling'):
            if self.stage_goal is None or nav.get('goal') is not self.stage_goal:
                self.assess_reverse('unavailable', '후진 판단 불가 · 다른 Nav2 목표 실행 중')
                return self.cancel(pending['reason'], 'error')
            if nav['state'] == 'active':
                self.node.cancel_goal()
            pending.pop('stopped_since', None)
            return
        if self.nav_busy or abs(odom.get('linear', 1.)) > .02 or abs(odom.get('angular', 1.)) > .04:
            pending.pop('stopped_since', None)
            return
        pending.setdefault('stopped_since', now)
        if now-pending['stopped_since'] < .4:
            return
        self.stage_goal = None
        self.pending_reverse_failure = None
        # Allow the bounded final assessment and one short retry even when the
        # original 120-second deadline was what triggered the failure.
        self.deadline = max(self.deadline, now+40.)
        self.begin_recovery(now, pending['reason'], align_first=pending['align_first'], final_reverse=True)

    def begin_recovery(self, now, reason, align_first=False, final_reverse=False):
        """Back straight out before replanning; never spin beside the table."""
        self.command()
        if not final_reverse and self.recovery_attempts >= 2:
            self.fail_motion(now, '후진 복구 후에도 접근 불가 · '+reason)
            return
        if self.nav_busy:
            self.cancel('다른 Nav2 이동 감지 · 접근 복구 중지', 'error')
            return
        if not final_reverse:
            self.recovery_attempts += 1
        self.recovery = dict(origin=None, wait_until=now+.4, complete=False, reason=reason,
                             final_reverse=final_reverse, reverse_distance=.15)
        self.reset_edge_tracking()
        self.finish_request = None
        self.arrived_since = self.point_blocked_since = None
        if final_reverse:
            self.assess_reverse('checking', 'Nav2 정지·입력 준비 확인 · 3D 보호를 통한 최종 후진 판단 시작',
                                requested_distance=self.recovery['reverse_distance'])
            self.report('final_reverse', f'실패 전 최종 후진 판단 · 최대 {round(self.recovery["reverse_distance"]*100)}cm 후진 후 재계획')
        else:
            self.report('recovering', f'전진/정렬 경로 막힘 · 직선 15cm 후진 후 재계획 · 복구 {self.recovery_attempts}/2')

    def advance_recovery(self, now, pose, odom):
        if self.nav_busy:
            return self.cancel('다른 Nav2 이동 감지 · 후진 복구 중지', 'error')
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
                self.resume_navigation(now)
                return
            recovery.update(origin=pose, started=now, best=0., progress_at=now)
            self.assess_reverse('checking', '3D 보호가 후진 궤적을 확인하며 저속 후진 판단 중',
                                requested_distance=recovery['reverse_distance'])
        origin = recovery['origin']
        progress = -((pose[0]-origin[0])*math.cos(origin[2])+(pose[1]-origin[1])*math.sin(origin[2]))
        lateral = -(pose[0]-origin[0])*math.sin(origin[2])+(pose[1]-origin[1])*math.cos(origin[2])
        yaw_error = math.atan2(math.sin(pose[2]-origin[2]), math.cos(pose[2]-origin[2]))
        if abs(lateral) > .03 or abs(yaw_error) > .10:
            return self.cancel('직선 후진 중 자세 이탈 · 복구 중지', 'error')
        goal, tolerance = recovery['reverse_distance'], .005
        stalled = now-recovery['progress_at'] > 1.2
        timed_out = now-recovery['started'] > 8.
        useful = progress >= .05
        if progress >= goal-tolerance or ((stalled or timed_out) and useful):
            self.command()
            self.assess_reverse('moved', f'후진 가능 · 실제 {max(0., progress)*100:.1f}cm 이동 · 재계획',
                                distance=max(0., progress))
            recovery.update(complete=True, wait_until=now+.4)
            self.report('final_reverse' if recovery['final_reverse'] else 'recovering', '복구 동작 완료 · 정지 후 Nav2 경로 재계획')
            return
        if progress > recovery['best']+.003:
            recovery.update(best=progress, progress_at=now)
        if stalled or timed_out:
            self.command()
            collision, _, _ = self.lidar.protection()
            blocked = collision.get('state') == 'blocked'
            detail = collision.get('blocking_link') or collision.get('reason') or '오도메트리 진행 없음'
            self.assess_reverse('blocked' if blocked else 'no_progress',
                                ('후진 불가 · ' if blocked else '후진 진행 확인 실패 · ')+detail,
                                distance=max(0., progress), blocking_point=collision.get('blocking_source_point_base'))
            return self.cancel('직선 후진 복구 진행 불가 · '+recovery['reason'], 'error')
        remaining = max(0., goal-progress)
        self.command(-min(.05, max(.015, .8*remaining)), 0.)

    def begin_plan(self, plan, now, force_navigation=False):
        self.reset_edge_tracking()
        self.plan = plan
        self.table_blocked_since = self.table_gap = None
        # Recovery can deliberately return to the outer staging pose. Do not
        # immediately cancel that retreat just because it starts in this lane.
        self.handoff_from_corridor = not force_navigation
        self.distance = plan['target_distance']
        self.last_pose = None
        self.arrived_since = self.point_blocked_since = None
        self.progress_best, self.progress_at = math.inf, now
        pose, _ = self.robot()
        stage = plan['stage']
        # A nearby pose on the opposite side must still use Nav2 around the
        # desk. Distance and lateral alignment alone do not establish a lane.
        if not force_navigation and (self.planner.in_docking_corridor(plan, pose)
                                     or self.planner.at_staging_pose(plan, pose)):
            self.stage_arrived(now)
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
        if not (self.planner.at_staging_pose(self.plan, pose)
                or self.planner.in_docking_corridor(self.plan, pose)):
            if self.candidates:
                self.begin_plan(self.candidates.pop(0), now)
            else:
                self.fail_motion(now, 'Nav2가 선택한 책상 면의 준비 위치에 도달하지 못했습니다.')
            return
        self.stage_goal = None
        self.last_pose = None
        self.arrived_since = None
        self.settle_until = now+1.2
        if self.plan['boundary_verified']:
            self.report('settling', 'Nav2 책상 접근 면 도착 · 물체 방향 정렬로 전환')
        else:
            self.surface_after = self.node.get_clock().now().nanoseconds/1e9
            self.surface_requested = 0.
            self.surface_note = '돌아온 책상 면에서 실제 가장자리 관측 중'
            self.report('observing_table', 'Nav2 책상 우회 위치 도착 · 이쪽 가장자리 LiDAR 확인 후 정밀 접근')

    def advance_docking_handoff(self, now, pose, odom):
        """Release Nav2's 2D envelope only after its action and motion stop."""
        self.command()
        nav = self.node.nav_status
        if nav.get('goal') is not self.stage_goal:
            return self.cancel('Nav2 목표가 다른 요청으로 변경됨 · 도킹 전환 중지', 'error')
        if now-self.phase_started > 10.:
            return self.fail_motion(now, '3D 도킹 전환 전 Nav2·차체 정지 확인 시간 초과')
        if nav['state'] in ('active', 'sending', 'canceling'):
            if nav['state'] == 'active':
                self.node.cancel_goal()
            self.arrived_since = None
            return
        if self.nav_busy or abs(odom.get('linear', 1.)) > .02 or abs(odom.get('angular', 1.)) > .04:
            self.arrived_since = None
            return
        self.arrived_since = self.arrived_since or now
        if now-self.arrived_since >= .4:
            # Recheck the selected face and entry area at the stopped pose.
            # An inferred far edge must still be observed before docking.
            if self.plan is None:
                self.stage_goal = None
                self.settle_until = now
                self.last_pose = None
                self.report('target_settling', 'Nav2 정지 완료 · 저장 LiDAR 좌표로 근접 접근')
            else:
                self.stage_arrived(now)

    def advance(self, now):
        if not self.active:
            return
        if self.phase == 'checking_reverse':
            if now-self.pending_reverse_failure['began'] > 10.:
                self.assess_reverse('unavailable', '후진 판단 불가 · Nav2/차체 정지 확인 시간 초과')
                return self.cancel(self.pending_reverse_failure['reason'], 'error')
        elif now > self.deadline:
            return self.fail_motion(now, '접근 제한 시간 초과')
        if self.phase in ('waiting_target', 'visual_waiting', 'visual_aligning',
                          'visual_approaching', 'visual_handoff'):
            self.advance_visual(now)
            return
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
                return self.fail_motion(now, '책상 우회 전환 시간 초과 · '+(self.surface_note or 'Nav2 정지 확인 실패'))
            self.try_grasp_plan(now)
            return
        if self.phase == 'navigating_target':
            if self.try_grasp_plan(now):
                return
            nav = self.node.nav_status
            if nav.get('goal') is not self.stage_goal:
                return self.cancel('3D 물체 이동 목표가 다른 요청으로 변경됨', 'error')
            point = self.relative(self.anchor)
            if (nav['state'] in ('active', 'sending') and math.hypot(point[0], point[1]) <= .9
                    and abs(math.atan2(point[1], point[0])) <= .6):
                self.command()
                self.last_pose = None
                self.arrived_since = None
                if nav['state'] == 'active':
                    self.node.cancel_goal()
                self.report('point_handoff', '물체 근접 구간 진입 · Nav2 정지 후 LiDAR 좌표로 계속 접근')
                return
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
                    return self.fail_motion(now, '3D 물체 좌표로 Nav2 이동 실패 · '+nav.get('message', ''))
            return
        if self.phase == 'navigating':
            nav = self.node.nav_status
            if nav.get('goal') is not self.stage_goal:
                return self.cancel('Nav2 목표가 다른 요청으로 변경됨', 'error')
            pose, _ = self.robot()
            if (self.planner.at_staging_pose(self.plan, pose)
                    or (self.handoff_from_corridor and self.planner.in_docking_corridor(self.plan, pose))):
                # Evaluate this while navigating, including during Nav2's
                # costmap recovery, rather than waiting for its final result.
                self.command()
                self.last_pose = None
                self.arrived_since = None
                if nav['state'] == 'active':
                    self.node.cancel_goal()
                self.report('docking_handoff', '책상 접근 구간 진입 · Nav2 2D 판정 종료 후 저속 3D 도킹으로 전환')
                return
            if nav['state'] == 'succeeded':
                self.stage_arrived(now)
            elif nav['state'] not in ('active', 'sending'):
                if nav['state'] in ('aborted', 'rejected', 'error') and self.candidates:
                    self.begin_plan(self.candidates.pop(0), now)
                else:
                    return self.fail_motion(now, 'Nav2 책상 준비 위치 이동 실패 · '+nav.get('message', ''))
            return
        self.require_protection_ready()
        pose, z = self.robot()
        position = self.np.array(pose[:2])
        if self.last_pose:
            then, previous, old_yaw = self.last_pose
            # Web callbacks can be delayed while the independent odometry and
            # guard remain live. Only actual source pose jumps invalidate the
            # saved target; a callback interval alone is not a frame reset.
            # Only a frame reset (a jump no executor delay can explain) invalidates the target; EKF message
            # continuity is checked by stamps in check_reference.
            if (self.np.linalg.norm(position-previous) > .5
                    or abs(math.atan2(math.sin(pose[2]-old_yaw), math.cos(pose[2]-old_yaw))) > .8):
                self.cancel('오도메트리 연속성 손실 · 저장 좌표 재관측 필요', 'error')
                self.clear()
                return
        self.last_pose = now, position, pose[2]
        odom, meta = self.lidar.odometry()
        ros_now = self.node.get_clock().now().nanoseconds/1e9
        received, source_stamp = meta.get('received'), meta.get('stamp')
        if (received is None or now-received > 1. or source_stamp is None
                or not -.1 <= ros_now-source_stamp <= 1.):
            return self.cancel('접근용 현재 오도메트리 수신 지연', 'error')
        if (self.plan is not None and self.plan.get('boundary_verified')
                and self.phase in ('approaching', 'final_align')):
            if not self.refresh_edge(now):
                return
        if self.plan is not None and self.plan.get('boundary_verified'):
            self.table_gap = self.planner.table_gap(pose, self.plan, self.arm_feedback.positions(), z)
        else:
            self.table_gap = None
        if self.phase in ('docking_handoff', 'point_handoff'):
            self.advance_docking_handoff(now, pose, odom)
            return
        if self.phase in ('target_settling', 'point_aligning', 'point_approaching'):
            self.advance_point(now, pose, odom)
            return
        if self.phase == 'checking_reverse':
            self.advance_reverse_check(now, pose, odom)
            return
        if self.phase in ('recovering', 'final_reverse'):
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
            self.report('aligning', '책상 법선 방향 정렬 · 정렬 전 전진 없음')
        if self.nav_busy:
            return self.cancel('다른 Nav2 이동 감지 · 정밀 접근 중지', 'error')
        point = self.relative(self.anchor)
        dock = self.plan['dock']
        heading = math.atan2(math.sin(dock[2]-pose[2]), math.cos(dock[2]-pose[2]))
        xy_error = math.hypot(dock[0]-pose[0], dock[1]-pose[1])
        if self.planner.docked(pose, self.plan, self.table_gap):
            self.begin_final_alignment(now, pose, 'dock', 'succeeded',
                                       self.label+' · 실측 책상 도킹 자세·간격 충족·정지 완료')
            return
        if self.phase == 'aligning':
            if abs(heading) > .035:
                self.command(0., max(-.25, min(.25, 1.2*heading)))
                metric = xy_error+.15*abs(heading)
                if metric < self.progress_best-.005:
                    self.progress_best, self.progress_at = metric, now
                elif now-self.progress_at > 3.:
                    self.begin_recovery(now, '물체 방향 회전 불가 · 현재 3D 충돌 보호')
                return
            # A staging arrival may face sideways. Align at the stopped
            # staging pose before requiring a front-facing edge observation.
            if not self.refresh_edge(now):
                return
            self.progress_best, self.progress_at = math.inf, now
            self.report('approaching', '실시간 책상 가장자리로 정밀 접근')
        self.arrived_since = None
        linear, angular, done = self.planner.command(pose, self.plan)
        if done:
            if self.planner.docked(pose, self.plan, self.table_gap):
                self.begin_final_alignment(now, pose, 'dock', 'succeeded', self.label+' · 책상 접근 완료 (도킹 선 끝)')
            else:
                self.begin_recovery(now, '파지 위치의 옆 방향 오차 · 후진 후 Nav2 재접근 필요')
            return
        old_v, old_w = self.last_command or (0., 0.)
        self.command(max(old_v-.04, min(old_v+.015, linear)), max(old_w-.06, min(old_w+.06, angular)))
        metric = xy_error+.15*abs(heading)
        if metric < self.progress_best-.0015:
            self.progress_best, self.progress_at = metric, now
        collision, _, received = self.lidar.protection()
        receipt = received.get('guard')
        command, output = collision.get('command') or [], collision.get('output') or []
        physical_limit = (receipt is not None and now-receipt <= .3 and collision.get('ready')
                          and collision.get('state') == 'blocked'
                          and collision.get('reason') == 'predicted_clearance_violation'
                          and len(command) == 2 and command[0] > 0
                          and len(output) == 2 and abs(output[0]) < 1e-6)
        self.table_blocked_since = (self.table_blocked_since or now) if physical_limit else None
        if (self.table_blocked_since is not None and now-self.table_blocked_since >= .6
                and self.planner.docked(pose, self.plan, self.table_gap)
                and math.hypot(point[0], point[1]) <= self.planner.max_reach):
            self.begin_final_alignment(now, pose, 'table_body', 'succeeded',
                                       self.label+' · 책상 상대 간격 충족·물체 방향 정렬 완료')
            return
        if now-self.progress_at > 3.:
            detail = collision.get('blocking_link') or collision.get('reason') or '진행 없음'
            self.begin_recovery(now, '책상 정밀 접근 진행 불가 · '+detail)

    def publish(self, now):
        detection = self.lidar.detection() or {}
        ros_now = self.node.get_clock().now().nanoseconds/1e9
        stamp = detection.get('stamp')
        yolo_visible = bool(detection.get('valid') and detection.get('measured')
            and isinstance(stamp, (int, float)) and -.1 <= ros_now-stamp <= .5
            and any(o.get('label') == self.label for o in detection.get('objects', [])))
        remembered = self.anchor is not None and (self.active or self.finish_request is not None)
        target = self.anchor if remembered else self.target if yolo_visible else None
        info = None
        if target is not None:
            info = dict(label=self.label, point_odom=target['point_odom'], capture_stamp=target['capture_stamp'],
                        lidar_stamp=target['lidar_stamp'], lidar_count=target['lidar_count'],
                        lidar_oldest_stamp=target['lidar_oldest_stamp'],
                        source_topic=target['source_topic'], point_method=target['point_method'],
                        table=target.get('support_surface'),
                        capture_age=max(0., self.node.get_clock().now().nanoseconds/1e9-target['capture_stamp']),
                        visible=yolo_visible, frame='odom', selection_source='head_yolo',
                        projection_verified=target.get('projection_verified', False))
            try:
                rotation, offset = self.transform('odom')
                info.update(point_map=(rotation @ self.np.array(target['point_odom'])+offset).tolist(), frame=self.node.fixed,
                            lidar_points_map=(self.np.array(target['lidar_points_odom']) @ rotation.T+offset).tolist())
                relative = self.relative(target)
                heading_error = float(math.atan2(relative[1], relative[0]))
                if self.plan is not None:
                    pose, _ = self.robot()
                    heading_error = self.planner.errors(pose, self.plan)[2]
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
                     position_note=self.position_note,
                     visual_target=self.visual,
                     inputs=self.lidar.diagnostics(), move_requested=self.active,
                     recovery_attempts=self.recovery_attempts,
                     reverse_assessment=self.reverse_assessment, final_reverse_used=self.final_reverse_used,
                     heading_tolerance=self.heading_tolerance, orientation_required=True,
                     position_tolerance=self.position_tolerance, heading_only=self.heading_only(),
                     table_gap=self.table_gap, table_gap_limit=self.planner.gap_limit,
                     grasp_distance_satisfied=self.grasp_gap_satisfied(),
                     edge_tracking=dict(state='frozen' if self.edge_frozen else 'tracked' if self.edge_seen is not None else 'waiting',
                                        stamp=self.edge_seen, note=self.surface_note),
                     control_hz=20., controller_process=__import__('os').getpid(),
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
            retreat = getattr(self.node, 'front_retreat', None)
            if retreat is None or not retreat.active:
                self.advance(now)
        except Exception as exc:
            if self.active:
                self.cancel(str(exc)[:300], 'error')
            elif self.target is None and self.label:
                self.report('searching', str(exc)[:200])
        if now-self.last_status_at >= .2:
            self.last_status_at = now
            self.publish(now)

    def close(self):
        if self.active:
            self.cancel('관제센터 종료 · 접근 취소')
        self.fusion_pool.shutdown(wait=True, cancel_futures=True)
        self.surface_pool.shutdown(wait=True, cancel_futures=True)
        self.edge_pool.shutdown(wait=True, cancel_futures=True)
        self.lidar.close()
