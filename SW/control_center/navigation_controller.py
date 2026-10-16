"""Navigation actions and 20 Hz precision control in the dedicated process."""
from collections import deque
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from detection_approach import DetectionApproach
from front_retreat import FrontRetreat


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class NavigationController(Node):
    def __init__(self, state, config):
        super().__init__('inha_docking_control')
        self.state = state
        self.fixed = config.get('fixed_frame', 'map')
        self.base = config.get('base_frame', 'base_link')
        self.nav_requests = deque(maxlen=32)
        self.nav_goal = None
        self.nav_cancel_target = None
        self.nav_status = dict(state='idle', message='목표 없음', goal=None)
        self.nav_started_ros = 0.
        from rclpy.action import ActionClient
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.nav_lifecycle = {name: self.create_client(GetState, '/'+name+'/get_state')
                              for name in ('bt_navigator', 'controller_server', 'planner_server')}
        self.nav_lifecycle_states, self.nav_lifecycle_pending = {}, {}
        self.detection_approach = DetectionApproach(self, state, config.get('detection', {}))
        self.front_retreat = FrontRetreat(self, state, config.get('front_retreat', {}))
        self.create_subscription(GoalStatusArray, '/backup/_action/status', self.backup_status,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Log, '/rosout', self.rosout, 10)
        self.create_timer(1., self.navigation_health)
        self.create_timer(.1, self.navigation)
        self.create_timer(.05, self.detection_approach.tick)

    def rosout(self, message):
        if message.level >= 30 and message.name != self.get_name():
            self.state.log(message.msg, 'error' if message.level >= 40 else 'warn', message.name)

    def stop_control(self):
        self.nav_requests.clear()
        self.detection_approach.requests.clear()
        self.front_retreat.cancel('관제센터 종료 · 짧은 후진 중지')
        self.detection_approach.cancel('관제센터 종료 · 접근 취소')
        if self.nav_goal is not None or self.nav_status['state'] == 'sending':
            self.cancel_goal()
        self.detection_approach.command()
        # Give queued action acceptance/cancellation callbacks a bounded chance
        # to cancel a goal that was still in flight when the bridge closed.
        import rclpy
        until = time.monotonic()+.4
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.02)
            self.detection_approach.command()
        self.detection_approach.close()

    def navigation(self):
        while self.nav_requests:
            request = self.nav_requests.popleft()
            try:
                if request['action'] == 'goal':
                    self.front_retreat.cancel('수동 Nav2 목표로 전환 · 짧은 후진 중지')
                    if self.detection_approach.active:
                        self.detection_approach.cancel('수동 Nav2 목표로 전환 · 접근 취소')
                    self.send_goal(request)
                elif request['action'] == 'cancel':
                    self.front_retreat.cancel('이동 취소 · 짧은 후진 중지')
                    if self.detection_approach.active:
                        self.detection_approach.cancel('이동 취소 · 물체 접근 중지')
                    self.cancel_goal()
            except Exception as exc:
                self.nav_update(state='error', message=str(exc)[:300])
        self.front_retreat.tick()
        self.state.put('navigation', dict(self.nav_status, available=self.nav_ready(),
            server_available=bool(self.nav_client and self.nav_client.server_is_ready()),
            lifecycle={k: v[1] for k, v in self.nav_lifecycle_states.items()},
            detection_active=self.detection_approach.active,
            detection_phase=self.detection_approach.phase))

    def navigation_health(self):
        now = time.monotonic()
        for name, client in self.nav_lifecycle.items():
            pending = self.nav_lifecycle_pending.get(name)
            if pending:
                began, future = pending
                if not future.done() and now-began < 2.5:
                    continue
                if not future.done():
                    future.cancel()
                self.nav_lifecycle_pending.pop(name, None)
            if not client.service_is_ready():
                self.nav_lifecycle_states.pop(name, None)
                continue
            future = client.call_async(GetState.Request())
            self.nav_lifecycle_pending[name] = now, future
            future.add_done_callback(lambda f, key=name: self.navigation_state(f, key))

    def navigation_state(self, future, name):
        if future.cancelled():
            return
        try:
            current = future.result().current_state
            self.nav_lifecycle_states[name] = int(current.id), current.label, time.monotonic()
        except Exception:
            self.nav_lifecycle_states.pop(name, None)

    def nav_ready(self):
        now = time.monotonic()
        return bool(self.nav_client and self.nav_client.server_is_ready() and all(
            name in self.nav_lifecycle_states and self.nav_lifecycle_states[name][0] == 3
            and now-self.nav_lifecycle_states[name][2] < 3. for name in self.nav_lifecycle))

    def backup_status(self, message):
        if self.nav_status.get('goal') is None or self.front_retreat.active:
            return
        statuses = [s for s in message.status_list
                    if stamp_seconds(s.goal_info.stamp) >= self.nav_started_ros]
        if not statuses:
            return
        status = max(statuses, key=lambda s: stamp_seconds(s.goal_info.stamp))
        descriptions = {1: ('checking', 'Nav2 후진 가능 여부 확인 시작'),
                        2: ('checking', 'Nav2 후진 판단·실행 중'),
                        3: ('canceling', 'Nav2 후진 중지 요청'),
                        4: ('moved', 'Nav2 후진 완료 · 재계획'),
                        5: ('canceled', 'Nav2 후진 취소'),
                        6: ('blocked', '후진 미완료 · 3D 보호·입력·실제 이동 원인 확인')}
        if status.status not in descriptions:
            return
        kind, reason = descriptions[status.status]
        value = dict(state=kind, reason=reason, source='nav2_backup',
                     goal_id=bytes(status.goal_info.goal_id.uuid).hex())
        if value != self.nav_status.get('reverse_assessment'):
            self.nav_status['reverse_assessment'] = value
            if self.nav_status.get('state') == 'aborted' and self.nav_status.get('failure_detail'):
                self.nav_status['message'] = self.nav_status['failure_detail']+' · '+reason
            self.state.log(reason, 'warn' if kind == 'blocked' else 'info', 'navigation')

    def nav_update(self, **values):
        self.nav_status.update(values)
        if values.get('state') in ('succeeded', 'aborted', 'canceled', 'rejected', 'error'):
            self.state.log('Nav2 · ' + self.nav_status.get('message', ''), 'info' if values['state'] == 'succeeded' else 'warn', 'navigation')

    def pose_msg(self, request, message_type):
        message = message_type()
        message.header.frame_id = self.fixed
        message.header.stamp = self.get_clock().now().to_msg()
        target = message.pose.pose if hasattr(message.pose, 'pose') else message.pose
        target.position.x, target.position.y = float(request['x']), float(request['y'])
        target.orientation.z, target.orientation.w = math.sin(request['yaw'] / 2), math.cos(request['yaw'] / 2)
        return message

    def send_goal(self, request):
        if self.state.sensor_manager:
            if request.get('owner') in ('detection', 'detection_target'):
                self.detection_approach.require_protection_ready()
            else:
                self.state.sensor_manager.require_collision_ready()
        if not self.nav_client:
            raise RuntimeError('nav2_msgs가 없습니다. Nav2를 설치하세요.')
        if not self.nav_client.server_is_ready():
            raise RuntimeError('Nav2 /navigate_to_pose 서버 대기 중입니다.')
        if not self.nav_ready():
            raise RuntimeError('Nav2 lifecycle 활성화 대기 중입니다.')
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_msg(request, type(goal.pose))
        owner = request.get('owner', 'navigation')
        if owner in ('detection', 'detection_target'):
            tree = 'navigate-to-detection-target.xml' if owner == 'detection_target' else 'navigate-to-detection.xml'
            goal.behavior_tree = str(Path(__file__).resolve().with_name(tree))
        target = dict(x=request['x'], y=request['y'], yaw=request['yaw'], frame=self.fixed, owner=owner)
        self.nav_started_ros = self.get_clock().now().nanoseconds/1e9
        self.nav_update(state='sending', message='목표 전송 중', goal=target, distance=None, eta=None,
                        recoveries=0, started=time.time(), reverse_assessment=None, failure_detail=None)
        self.state.log(f'Nav2 목표 전송 · {self.fixed} ({target["x"]:.2f}, {target["y"]:.2f}) '
                  f'{math.degrees(target["yaw"]):.0f}°', source='navigation')
        future = self.nav_client.send_goal_async(goal, feedback_callback=lambda f, t=target: self.nav_feedback(f, t))
        future.add_done_callback(lambda f, t=target: self.nav_accepted(f, t))
        return target

    def nav_accepted(self, future, target):
        try:
            handle = future.result()
        except Exception as exc:
            if self.nav_status.get('goal') is target:
                self.nav_update(state='error', message=str(exc)[:300])
            return
        if self.nav_status.get('goal') is not target:
            if handle.accepted:
                handle.cancel_goal_async()  # superseded while it was being sent
            return
        if not handle.accepted:
            self.nav_update(state='rejected', message='Nav2가 목표를 거부했습니다.')
            return
        self.nav_goal = handle
        handle.get_result_async().add_done_callback(lambda f, h=handle, t=target: self.nav_result(f, h, t))
        if target is self.nav_cancel_target:
            handle.cancel_goal_async()
            self.nav_update(state='canceling', message='취소 요청')
        else:
            self.nav_update(state='active', message='이동 중')

    def nav_feedback(self, message, target):
        if self.nav_status.get('goal') is not target:
            return
        feedback = message.feedback
        self.nav_status.update(distance=float(feedback.distance_remaining),
                               eta=feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9,
                               recoveries=int(feedback.number_of_recoveries))

    def nav_result(self, future, handle, target):
        if handle is not self.nav_goal or self.nav_status.get('goal') is not target:
            return
        self.nav_goal = None
        try:
            code = future.result().status
        except Exception as exc:
            self.nav_update(state='error', message=str(exc)[:300])
            return
        names = {4: ('succeeded', '목표 도착'), 5: ('canceled', '이동 취소됨'), 6: ('aborted', '이동 실패 · 경로 또는 진행 불가')}
        result, message = names.get(code, ('aborted', f'종료 상태 {code}'))
        if code == 6:
            with self.state.condition:
                details = next((log['message'] for log in reversed(self.state.logs)
                                if time.time()-log['time'] < 20 and any(text in log['message'] for text in
                                ('Trajectory Hits Obstacle', 'No valid trajectories', 'Failed to make progress', 'Failed to create plan', 'Collision Ahead'))), '')
            if details:
                message += ' · '+details[:180]
            self.nav_status['failure_detail'] = message
            assessment = self.nav_status.get('reverse_assessment')
            if assessment and assessment['state'] not in ('checking', 'canceling'):
                message += ' · '+assessment['reason']
        self.nav_update(state=result, message=message)

    def cancel_goal(self):
        self.nav_cancel_target = self.nav_status.get('goal')
        if self.nav_goal is not None:
            self.nav_goal.cancel_goal_async()
            self.nav_update(state='canceling', message='취소 요청')
        else:  # a goal still being sent is cancelled as soon as it is accepted
            self.nav_update(state='idle', message='목표 취소', goal=None)

