"""Bounded ROS manipulation monitor/action client; never starts hardware on import."""
from collections import deque
import json
import math
import threading
import time


STAGES = {
    'WAIT_FOR_TARGET': '물체 좌표 대기', 'WAIT_FOR_APPROACH': '접근 완료 대기',
    'LOCK_BASE': '베이스 잠금', 'MOVE_TO_OBSERVATION': '손목 관측 위치 이동',
    'OBSERVE': '손목 재검출·세그멘테이션', 'PLAN_GRASP': '파지 계획',
    'GRASP': '파지', 'PLACE': '내려놓기', 'RETURN': '팔 복귀',
    'DONE': '작업 종료',
    'WRIST_REDETECT': '손목 재검출·세그멘테이션', 'COMPUTE_GRASP': '파지 위치 계산',
    'MOVE_TO_PREGRASP': '파지 전 위치 이동', 'DESCEND': '물체로 하강',
    'CLOSE_GRIPPER': '그리퍼 닫기', 'VERIFY_GRASP': '파지 확인', 'LIFT': '들어 올리기',
    'MOVE_TO_PLACE': '내려놓을 위치 이동', 'RELEASE': '물체 놓기',
    'RETREAT': '팔 후퇴', 'RETURN_ARM': '팔 복귀', 'COMPLETE': '작업 종료',
    'OBSERVE_PLATE': '손목 SAM 접시 관측·놓을 공간 확인',
}
IDLE_REASONS = {'', 'WAITING_FOR_NEW_APPROACH', 'APPROACH_RUNNING',
                'WAITING_FOR_MEASURED_TARGET', 'WAITING_FOR_MEASURED_STOP',
                'HANDED_OFF_EXTERNAL_BASE_LEASE_REQUIRED'}


class ManipulationMonitor:
    """Pure state/event model. Shared event list stays in RAM, no extra log file."""
    def __init__(self, state, clock=time.monotonic):
        self.state, self.clock = state, clock
        self.values, self.received, self.signatures = {}, {}, {}
        self.started = clock()
        self.available = False
        self.active = False
        self.pending = False
        self.task_id = ''
        self.reason = '매니퓰레이션 ROS 인터페이스 연결 대기'

    def event(self, key, signature, message, level='info', source='manipulation'):
        if self.signatures.get(key) != signature:
            self.signatures[key] = signature
            self.state.log(message, level, source)

    def receive(self, key, value):
        if not isinstance(value, dict):
            raise ValueError('상태 메시지는 JSON 객체여야 합니다.')
        self.values[key], self.received[key] = value, self.clock()
        if key == 'status':
            stage, outcome, reason = (str(value.get(k, '')) for k in ('stage', 'outcome', 'reason'))
            task = str(value.get('task_id', ''))
            if not task:
                raise ValueError('task_id 누락')
            self.task_id = task
            self.active = outcome not in ('SUCCESS', 'FAILED', 'CANCELED', 'REJECTED')
            level = 'error' if outcome in ('FAILED', 'REJECTED') else 'warn' if outcome == 'CANCELED' else 'info'
            message = f'매니퓰 · {STAGES.get(stage, stage)} · {outcome} · {reason or "진행 중"} · 작업 {task[:12]}'
            self.event('task', (task, stage, outcome, reason), message, level)
        elif key == 'readiness':
            errors = value.get('errors', [])
            if not isinstance(errors, list):
                raise ValueError('readiness.errors 형식 오류')
            errors = [str(x) for x in errors][:20]
            self.event('readiness', tuple(errors),
                '매니퓰 실행 준비 · '+(' / '.join(errors) if errors else '정상'),
                'warn' if errors else 'info', 'manipulation-readiness')
        elif key in ('adapter', 'wrist', 'base'):
            reason = str(value.get('reason') or '')
            problem = reason not in IDLE_REASONS
            # Explicit errors only: a wrist waiting for a request is normal.
            if key == 'base' and value.get('ready') is True:
                problem = False
            self.event(key, reason if problem else '',
                f'매니퓰 {key} · {reason or "연결 정상"}',
                'warn' if problem else 'info', 'manipulation-'+key)

    def fresh(self, key, age=3.):
        return key in self.received and 0 <= self.clock()-self.received[key] <= age

    def tick(self):
        if self.clock()-self.started >= 5:
            for key in ('readiness', 'adapter', 'base', 'wrist'):
                ok = self.fresh(key)
                self.event('connection-'+key, ok,
                    f'매니퓰 {key} · '+('상태 수신 복구' if ok else '상태 수신 없음 / 연결 지연'),
                    'info' if ok else 'warn', 'manipulation-'+key)
        result = dict(self.values)
        result.update(available=self.available, active=self.active, pending=self.pending,
            task_id=self.task_id, reason=self.reason,
            ages={k: max(0., self.clock()-v) for k, v in self.received.items()})
        self.state.put('manipulation', result)


class ManipulationControl:
    """ROS callbacks and action ownership stay on the bridge executor thread."""
    def __init__(self, node, state):
        self.node, self.state = node, state
        self.monitor = ManipulationMonitor(state)
        self.requests = deque()
        self.request_lock = threading.Lock()
        self.goal = None
        self.subscriptions = []
        self.targets, self.approaches = {}, {}
        self.consumed = deque(maxlen=128)
        self.client = None
        self.seen_sessions = deque(maxlen=32)
        self.armed = False
        self.armed_at = 0.
        self.armed_task = None
        self.last_error = ''
        self.cancel_pending = False
        self.started_ros = self.now()
        self.armed_at_ros = 0.
        state.manipulation_interlock = self.interlock
        state.manipulation_request = self.request
        try:
            from rclpy.action import ActionClient
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from std_msgs.msg import String
            from robocup_real_manipulation_msgs.msg import TaskStatus, Target, ApproachResult
            from robocup_real_manipulation_msgs.action import Manipulate
            self.Action = Manipulate
            self.client = ActionClient(node, Manipulate, '/manipulation/run')
            latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
            for topic in ('/manipulation/status', '/manipulation/result'):
                self.subscriptions.append(node.create_subscription(TaskStatus, topic, self.status, latched_qos))
            for key, topic in [('readiness', '/manipulation/readiness'),
                               ('adapter', '/manipulation/control_center/status'),
                               ('base', '/manipulation/base_gate/status'),
                               ('wrist', '/manipulation/wrist/status')]:
                self.subscriptions.append(node.create_subscription(String, topic,
                    lambda msg, key=key: self.json_status(key, msg), 1))
            # Match the adapter's retained, original-stamped terminal pair.
            handoff_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
            for cls, topic, callback in [(Target, '/manipulation/target', self.target),
                    (ApproachResult, '/manipulation/approach_result', self.approach)]:
                self.subscriptions.append(node.create_subscription(cls, topic, callback, handoff_qos))
            self.monitor.available = True
            self.monitor.reason = '실제 매니퓰레이션 상태 수신 대기'
        except ImportError as exc:
            self.monitor.reason = '매니퓰 메시지 패키지/overlay 없음: '+str(exc)
            state.log(self.monitor.reason, 'warn', 'manipulation')
        node.create_timer(.5, self.tick)

    def now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def interlock(self):
        m = self.monitor
        return bool(m.active or m.pending or self.goal or
                    m.values.get('base', {}).get('locked') or
                    m.values.get('readiness', {}).get('locked_task'))

    def status(self, msg):
        # Transient historical results are not replayed as a new running task.
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not -.1 <= self.now()-stamp <= 3.:
            return
        value = {k: getattr(msg, k) for k in ('task_id', 'class_id', 'stage', 'outcome',
            'reason', 'holding', 'place_success', 'return_success', 'stop_confirmed')}
        p = msg.precise_center
        value['precise_center'] = dict(frame=p.header.frame_id, xyz=[p.point.x, p.point.y, p.point.z])
        self.monitor.receive('status', value)

    def json_status(self, key, msg):
        try:
            if len(msg.data) > 16384:
                raise ValueError('상태 메시지 크기 초과')
            self.monitor.receive(key, json.loads(msg.data))
            if key == 'adapter':
                value = self.monitor.values[key]
                task = value.get('task_id')
                started = value.get('approach_started_at')
                new_session = (type(started) in (int, float) and math.isfinite(started)
                               and self.started_ros-.1 <= started <= self.now()+.1)
                observed = value.get('reason') == 'APPROACH_RUNNING' or (
                    value.get('reason') == 'HANDED_OFF_EXTERNAL_BASE_LEASE_REQUIRED' and new_session)
                if task and observed and task not in self.seen_sessions:
                    self.seen_sessions.append(task)
                    if self.armed and (value.get('reason') == 'APPROACH_RUNNING' or
                                       new_session and started >= self.armed_at_ros):
                        self.armed_task = task
                if self.armed and value.get('reason') not in IDLE_REASONS:
                    self.armed = False
                    self.state.log('매니퓰 자동 인계 중단 · '+str(value.get('reason')), 'warn', 'manipulation')
        except (ValueError, TypeError) as exc:
            self.monitor.event('invalid-'+key, str(exc), f'매니퓰 {key} 상태 해석 실패: {exc}',
                               'warn', 'manipulation-'+key)

    def target(self, msg):
        if msg.task_id not in self.targets:
            if len(self.targets) >= 8:
                self.targets.pop(next(iter(self.targets)))
            self.targets[msg.task_id] = (msg, time.monotonic())

    def approach(self, msg):
        if msg.task_id not in self.approaches:
            if len(self.approaches) >= 8:
                self.approaches.pop(next(iter(self.approaches)))
            self.approaches[msg.task_id] = (msg, time.monotonic())

    def candidate(self):
        for task in reversed(self.approaches):
            msg, stamp = self.approaches[task]
            source_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            target = self.targets.get(task, (None,))[0]
            target_stamp = (target.header.stamp.sec+target.header.stamp.nanosec*1e-9) if target else 0.
            valid_target = (target is not None and target.header.frame_id == 'odom'
                and 0 <= target.class_id < 6 and -.1 <= self.now()-target_stamp <= 120.
                and all(math.isfinite(v) for v in (target.coarse_position.x,
                    target.coarse_position.y, target.coarse_position.z)))
            if (valid_target and task in self.seen_sessions and task not in self.consumed and msg.outcome == 'SUCCESS'
                    and msg.base_stopped and 0 <= time.monotonic()-stamp <= 30.
                    and -.1 <= self.now()-source_stamp <= 30.
                    and self.monitor.fresh('adapter')
                    and self.monitor.values['adapter'].get('task_id') == task
                    and self.monitor.values['adapter'].get('reason') == 'HANDED_OFF_EXTERNAL_BASE_LEASE_REQUIRED'
                    and self.monitor.values['adapter'].get('measured_stop_confirmed') is True
                    and self.monitor.fresh('base') and self.monitor.values['base'].get('stopped') is True):
                return task
        return None

    def handoff_state(self):
        adapter = self.monitor.values.get('adapter', {})
        task = adapter.get('task_id')
        ready = self.candidate()
        reason = adapter.get('reason', 'WAITING_FOR_NEW_APPROACH')
        messages = {'WAITING_FOR_NEW_APPROACH': '새 물체 도킹 완료 대기',
            'WAITING_FOR_MEASURED_TARGET': '접근 중 · 실측 물체 좌표 수집 대기',
            'APPROACH_RUNNING': '물체 접근·도킹 진행 중',
            'WAITING_FOR_MEASURED_STOP': '도킹 완료 · 실측 정지 확인 중',
            'HANDED_OFF_EXTERNAL_BASE_LEASE_REQUIRED': '도킹 인계 확인 중'}
        message = '도킹 완료·실측 정지 인계 완료' if ready else messages.get(reason, '인계 대기 · '+str(reason))
        if reason == 'HANDED_OFF_EXTERNAL_BASE_LEASE_REQUIRED' and not ready:
            message = '인계 유효기간·연결·현재 정지 상태 확인 필요'
        if not self.monitor.fresh('adapter'):
            message = '디텍션 인계 상태 수신 대기 / 지연'
        target = self.targets.get(task, (None,))[0]
        return dict(ready=bool(ready), task_id=task, reason=reason, message=message,
            coarse_position=dict(frame=target.header.frame_id,
                xyz=[target.coarse_position.x, target.coarse_position.y, target.coarse_position.z]) if target else None,
            base_lock_policy='실행 요청 후 실제 베이스 잠금 확인, 확인 전 팔 동작 금지')

    def blockers(self, require_handoff=True):
        result = []
        m = self.monitor
        if not self.client or not self.client.server_is_ready():
            result.append('실행 Action 서버 연결 없음')
        if not m.fresh('readiness') or m.values.get('readiness', {}).get('ready') is not True:
            result.append('실행기 준비 미완료')
        if not m.fresh('base') or m.values.get('base', {}).get('ready') is not True:
            result.append('최종 베이스 명령 잠금 경로 미확인')
        if not m.fresh('wrist') or m.values.get('wrist', {}).get('valid') is not True:
            result.append('손목 재검출 준비 미완료')
        if m.active or self.goal or m.pending:
            result.append('기존 작업 진행 중')
        if require_handoff and not self.candidate():
            result.append('새 접근 완료·실측 정지 인계 대기')
        return result

    def request(self, request):
        with self.request_lock:
            if request.get('action') == 'cancel':
                self.requests.clear()
                self.cancel_pending = True
            if len(self.requests) >= 4:
                raise RuntimeError('매니퓰 명령 처리 중입니다.')
            self.requests.append(request)

    def tick(self):
        with self.request_lock:
            request = self.requests.popleft() if self.requests else None
        if request:
            try:
                if request['action'] == 'arm':
                    errors = self.blockers(require_handoff=False)
                    if errors:
                        raise RuntimeError(' / '.join(errors))
                    self.armed, self.armed_at, self.armed_task = True, time.monotonic(), None
                    self.armed_at_ros = self.now()
                    self.state.log('매니퓰 자동 인계 대기 · 이후 시작하는 새 Detection 접근 완료 후 실행', source='manipulation')
                elif request['action'] == 'run':
                    self.last_error = ''
                    errors = self.blockers()
                    if errors:
                        raise RuntimeError(' / '.join(errors))
                    task = self.candidate()
                    if request.get('task_id', task) != task:
                        raise RuntimeError('전체 Task와 도킹 인계 ID 불일치')
                    self.cancel_pending = False
                    self.armed = False
                    self.monitor.pending = True
                    self.consumed.append(task)
                    self.client.send_goal_async(self.Action.Goal(task_id=task, operation=request.get('operation', '')),
                        feedback_callback=lambda f: self.status(f.feedback.status)).add_done_callback(self.accepted)
                    self.state.log('매니퓰 실행 요청 · 작업 '+task[:12], source='manipulation')
                elif request['action'] == 'cancel':
                    if self.armed:
                        self.armed = False
                        self.state.log('매니퓰 자동 인계 예약 취소', source='manipulation')
                        if not self.goal:
                            return
                    if not self.goal:
                        self.monitor.tick()
                        return  # A late acceptance is canceled in accepted().
                    self.goal.cancel_goal_async().add_done_callback(self.canceled)
            except Exception as exc:
                self.last_error = str(exc)
                self.state.log('매니퓰 명령 거부 · '+str(exc), 'warn', 'manipulation')
        if self.armed:
            if time.monotonic()-self.armed_at > 300:
                self.armed = False
                self.state.log('매니퓰 자동 인계 시간 초과 · 예약 해제', 'warn', 'manipulation')
            elif self.armed_task and self.candidate() == self.armed_task:
                self.armed = False
                self.request(dict(action='run'))
        handoff = self.handoff_state()
        if handoff['ready']:
            self.monitor.event('handoff', handoff['task_id'],
                '매니퓰 · 도킹 완료·실측 정지 인계 완료 · 작업 '+handoff['task_id'][:12],
                'info', 'manipulation-handoff')
        self.monitor.values['execution'] = dict(blockers=self.blockers(),
            arm_blockers=self.blockers(require_handoff=False), armed=self.armed,
            handoff_task=self.candidate(), handoff=handoff)
        self.monitor.tick()

    def accepted(self, future):
        self.monitor.pending = False
        try:
            goal = future.result()
            if not goal.accepted:
                raise RuntimeError('Action 서버가 작업을 거부했습니다.')
            self.goal = goal
            self.monitor.active = True
            goal.get_result_async().add_done_callback(self.finished)
            if self.cancel_pending:
                goal.cancel_goal_async().add_done_callback(self.canceled)
        except Exception as exc:
            self.last_error = str(exc)
            self.state.log('매니퓰 실행 거부 · '+str(exc), 'error', 'manipulation')

    def canceled(self, future):
        try:
            accepted = bool(future.result().goals_canceling)
            self.state.log('매니퓰 취소 '+('접수 · 정지 확인 대기' if accepted else '거부'),
                           'warn', 'manipulation')
        except Exception as exc:
            self.state.log('매니퓰 취소 응답 실패 · '+str(exc), 'error', 'manipulation')

    def finished(self, future):
        try:
            msg = future.result().result.status
            # Action result remains authoritative even if execution took >3 s.
            self.monitor.receive('status', {k: getattr(msg, k) for k in ('task_id', 'class_id',
                'stage', 'outcome', 'reason', 'holding', 'place_success', 'return_success', 'stop_confirmed')})
        except Exception as exc:
            self.state.log('매니퓰 결과 수신 실패 · '+str(exc)+' · 로봇 정지 여부 확인 필요',
                           'error', 'manipulation')
        finally:
            self.goal = None
            # A failed transport is not proof of a stopped robot.
