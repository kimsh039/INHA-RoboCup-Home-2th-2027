"""Own navigation/docking in a spawned process, independent of web encoding.

Only bounded commands, status snapshots and log events cross the private pipe.
Pipe I/O runs in helper threads so a slow web consumer never stalls control.
"""
from collections import deque
import multiprocessing
import threading
import time


class Outbox:
    def __init__(self):
        self.lock = threading.Lock()
        self.events = deque(maxlen=128)
        self.latest = {}

    def put(self, message):
        with self.lock:
            if message[0] == 'state':
                self.latest[message[1]] = message
            else:
                self.events.append(message)

    def drain(self):
        with self.lock:
            batch = list(self.events)+list(self.latest.values())
            self.events.clear()
            self.latest.clear()
        return batch


class ControlSensors:
    """SensorManager interface backed by the parent's service snapshot."""
    def __init__(self, state, outgoing, require_collision_ready):
        self.state = state
        self.outgoing = outgoing
        self.require_collision_ready = require_collision_ready

    def service_request(self, key, action):
        self.outgoing.put(('service', key, action))

    def navigation_available(self):
        with self.state.condition:
            service = (self.state.values.get('service_control') or {}).get('navigation') or {}
            # Availability prevents duplicate starts; lifecycle readiness is
            # checked separately by NavigationController before sending goals.
            return bool(service.get('managed') or service.get('publishers'))


class ControllerProcess:
    def __init__(self, state, config, use_sim_time):
        self.state = state
        self.lock = threading.Lock()
        self.requests = deque()
        self.services = deque(maxlen=16)
        self.closed = threading.Event()
        self.stopping = False
        context = multiprocessing.get_context('spawn')
        self.connection, remote = context.Pipe()
        self.process = context.Process(target=run_controller,
            args=(remote, config, use_sim_time), name='inha-docking-control')
        self.process.start()
        remote.close()
        self.thread = threading.Thread(target=self.pump, name='control-process-ipc', daemon=True)
        self.thread.start()
        state.navigation_request = lambda request: self.request('navigation', request)
        state.detection_request = lambda request: self.request('detection', request)

    def request(self, kind, value):
        with self.lock:
            if self.stopping or self.closed.is_set() or not self.process.is_alive():
                raise RuntimeError('독립 주행·도킹 제어 프로세스 연결 없음')
            if len(self.requests) >= 32:
                raise RuntimeError('주행·도킹 명령 처리 대기 중')
            self.requests.append((kind, dict(value), time.monotonic()))

    def pump(self):
        try:
            while not self.closed.is_set() and not self.state.closed.is_set():
                with self.lock:
                    batch = list(self.requests)
                    self.requests.clear()
                with self.state.condition:
                    services = self.state.values.get('service_control') or {}
                    interlocked = self.state.manipulation_interlock()
                self.connection.send(('heartbeat', time.monotonic(), services, interlocked))
                for request in batch:
                    self.connection.send(request)
                if self.stopping:
                    self.connection.send(('shutdown',))
                until = time.monotonic()+.1
                while time.monotonic() < until and self.connection.poll(.01):
                    message = self.connection.recv()
                    if message[0] == 'state':
                        self.state.put(message[1], message[2])
                    elif message[0] == 'log':
                        self.state.log(*message[1:])
                    elif message[0] == 'service':
                        with self.lock:
                            self.services.append(message[1:])
                    elif message[0] == 'closed':
                        return
                if not self.process.is_alive():
                    raise RuntimeError('control_process_exited')
                self.closed.wait(.05)
        except (EOFError, OSError, RuntimeError) as exc:
            if not self.stopping:
                self.state.log('독립 주행·도킹 제어 종료 · '+str(exc), 'error', 'navigation')
                self.state.put('navigation', dict(state='error', available=False, message='제어 프로세스 연결 끊김'))
                self.state.put('detection_control', dict(active=False, phase='error', message='제어 프로세스 연결 끊김'))
        finally:
            self.closed.set()

    def dispatch_services(self):
        with self.lock:
            requests = list(self.services)
            self.services.clear()
        for key, action in requests:
            try:
                if self.state.sensor_manager:
                    self.state.sensor_manager.service_request(key, action)
            except (ValueError, RuntimeError) as exc:
                self.state.log(str(exc), 'warn', 'sensor-control')

    def close(self):
        self.stopping = True
        self.process.join(timeout=4.)
        if self.process.is_alive():
            # Loss of the pipe cancels the controller; actuator watchdogs also
            # expire its last velocity independently if the child is wedged.
            self.connection.close()
            self.process.join(timeout=2.)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=2.)
        self.closed.set()
        self.thread.join(timeout=1.)
        self.connection.close()


def run_controller(connection, config, use_sim_time):
    import rclpy
    import signal
    from rclpy.signals import SignalHandlerOptions
    from navigation_controller import NavigationController
    from server import State

    outgoing = Outbox()
    commands = deque()
    disconnected = threading.Event()
    interrupted = threading.Event()
    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGINT, lambda *_: interrupted.set())
        signal.signal(signal.SIGTERM, lambda *_: interrupted.set())
    context = dict(received=time.monotonic(), services={}, interlocked=True)
    context_lock = threading.Lock()

    def transport():
        try:
            while not disconnected.is_set():
                for message in outgoing.drain():
                    connection.send(message)
                    if message[0] == 'closed':
                        return
                if not connection.poll(.01):
                    continue
                message = connection.recv()
                if message[0] == 'heartbeat':
                    with context_lock:
                        context.update(received=message[1], services=message[2], interlocked=message[3])
                else:
                    commands.append(message)
        except (EOFError, OSError):
            disconnected.set()

    class ControlState(State):
        def put(self, key, value, *args, **kwargs):
            super().put(key, value, *args, **kwargs)
            outgoing.put(('state', key, self.values[key]))

        def log(self, message, level='info', source='bridge'):
            super().log(message, level, source)
            outgoing.put(('log', message, level, source))

    node = None
    thread = threading.Thread(target=transport, name='control-web-ipc', daemon=True)
    thread.start()
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:='+str(use_sim_time).lower()],
               signal_handler_options=SignalHandlerOptions.NO)
    state = ControlState('live', config)
    state.sensor_manager = ControlSensors(state, outgoing,
        lambda: node.detection_approach.require_protection_ready())
    shutdown = False
    try:
        node = NavigationController(state, config)
        while rclpy.ok() and not shutdown and not interrupted.is_set():
            with context_lock:
                received, services, interlocked = context['received'], context['services'], context['interlocked']
            if disconnected.is_set() or time.monotonic()-received > 1.5:
                state.log('관제센터 연결 끊김 · 주행·도킹 취소', 'error', 'navigation')
                break
            with state.condition:
                state.values['service_control'] = services
            while commands:
                message = commands.popleft()
                kind = message[0]
                if kind == 'shutdown':
                    shutdown = True
                    break
                if len(message) != 3 or kind not in ('navigation', 'detection'):
                    continue
                request, queued = message[1:]
                canceling = request.get('action') in ('cancel', 'stop', 'clear')
                departure = kind == 'navigation' and request.get('action') == 'departure'
                if not canceling and (time.monotonic()-queued > 2. or (interlocked and not departure)):
                    state.log('주행 명령 거부 · 지연 또는 매니퓰레이션 잠금', 'warn', kind)
                    continue
                if kind == 'navigation':
                    node.nav_requests.append(request)
                else:
                    node.detection_approach.requests.append(request)
            if interlocked and (node.detection_approach.active or node.nav_status['state'] in ('active', 'sending')):
                node.front_retreat.cancel('매니퓰레이션 잠금 · 주행 중지')
                node.detection_approach.cancel('매니퓰레이션 잠금 · 접근 중지')
                node.cancel_goal()
            if not shutdown:
                rclpy.spin_once(node, timeout_sec=.025)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        state.log('독립 제어 오류 · '+str(exc), 'error', 'navigation')
    finally:
        if node is not None:
            try:
                node.stop_control()
            except Exception as exc:
                state.log('제어 종료 처리 오류 · '+str(exc), 'error', 'navigation')
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        outgoing.put(('closed',))
        # Let the transport finish its bounded status batch without making
        # the real-time loop wait for web rendering at any point.
        thread.join(timeout=.2)
        disconnected.set()
        connection.close()
