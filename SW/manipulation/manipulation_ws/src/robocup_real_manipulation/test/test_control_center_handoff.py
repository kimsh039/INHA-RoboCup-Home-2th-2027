"""Web handoff gating without ROS, hardware or action execution."""
from collections import deque
from pathlib import Path
from types import SimpleNamespace as N
import importlib.util
import json
import threading
import time

path = Path(__file__).resolve().parents[5] / 'control_center/manipulation_control.py'
spec = importlib.util.spec_from_file_location('handoff_web', path)
web = importlib.util.module_from_spec(spec); spec.loader.exec_module(web)


class State:
    def __init__(self): self.logs = []; self.values = {}
    def log(self, message, level='info', source=''):
        self.logs.append(dict(message=message, level=level, source=source))
    def put(self, key, value): self.values[key] = value


def setup():
    c = web.ManipulationControl.__new__(web.ManipulationControl)
    c.state = State(); c.monitor = web.ManipulationMonitor(c.state, clock=lambda: 50.)
    c.now = lambda: 100.; c.started_ros = 90.; c.armed_at_ros = 95.
    c.armed = False; c.armed_task = None; c.armed_at = time.monotonic()
    c.targets = {}; c.approaches = {}; c.seen_sessions = deque(maxlen=32)
    c.consumed = deque(maxlen=128); c.goal = None
    c.requests = deque(); c.request_lock = threading.Lock()
    c.client = N(server_is_ready=lambda: True)
    c.monitor.receive('base', dict(ready=True, stopped=True, locked=False))
    c.monitor.receive('wrist', dict(valid=True))
    c.monitor.receive('readiness', dict(ready=False, errors=['READ_ONLY_MODE']))
    point = N(x=.4, y=.1, z=.6)
    c.target(N(task_id='dock-A', class_id=3, header=N(frame_id='odom', stamp=N(sec=99, nanosec=0)), coarse_position=point))
    c.approach(N(task_id='dock-A', outcome='SUCCESS', base_stopped=True, header=N(stamp=N(sec=100, nanosec=0))))
    return c


def complete(c, started=96., stopped=True):
    c.json_status('adapter', N(data=json.dumps(dict(task_id='dock-A',
        reason='HANDED_OFF_EXTERNAL_BASE_LEASE_REQUIRED', approach_started_at=started,
        measured_stop_confirmed=stopped, handed_off=True))))


def test_completion_arriving_before_running_status_still_joins_same_new_job():
    c = setup(); assert c.candidate() is None
    complete(c)
    assert c.candidate() == 'dock-A'
    assert c.handoff_state()['coarse_position'] == dict(frame='odom', xyz=[.4, .1, .6])
    assert c.handoff_state()['ready']


def test_old_session_and_expired_pair_never_become_new_handoff():
    c = setup(); complete(c, started=80.)
    assert c.candidate() is None
    c = setup(); complete(c); c.approaches['dock-A'][0].header.stamp.sec = 69
    assert c.candidate() is None


def test_handoff_does_not_fake_hardware_readiness_or_send_an_action():
    c = setup(); complete(c); c.tick(); c.tick()
    assert c.monitor.values['execution']['handoff']['ready']
    assert c.blockers() == ['실행기 준비 미완료']
    assert len([e for e in c.state.logs if e['source'] == 'manipulation-handoff']) == 1
    c.monitor.receive('readiness', dict(ready=True, errors=[]))
    assert c.blockers() == []


def test_current_stop_and_unconsumed_id_are_required():
    c = setup(); complete(c, stopped=False); assert c.candidate() is None
    complete(c); c.monitor.receive('base', dict(ready=True, stopped=False)); assert c.candidate() is None
    c.monitor.receive('base', dict(ready=True, stopped=True)); assert c.candidate() == 'dock-A'
    c.consumed.append('dock-A'); assert c.candidate() is None


def test_automatic_reservation_only_adopts_a_subsequent_session():
    c = setup(); c.armed = True; complete(c, started=94.)
    assert c.armed_task is None
    c = setup(); c.armed = True; complete(c, started=96.)
    assert c.armed_task == 'dock-A'
