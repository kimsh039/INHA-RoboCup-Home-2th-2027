"""No robot commands: event model and authenticated HTTP routing."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
from urllib.request import Request, urlopen
from urllib.error import HTTPError

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from server import State, make_server
from manipulation_control import ManipulationMonitor


def fixture():
    now = [10.]
    state = State(mode='live', config={})
    monitor = ManipulationMonitor(state, lambda: now[0])
    return state, monitor, now


def test_failure_goes_to_existing_event_list_once():
    state, monitor, now = fixture()
    msg = dict(task_id='task1', stage='OBSERVE', outcome='FAILED', reason='CAMERA_SYNC')
    for _ in range(200):
        monitor.receive('status', msg)
    assert len(state.logs) == 1
    event = state.logs[0]
    assert event['level'] == 'error' and event['source'] == 'manipulation'
    assert 'CAMERA_SYNC' in event['message'] and '손목' in event['message']
    assert isinstance(event['time'], float)


def test_stage_result_transitions_not_lost():
    state, monitor, _ = fixture()
    for stage, outcome, reason in [('OBSERVE','RUNNING',''), ('GRASP','RUNNING',''),
                                  ('DONE','SUCCESS','BASE_LOCK_RETAINED')]:
        monitor.receive('status', dict(task_id='t',stage=stage,outcome=outcome,reason=reason))
    assert len(state.logs) == 3
    assert not monitor.active
    assert state.logs[-1]['level'] == 'info'


def test_disconnect_is_logged_once_and_recovery_once():
    state, monitor, now = fixture()
    monitor.receive('readiness', dict(ready=True, errors=[]))
    now[0] += 6
    for _ in range(100): monitor.tick()
    missing = [x for x in state.logs if x['source']=='manipulation-readiness' and '상태 수신 없음' in x['message']]
    assert len(missing)==1
    monitor.receive('readiness', dict(ready=True, errors=[]))
    monitor.tick()
    assert any('상태 수신 복구' in x['message'] for x in state.logs)
    now[0] += 4
    monitor.tick()
    assert sum('상태 수신 없음' in x['message'] and x['source']=='manipulation-readiness' for x in state.logs)==2


def test_readiness_wrist_and_adapter_errors_use_shared_events():
    state, monitor, _ = fixture()
    monitor.receive('readiness', dict(ready=False,errors=['MODEL_JOINT_FEEDBACK_STALE']))
    monitor.receive('wrist', dict(valid=False,reason='CAMERA_SYNC'))
    monitor.receive('adapter', dict(reason='CONTROL_CENTER_CLASS_SCHEMA'))
    assert len(state.logs)==3 and all(x['level']=='warn' for x in state.logs)
    assert {x['source'] for x in state.logs}=={'manipulation-readiness','manipulation-wrist','manipulation-adapter'}


def test_malformed_status_rejected():
    _, monitor, _ = fixture()
    with pytest.raises(ValueError): monitor.receive('status', {})
    with pytest.raises(ValueError): monitor.receive('readiness', dict(errors='invalid'))


def test_http_action_is_authenticated_and_validated():
    state, _, _ = fixture()
    commands=[]
    state.manipulation_request=commands.append
    server=make_server(state,port=0)
    worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
    url='http://127.0.0.1:'+str(server.server_port)+'/api/manipulation'
    def post(body,token):
        request=Request(url,data=json.dumps(body).encode(),headers={
            'Content-Type':'application/json','X-Control-Token':token})
        try:
            with urlopen(request,timeout=2) as r: return r.status
        except HTTPError as e: return e.code
    try:
        assert post({'action':'run'},'wrong')==403
        assert post({'action':'shell'},state.control_token)==400
        assert post({'action':'run','task_id':'invented'},state.control_token)==400
        assert post({'action':'run'},state.control_token)==202
        assert post({'action':'cancel'},state.control_token)==202
        assert commands==[{'action':'run'},{'action':'cancel'}]
    finally:
        server.shutdown();server.server_close();worker.join()


def test_manipulation_tab_uses_existing_theme_and_events():
    html=(ROOT/'static/index.html').read_text()
    js=(ROOT/'static/app.js').read_text()
    assert 'data-mode-button="manipulation"' in html
    assert 'mode-section mode-panel hidden" data-mode="manipulation"' in html
    assert html.count('id="event-list"')==1
    assert 'manipulation: 1' in js
    # The full Task panel may render between these two independent panels.
    assert 'renderManipulation();' in js
    assert 'renderLogs();' in js


def controller():
    from types import SimpleNamespace as NS
    from collections import deque
    from manipulation_control import ManipulationControl
    c=ManipulationControl.__new__(ManipulationControl)
    c.state,c.monitor,_=fixture();c.monitor.state=c.state
    c.client=NS(server_is_ready=lambda:True)
    c.goal=None;c.targets={};c.approaches={};c.consumed=deque(maxlen=128)
    c.seen_sessions=deque(maxlen=32);c.now=lambda:100.
    c.armed=False;c.armed_at=0.;c.armed_task=None;c.started_ros=90.;c.armed_at_ros=0.
    c.requests=deque();c.request_lock=threading.Lock()
    return c


def test_stale_or_replayed_handoff_cannot_be_run():
    from types import SimpleNamespace as NS
    import time
    c=controller()
    c.targets['t']=(NS(class_id=0, header=NS(frame_id='odom', stamp=NS(sec=99,nanosec=0)),
        coarse_position=NS(x=.4,y=.1,z=.6)),time.monotonic())
    c.approaches['t']=(NS(outcome='SUCCESS',base_stopped=True,
        header=NS(stamp=NS(sec=100,nanosec=0))),time.monotonic())
    c.monitor.receive('adapter',dict(task_id='t',reason='HANDED_OFF_EXTERNAL_BASE_LEASE_REQUIRED', measured_stop_confirmed=True))
    c.monitor.receive('base',dict(ready=True,stopped=True,locked=False))
    assert c.candidate() is None # No observed active approach in this session.
    c.seen_sessions.append('t');assert c.candidate()=='t'
    c.approaches['t'][0].header.stamp.sec=69;assert c.candidate() is None
    c.approaches['t'][0].header.stamp.sec=100;c.consumed.append('t');assert c.candidate() is None


def test_run_requires_all_live_gate_inputs():
    c=controller()
    assert len(c.blockers())>=4
    c.monitor.receive('readiness',dict(ready=True,errors=[]))
    c.monitor.receive('wrist',dict(valid=True))
    c.monitor.receive('base',dict(ready=True,locked=False))
    assert c.blockers(require_handoff=False)==[]
    assert c.blockers()==['새 접근 완료·실측 정지 인계 대기']
    c.monitor.active=True
    assert '기존 작업 진행 중' in c.blockers()


def test_reservation_canceled_on_adapter_failure():
    from types import SimpleNamespace as NS
    c=controller();c.armed=True
    c.json_status('adapter',NS(data=json.dumps(dict(reason='CONTROL_CENTER_CLASS_SCHEMA'))))
    assert not c.armed
    assert any('자동 인계 중단' in e['message'] for e in c.state.logs)


def test_base_lock_interlock_survives_stale_stream():
    c=controller()
    c.monitor.receive('base',dict(ready=True,locked=True))
    c.monitor.received['base']=-100
    assert c.interlock() # Stale data is no evidence of an unlock.
