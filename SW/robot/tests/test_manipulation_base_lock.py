import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import pytest

spec=importlib.util.spec_from_file_location('base_gate',Path(__file__).resolve().parents[1]/'manipulation_base_lock.py')
gate=importlib.util.module_from_spec(spec);spec.loader.exec_module(gate)


def adapter(tmp_path):
    obj=gate.BaseLockAdapter.__new__(gate.BaseLockAdapter)
    obj.path=tmp_path/'lock.json';obj.task='';obj.release_task='';obj.error=''
    obj.node=NS(owner_valid=True,get_publishers_info_by_topic=lambda topic:[NS(node_name='real_manipulation')])
    obj.now=lambda:100.
    obj.stop_since=None;obj.odom_stamp=None
    return obj


def request(task='task', locked=True, stamp=100):
    return NS(task_id=task,locked=locked,header=NS(stamp=NS(sec=stamp,nanosec=0)))


def test_lock_is_latched_and_persisted(tmp_path):
    obj=adapter(tmp_path);obj.request(request())
    assert obj.task=='task' and 'task' in obj.path.read_text()
    obj.request(request('other'))
    assert obj.task=='task' and obj.error=='OTHER_BASE_LOCK_OWNER'


def test_lock_stays_closed_if_persistence_fails(tmp_path):
    obj=adapter(tmp_path)
    with patch.object(obj,'persist',side_effect=OSError):obj.request(request())
    assert obj.task=='task' and obj.error=='BASE_LOCK_PERSIST_FAILED'


def test_no_unlock_without_actual_stop_or_single_owner(tmp_path):
    obj=adapter(tmp_path);obj.request(request());obj.stopped=lambda:False
    obj.request(request(locked=False));assert obj.task=='task'
    obj.stopped=lambda:True;obj.node.owner_valid=False
    obj.request(request(locked=False));assert obj.task=='task'
    obj.node.owner_valid=True;obj.request(request(locked=False))
    assert obj.task=='' and obj.release_task=='task'


def test_stale_request_cannot_lock(tmp_path):
    obj=adapter(tmp_path);obj.request(request(stamp=98))
    assert not obj.task and not obj.path.exists()


def test_measured_stop_requires_fresh_correct_frame_and_stationary_twist(tmp_path):
    obj=adapter(tmp_path)
    msg=NS(header=NS(frame_id='odom',stamp=NS(sec=100,nanosec=0)),child_frame_id='base_link',
           twist=NS(twist=NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))))
    with patch.object(gate.time,'monotonic',return_value=10.):obj.odometry(msg)
    with patch.object(gate.time,'monotonic',return_value=10.7):assert obj.stopped()
    obj.now=lambda:101.
    assert not obj.stopped()
    obj.now=lambda:100.;msg.header.frame_id='map';obj.odometry(msg)
    assert not obj.stopped()


def test_real_locked_zero_remains_acknowledged_without_upstream_approval(tmp_path):
    obj=adapter(tmp_path);obj.task='task';obj.last_ack=0
    obj.stopped=lambda:True
    obj.node.fresh=lambda:False
    obj.node.output='/tracer/cmd_vel_safe'
    obj.node.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS(sec=100,nanosec=0)))
    obj.Lease=lambda **kw:NS(header=NS(stamp=None),**kw)
    obj.String=lambda **kw:NS(**kw)
    leases=[];obj.pub=NS(publish=leases.append);obj.status_pub=NS(publish=lambda msg:None)
    with patch.object(gate.time,'monotonic',return_value=10.):obj.tick(zero_output=True)
    assert len(leases)==1 and leases[0].locked and leases[0].stopped
