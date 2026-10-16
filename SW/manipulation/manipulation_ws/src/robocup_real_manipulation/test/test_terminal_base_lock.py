import copy
from types import SimpleNamespace
from unittest.mock import Mock
import pytest

from robocup_real_manipulation.core import Fault, validate
from robocup_real_manipulation.node import Coordinator
from robocup_real_manipulation.sequence import Sequence
from test_core import CFG


def test_default_completion_keeps_real_lease_and_does_not_request_unlock():
    lease = object()
    backend = SimpleNamespace(c={}, lease=lease, lock_requested=True,
                              guard=Mock(), publish_lock=Mock())
    assert Coordinator.finish_success(backend, object()) == 'BASE_LOCK_RETAINED'
    backend.guard.assert_called_once()
    backend.publish_lock.assert_called_once()
    assert backend.lock_requested and backend.lease is lease


def test_hold_completion_cannot_bypass_missing_real_lease():
    backend = SimpleNamespace(c={'base_completion_policy': 'hold'}, lock_requested=True,
                              guard=Mock(side_effect=Fault('BASE_NOT_LOCKED_AND_STOPPED')), publish_lock=Mock())
    with pytest.raises(Fault, match='BASE_NOT_LOCKED'):
        Coordinator.finish_success(backend, object())
    backend.publish_lock.assert_not_called()


def test_sequence_success_reports_retained_base_lock():
    backend = SimpleNamespace(guard=lambda: None, perform=lambda stage, seq: None,
                              status=lambda *args: None, finish_success=lambda seq: 'BASE_LOCK_RETAINED')
    seq = Sequence(backend)
    assert seq.run() == ('SUCCESS', 'BASE_LOCK_RETAINED')
    assert seq.stage == 'COMPLETE' and seq.place_success and seq.return_success


@pytest.mark.parametrize('value', ['auto', False, None])
def test_unknown_completion_policy_is_rejected(value):
    config = copy.deepcopy(CFG); config['base_completion_policy'] = value
    with pytest.raises(Fault, match='BASE_COMPLETION_POLICY'):
        validate(config)
