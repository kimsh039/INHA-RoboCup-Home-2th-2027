import importlib.util
from pathlib import Path

path = Path(__file__).resolve().parents[4]/'tools/check_control_center_source.py'
spec = importlib.util.spec_from_file_location('source_contract_check', path)
check = importlib.util.module_from_spec(spec); spec.loader.exec_module(check)
BASE = f'''CLASSES = {check.CLASSES!r}
STATUS = '/detection/closed_approach/status'
TARGET = '/detection/closed_approach/target'
'''


def test_current_realign_and_deferred_success_are_recognized():
    source = BASE+'''def phase(self):
    self.report('realigning', 'wait')
    self.begin_final_alignment(now, pose, 'dock', 'succeeded', 'done')
    self.begin_final_alignment(now, pose, 'point', 'at_target', 'unconfirmed')
'''
    result = check.inspect_source(source)
    assert result['ok'] and 'succeeded' in result['observed_phases']


def test_unknown_phase_requires_semantic_review():
    result = check.inspect_source(BASE+"self.report('new_unreviewed_phase', '')\n")
    assert not result['ok'] and result['unknown_phases'] == ['new_unreviewed_phase']


def test_topic_or_class_schema_changes_are_flagged():
    assert not check.inspect_source(BASE.replace('/detection/closed_approach/status', '/changed/status'))['ok']
    assert not check.inspect_source(BASE.replace('fanta_can', 'can'))['ok']
