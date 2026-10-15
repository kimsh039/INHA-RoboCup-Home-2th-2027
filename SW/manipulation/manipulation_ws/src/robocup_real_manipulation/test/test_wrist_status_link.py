import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import pytest

tool = Path(__file__).resolve().parents[4]/'tools/link_control_center_wrist_status.py'
spec = importlib.util.spec_from_file_location('wrist_status_link', tool)
link = importlib.util.module_from_spec(spec); spec.loader.exec_module(link)


def test_preserves_existing_topics_and_other_settings():
    original = dict(channels=dict(wrist_status=['/detection/wrist/status'],
                                 wrist_image=['/wrist/wrist/color/image_raw']),
                    detection=dict(model_joint_topic='/robot/joint_states'), private_setting='keep')
    frozen = copy.deepcopy(original)
    result = link.linked_config(original)
    assert original == frozen
    assert result['channels']['wrist_status'] == ['/manipulation/wrist/status', '/detection/wrist/status']
    result['channels']['wrist_status'] = frozen['channels']['wrist_status']
    assert result == original


def test_missing_channel_and_duplicates():
    assert link.linked_config(dict(channels={}))['channels']['wrist_status'] == [link.TOPIC]
    result = link.linked_config(dict(channels=dict(wrist_status=[link.TOPIC, link.TOPIC, '/other', '/other'])))
    assert result['channels']['wrist_status'] == [link.TOPIC, '/other']


def test_malformed_config_is_not_silently_replaced():
    for value in ({}, dict(channels=[]), dict(channels=dict(wrist_status='/wrong')), []):
        with pytest.raises(ValueError):
            link.linked_config(value)


def test_check_only_backup_and_idempotence():
    with tempfile.TemporaryDirectory() as td:
        path = Path(td)/'config.json'; raw = '{"channels":{"wrist_status":["/detection/wrist/status"]}}\n'
        path.write_text(raw)
        assert link.apply(path, check=True)[1] is None and path.read_text() == raw
        topics, backup = link.apply(path)
        assert topics[0] == link.TOPIC and backup.read_text() == raw
        assert json.loads(path.read_text())['channels']['wrist_status'] == topics
        assert link.apply(path)[1] is None
