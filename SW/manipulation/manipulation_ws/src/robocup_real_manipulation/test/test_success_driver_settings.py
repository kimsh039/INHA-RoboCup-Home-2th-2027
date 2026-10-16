import copy
import importlib.util
import json
from pathlib import Path
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location('success_driver_settings', ROOT/'tools/restore_success_driver_settings.py')
restore = importlib.util.module_from_spec(spec); spec.loader.exec_module(restore)
REFERENCE = json.loads((ROOT/'references/success_driver_contract_20261015.json').read_text())


def test_only_missing_driver_settings_are_restored():
    cfg = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/robot.yaml').read_text())
    frozen = copy.deepcopy(cfg); result = restore.restored(cfg, REFERENCE)
    assert cfg == frozen
    assert result['joint_offsets'] == [-1.6, 0, 0, 0, 0, 0]
    assert result['can_port'] == 'can_arm'
    for key in cfg.keys()-set(restore.FIELDS):
        assert result[key] == cfg[key]
    assert not result['execution_enabled'] and not result['commissioned']
    assert result['width_scale'] is None and not result['width_mapping_verified']


def test_existing_mapping_is_not_overwritten():
    with pytest.raises(ValueError, match='EXISTING_DRIVER_SETTING_DIFFERS:joint_offsets'):
        restore.restored(dict(joint_offsets=[0.]*6), REFERENCE)


def test_source_success_task_hash_and_trial_uncertainty_are_preserved():
    assert REFERENCE['task_id'] == '7ece08d387f547cd942d794076f82bb0'
    assert REFERENCE['config_sha256'] == '2d6245769bc5b9a4488c6d5a765120b3d797aa453bdc5f90eb45ceb674f9474a'
    assert REFERENCE['trial_gripper_settings']['width_mapping_mode'] == 'nominal_trial'
    assert not REFERENCE['trial_gripper_settings']['width_mapping_verified']
