#!/usr/bin/env python3
"""Fill missing driver settings from hash-matched historical success evidence.

Writes a new config only. Never enables hardware, copies fixture/scene settings,
imports nominal gripper calibration, starts a driver or fabricates live feedback.
"""
import argparse
import copy
import json
from pathlib import Path
import yaml

FIELDS = ('driver_joint_names', 'joint_names', 'joint_signs', 'joint_offsets',
          'feedback_topic', 'gripper_feedback_topic', 'arm_status_topic',
          'command_topic', 'enable_service', 'can_port', 'firmware')


def restored(config, reference):
    result = copy.deepcopy(config)
    for key in FIELDS:
        value = reference['driver_settings'][key]
        if result.get(key) is None:
            result[key] = copy.deepcopy(value)
        elif result[key] != value:
            raise ValueError('EXISTING_DRIVER_SETTING_DIFFERS:'+key)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--reference', type=Path, default=Path(__file__).resolve().parents[1]/
                   'references/success_driver_contract_20261015.json')
    a = p.parse_args()
    reference = json.loads(a.reference.read_text())
    result = restored(yaml.safe_load(a.config.read_text()), reference)
    with a.output.open('x') as f:
        yaml.safe_dump(result, f, allow_unicode=True, sort_keys=False)
    print(json.dumps(dict(output=str(a.output), reference_task_id=reference['task_id'],
        reference_config_sha256=reference['config_sha256'], hardware_started=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
