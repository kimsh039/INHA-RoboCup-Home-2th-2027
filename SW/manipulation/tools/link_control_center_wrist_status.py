#!/usr/bin/env python3
"""Connect the web status channel to the real manipulation wrist publisher.

Updates only channel selection. Does not publish, start or restart anything.
"""
import argparse
import copy
import datetime
import json
import os
from pathlib import Path
import shutil
import tempfile

TOPIC = '/manipulation/wrist/status'


def linked_config(config):
    result = copy.deepcopy(config)
    if not isinstance(result, dict) or not isinstance(result.get('channels'), dict):
        raise ValueError('invalid_control_center_channels')
    current = result['channels'].get('wrist_status', [])
    if not isinstance(current, list) or not all(isinstance(t, str) and t.startswith('/') for t in current):
        raise ValueError('invalid_wrist_status_candidates')
    result['channels']['wrist_status'] = [TOPIC]+list(dict.fromkeys(t for t in current if t != TOPIC))
    return result


def apply(path, check=False):
    original = path.read_bytes()
    config = json.loads(original)
    result = linked_config(config)
    if check or result == config:
        return result['channels']['wrist_status'], None
    backup = path.with_name(path.name+'.bak-manip-wrist-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    if path.read_bytes() != original:
        raise RuntimeError('concurrent_config_change')
    shutil.copy2(path, backup)
    fd, name = tempfile.mkstemp(prefix='.wrist-status-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(result, f, ensure_ascii=False, indent=2); f.write('\n')
        os.chmod(name, path.stat().st_mode)
        if path.read_bytes() != original:
            raise RuntimeError('concurrent_config_change_backup_preserved')
        os.replace(name, path)
    finally:
        if Path(name).exists():
            Path(name).unlink()
    return result['channels']['wrist_status'], backup


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--check', action='store_true')
    a = p.parse_args()
    topics, backup = apply(a.config.resolve(), a.check)
    print(json.dumps(dict(wrist_status_topics=topics, backup=str(backup) if backup else None,
                         check_only=a.check, process_restarted=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
