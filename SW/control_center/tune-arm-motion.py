#!/usr/bin/env python3
"""Tune only arm-motion classification; preserve all other collision settings."""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import shutil


def digest(data):
    return hashlib.sha256(data).hexdigest()


def replace_value(text, key, value):
    pattern = rf'(?m)^(\s*{re.escape(key)}:\s*)[^\s#]+(\s*(?:#.*)?)$'
    text, count = re.subn(pattern, lambda m: m[1] + str(value) + m[2], text)
    if count != 1:
        raise ValueError(f'Expected one {key}, found {count}')
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parent.parent / 'robot/config/collision_safety.yaml')
    parser.add_argument('--speed', type=float, default=0.5, help='rad/s, shared by cloud and guard')
    parser.add_argument('--settle', type=float, default=0.4, help='seconds')
    parser.add_argument('--restore', type=Path, help='backup directory printed by an earlier invocation')
    args = parser.parse_args()
    path = args.config.resolve()
    before = path.read_bytes()
    if args.restore:
        manifest = json.loads((args.restore / 'manifest.json').read_text())
        if str(path) != manifest['path'] or digest(before) != manifest['after_sha256']:
            raise SystemExit('Configuration changed since tuning; refusing to overwrite concurrent edits.')
        original = (args.restore / path.name).read_bytes()
        if digest(original) != manifest['before_sha256']:
            raise SystemExit('Backup checksum mismatch')
        path.write_bytes(original)
        print('Restored:', path)
        return
    if not all(math.isfinite(x) and x > 0 for x in (args.speed, args.settle)):
        raise SystemExit('speed/settle must be finite positive values')
    text = replace_value(before.decode(), 'arm_motion_limit', args.speed)
    after = replace_value(text, 'arm_settle_sec', args.settle).encode()
    backup = path.parent / ('.arm-motion-backup-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    backup.mkdir()
    shutil.copy2(path, backup / path.name)
    (backup / 'manifest.json').write_text(json.dumps(dict(path=str(path), before_sha256=digest(before), after_sha256=digest(after), speed=args.speed, settle=args.settle), indent=2) + '\n')
    if path.read_bytes() != before:
        raise SystemExit('Configuration changed during tuning; no changes applied.')
    path.write_bytes(after)
    print('Backup:', backup)
    print('arm_motion_limit:', args.speed, 'rad/s; arm_settle_sec:', args.settle)
    print('Existing nodes require restart. This tool does not start or move hardware.')


if __name__ == '__main__':
    main()
