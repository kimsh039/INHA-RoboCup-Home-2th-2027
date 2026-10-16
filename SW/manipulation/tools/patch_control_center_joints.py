#!/usr/bin/env python3
"""Patch only model-joint input sites in the user's current web source.

Never launches/restarts a server or writes a robot command. Keeps source backups.
"""
import argparse
import datetime
import os
from pathlib import Path
import re
import shutil
import tempfile


def rewrite(source):
    if 'from joint_contract import ModelJointFeedback' in source:
        if ('self.arm_feedback.positions()' not in source
                or "self.state.values.get('joints')" in source):
            raise ValueError('incomplete_existing_joint_patch')
        return source, 0
    needle = '        self.planner = GraspPlanner(settings)\n'
    if source.count(needle) != 1:
        raise ValueError('constructor_changed_inspect_before_patching')
    result = source.replace(needle, needle+
        '        from joint_contract import ModelJointFeedback\n'
        '        self.arm_feedback = ModelJointFeedback(node, settings)\n')
    pattern = (r"(?m)^( +)with self\.state\.condition:\n\1    joints = self\.state\.values\.get\('joints'\) or \{\}\n"
               r"\1positions = dict\(zip\(joints\.get\('names', \[\]\), joints\.get\('positions', \[\]\)\)\)\n")
    result, count = re.subn(pattern, lambda m:m[1]+'positions = self.arm_feedback.positions()\n', result)
    if count not in (1, 2):
        raise ValueError(f'unexpected_joint_input_sites:{count}')
    compile(result, 'detection_approach.py', 'exec')
    return result, count


def apply(folder, helper, check=False):
    target = folder/'detection_approach.py'
    original = target.read_bytes()
    result, count = rewrite(original.decode())
    helper_data = helper.read_bytes()
    compile(helper_data, str(helper), 'exec')
    if check:
        return count, None
    if not count and (folder/'joint_contract.py').read_bytes() == helper_data:
        return 0, None
    # Refuse an intervening edit instead of overwriting another developer's work.
    if target.read_bytes() != original:
        raise RuntimeError('concurrent_source_change_retry_after_review')
    backup = folder/('.joint-input-backup-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    backup.mkdir()
    shutil.copy2(target, backup/target.name)
    destination = folder/'joint_contract.py'
    if destination.exists():
        shutil.copy2(destination, backup/destination.name)
    for path, data in [(destination, helper_data), (target, result.encode())]:
        fd, name = tempfile.mkstemp(prefix='.joint-fix-', dir=folder)
        try:
            with os.fdopen(fd, 'wb') as f:
                f.write(data)
            os.chmod(name, path.stat().st_mode if path.exists() else 0o644)
            if path == target and target.read_bytes() != original:
                raise RuntimeError('concurrent_source_change_backup_preserved')
            os.replace(name, path)
        finally:
            if Path(name).exists():
                Path(name).unlink()
    return count, backup


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--control-center', type=Path, required=True)
    p.add_argument('--helper', type=Path, default=Path(__file__).resolve().parents[2]/'control_center/joint_contract.py')
    p.add_argument('--check', action='store_true')
    a = p.parse_args()
    count, backup = apply(a.control_center.resolve(), a.helper.resolve(), a.check)
    print(f'joint input sites: {count}; check_only={a.check}; backup={backup}')
    print('No process restarted. Load the new source during the next coordinated control-center restart.')


if __name__ == '__main__':
    main()
