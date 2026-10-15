#!/usr/bin/env python3
"""Operator-accepted existing geometry for the stationary green-apple trial.

Never enables motors or sends movement. Keeps width calibration unverified.
The operator explicitly accepts the existing mounting/calibration/table basis;
this is not a new measurement or an end-to-end robot validation.
"""
import argparse
from pathlib import Path
import sys
import yaml

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--repo', type=Path, required=True)
p.add_argument('--config', type=Path, required=True)
p.add_argument('--joint-snapshot', type=Path, required=True)
p.add_argument('--tuning-file', type=Path, required=True)
p.add_argument('--accept-existing-geometry-and-cleared-scene', action='store_true', required=True)
a = p.parse_args()
root = a.repo/'SW/manipulation/manipulation_ws/src/robocup_real_manipulation'
sys.path.insert(0, str(root))
from robocup_real_manipulation.core import validate
from robocup_real_manipulation.tuning import overlay

c = yaml.safe_load(a.config.read_text())
msg = next(yaml.safe_load_all(a.joint_snapshot.read_text()))
q = dict(zip(msg['name'], msg['position']))
c['return_joints'] = [s*q[n]+o for n,s,o in
    zip(c['driver_joint_names'],c['joint_signs'],c['joint_offsets'])]
c['width_mapping_mode'] = 'nominal_trial'
c['width_mapping_verified'] = False
c['driver_width_max'] = .1
c['coarse_fixed_frame'] = 'base_link'
for name, rule in c['classes'].items():
    rule['enabled'] = name == 'green_apple'
# These are explicit operator acknowledgements of the existing startup basis.
# Numerical requirements, actual feedback and MoveIt collision gates still apply.
for flag in ('execution_enabled','commissioned','calibration_verified',
             'scene_verified','model_names_verified','gravity_frame_verified'):
    c[flag] = True
c['trial_basis'] = {
    'geometry': 'operator accepts existing URDF mounting, stored hand-eye/CAD TCP, level base and placed table',
    'scene': 'operator clears workspace except the requested green apple',
    'width': 'initial approximation; whole-range calibration NOT verified',
    'grasp_effort': 'initial trial threshold, not measured contact classification',
    'navigation': 'stationary operator-isolated base; temporary initial target',
}
validate(overlay(c,yaml.safe_load(a.tuning_file.read_text())),hardware=True)
backup = a.config.with_suffix('.before_width_trial.yaml')
if not backup.exists():
    backup.write_text(a.config.read_text())
a.config.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False))
print('청사과 정지 시험 설정 저장. 폭 보정 완료 표시 없음. 모터 명령 없음.')
