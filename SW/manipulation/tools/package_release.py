#!/usr/bin/env python3
"""Package portable source only; never connects to a robot or uploads."""
import argparse
import hashlib
import io
from pathlib import Path
import tarfile

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
manip=Path(__file__).resolve().parents[1];repo=manip.parents[1]
roots=[manip,repo/'SW/detection/head/head_detection_ws/src/robocup_detection_msgs',repo/'SW/detection/head/head_detection_ws/src/robocup_head_detection',repo/'SW/control_center']
current_validation={'integration_build.log','integration_tests.log','integration_ros_contract.log',
    'integration_final_verification.log','jetson_passive_20261015.json','jetson_integration_build.log',
    'integrated_launch_args.log','task_client_help.log','unconfigured_preflight.json'}
files=[]
for root in roots:
    for path in sorted(root.rglob('*')):
        if not path.is_file() or path.is_symlink():continue
        relative=path.relative_to(repo)
        if any(x in ('build','install','log','__pycache__','.pytest_cache') or x.endswith('.egg-info') for x in relative.parts):continue
        if path.name in ('config.json',) or path.name.startswith('config.json.bak') or path.suffix in ('.orig','.rej'):continue
        if any(x.startswith(('.manipulation-backup-', '.joint-input-backup-')) for x in relative.parts):continue
        if root.name=='control_center' and path.suffix=='.log':continue
        if 'validation' in relative.parts and path.name not in current_validation:continue
        files.append((path,relative.as_posix()))
# Matched final-gate source; deploy through expected hashes, never overwrite a
# teammate's entire running web/robot workspace with this source snapshot.
for name in ('manipulation_base_lock.py','tracer_safety_watchdog.py','tests/test_manipulation_base_lock.py'):
    path=repo/'SW/robot'/name
    files.append((path,path.relative_to(repo).as_posix()))
a.output.parent.mkdir(parents=True,exist_ok=True)
manifest=''.join(hashlib.sha256(path.read_bytes()).hexdigest()+'  '+name+'\n' for path,name in files)
with tarfile.open(a.output,'w:gz') as tar:
    for path,name in files:tar.add(path,arcname=name,recursive=False)
    entry=tarfile.TarInfo('MANIFEST.sha256');raw=manifest.encode();entry.size=len(raw);entry.mode=0o644;tar.addfile(entry,io.BytesIO(raw))
sha=hashlib.sha256(a.output.read_bytes()).hexdigest()
# Relative to the user's project cwd when invoked from that cwd.
try: label=str(a.output.relative_to(Path.cwd()))
except ValueError:label=str(a.output)
a.output.with_name(a.output.name+'.sha256').write_text(sha+'  '+label+'\n')
print(f'{a.output}: {len(files)} source/document/evidence files, {a.output.stat().st_size} bytes, sha256={sha}')
