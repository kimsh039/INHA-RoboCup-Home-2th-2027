#!/usr/bin/env python3
"""Deploy only reviewed files with expected hashes; preserve concurrent edits.

No process restart, motor command, config flags, sensor data or log copy.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile

REMOTE = r'''
import hashlib, io, json, os
from pathlib import Path
import sys, tarfile, tempfile
from datetime import datetime
repo=Path.home()/'INHA-RoboCup-Home-2th-2027'
archive=tarfile.open(fileobj=io.BytesIO(sys.stdin.buffer.read()),mode='r:gz')
manifest=json.load(archive.extractfile('manifest.json'))
for name,item in manifest['files'].items():
    path=repo/name
    actual=hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None
    if actual!=item['before']: raise SystemExit('CONCURRENT_CHANGE: '+name)
runtime=Path.home()/'robot_setup/manipulation-runtime/releases/20261015-web-integration'
if not (runtime/'install/local_setup.bash').is_file(): raise SystemExit('RUNTIME_NOT_BUILT')
backup=repo/'SW/control_center'/('.manipulation-backup-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
backup.mkdir(mode=0o700)
pointer=runtime.parent.parent/'current'
manifest['old_pointer']=os.readlink(pointer) if pointer.is_symlink() else None
if pointer.exists() and not pointer.is_symlink():raise SystemExit('CURRENT_IS_NOT_SYMLINK')
for name,item in manifest['files'].items():
    path=repo/name
    if path.exists():
        saved=backup/name;saved.parent.mkdir(parents=True,exist_ok=True);saved.write_bytes(path.read_bytes())
(backup/'manifest.json').write_text(json.dumps(manifest,indent=2))
(backup/'rollback.py').write_bytes(archive.extractfile('rollback.py').read())
for name,item in manifest['files'].items():
    payload=archive.extractfile('payload/'+name).read()
    if hashlib.sha256(payload).hexdigest()!=item['after']: raise SystemExit('PAYLOAD_HASH: '+name)
for name in manifest['files']:
    path=repo/name;path.parent.mkdir(parents=True,exist_ok=True)
    mode=path.stat().st_mode & 0o777 if path.exists() else 0o644
    fd,tmp=tempfile.mkstemp(dir=path.parent,prefix='.manipulation-')
    with os.fdopen(fd,'wb') as f:f.write(archive.extractfile('payload/'+name).read());f.flush();os.fsync(f.fileno())
    os.chmod(tmp,mode);os.replace(tmp,path)
temporary=pointer.with_name('.current-'+str(os.getpid()))
temporary.symlink_to(runtime);os.replace(temporary,pointer)
print(json.dumps(dict(backup=str(backup),files=len(manifest['files']),runtime=str(runtime),restart_required=True)))
'''

ROLLBACK = r'''
"""Restore source/pointer only; never stop robot processes or release a base lock."""
import hashlib,json,os,tempfile
from pathlib import Path
backup=Path(__file__).resolve().parent
repo=Path.home()/'INHA-RoboCup-Home-2th-2027'
m=json.loads((backup/'manifest.json').read_text())
pointer=Path.home()/'robot_setup/manipulation-runtime/current'
expected=Path.home()/'robot_setup/manipulation-runtime/releases/20261015-web-integration'
if not pointer.is_symlink() or pointer.resolve()!=expected:raise SystemExit('Runtime pointer has changed; restore it manually.')
for name,item in m['files'].items():
    path=repo/name
    if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest()!=item['after']:
        raise SystemExit('Rollback stopped: later edit in '+name)
    if item['before'] and hashlib.sha256((backup/name).read_bytes()).hexdigest()!=item['before']:
        raise SystemExit('Backup hash mismatch: '+name)
for name,item in m['files'].items():
    path=repo/name
    if item['before'] is None:path.unlink()
    else:
        data=(backup/name).read_bytes()
        if hashlib.sha256(data).hexdigest()!=item['before']:raise SystemExit('Backup hash mismatch: '+name)
        mode=path.stat().st_mode&0o777
        fd,tmp=tempfile.mkstemp(dir=path.parent,prefix='.rollback-')
        with os.fdopen(fd,'wb') as f:f.write(data)
        os.chmod(tmp,mode);os.replace(tmp,path)
if m['old_pointer']:
    temp=pointer.with_name('.rollback-current');temp.symlink_to(m['old_pointer']);os.replace(temp,pointer)
else:pointer.unlink()
print('Source restored. Restart the web server only after confirming no arm task is active. Base locks/journals were preserved.')
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',required=True,type=Path,help='Read-only Jetson source baseline directory containing SW/')
    parser.add_argument('--host',default='jetson')
    parser.add_argument('--apply',action='store_true')
    args=parser.parse_args()
    repo=Path(__file__).resolve().parents[3]
    names=['SW/control_center/'+x for x in ('server.py','ros_bridge.py','run-robot.sh',
        'manipulation_control.py','manipulation-monitor.sh','MANIPULATION.md','static/index.html','static/app.js')]
    names+=['SW/robot/manipulation_base_lock.py','SW/robot/tracer_safety_watchdog.py']
    manifest={'files':{}}
    for name in names:
        old=args.baseline/name
        manifest['files'][name]={'before':hashlib.sha256(old.read_bytes()).hexdigest() if old.exists() else None,
                                'after':hashlib.sha256((repo/name).read_bytes()).hexdigest()}
    if not args.apply:
        print(json.dumps(manifest,indent=2));return
    buf=io.BytesIO()
    with tarfile.open(fileobj=buf,mode='w:gz') as archive:
        for name in names:archive.add(repo/name,arcname='payload/'+name)
        for name,data in [('manifest.json',json.dumps(manifest).encode()),('rollback.py',ROLLBACK.encode())]:
            info=tarfile.TarInfo(name);info.size=len(data);archive.addfile(info,io.BytesIO(data))
    subprocess.run(['ssh',args.host,'python3 -c '+shlex.quote(REMOTE)],input=buf.getvalue(),check=True)

if __name__=='__main__':main()
