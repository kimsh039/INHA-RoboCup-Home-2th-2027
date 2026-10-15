import hashlib, importlib.util, io, json, os, subprocess, sys, tarfile
from pathlib import Path

path=Path(__file__).resolve().parents[2]/'manipulation/tools/deploy_control_center.py'
spec=importlib.util.spec_from_file_location('deploy_manip',path)
deploy=importlib.util.module_from_spec(spec);spec.loader.exec_module(deploy)


def make_deployment(tmp_path):
    repo=tmp_path/'INHA-RoboCup-Home-2th-2027'
    old=repo/'SW/control_center/server.py';old.parent.mkdir(parents=True);old.write_bytes(b'old')
    runtime=tmp_path/'robot_setup/manipulation-runtime/releases/20261015-web-integration/install'
    runtime.mkdir(parents=True);(runtime/'local_setup.bash').write_text('')
    files={'SW/control_center/server.py':b'new', 'SW/control_center/new.py':b'helper'}
    manifest={'files':{name:dict(before=hashlib.sha256(b'old').hexdigest() if name.endswith('server.py') else None,
            after=hashlib.sha256(data).hexdigest()) for name,data in files.items()}}
    buf=io.BytesIO()
    with tarfile.open(fileobj=buf,mode='w:gz') as archive:
        for name,data in [('manifest.json',json.dumps(manifest).encode()),('rollback.py',deploy.ROLLBACK.encode()),
                          *[('payload/'+name,data) for name,data in files.items()]]:
            info=tarfile.TarInfo(name);info.size=len(data);archive.addfile(info,io.BytesIO(data))
    return repo,old,buf.getvalue(),dict(os.environ,HOME=str(tmp_path))


def test_apply_and_rollback_restore_exact_source(tmp_path):
    repo,old,payload,env=make_deployment(tmp_path)
    result=subprocess.run([sys.executable,'-c',deploy.REMOTE],input=payload,capture_output=True,env=env)
    assert result.returncode==0,result.stderr
    backup=Path(json.loads(result.stdout)['backup'])
    assert old.read_bytes()==b'new'
    result=subprocess.run([sys.executable,str(backup/'rollback.py')],capture_output=True,env=env)
    assert result.returncode==0,result.stderr
    assert old.read_bytes()==b'old' and not (old.parent/'new.py').exists()
    assert not (tmp_path/'robot_setup/manipulation-runtime/current').exists()


def test_concurrent_change_aborts_before_any_write(tmp_path):
    repo,old,payload,env=make_deployment(tmp_path);old.write_bytes(b'other developer')
    result=subprocess.run([sys.executable,'-c',deploy.REMOTE],input=payload,capture_output=True,env=env)
    assert result.returncode!=0 and b'CONCURRENT_CHANGE' in result.stderr
    assert old.read_bytes()==b'other developer' and not list(old.parent.glob('.manipulation-backup-*'))


def test_rollback_preserves_later_developer_edits(tmp_path):
    repo,old,payload,env=make_deployment(tmp_path)
    result=subprocess.run([sys.executable,'-c',deploy.REMOTE],input=payload,capture_output=True,env=env)
    backup=Path(json.loads(result.stdout)['backup']);old.write_bytes(b'later edit')
    result=subprocess.run([sys.executable,str(backup/'rollback.py')],capture_output=True,env=env)
    assert result.returncode!=0 and old.read_bytes()==b'later edit'
