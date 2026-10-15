#!/usr/bin/env bash
# Stationary, operator-isolated base. Real wrist/arm/MoveIt; temporary coarse only.
set -e
PHASE1_REPO=/home/sparo/INHA-RoboCup-Home-2th-2027
PHASE1_CFG="$HOME/robot_config/robot.measured.yaml"
PHASE1_LOG="$HOME/.ros/robocup_real_manipulation/phase1_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$PHASE1_LOG"
source /opt/ros/humble/setup.bash
source "$HOME/piper_ros/install/setup.bash"
source "$PHASE1_REPO/SW/detection/head/head_detection_ws/install/setup.bash"
source "$PHASE1_REPO/SW/manipulation/manipulation_ws/install/setup.bash"
export PYTHONPATH="$HOME/robot_setup/piper-python${PYTHONPATH:+:$PYTHONPATH}"
export PHASE1_REPO PHASE1_CFG PHASE1_LOG
PHASE1_PIPER_PID=''; PHASE1_LAUNCH_PID=''; PHASE1_POINT_PID=''
phase1_cleanup() {
    if [ -n "$PHASE1_LAUNCH_PID" ]; then
        timeout 4s ros2 service call /manipulation/stop std_srvs/srv/Trigger '{}' > /dev/null 2>&1 || true
    fi
    for phase1_pid in "$PHASE1_POINT_PID" "$PHASE1_LAUNCH_PID" "$PHASE1_PIPER_PID"; do
        if [ -n "$phase1_pid" ]; then kill -- "-$phase1_pid" 2>/dev/null || true; fi
    done
    echo "실행 로그: $PHASE1_LOG"
}
trap phase1_cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
# Restart only this robot's previous manipulation stack, not unrelated ROS apps.
# A prior backend must acknowledge stop before its processes are terminated.
python3 - <<'PY_RESTART'
import os, signal, subprocess, time
from pathlib import Path
repo=os.environ['PHASE1_REPO']; log=os.environ['PHASE1_LOG']
processes={}
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit(): continue
    try:
        status=(entry/'status').read_text()
        uid=int(next(x for x in status.splitlines() if x.startswith('Uid:')).split()[1])
        if uid!=os.getuid(): continue
        parent=int(next(x for x in status.splitlines() if x.startswith('PPid:')).split()[1])
        cmd=(entry/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        env=dict(x.split(b'=',1) for x in (entry/'environ').read_bytes().split(b'\0') if b'=' in x)
        token=(entry/'stat').read_text().rsplit(')',1)[1].split()[19]
        processes[int(entry.name)]=(parent,cmd,env,token)
    except (OSError,StopIteration,ValueError,IndexError): continue
ancestors=set(); pid=os.getpid()
while pid and pid not in ancestors:
    ancestors.add(pid); pid=processes.get(pid,(0,))[0]
roots=set(); backend=False
for pid,(_,cmd,env,_) in processes.items():
    if pid in ancestors: continue
    launch=' launch robocup_real_manipulation ' in cmd and any(x in cmd for x in ('phase1.launch.py','bringup.launch.py'))
    module='robocup_real_manipulation/' in cmd and any('/'+x in cmd for x in ('piper_bridge','coordinator','phase1_pick','wrist_frontend','head_input_adapter'))
    piper='/piper/piper_single_ctrl' in cmd or ' run piper piper_single_ctrl' in cmd
    owned=env.get(b'PHASE1_REPO')==repo.encode() and bool(env.get(b'PHASE1_LOG')) and env[b'PHASE1_LOG']!=log.encode()
    if launch or module or piper or owned: roots.add(pid)
    if module and '/piper_bridge' in cmd: backend=True
if backend:
    try:
        reply=subprocess.run(['ros2','service','call','/manipulation/stop','std_srvs/srv/Trigger','{}'],capture_output=True,text=True,timeout=8)
    except subprocess.TimeoutExpired:
        raise SystemExit('이전 실행기의 정지 응답이 없어 새 동작을 시작하지 않습니다.')
    if reply.returncode or not any(x in reply.stdout for x in ('success=True','success: true','success=true')):
        raise SystemExit('이전 실행기의 정지가 확인되지 않아 새 동작을 시작하지 않습니다.')
selected=set(roots)
while True:
    children={p for p,(parent,_,_,_) in processes.items() if parent in selected and p not in ancestors}
    newer=selected|children
    if newer==selected: break
    selected=newer
def alive(pid):
    try:
        state=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
        return state[0]!='Z' and state[19]==processes[pid][3]
    except (OSError,IndexError): return False
for sig,seconds in ((signal.SIGINT,3),(signal.SIGTERM,3)):
    for pid in selected:
        if alive(pid):
            try: os.kill(pid,sig)
            except ProcessLookupError: pass
    deadline=time.monotonic()+seconds
    while any(alive(p) for p in selected) and time.monotonic()<deadline: time.sleep(.1)
remaining=[p for p in selected if alive(p)]
# Only a reader/control process owned by this previous runner can survive its
# ROS shutdown because of SDK threads. The backend stop above must succeed if
# an old bridge exists; other surviving processes are never force-killed.
for pid in remaining:
    _,cmd,env,_=processes[pid]
    owned=env.get(b'PHASE1_REPO')==repo.encode() and bool(env.get(b'PHASE1_LOG')) and env[b'PHASE1_LOG']!=log.encode()
    if owned and '/piper/piper_single_ctrl ' in cmd and '-p can_port:=can_arm' in cmd and alive(pid):
        os.kill(pid,signal.SIGKILL)
        print('종료되지 않은 이전 실행기의 Piper SDK 프로세스 정리:',pid)
deadline=time.monotonic()+2.
while any(alive(p) for p in remaining) and time.monotonic()<deadline: time.sleep(.1)
remaining=[p for p in selected if alive(p)]
if remaining: raise SystemExit('이전 프로세스가 종료되지 않아 중복 구동을 중단합니다: '+str(remaining))
if selected: print('이전 매니퓰레이션 프로세스 정리 완료')
PY_RESTART
PHASE1_NODES="$(ros2 node list --no-daemon --spin-time 2)"
# PY_RESTART verified real old processes have exited. DDS may still advertise
# their names after SIGKILL; never mistake that stale graph for a live driver.
setsid ros2 run piper piper_single_ctrl --ros-args -p can_port:=can_arm \
        -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=1 \
        -p rx_filter:=true -p publish_hz:=100.0 -p publish_end_pose:=false \
        > "$PHASE1_LOG/piper.log" 2>&1 &
PHASE1_PIPER_PID=$!
# Actual startup feedback, not synthetic joints or a new manual measurement.
if ! timeout 30s ros2 topic echo /joint_states_feedback sensor_msgs/msg/JointState \
    --qos-reliability best_effort --once > "$PHASE1_LOG/start_joints.yaml"; then
    echo '실제 팔 피드백을 받지 못했습니다. Piper 드라이버 출력:' >&2
    if [ -f "$PHASE1_LOG/piper.log" ]; then tail -n 30 "$PHASE1_LOG/piper.log"; fi
    exit 1
fi
python3 - <<'PY_PREP'
import importlib, importlib.util, os, json, shutil, time
from pathlib import Path
import yaml
repo=Path(os.environ['PHASE1_REPO']); cfg_path=Path(os.environ['PHASE1_CFG'])
source=repo/'SW/manipulation/manipulation_ws/src/robocup_real_manipulation/robocup_real_manipulation/core.py'
installed=Path(importlib.util.find_spec('robocup_real_manipulation.core').origin).resolve()
guard="""        width_trial = cfg.get('width_mapping_mode', 'measured') == 'nominal_trial'
        require(cfg.get('width_mapping_verified') is True or width_trial,
                'NOT_VERIFIED:width_mapping_verified')
        if width_trial:
            require(cfg['velocity_scale'] <= .05 and cfg['acceleration_scale'] <= .05,
                    'WIDTH_TRIAL_SPEED')
            enabled = [n for n, r in cfg['classes'].items() if r['enabled']]
            require(enabled == ['green_apple'], 'WIDTH_TRIAL_GREEN_APPLE_ONLY')
            require(.07 <= cfg['classes']['green_apple']['close_m'] <
                    cfg['classes']['green_apple']['open_m'] <= .10,
                    'WIDTH_TRIAL_APERTURE')
"""
for path in {source.resolve(), installed}:
    text=path.read_text()
    if 'WIDTH_TRIAL_GREEN_APPLE_ONLY' in text: continue
    old="'gravity_frame_verified','width_mapping_verified'):"
    anchor="        for key in ('joint_signs','joint_offsets','return_joints',"
    if text.count(old)!=1 or text.count(anchor)!=1:
        raise SystemExit('패키지 버전이 달라 자동 수정 없이 중단합니다.')
    backup=path.with_suffix('.py.before_width_trial')
    if not backup.exists(): backup.write_text(text)
    text=text.replace(old,"'gravity_frame_verified'):",1)
    path.write_text(text.replace(anchor,guard+anchor,1))
# Use the existing collision-checked transit stage for the persistent initial pose.
old_pose="""            if c['observation_joints'] is not None:
                self.motion.joint_move(c['observation_joints'],self.guard)"""
new_pose="""            if c['observation_joints'] is not None:
                self.status(seq,'RUNNING','MOVE_TO_INITIAL_POSE')
                with self.mutex:
                    values=dict(zip(self.joint.name,self.joint.position))
                require(all(math.isfinite(values[n]) for n in c['joint_names']),
                        'INVALID_INITIAL_POSE_FEEDBACK')
                if any(abs(values[n]-q)>c['goal_tolerance_rad'] for n,q in
                       zip(c['joint_names'],c['observation_joints'])):
                    self.motion.joint_move(c['observation_joints'],self.guard)"""
for core_path in {source.resolve(),installed}:
    path=core_path.with_name('node.py'); text=path.read_text()
    if 'INVALID_INITIAL_POSE_FEEDBACK' in text: continue
    if text.count(old_pose)!=1: raise SystemExit('초기 자세 연결부 버전이 달라 동작 없이 중단합니다.')
    backup=path.with_suffix('.py.before_initial_pose')
    if not backup.exists(): backup.write_text(text)
    path.write_text(text.replace(old_pose,new_pose,1))
c=yaml.safe_load(cfg_path.read_text())
# A failed initial plan executed no grasp. Recover only recorded EMPTY +
# confirmed-stop pre-observation failures; retain completed IDs and backups.
journal=Path(c['journal_path']).expanduser()
if journal.exists():
    saved=json.loads(journal.read_text()); locked=saved.get('locked_task','')
    if locked:
        rows=[json.loads(x) for x in Path(c['log_path']).expanduser().read_text().splitlines() if x.strip()]
        rows=[r for r in rows if r.get('task_id')==locked and 'outcome' in r]
        final=rows[-1] if rows else {}
        safe=(bool(rows) and all(r.get('stage') in ('WAIT_FOR_APPROACH','MOVE_TO_OBSERVATION') and r.get('holding')=='EMPTY' for r in rows)
              and final.get('outcome')=='FAILED' and final.get('reason') in ('PLAN_FAILED','ACTION_NOT_SUCCEEDED','GOAL_REJECTED') and final.get('stop_confirmed') is True)
        if not safe: raise SystemExit('정지 확인된 초기 계획 실패와 다른 복구 잠금입니다. 자동 해제 없이 중단합니다.')
        shutil.copy2(journal,journal.with_name(journal.name+'.backup_'+str(time.time_ns())))
        saved['locked_task']=''; tmp=journal.with_name(journal.name+'.recovery.tmp')
        tmp.write_text(json.dumps(saved)); tmp.replace(journal)
        print('정지/미보유 확인된 초기 계획 실패 잠금 복구:',locked)
# [TUNE:INITIAL_POSE] Keep one initial/return pose across future SSH sessions.
# This actual driver pose was supplied and accepted by the operator in this chat.
home_path=cfg_path.parent/'phase1.home.yaml'
if not home_path.exists():
    raw=[1.541299508,0.,0.,.013850536,.486094504,.029986236]
    home={'joint_names':c['joint_names'],
          'positions':[s*q+o for s,q,o in zip(c['joint_signs'],raw,c['joint_offsets'])],
          'source':'operator-confirmed original pose, actual ROS feedback stamp 1791909997'}
    home_path.write_text('# [TUNE:INITIAL_POSE] URDF joint positions, rad.\n'+yaml.safe_dump(home,allow_unicode=True,sort_keys=False))
home=yaml.safe_load(home_path.read_text())
if home['joint_names']!=c['joint_names']: raise SystemExit('저장된 초기 자세의 관절 이름이 다릅니다.')
c['return_joints']=home['positions']; c['observation_joints']=home['positions']
c.update(width_mapping_mode='nominal_trial',width_mapping_verified=False,
         driver_width_max=.1,coarse_fixed_frame='base_link',stage_timeout_s=600.,
         startup_bounds_tolerance_rad=.07,feedback_age_s=1.5)
for name,r in c['classes'].items(): r['enabled']=name=='green_apple'
# Operator acceptance of the already supplied URDF mounting, stored calibration,
# level stationary base, cleared scene and placed table. Not a new calibration.
for key in ('execution_enabled','commissioned','calibration_verified',
            'scene_verified','model_names_verified','gravity_frame_verified'):
    c[key]=True
c['trial_basis']={'geometry':'operator accepts existing URDF/hand-eye/CAD TCP and placed table',
                  'scene':'stationary isolated base, cleared workspace except green apple',
        'width':'initial approximation; whole-range calibration unverified',
                  'contact':'initial heuristic threshold, not measured contact classification'}
from robocup_real_manipulation import core
importlib.reload(core)
from robocup_real_manipulation.tuning import overlay
tuning=yaml.safe_load((cfg_path.parent/'phase1.tuning.yaml').read_text())
core.validate(overlay(c,tuning),hardware=True)
backup=cfg_path.with_suffix('.before_width_trial.yaml')
if not backup.exists(): backup.write_text(cfg_path.read_text())
cfg_path.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False))
print('청사과 정지 시험 시작: 폭 근사값 사용, MoveIt 충돌 검사 유지.')
PY_PREP
PHASE1_CAMERA=true; PHASE1_MOUNT=true
if echo "$PHASE1_NODES" | grep -Fxq /wrist/wrist; then PHASE1_CAMERA=false; fi
if echo "$PHASE1_NODES" | grep -Fxq /robot_sensor_calibration_tf; then PHASE1_MOUNT=false; fi
setsid ros2 launch robocup_real_manipulation phase1.launch.py \
    repo:="$PHASE1_REPO" config:="$PHASE1_CFG" \
    vision_config:="$HOME/robot_config/wrist.yaml" vision_python:="$HOME/venvs/sam21/bin/python" \
    tuning_file:="$HOME/robot_config/phase1.tuning.yaml" start_driver:=false \
    start_camera:="$PHASE1_CAMERA" start_mount_tf:="$PHASE1_MOUNT" start_head_adapter:=false \
    > "$PHASE1_LOG/manipulation.log" 2>&1 &
PHASE1_LAUNCH_PID=$!
setsid python3 - <<'PY_POINT' > "$PHASE1_LOG/initial_point.log" 2>&1 &
import os, yaml, rclpy
from pathlib import Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from robocup_real_manipulation_msgs.msg import Target
c=yaml.safe_load(Path(os.environ['PHASE1_CFG']).read_text())
rclpy.init(); n=rclpy.create_node('temporary_initial_green_apple')
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
pub=n.create_publisher(Target,'/manipulation/coarse_target',qos)
def send():
    m=Target(); m.header.stamp=n.get_clock().now().to_msg(); m.header.frame_id='base_link'
    m.task_id='temporary-stationary-green-apple'; m.class_id=c['classes']['green_apple']['class_id']
    m.coarse_position.x=c['base_arm_transform']['xyz'][0]+.30
    m.coarse_position.y=c['base_arm_transform']['xyz'][1]
    m.coarse_position.z=c['table_z_m']+c['classes']['green_apple']['center_height_m']
    pub.publish(m)
n.create_timer(.5,send); rclpy.spin(n)
PY_POINT
PHASE1_POINT_PID=$!
if ! timeout 60s ros2 service call /enable_srv piper_msgs/srv/Enable '{enable_request: true}' \
    > "$PHASE1_LOG/enable.log" 2>&1; then
    echo 'Piper 활성화 서비스 연결/응답 실패. 실제 기동 로그:' >&2
    cat "$PHASE1_LOG/enable.log"
    if [ -f "$PHASE1_LOG/piper.log" ]; then cat "$PHASE1_LOG/piper.log"; fi
    exit 1
fi
if ! grep -Eq 'enable_response[=:][[:space:]]*[Tt]rue' "$PHASE1_LOG/enable.log"; then
    cat "$PHASE1_LOG/enable.log"
    exit 1
fi
ros2 run robocup_real_manipulation phase1_pick green_apple \
    --config "$PHASE1_CFG" --already-docked --base-drive-isolated --input-durable --timeout 1800
