"""Run real Gazebo captures and held-out evaluation, sequentially in private partitions."""
import argparse,csv,gzip,hashlib,json,os,signal,subprocess,sys,time
from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from kinematics import fk
ROOT=Path(__file__).resolve().parents[2]
RUN=Path(__file__).resolve().parent
PYTHON=ROOT/'.venv/bin/python'
URDF=ROOT/'robot/simulation/robot_description/robocup.urdf'
HEAD=ROOT/'calibration_data/sim/head_mid360_01'
WRIST=ROOT/'calibration_data/sim/wrist_handeye_02'
ZERO={f'piper_joint{i}':0. for i in range(1,7)}
ENV=os.environ.copy();ENV.update(GZ_IP='127.0.0.1',GZ_PARTITION='robocup_auto_20261006')
ENV.pop('PYTHONPATH',None);ENV.pop('PYTHONHOME',None)
COMMANDS=RUN/'commands.jsonl'
STATE=RUN/'progress.json'

def write(path,value):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,indent=2));tmp.replace(path)

def command(args,timeout=40,check=True):
 args=[str(s) for s in args]
 with COMMANDS.open('a') as f:f.write(json.dumps({'time_kst':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),'partition':ENV['GZ_PARTITION'],'argv':args})+'\n')
 r=subprocess.run(args,cwd=ROOT,env=ENV,text=True,capture_output=True,timeout=timeout)
 if r.returncode and check:raise RuntimeError('Command failed: '+json.dumps(args)+'\n'+r.stdout+'\n'+r.stderr)
 return r

def tool(name,*args,timeout=40,check=True):return command([PYTHON,ROOT/name,*args],timeout,check)
def pose(text):
 v=np.fromstring(text or '0 0 0 0 0 0',sep=' ');t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',v[3:]).as_matrix();t[:3,3]=v[:3];return t

def pose_command(name,t):
 q=Rotation.from_matrix(t[:3,:3]).as_quat();p=t[:3,3]
 req=f'name: "{name}" position {{ x: {p[0]:.17g} y: {p[1]:.17g} z: {p[2]:.17g} }} orientation {{ x: {q[0]:.17g} y: {q[1]:.17g} z: {q[2]:.17g} w: {q[3]:.17g} }}'
 r=command(['gz','service','-s','/world/robocup_motion/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','4000','--req',req],timeout=9)
 if 'true' not in r.stdout:raise RuntimeError('Pose command was not accepted: '+r.stdout+r.stderr)
 return req

def sample_status(stage,**kw):
 value={'stage':stage,'time_kst':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),**kw};write(STATE,value);print(json.dumps(value),flush=True)

def create_plan():
 head=[]
 head.append({'split':'train','sample_id':'001','roll_deg':0,'pitch_deg':0,'depth_command_m':2.,'existing_user_observation':True})
 for roll in [-24,-12,0,12,24]:
  for pitch in [-24,-12,0,12,24]:
   if roll==0 and pitch==0:continue
   i=len(head);head.append({'split':'train','sample_id':f'{i+1:03d}','roll_deg':roll,'pitch_deg':pitch,'depth_command_m':1.0+.04*((i*3)%8),'existing_user_observation':False})
 for i,(roll,pitch) in enumerate([(-18,6),(-6,18),(18,-6),(6,-18),(18,18),(-18,-18),(-6,-6),(6,6),(-18,18),(18,-18)],1):
  head.append({'split':'holdout','sample_id':f'{i:03d}','roll_deg':roll,'pitch_deg':pitch,'depth_command_m':1.03+.03*(i%7),'existing_user_observation':False})
 wrist=[]
 for filename,split in [('wrist_train.csv','train'),('wrist_holdout10.csv','holdout')]:
  for row in csv.DictReader((ROOT/'cli/poses'/filename).open()):wrist.append({'split':split,'sample_id':row['sample_id'],'joint_targets_rad':[float(row[f'joint{i}_rad']) for i in range(1,7)],'table':str(ROOT/'cli/poses'/filename)})
 head_rot=[Rotation.from_euler('xyz',[r['roll_deg'],r['pitch_deg'],0],degrees=True) for r in head]
 hdist=[(float(np.degrees((a.inv()*b).magnitude())),i,j) for i,a in enumerate(head_rot) for j,b in enumerate(head_rot) if j>i]
 wposes=[fk(URDF,{f'piper_joint{i}':r['joint_targets_rad'][i-1] for i in range(1,7)},'piper_base_link','piper_link6') for r in wrist]
 wdist=[]
 for i,a in enumerate(wrist):
  for j,b in enumerate(wrist):
   if j<=i:continue
   angle=float(np.degrees(Rotation.from_matrix(wposes[i][:3,:3].T@wposes[j][:3,:3]).magnitude()));dist=float(np.linalg.norm(wposes[i][:3,3]-wposes[j][:3,3]));wdist.append((float(np.linalg.norm(np.array(a['joint_targets_rad'])-b['joint_targets_rad'])),angle,dist,i,j))
 if min(x[0] for x in hdist)<1e-4 or min(x[0] for x in wdist)<1e-8 or any(x[1]<1e-5 and x[2]<1e-8 for x in wdist):raise RuntimeError('Duplicate calibration command poses')
 report={'head_count':len(head),'wrist_count':len(wrist),'head_duplicate_orientations':False,'head_minimum_orientation_separation_deg':min(x[0] for x in hdist),'head_train_holdout_minimum_orientation_separation_deg':min(x[0] for x in hdist if head[x[1]]['split']!=head[x[2]]['split']),'wrist_duplicate_joint_commands':False,'wrist_duplicate_flange_poses':False,'wrist_minimum_joint_distance_rad':min(x[0] for x in wdist),'wrist_train_holdout_minimum_joint_distance_rad':min(x[0] for x in wdist if wrist[x[3]]['split']!=wrist[x[4]]['split']),'wrist_minimum_flange_orientation_separation_deg':min(x[1] for x in wdist),'actual_measured_pose_overlap_checked_after_capture':True,'ground_truth_usage':'SDF/CAD is used only for fixture placement and final evaluation; solver inputs are image PnP, measured joints and measured clouds.'}
 write(RUN/'angle_overlap_plan.json',{'checks':report,'head':head,'wrist':wrist});return head,wrist

class Server:
 def __init__(self,world,label):self.world=world;self.label=label;self.processes=[];self.handles=[]
 def __enter__(self):
  for mode,name in [('-s','server'),('-g','gui')]:
   args=['gz','sim',mode,'--render-engine-api-backend','metal']
   if mode=='-s':args+=['-r',str(self.world)]
   log=(RUN/f'{self.label}_{name}.log').open('a');self.handles.append(log)
   self.processes.append(subprocess.Popen(args,cwd=ROOT,env=ENV,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
  for i in range(12):
   if self.processes[0].poll() is not None:raise RuntimeError('Gazebo server exited; see '+str(RUN/f'{self.label}_server.log'))
   try:
    req='pause: false';r=command(['gz','service','-s','/world/robocup_motion/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','1000','--req',req],timeout=4,check=False)
    if 'true' in r.stdout:time.sleep(2);return self
   except subprocess.TimeoutExpired:pass
   time.sleep(.5)
  raise RuntimeError('Gazebo server readiness timeout')
 def __exit__(self,*args):
  for p in reversed(self.processes):
   try:os.killpg(p.pid,signal.SIGINT)
   except ProcessLookupError:pass
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:
    try:os.killpg(p.pid,signal.SIGKILL)
    except ProcessLookupError:pass
    p.wait()
  for log in self.handles:log.close()

def capture(session,camera,split):
 dataset=session/('head_samples.json' if camera=='head' else 'dataset.json')
 r=tool('capture_pose.py','--camera',camera,'--partition',ENV['GZ_PARTITION'],'--dataset',dataset,'--split',split,'--timeout','22',timeout=35)
 records=json.loads(dataset.read_text());sample=records['samples'][-1];return dataset,sample,r.stdout

def raw_cloud(path):
 r=command(['gz','topic','-e','-n','1','--json-output','-t','/robocup/mid360s/scan/points'],timeout=18)
 v=json.loads(r.stdout)
 if not v.get('data'):raise RuntimeError('Empty point cloud message')
 with gzip.open(path,'wt') as f:json.dump(v,f)
 return {'stamp':v.get('header',{}).get('stamp'),'frame':v.get('header',{}).get('data'),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}

def eval_error(estimate,truth):
 t=np.array(estimate);return {'translation_mm':float(np.linalg.norm(t[:3,3]-truth[:3,3])*1000),'rotation_deg':float(np.degrees(Rotation.from_matrix(truth[:3,:3].T@t[:3,:3]).magnitude()))}

def truth_head(world):
 model=ET.parse(world).getroot().find("world/model[@name='robocup']");base=model.find("link[@name='base_link']")
 lidar=pose(base.find("sensor[@name='livox_mid360s']").findtext('pose'));camera=pose(base.find("sensor[@name='d435f_color']").findtext('pose'))
 optical=np.eye(4);optical[:3,:3]=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
 return np.linalg.inv(camera@optical)@lidar

def measured_overlap(samples):
 checks=[]
 for i,a in enumerate(samples):
  for j,b in enumerate(samples):
   if j<=i:continue
   ga=np.array(a['T_base_flange']);gb=np.array(b['T_base_flange']);q=np.array([a['measured_joint_positions'][f'piper_joint{k}']-b['measured_joint_positions'][f'piper_joint{k}'] for k in range(1,7)])
   checks.append({'a':a['id'],'b':b['id'],'cross_split':a['split']!=b['split'],'joint_distance_rad':float(np.linalg.norm(q)),'flange_translation_mm':float(np.linalg.norm(ga[:3,3]-gb[:3,3])*1000),'flange_rotation_deg':float(np.degrees(Rotation.from_matrix(ga[:3,:3].T@gb[:3,:3]).magnitude()))})
 duplicates=[r for r in checks if r['joint_distance_rad']<.002 or (r['flange_translation_mm']<.05 and r['flange_rotation_deg']<.05)]
 return {'count':len(samples),'duplicates':duplicates,'minimum_joint_distance_rad':min(r['joint_distance_rad'] for r in checks),'minimum_flange_rotation_deg':min(r['flange_rotation_deg'] for r in checks),'closest_train_holdout':min((r for r in checks if r['cross_split']),key=lambda r:r['joint_distance_rad'])}

def run_head(plan):
 world=HEAD/'config/head_mid.world.sdf';write(HEAD/'config/automated_pose_plan.json',plan)
 tree=ET.parse(world);model=tree.getroot().find("world/model[@name='robocup']");base=pose(model.findtext('pose'))
 camera=base@fk(URDF,ZERO,'base_link','camera_optical_frame');flip=np.diag([1.,-1.,-1.]);done=[]
 with Server(world,'head'):
  for n,row in enumerate(plan):
   if row['existing_user_observation']:
    path=HEAD/'train/001';(path/'capture_id.txt').write_text('0\n') if not (path/'capture_id.txt').exists() else None
    done.append({'sample_id':'001','split':'train','source':'existing_user_capture','image_id':0});continue
   folder=HEAD/row['split']/row['sample_id']
   if (folder/'head_plane.json').exists():done.append({'sample_id':row['sample_id'],'split':row['split'],'source':'previous_automated_capture'});continue
   folder.mkdir(parents=True,exist_ok=True)
   ct=np.eye(4);ct[:3,:3]=flip@Rotation.from_euler('xyz',[row['roll_deg'],row['pitch_deg'],0],degrees=True).as_matrix();ct[:3,3]=[.05*np.sin(n*1.9),.045*np.cos(n*1.3),row['depth_command_m']]
   target=camera@ct;request=pose_command('calibration_board',target);time.sleep(.7)
   dataset,sample,out=capture(HEAD,'head',row['split']);(folder/'capture_id.txt').write_text(str(sample['id'])+'\n')
   raw=raw_cloud(folder/'points_raw.json.gz')
   tool('calibration_workflow.py','decode-cloud','--input',folder/'points_raw.json.gz','--output',folder/'xyz.npz')
   tool('calibration_workflow.py','tag-plane','--dataset',dataset,'--sample-id',sample['id'],'--tag-to-plane',HEAD/'config/tag_to_plane.json','--plane-id',('board' if row['split']=='train' else 'holdout_board')+row['sample_id'],'--output',folder/'head_plane.json')
   write(folder/'capture_record.json',{'requested_pose':row,'T_world_board_command':target.tolist(),'set_pose_request':request,'image_id':sample['id'],'image_sim_time':sample['image_sim_time'],'cloud':raw,'board_stationary_during_pair_capture':True,'partition':ENV['GZ_PARTITION'],'world_sha256':hashlib.sha256(world.read_bytes()).hexdigest(),'ground_truth_not_used_in_solver':True})
   done.append({'sample_id':row['sample_id'],'split':row['split'],'image_id':sample['id']});sample_status('head_capture',complete=len(done),target=len(plan),sample=row['sample_id'],split=row['split'])
 sample_status('head_solve')
 results=HEAD/'results/automated_01';results.mkdir(parents=True,exist_ok=True)
 for split,ref in [('train',None),('holdout',results/'head_mid360.json')]:
  args=['auto-board-pairs','--samples-root',HEAD/split,'--output-dir',results/('auto_board_'+split)]
  if ref:args+=['--reference',ref]
  tool('calibration_workflow.py',*args,timeout=180)
  if split=='train':tool('calibration_workflow.py','planes','--train',results/'auto_board_train/pairs.csv','--parent','camera_optical_frame','--child','livox_frame','--output',results/'head_mid360.json')
 check=tool('calibration_workflow.py','validate-planes','--result',results/'head_mid360.json','--validation',results/'auto_board_holdout/pairs.csv','--max-angle-deg','1','--max-offset-mm','10','--output',results/'head_mid360_validation.json',check=False)
 report=json.loads((results/'head_mid360_validation.json').read_text());estimate=json.loads((results/'head_mid360.json').read_text());truth=truth_head(world)
 output={'train_count':sum(r['split']=='train' for r in done),'holdout_count':sum(r['split']=='holdout' for r in done),'held_out':report,'ground_truth_error':eval_error(estimate['matrix4x4'],truth),'T_head_mid360_ground_truth':truth.tolist(),'result':str(results/'head_mid360.json'),'validation':str(results/'head_mid360_validation.json'),'source_scene':str(world)}
 write(results/'evaluation_summary.json',output);write(RUN/'head_summary.json',output);sample_status('head_finished',summary=output);return output

def run_wrist(plan):
 WRIST.mkdir(parents=True,exist_ok=True);(WRIST/'config').mkdir(exist_ok=True);(WRIST/'results').mkdir(exist_ok=True)
 world=ROOT/'cli/world/wrist.world.sdf';write(WRIST/'config/automated_pose_plan.json',plan)
 # Copy the exact scene as a manifest. Relative mesh/texture paths remain based on the original scene, which is executed.
 write(WRIST/'config/scene_source.json',{'scene':str(world),'sha256':hashlib.sha256(world.read_bytes()).hexdigest(),'urdf':str(URDF),'urdf_sha256':hashlib.sha256(URDF.read_bytes()).hexdigest(),'partition':ENV['GZ_PARTITION']})
 done=[]
 with Server(world,'wrist'):
  for row in plan:
   tool('move_pose.py','--table',row['table'],'--sample',row['sample_id']);time.sleep(.65)
   dataset,sample,out=capture(WRIST,'wrist',row['split'])
   requested=np.array(row['joint_targets_rad']);actual=np.array([sample['measured_joint_positions'][f'piper_joint{i}'] for i in range(1,7)])
   if np.max(np.abs(actual-requested))>.015:raise RuntimeError('Actual wrist joints did not reach command target')
   folder=WRIST/row['split']/row['sample_id'];folder.mkdir(parents=True,exist_ok=True)
   write(folder/'capture_record.json',{'requested_pose':row,'image_id':sample['id'],'measured_joint_positions':sample['measured_joint_positions'],'max_joint_command_error_rad':float(np.max(np.abs(actual-requested))),'image_sim_time':sample['image_sim_time'],'time_skew_s':sample['time_skew_s'],'partition':ENV['GZ_PARTITION']})
   done.append(sample);sample_status('wrist_capture',complete=len(done),target=len(plan),sample=row['sample_id'],split=row['split'])
 results=WRIST/'results';dataset=WRIST/'dataset.json';overlap=measured_overlap(done);write(results/'actual_pose_overlap.json',overlap)
 if overlap['duplicates']:raise RuntimeError('Duplicate actual wrist poses; saved overlap report')
 sample_status('wrist_solve')
 tool('solve_handeye.py',dataset,'--output',results/'handeye_01.json')
 tool('calibration_workflow.py','normalize-handeye','--input',results/'handeye_01.json','--parent','piper_link6','--child','wrist_camera_optical_frame','--output',results/'flange_wrist.json')
 tool('calibration_workflow.py','validate-handeye','--result',results/'handeye_01.json','--dataset',dataset,'--max-mm','5','--max-deg','1','--output',results/'handeye_validation_01.json',check=False)
 hand=json.loads((results/'handeye_01.json').read_text());validation=json.loads((results/'handeye_validation_01.json').read_text())
 model=ET.parse(world).getroot().find("world/model[@name='robocup']");link=model.find("link[@name='piper_gripper_base']");camera=pose(link.find("sensor[@name='wrist_d435f_color']").findtext('pose'));optical=np.eye(4);optical[:3,:3]=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
 truth=fk(URDF,ZERO,'piper_link6','piper_gripper_base')@camera@optical
 output={'train_count':hand['training_count'],'holdout_count':hand['holdout_count'],'held_out':validation,'ground_truth_error':eval_error(hand['matrix'],truth),'T_flange_wrist_ground_truth':truth.tolist(),'pose_overlap':overlap,'result':str(results/'flange_wrist.json'),'validation':str(results/'handeye_validation_01.json'),'source_scene':str(world)}
 write(results/'evaluation_summary.json',output);write(RUN/'wrist_summary.json',output);sample_status('wrist_finished',summary=output);return output

if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--only',choices=['head','wrist','plan'],default=None);args=parser.parse_args()
 plans=create_plan();sample_status('angles_checked',plan=str(RUN/'angle_overlap_plan.json'))
 summary={}
 if args.only=='plan':sys.exit(0)
 for name,func,plan in [('head',run_head,plans[0]),('wrist',run_wrist,plans[1])]:
  if args.only and args.only!=name:continue
  try:summary[name]=func(plan)
  except Exception as e:
   summary[name]={'error':str(e)};write(RUN/f'{name}_failure.json',summary[name]);sample_status(name+'_failed',error=str(e))
 write(RUN/'summary.json',summary);sample_status('finished',summary=summary)
 sys.exit(0 if all('error' not in v for v in summary.values()) else 1)
