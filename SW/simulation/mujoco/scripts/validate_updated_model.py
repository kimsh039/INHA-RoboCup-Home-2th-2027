"""새 장착·영점·초기 자세·접힘 자세를 CPU에서 검사한다. 파지 성공 판정은 하지 않는다."""
import json,sys
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from manipulation.sim_model import load_model,write_moveit_files,verify_exported_fk,config,body_transform,jaw_opening
from manipulation.observation import top_down_pose
m,d=load_model();write_moveit_files(m);cfg=config();fk=verify_exported_fk(m,d)
arm=[int(m.joint(f'joint{i}').qposadr[0]) for i in range(1,7)];act=[m.actuator(f'joint{i}').id for i in range(1,7)];dof=[int(m.joint(f'joint{i}').dofadr[0]) for i in range(1,7)]
def contacts():
 return [{'geom1':m.geom(c.geom1).name,'geom2':m.geom(c.geom2).name,'body1':m.body(m.geom_bodyid[c.geom1]).name,'body2':m.body(m.geom_bodyid[c.geom2]).name,'depth_m':float(-c.dist)} for c in d.contact if c.dist < -cfg['penetration_tolerance_m']]
report={'fk':fk,'initial':{'joint_positions_rad':d.qpos[arm].tolist(),'T_world_arm':body_transform(m,d,'arm_base').tolist(),'contacts':contacts()},'observation_plan':top_down_pose(m,d,cfg['observation_camera_height_m']),'checks':[],'scope':'scratch stationary holding with original bridge bias compensation and actuator clipping; strict joint limits; no live trajectory or grasp claim'}
# 접힘은 별도 scratch 상태로 검사하며 실제 관측 이동을 순간 이동하지 않는다.
for stage,q,grip in [('initial_hold',cfg['initial_joints_rad'],.035),('folded_hold',cfg['folded_candidate_rad'],.035),('folded_gripper_close',cfg['folded_candidate_rad'],0),('folded_gripper_open',cfg['folded_candidate_rad'],.035)]:
 d=mujoco.MjData(m)
 for i,value in zip(arm,q):d.qpos[i]=value
 d.qpos[m.joint('joint7').qposadr[0]]=.035;d.qpos[m.joint('joint8').qposadr[0]]=-.035
 d.ctrl[act]=q;d.ctrl[m.actuator('gripper').id]=.035;mujoco.mj_forward(m,d)
 bad=[]
 for _ in range(2000):
  # 실제 성공 브리지의 편향 토크/입력 범위 정책을 그대로 적용한다.
  # 실제 브리지처럼 닫기 목표를 3초에 걸쳐 천천히 적용한다.
  g=m.actuator('gripper').id;rate=.035/cfg['grasp_close_duration_s']*m.opt.timestep
  d.ctrl[g]+=np.clip(grip-d.ctrl[g],-rate,rate)
  ranges=m.actuator_ctrlrange[act]
  d.ctrl[act]=np.clip(np.array(q)+d.qfrc_bias[dof]/m.actuator_gainprm[act,0],ranges[:,0],ranges[:,1])
  mujoco.mj_step(m,d)
  bad=contacts()
  if bad:break
 err=float(np.max(np.abs(d.qpos[arm]-q)))
 gap=jaw_opening(m,d);gap_ok=(gap<.002 if grip==0 else gap>.068)
 limits=np.array([m.joint(f'joint{i}').range for i in range(1,7)]);limits_ok=bool(np.all(d.qpos[arm]>=limits[:,0]) and np.all(d.qpos[arm]<=limits[:,1]))
 report['checks'].append({'stage':stage,'simulation_time_s':float(d.time),'max_arm_error_rad':err,'measured_arm_joints_rad':d.qpos[arm].tolist(),'joint_limits_rad':limits.tolist(),'limit_violations_rad':np.maximum(limits[:,0]-d.qpos[arm],np.maximum(d.qpos[arm]-limits[:,1],0)).tolist(),'jaw_gap_m':jaw_opening(m,d),'contacts':bad,'physics_warning_count':sum(int(w.number) for w in d.warning),'within_joint_limits':limits_ok,'finite':bool(np.isfinite(d.qpos).all()),'pass':err<cfg['arm_goal_tolerance_rad'] and gap_ok and limits_ok and bool(np.isfinite(d.qpos).all()) and not bad and not any(w.number for w in d.warning)})
report['result']='PASS' if all(x['pass'] for x in report['checks']) else 'FAIL'
(ROOT/'reports/updated_model_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if report['result']!='PASS':raise SystemExit(1)
