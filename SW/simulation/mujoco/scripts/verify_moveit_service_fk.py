"""실행 중인 MoveIt /compute_fk 결과를 MuJoCo FK와 직접 비교한다."""
import json,sys,time
from pathlib import Path
import numpy as np
import mujoco,rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from manipulation.sim_model import load_model,JOINTS,body_transform,tcp_transform
m,d=load_model();rclpy.init();node=Node('updated_fk_verification');client=node.create_client(GetPositionFK,'/compute_fk')
report={'result':'NOT_RUN','samples':20,'frames':['arm_base','tcp','wrist_camera_optical_frame'],'max_position_error_m':0.,'max_rotation_error_rad':0.}
try:
 if not client.wait_for_service(timeout_sec=15):raise RuntimeError('MOVEIT_FK_SERVICE_UNAVAILABLE')
 rng=np.random.default_rng(42)
 for _ in range(20):
  req=GetPositionFK.Request();req.header.frame_id='world';req.fk_link_names=report['frames'];names=[*JOINTS,'joint7','joint8'];values=[]
  for n in names:
   q=-values[-1] if n=='joint8' else float(rng.uniform(*m.joint(n).range));values.append(q);d.qpos[m.joint(n).qposadr[0]]=q
  mujoco.mj_forward(m,d);req.robot_state.joint_state.name=names;req.robot_state.joint_state.position=values
  future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=15)
  if not future.done():raise RuntimeError('MOVEIT_FK_TIMEOUT')
  result=future.result()
  if result.error_code.val!=1:raise RuntimeError(f'MOVEIT_FK_ERROR:{result.error_code.val}')
  for name,p in zip(result.fk_link_names,result.pose_stamped):
   expected=tcp_transform(m,d) if name=='tcp' else body_transform(m,d,name)
   q=p.pose.orientation;R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();xyz=p.pose.position
   report['max_position_error_m']=max(report['max_position_error_m'],float(np.linalg.norm(expected[:3,3]-[xyz.x,xyz.y,xyz.z])))
   report['max_rotation_error_rad']=max(report['max_rotation_error_rad'],float(Rotation.from_matrix(expected[:3,:3].T@R).magnitude()))
 report['result']='PASS' if report['max_position_error_m']<1e-7 and report['max_rotation_error_rad']<1e-7 else 'FAIL'
except Exception as error:report.update(result='FAIL',failure=str(error));raise
finally:
 (ROOT/'reports/moveit_service_fk.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));node.destroy_node();rclpy.shutdown()
if report['result']!='PASS':raise SystemExit(1)
