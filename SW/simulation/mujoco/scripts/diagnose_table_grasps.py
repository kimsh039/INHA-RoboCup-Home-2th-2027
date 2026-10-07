"""Independent table/width diagnostics on untouched model poses.
Uses the GraspNetAPI visualization boxes, not calibrated PiPER collision geometry.
"""
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from manipulation.sim_model import ROOT,config

def obb_intersects_aabb(center,rotation,half,box_center,box_half):
    axes=[*np.eye(3),*rotation.T]
    axes += [np.cross(a,b) for a in np.eye(3) for b in rotation.T]
    offset=np.asarray(center)-box_center
    for axis in axes:
        if np.linalg.norm(axis)<1e-10:continue
        r=np.abs(axis)@box_half+np.abs(rotation.T@axis)@half
        if abs(offset@axis)>r+1e-9:return False
    return True

def gripper_boxes(g):
    w,d=g['width'],g['depth'];f=.004;h=.004;b=.02
    return [([-b-f,-w/2-f,-h/2],[d,-w/2,h/2]),
            ([-b-f,w/2,-h/2],[d,w/2+f,h/2]),
            ([-b-f,-w/2,-h/2],[-b,w/2,h/2]),
            ([-.04-b-f,-f/2,-h/2],[-b-f,f/2,h/2])]

def main():
    cfg=config();source=json.loads((ROOT/'data/grasps/cube/grasps.json').read_text())
    center=np.array(cfg['object_world_m'])
    result_file=ROOT/'reports/grasp_simulation_result.json'
    if result_file.exists():
        status=json.loads(result_file.read_text()).get('last_simulation_status')
        if status:center=np.array(status['T_world_object'])[:3,3]
    rows=[]
    for g in source['grasps']:
        R=np.array(g['rotation_matrix']);p=center+np.array(g['translation']);hits=[]
        for name,(lo,hi) in zip(('finger_left','finger_right','palm','tail'),gripper_boxes(g)):
            lo=np.array(lo);hi=np.array(hi)
            if obb_intersects_aabb(p+R@((lo+hi)/2),R,(hi-lo)/2,np.array(cfg['table_center_world_m']),np.array(cfg['table_half_size_m'])):hits.append(name)
        rows.append({'candidate_id':g['candidate_id'],'width_m':g['width'],'width_valid':g['width']<=.07,'template_table_collision':bool(hits),'colliding_parts':hits})
    out={'candidate_count':len(rows),'template_table_clear':sum(not x['template_table_collision'] for x in rows),'template_table_clear_and_width_valid':sum(not x['template_table_collision'] and x['width_valid'] for x in rows),'template_table_clear_ids':[x['candidate_id'] for x in rows if not x['template_table_collision']],'template_table_clear_width_valid_ids':[x['candidate_id'] for x in rows if not x['template_table_collision'] and x['width_valid']],'method':'15-axis separating-axis test: rotated API template boxes vs world tabletop AABB; current object orientation identity','limitations':'Checks final grasp only, not pre-grasp or approach sweep. API visual template is not PiPER physical geometry. Table-clear does not imply valid width, IK or successful contact. All original model poses/widths retained.','rows':rows}
    (ROOT/'reports/table_grasp_diagnosis.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='rows'},indent=2))
if __name__=='__main__':main()
