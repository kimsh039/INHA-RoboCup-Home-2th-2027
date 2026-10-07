"""Offline eye-in-hand / eye-on-base calibration. T_parent_child maps child to parent."""
import argparse,json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

def transform(value):
    t=np.asarray(value,dtype=float)
    if t.shape!=(4,4) or not np.isfinite(t).all(): raise ValueError('Transform must be finite 4x4')
    if not np.allclose(t[3],[0,0,0,1],atol=1e-8): raise ValueError('Invalid homogeneous bottom row')
    r=t[:3,:3]
    if not np.allclose(r.T@r,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(r),1,atol=1e-6):
        raise ValueError('Rotation must be orthonormal with determinant +1')
    return t

def mean_transform(values):
    result=np.eye(4); result[:3,:3]=Rotation.from_matrix(np.array([x[:3,:3] for x in values])).mean().as_matrix()
    result[:3,3]=np.mean([x[:3,3] for x in values],axis=0)
    return result

def pose_error(estimated,reference):
    return {'translation_m':float(np.linalg.norm(estimated[:3,3]-reference[:3,3])),
            'rotation_deg':float(np.degrees(Rotation.from_matrix(reference[:3,:3].T@estimated[:3,:3]).magnitude()))}

def solve(payload):
    mode=payload['mode']
    if mode not in ('eye_in_hand','eye_on_base'): raise ValueError('Unsupported mode')
    if payload.get('translation_unit')!='m': raise ValueError('Explicit translation_unit=m required')
    rows=payload['samples'];train=[];hold=[]
    for row in rows:
        g=transform(row['T_base_flange']);c=transform(row['T_camera_target'])
        if row.get('split') not in ('train','holdout'):raise ValueError('Every sample needs train/holdout split')
        (train if row['split']=='train' else hold).append((g,c))
    if len(train)<6 or len(hold)<3: raise ValueError('This workflow requires >=6 train and >=3 held-out poses')
    relative=np.array([Rotation.from_matrix(g[:3,:3]@train[0][0][:3,:3].T).as_rotvec() for g,_ in train])
    singular=np.linalg.svd(relative,compute_uv=False)
    if singular[0]<np.radians(10) or singular[1]<0.05*singular[0]:
        raise ValueError('Degenerate poses: rotate about at least two axes')
    # Eye-on-base uses inverse robot poses; A * X * C is then the fixed flange target.
    robot=[g if mode=='eye_in_hand' else np.linalg.inv(g) for g,_ in train]
    camera=[c for _,c in train]
    r,t=cv2.calibrateHandEye([g[:3,:3] for g in robot],[g[:3,3] for g in robot],
        [c[:3,:3] for c in camera],[c[:3,3] for c in camera],method=cv2.CALIB_HAND_EYE_PARK)
    x=np.eye(4);x[:3,:3]=r;x[:3,3]=np.asarray(t).reshape(3);transform(x)
    def invariant(pair):
        g,c=pair;return (g if mode=='eye_in_hand' else np.linalg.inv(g))@x@c
    fixed=mean_transform([invariant(pair) for pair in train])
    errors=[pose_error(invariant(pair),fixed) for pair in hold]
    result={'mode':mode,'translation_unit':'m','synthetic':bool(payload.get('synthetic',False)),
        'transform_convention':'T_parent_child maps child coordinates to parent coordinates',
        'result_frame':'T_flange_camera' if mode=='eye_in_hand' else 'T_base_camera',
        'matrix':x.tolist(),'translation_xyz_m':x[:3,3].tolist(),'quaternion_xyzw':Rotation.from_matrix(x[:3,:3]).as_quat().tolist(),
        'training_count':len(train),'holdout_count':len(hold),'method':'OpenCV PARK',
        'held_out_invariant_errors':errors,
        'max_holdout_translation_m':max(e['translation_m'] for e in errors),
        'max_holdout_rotation_deg':max(e['rotation_deg'] for e in errors),
        'deployment_ready':False}
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('input',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=solve(json.loads(a.input.read_text()));a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:result[k] for k in ('result_frame','training_count','holdout_count','max_holdout_translation_m','max_holdout_rotation_deg','synthetic')}))

if __name__=='__main__':main()
