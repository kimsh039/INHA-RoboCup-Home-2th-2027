"""Whole-cloud plane association. No user-drawn ROI or sensor mount ground truth."""
import csv,itertools,json,math
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from calibration_workflow import plane,read,rows,sha,transform,write,result

def load_cloud(path):
    with np.load(path,allow_pickle=False) as data:
        points=np.asarray(data['xyz'],dtype=float)
        raw_hash=str(data['source_sha256'])
    if points.ndim!=2 or points.shape[1]!=3:raise ValueError('Expected XYZ point array')
    points=points[np.isfinite(points).all(axis=1)]
    if len(points)<80:raise ValueError('Too few finite returns')
    return points,raw_hash

def extract(points,a):
    """Sequential RANSAC over the whole finite cloud; automatic plane inliers only."""
    if a.min_inliers<30 or a.iterations<64 or a.max_planes<3 or a.ransac_threshold<=0:raise ValueError('Invalid automatic extraction parameters')
    remaining=points.copy();rng=np.random.default_rng(0);candidates=[]
    for _ in range(a.max_planes):
        if len(remaining)<a.min_inliers:break
        work=remaining if len(remaining)<=6000 else remaining[rng.choice(len(remaining),6000,replace=False)]
        best_score=0;best_normal=None;best_d=None
        for start in range(0,a.iterations,64):
            count=min(64,a.iterations-start);indices=rng.integers(0,len(work),size=(count,3));q=work[indices]
            normals=np.cross(q[:,1]-q[:,0],q[:,2]-q[:,0]);lengths=np.linalg.norm(normals,axis=1);valid=lengths>1e-10
            if not valid.any():continue
            normals=normals[valid]/lengths[valid,None];distances=-np.sum(normals*q[valid,0],axis=1)
            scores=np.sum(np.abs(work@normals.T+distances)<a.ransac_threshold,axis=0);i=int(np.argmax(scores))
            if int(scores[i])>best_score:best_score=int(scores[i]);best_normal=normals[i];best_d=distances[i]
        if best_normal is None:break
        mask=np.abs(remaining@best_normal+best_d)<a.ransac_threshold
        if mask.sum()<a.min_inliers:break
        selected=remaining[mask];center=selected.mean(axis=0);_,s,vt=np.linalg.svd(selected-center,full_matrices=False)
        n=vt[-1];d=-float(n@center)
        for _ in range(2):
            mask=np.abs(remaining@n+d)<a.ransac_threshold
            if mask.sum()<a.min_inliers:break
            selected=remaining[mask];center=selected.mean(axis=0);_,s,vt=np.linalg.svd(selected-center,full_matrices=False);n=vt[-1];d=-float(n@center)
        remaining=remaining[~mask]
        if len(selected)<a.min_inliers or s[1]/math.sqrt(len(selected))<.04 or s[1]<.01*s[0]:continue
        # Visible board face points towards the sensor; signed distance is positive.
        if d<0:n=-n;d=-d
        residual=selected@n+d
        candidates.append(dict(normal=n.tolist(),d=d,points=int(len(selected)),rms_m=float(np.sqrt(np.mean(residual**2))),centroid_m=center.tolist()))
    if len(candidates)<3:raise ValueError('Too few extended planes in the whole cloud; collect a view with more planar returns')
    return candidates

def fit_pairs(source_normals,source_d,target_normals,target_d):
    if np.linalg.matrix_rank(source_normals,tol=1e-3)<3 or np.linalg.matrix_rank(target_normals,tol=1e-3)<3:raise ValueError('Plane normals do not span three axes; change target orientations')
    u,_,vt=np.linalg.svd(target_normals.T@source_normals);r=u@np.diag([1,1,np.linalg.det(u@vt)])@vt
    t=np.linalg.lstsq(target_normals,source_d-target_d,rcond=None)[0]
    return r,t

def save_pairs(path,items):
    with Path(path).open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['plane_id','source_nx','source_ny','source_nz','source_d','target_nx','target_ny','target_nz','target_d','source_sha256','target_sha256']);writer.writeheader();writer.writerows(items)

def pair_row(ident,n,d,m,e,source_hash,target_hash):
    return dict(plane_id=ident,source_nx=float(n[0]),source_ny=float(n[1]),source_nz=float(n[2]),source_d=float(d),target_nx=float(m[0]),target_ny=float(m[1]),target_nz=float(m[2]),target_d=float(e),source_sha256=source_hash,target_sha256=target_hash)

def auto_room(a):
    points,raw_hash=load_cloud(a.input);targets=rows(a.targets);identifiers=[v['plane_id'] for v in targets]
    if not set(['floor','front','side','back','other_side']).issubset(identifiers):raise ValueError('Generate room targets again with room-targets --all-walls; all four walls resolve false matches')
    if any(v.get('frame')!='base_link' or v.get('unit')!='m' or not v.get('source') for v in targets):raise ValueError('Room targets must have independent source, base_link frame, and metres')
    nd=[plane(v) for v in targets];m=np.array([v[0] for v in nd]);e=np.array([v[1] for v in nd]);candidates=extract(points,a)
    if a.reference:
        reference,matrix=result(a.reference)
        if (reference['parent_frame'],reference['child_frame'])!=('base_link','livox_frame'):raise ValueError('Reference frames must be base_link<-livox_frame')
        r=matrix[:3,:3];t=matrix[:3,3];refitted=False
    else:
        if a.distance_threshold<=0 or a.max_translation_m<=0 or not 0<a.max_rotation_deg<90:raise ValueError('Invalid matching thresholds or orientation prior')
        initial=Rotation.from_euler('xyz',a.initial_rpy_deg,degrees=True).as_matrix();triplet=[identifiers.index(k) for k in ('floor','front','side')];mt=m[triplet];et=e[triplet]
        sample=points[::max(1,len(points)//6000)];minimum=max(8,int(math.ceil(a.min_inliers*len(sample)/len(points))));best=None
        for choices in itertools.permutations(range(len(candidates)),3):
            ns=np.array([candidates[i]['normal'] for i in choices]);ds=np.array([candidates[i]['d'] for i in choices])
            if np.max(np.abs(ns@ns.T-np.eye(3)))>.25:continue
            for signs in itertools.product([-1,1],repeat=3):
                ss=np.array(signs);rr,tt=fit_pairs(ns*ss[:,None],ds*ss,mt,et)
                if np.degrees(Rotation.from_matrix(initial.T@rr).magnitude())>a.max_rotation_deg or np.linalg.norm(tt)>a.max_translation_m:continue
                distances=np.abs((sample@rr.T+tt)@m.T+e);labels=np.argmin(distances,axis=1);near=distances[np.arange(len(sample)),labels]<a.distance_threshold;counts=np.bincount(labels[near],minlength=len(m))
                if (counts<minimum).any():continue
                score=(int(near.sum()),-float(np.mean(distances[np.arange(len(sample))[near],labels[near]]**2)))
                if best is None or score>best[0]:best=(score,rr,tt)
        if best is None:raise ValueError('No whole-cloud room correspondence fits all four walls and the orientation prior; collect more room surfaces or specify the actual design orientation')
        _,r,t=best
        for _ in range(3):
            transformed=points@r.T+t;distances=np.abs(transformed@m.T+e);labels=np.argmin(distances,axis=1);mask=distances[np.arange(len(points)),labels]<a.distance_threshold;p=points[mask];nm=m[labels[mask]];dd=e[labels[mask]]
            if len(p)<3*a.min_inliers:raise ValueError('Insufficient whole-cloud plane support')
            def residual(v):return np.sum((p@Rotation.from_rotvec(v[:3]).as_matrix().T+v[3:])*nm,axis=1)+dd
            solution=least_squares(residual,np.r_[Rotation.from_matrix(r).as_rotvec(),t],loss='soft_l1',f_scale=a.ransac_threshold,max_nfev=150)
            if not solution.success:raise ValueError('Automatic room optimization did not converge')
            r=Rotation.from_rotvec(solution.x[:3]).as_matrix();t=solution.x[3:]
        if np.degrees(Rotation.from_matrix(initial.T@r).magnitude())>a.max_rotation_deg or np.linalg.norm(t)>a.max_translation_m:raise ValueError('Estimated mount is outside the stated design prior')
        refitted=True
    distances=np.abs((points@r.T+t)@m.T+e);labels=np.argmin(distances,axis=1);near=distances[np.arange(len(points)),labels]<a.distance_threshold
    output=Path(a.output_dir);output.mkdir(parents=True);items=[];observed=[]
    for i,ident in enumerate(identifiers):
        p=points[near&(labels==i)]
        if len(p)<a.min_inliers:raise ValueError('Insufficient automatic support for '+ident)
        center=p.mean(axis=0);_,s,vt=np.linalg.svd(p-center,full_matrices=False)
        if s[1]/math.sqrt(len(p))<.04:raise ValueError('Plane support is too narrow for '+ident)
        n=vt[-1];d=-float(n@center)
        if (r@n)@m[i]<0:n=-n;d=-d
        rms=float(np.sqrt(np.mean((p@n+d)**2)))
        obs=dict(plane_id=ident,nx=float(n[0]),ny=float(n[1]),nz=float(n[2]),d=d,unit='m',frame='livox_frame',inlier_count=len(p),rms_m=rms,manual_roi=False,cloud_sha256=sha(a.input),raw_sha256=raw_hash)
        write(output/(ident+'.json'),obs);observed.append(obs);items.append(pair_row(ident,n,d,m[i],e[i],sha(a.input),sha(a.targets)))
    save_pairs(output/'pairs.csv',items)
    report=dict(method='whole-cloud sequential RANSAC, room hypothesis matching, robust plane residual fitting',manual_roi=False,finite_input_points=len(points),automatically_assigned_points=int(near.sum()),unassigned_points=int((~near).sum()),plane_candidates=candidates,observed_planes=observed,cloud_sha256=sha(a.input),raw_sha256=raw_hash,targets_sha256=sha(a.targets),extrinsic_refitted=refitted)
    if a.reference:report['reference_sha256']=sha(a.reference)
    else:
        matrix=np.eye(4);matrix[:3,:3]=r;matrix[:3,3]=t
        write(output/'base_mid360.json',transform(matrix,'base_link','livox_frame',status='computed_validation_pending',manual_roi=False,method=report['method'],training_sha256=sha(output/'pairs.csv'),cloud_sha256=sha(a.input),targets_sha256=sha(a.targets),initial_rpy_deg=a.initial_rpy_deg,max_rotation_deg=a.max_rotation_deg,max_translation_m=a.max_translation_m))
    write(output/'automatic_extraction.json',report);print(str(output/'pairs.csv' if a.reference else output/'base_mid360.json'))

def auto_board_pairs(a):
    paths=sorted(Path(a.samples_root).glob('*/head_plane.json'))
    minimum=3 if a.reference else 6
    if len(paths)<minimum:raise ValueError(f'Need at least {minimum} different stationary board poses; collect 15-25 train poses')
    targets=[read(p) for p in paths];ids=[t['plane_id'] for t in targets]
    if len(set(ids))!=len(ids) or any(t.get('frame')!='camera_optical_frame' or t.get('unit')!='m' for t in targets):raise ValueError('Head plane IDs must be unique, camera optical frame, metres')
    nd=[plane(t) for t in targets];m=np.array([v[0] for v in nd]);e=np.array([v[1] for v in nd])
    if not a.reference and np.linalg.matrix_rank(m,tol=1e-3)<3:raise ValueError('Rotate the board about more axes; Head plane normals lack rank three')
    clouds=[p.parent/'xyz.npz' for p in paths];all_candidates=[]
    for path in clouds:
        points,_=load_cloud(path);all_candidates.append(extract(points,a))
    choices=[];persistent_limit=max(2,int(math.ceil(.6*len(paths))))
    for i,candidates in enumerate(all_candidates):
        moving=[]
        for c in candidates:
            n=np.array(c['normal']);persistent=0
            for j,other in enumerate(all_candidates):
                if j==i:continue
                if any(float(n@np.array(o['normal']))>math.cos(math.radians(2)) and abs(c['d']-o['d'])<.025 for o in other):persistent+=1
            if persistent<persistent_limit:moving.append(c)
        if not moving:raise ValueError('No changing plane found for '+ids[i]+'; keep sensors fixed and change board pose')
        choices.append(moving)
    if a.reference:
        reference,matrix=result(a.reference)
        if (reference['parent_frame'],reference['child_frame'])!=('camera_optical_frame','livox_frame'):raise ValueError('Reference frames must be camera_optical_frame<-livox_frame')
        r=matrix[:3,:3];t=matrix[:3,3];selected=[]
        for i,candidates in enumerate(choices):
            scores=[]
            for k,c in enumerate(candidates):
                angle=float(np.degrees(np.arccos(np.clip((r@np.array(c['normal']))@m[i],-1,1))));offset=float(m[i]@t+e[i]-c['d'])
                if angle<=a.max_normal_deg and abs(offset)<=a.max_offset_m:scores.append((angle**2+(offset*100)**2,k))
            if not scores:raise ValueError('No moving board correspondence under the frozen reference for '+ids[i])
            selected.append(min(scores)[1])
    else:
        # Unknown rotation preserves pairwise normal angles.
        beams=[(0.,[])]
        for i,candidates in enumerate(choices):
            next_beams=[]
            for score,selected in beams:
                for index,c in enumerate(candidates):
                    n=np.array(c['normal']);increment=sum((float(n@np.array(choices[j][k]['normal']))-float(m[i]@m[j]))**2 for j,k in enumerate(selected))
                    next_beams.append((score+increment,selected+[index]))
            beams=sorted(next_beams,key=lambda v:v[0])[:64]
        best=None
        for normal_cost,selected in beams:
            ns=np.array([choices[i][k]['normal'] for i,k in enumerate(selected)]);ds=np.array([choices[i][k]['d'] for i,k in enumerate(selected)])
            try:r,t=fit_pairs(ns,ds,m,e)
            except ValueError:continue
            angles=np.degrees(np.arccos(np.clip(np.sum((ns@r.T)*m,axis=1),-1,1)));offsets=m@t+e-ds;score=float(np.mean(angles**2)+np.mean((offsets*100)**2))
            if np.max(angles)>a.max_normal_deg or np.max(np.abs(offsets))>a.max_offset_m:continue
            if best is None or score<best[0]:best=(score,selected,r,t)
        if best is None:raise ValueError('No consistent moving-board association across whole clouds; collect more board returns and varied orientations')
        _,selected,_,_=best
    output=Path(a.output_dir);output.mkdir(parents=True);items=[];assignments=[]
    for i,k in enumerate(selected):
        c=choices[i][k];n=np.array(c['normal']);d=c['d'];items.append(pair_row(ids[i],n,d,m[i],e[i],sha(clouds[i]),sha(paths[i])));assignments.append(dict(plane_id=ids[i],cloud=str(clouds[i]),head_plane=str(paths[i]),selected_plane=c,candidate_count=len(choices[i])))
    save_pairs(output/'pairs.csv',items);write(output/'automatic_association.json',dict(manual_roi=False,method='whole-cloud plane extraction, persistent-background rejection, multi-pose normal/distance correspondence',same_visible_board_face_required=True,sensors_must_remain_stationary=True,assignments=assignments,calibration_complete=False,reference_sha256=sha(a.reference) if a.reference else None,reference_refitted=False));print(str(output/'pairs.csv'))

def add_commands(command,arg):
    def extraction(q):
        q.add_argument('--ransac-threshold',type=float,default=.012);q.add_argument('--min-inliers',type=int,default=80);q.add_argument('--iterations',type=int,default=512);q.add_argument('--max-planes',type=int,default=12)
    q=command('auto-room',auto_room);arg(q,'input');arg(q,'targets');arg(q,'output-dir');q.add_argument('--reference');q.add_argument('--initial-rpy-deg',nargs=3,type=float,default=[0,0,0]);q.add_argument('--max-rotation-deg',type=float,default=40);q.add_argument('--max-translation-m',type=float,default=1.5);q.add_argument('--distance-threshold',type=float,default=.05);extraction(q)
    q=command('auto-board-pairs',auto_board_pairs);arg(q,'samples-root');arg(q,'output-dir');q.add_argument('--reference');q.add_argument('--max-normal-deg',type=float,default=5);q.add_argument('--max-offset-m',type=float,default=.05);extraction(q)
