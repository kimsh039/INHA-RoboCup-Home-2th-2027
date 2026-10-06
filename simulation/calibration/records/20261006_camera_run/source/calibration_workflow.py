#!/usr/bin/env python3
"""Offline measured-plane, mount, pivot and URDF tools. All transforms are parent<-child, metres."""
import argparse, base64, csv, gzip, hashlib, json, os, sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):
    p=Path(p)
    with (gzip.open(p,'rt') if p.suffix=='.gz' else p.open()) as f: return json.load(f)
def write(p,obj):
    p=Path(p); p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f: json.dump(obj,f,indent=2,allow_nan=False); f.write('\n')
def se3(v):
    t=np.array(v,dtype=float)
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1],atol=1e-8): raise ValueError('Expected finite 4x4 SE(3)')
    r=t[:3,:3]
    if not np.allclose(r.T@r,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(r),1,atol=1e-6): raise ValueError('Invalid rotation')
    return t
def result(p):
    v=read(p)
    if v.get('translation_unit')!='m': raise ValueError('Result unit must be m')
    return v,se3(v['matrix4x4'])
def transform(t,parent,child,**extra):
    t=se3(t)
    return dict(parent_frame=parent,child_frame=child,translation_unit='m',matrix4x4=t.tolist(),translation_xyz_m=t[:3,3].tolist(),quaternion_xyzw=Rotation.from_matrix(t[:3,:3]).as_quat().tolist(),**extra)
def error(a,b):
    delta=np.linalg.inv(a)@b
    return dict(translation_mm=float(np.linalg.norm(delta[:3,3])*1000),rotation_deg=float(np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude())))
def fixed_fk(root,base,child):
    parents={j.find('child').get('link'):j for j in root.findall('joint')}; chain=[]; seen=set()
    while child!=base:
        if child in seen or child not in parents: raise ValueError(f'No fixed ancestor path from {base} to {child}')
        seen.add(child); j=parents[child]
        if j.get('type')!='fixed': raise ValueError('Path crosses moving joint '+j.get('name'))
        chain.append(j); child=j.find('parent').get('link')
    t=np.eye(4)
    for j in reversed(chain):
        o=j.find('origin'); local=np.eye(4)
        if o is not None:
            local[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
            local[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
        t=t@local
    return t
def rows(p):
    with Path(p).open(newline='') as f: return list(csv.DictReader(f))
def plane(row):
    if row.get('unit','m')!='m': raise ValueError('Plane distances must be m')
    n=np.array([float(row[k]) for k in ('nx','ny','nz')]); length=np.linalg.norm(n)
    if not np.isfinite(length) or length<1e-10: raise ValueError('Invalid plane normal')
    d=float(row['d'])
    if not np.isfinite(d): raise ValueError('Invalid plane distance')
    return n/length,d/length
def plane_pairs(p):
    data=rows(p)
    if len(data)<3: raise ValueError('At least three plane pairs required')
    nn,dd,mm,ee=[],[],[],[]
    for row in data:
        n,d=plane({k:row['source_'+k] for k in ('nx','ny','nz','d')})
        m,e=plane({k:row['target_'+k] for k in ('nx','ny','nz','d')})
        nn.append(n); dd.append(d); mm.append(m); ee.append(e)
    return np.array(nn),np.array(dd),np.array(mm),np.array(ee)
def cloud(a):
    v=read(a.input); data=base64.b64decode(v['data'],validate=True)
    fields=v.get('field',v.get('fields',[])); fmap={f['name']:f for f in fields}
    types={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
    step=int(v.get('pointStep',v.get('point_step',0))); width=int(v['width']); height=int(v.get('height',1)); rowstep=int(v.get('rowStep',v.get('row_step',width*step)))
    big=v.get('isBigendian',v.get('is_bigendian',False)); endian='>' if big else '<'
    if step<=0 or width<=0 or height<=0 or rowstep<width*step or len(data)<height*rowstep: raise ValueError('Malformed cloud layout')
    columns=[]
    for name in ('x','y','z'):
        f=fmap[name]; kind=f.get('datatype',f.get('dataType'))
        if isinstance(kind,str): kind={'FLOAT32':7,'FLOAT64':8}.get(kind,kind)
        dtype=np.dtype(endian+types[int(kind)]); offset=int(f.get('offset',0))
        if offset<0 or offset+dtype.itemsize>step or int(f.get('count',1))!=1: raise ValueError('Unsupported XYZ field')
        columns.append(np.ndarray((height,width),dtype=dtype,buffer=data,offset=offset,strides=(rowstep,step)).reshape(-1))
    xyz=np.column_stack(columns).astype(float); xyz=xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz): raise ValueError('No finite points')
    out=Path(a.output); out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('xb') as f: np.savez_compressed(f,xyz=xyz,source_sha256=sha(a.input))
    view=out.with_suffix('.html'); points=xyz[::max(1,len(xyz)//12000)].tolist()
    html='''<!doctype html><meta charset="utf-8"><title>Raw cloud — sensor frame / metres</title><style>body{font:16px system-ui}canvas{border:1px solid #aaa;margin:8px}label{display:inline-block;margin:8px}</style><h2>Raw XYZ — sensor frame / metres</h2><p>전체 원시 점군의 XY / XZ / YZ 표시입니다. 캘리브레이션 입력에는 전체 점군을 사용하며 수동 ROI를 선택하지 않습니다.</p><div id="views"></div><pre id="coords"></pre><script>const pts=POINTS;for(const axes of [[0,1],[0,2],[1,2]]){let c=document.createElement('canvas');c.width=440;c.height=440;document.getElementById('views').append(c);let ctx=c.getContext('2d');let lo=axes.map(i=>Math.min(...pts.map(p=>p[i]))),hi=axes.map(i=>Math.max(...pts.map(p=>p[i])));ctx.fillText(axes.map(i=>'xyz'[i]).join(' / '),10,18);for(let p of pts){let x=20+400*(p[axes[0]]-lo[0])/(hi[0]-lo[0]||1),y=420-400*(p[axes[1]]-lo[1])/(hi[1]-lo[1]||1);ctx.fillStyle='#146d9d';ctx.fillRect(x,y,1.5,1.5)}c.onmousemove=e=>{let b=c.getBoundingClientRect();document.getElementById('coords').textContent=axes.map((i,j)=>'xyz'[i]+' = '+(lo[j]+(j?420-(e.clientY-b.top):e.clientX-b.left-20)/400*(hi[j]-lo[j])).toFixed(3)+' m').join(', ')}}</script>'''.replace('POINTS',json.dumps(points))
    with view.open('x') as f:f.write(html)
    print(json.dumps({'points':len(xyz),'min_xyz_m':xyz.min(axis=0).tolist(),'max_xyz_m':xyz.max(axis=0).tolist(),'viewer':str(view)}))
def fit_plane(a):
    with np.load(a.input,allow_pickle=False) as z: xyz=z['xyz']; source_hash=str(z['source_sha256'])
    bounds=np.array(a.bounds).reshape(3,2)
    if (bounds[:,0]>=bounds[:,1]).any(): raise ValueError('Bounds must be xmin<xmax, ymin<ymax, zmin<zmax')
    p=xyz[((xyz>=bounds[:,0])&(xyz<=bounds[:,1])).all(axis=1)]
    if len(p)<30: raise ValueError('ROI has fewer than 30 points')
    rng=np.random.default_rng(0); best=None
    for _ in range(400):
        q=p[rng.choice(len(p),3,replace=False)]; n=np.cross(q[1]-q[0],q[2]-q[0]); length=np.linalg.norm(n)
        if length<1e-10: continue
        n/=length; mask=np.abs(p@n-n@q[0])<a.threshold
        if best is None or mask.sum()>best.sum():best=mask
    if best is None or best.sum()<30 or best.mean()<.6: raise ValueError('Insufficient planar inliers; adjust ROI')
    inliers=p[best]; center=inliers.mean(axis=0); _,s,vt=np.linalg.svd(inliers-center,full_matrices=False)
    if s[1]<1e-6 or s[1]<.001*s[0]: raise ValueError('ROI is collinear')
    n=vt[-1]; direction=np.zeros(3); direction['xyz'.index(a.normal_axis)]=a.normal_sign
    if n@direction<0:n=-n
    d=-n@center; residual=inliers@n+d
    write(a.output,dict(plane_id=a.plane_id,nx=float(n[0]),ny=float(n[1]),nz=float(n[2]),d=float(d),unit='m',frame=a.frame,inlier_count=int(len(inliers)),roi_count=int(len(p)),rms_m=float(np.sqrt(np.mean(residual**2))),bounds_m=a.bounds,cloud_sha256=sha(a.input),raw_sha256=source_hash))
def pairs(a):
    target={r['plane_id']:r for r in rows(a.targets)}; out=[]
    for f in a.observed:
        v=read(f); t=target[v['plane_id']]
        if t.get('unit')!='m' or not t.get('source'): raise ValueError('Targets require unit=m and independent source')
        out.append(dict(plane_id=v['plane_id'],observed_sha256=sha(f),**{'source_'+k:v[k] for k in ('nx','ny','nz','d')},**{'target_'+k:t[k] for k in ('nx','ny','nz','d')}))
    p=Path(a.output); p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(out[0]));w.writeheader();w.writerows(out)
def targets(a):
    values=[read(p) for p in a.planes]
    if len({v['plane_id'] for v in values})!=len(values) or len({v['frame'] for v in values})!=1:raise ValueError('Target IDs must be unique and target frame must agree')
    with Path(a.output).open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['plane_id','nx','ny','nz','d','frame','unit','source']);w.writeheader()
        for p,v in zip(a.planes,values):w.writerow(dict(**{k:v[k] for k in ('plane_id','nx','ny','nz','d','frame','unit')},source=f'measured plane {p}; SHA256 {sha(p)}'))
def pivot_sample(a):
    v,t=result(a.pose)
    if v['child_frame']!='piper_link6' or 'measured_joint_positions' not in v:raise ValueError('Expected measured FK pose')
    path=Path(a.dataset);data=read(path) if path.exists() else dict(translation_unit='m',base_frame=v['parent_frame'],samples=[])
    if data.get('base_frame')!=v['parent_frame']:raise ValueError('Base frame differs')
    h=sha(a.pose)
    if any(s.get('pose_sha256')==h for s in data['samples']):raise ValueError('Same pose file already added')
    data['samples'].append(dict(id=len(data['samples']),split=a.split,T_base_flange=t.tolist(),pose_sha256=h,contact_note=a.contact_note,simulation_stamp=v.get('simulation_stamp'),measured_joint_positions=v['measured_joint_positions']))
    path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_suffix('.tmp')
    with tmp.open('x') as f:json.dump(data,f,indent=2,allow_nan=False)
    tmp.replace(path)
def planes(a):
    n,d,m,e=plane_pairs(a.train)
    if np.linalg.matrix_rank(m,tol=1e-3)<3 or np.linalg.matrix_rank(n,tol=1e-3)<3: raise ValueError('Plane normals must span three dimensions')
    u,sv,vt=np.linalg.svd(m.T@n); r=u@np.diag([1,1,np.linalg.det(u@vt)])@vt
    t,res,rank,svt=np.linalg.lstsq(m,d-e,rcond=None); matrix=np.eye(4);matrix[:3,:3]=r;matrix[:3,3]=t
    normal_angles=np.degrees(np.arccos(np.clip(np.sum((n@r.T)*m,axis=1),-1,1)))
    offsets=m@t-(d-e)
    write(a.output,transform(matrix,a.parent,a.child,status='computed_validation_pending',method='plane-normal SVD and plane-offset least squares',training_sha256=sha(a.train),normal_condition_number=float(np.linalg.cond(m)),max_training_normal_deg=float(normal_angles.max()),training_offset_rms_mm=float(np.sqrt(np.mean(offsets**2))*1000)))
def validate_planes(a):
    v,t=result(a.result); n,d,m,e=plane_pairs(a.validation)
    if v.get('training_sha256')==sha(a.validation): raise ValueError('Validation file is identical to training file')
    angles=np.degrees(np.arccos(np.clip(np.sum((n@t[:3,:3].T)*m,axis=1),-1,1))); offsets=(m@t[:3,3]+e-d)*1000
    ok=bool(np.max(angles)<=a.max_angle_deg and np.max(np.abs(offsets))<=a.max_offset_mm)
    write(a.output,dict(status='passed_provisional_limits' if ok else 'failed_provisional_limits',result_sha256=sha(a.result),validation_sha256=sha(a.validation),refitted=False,normal_errors_deg=angles.tolist(),offset_errors_mm=offsets.tolist(),limits=dict(max_angle_deg=a.max_angle_deg,max_offset_mm=a.max_offset_mm)))
    if not ok: raise ValueError('Validation failed; report saved')
def mount(a):
    data={r['point_id']:np.array([float(r[k]) for k in ('x_m','y_m','z_m')]) for r in rows(a.points)}
    o,x,y=[data[k] for k in ('origin','x_axis','y_axis')]; x=x-o; y=y-o
    if not np.isfinite(np.r_[o,x,y]).all() or np.linalg.norm(x)<.01 or np.linalg.norm(y)<.01:raise ValueError('Invalid or too-close axis points')
    x/=np.linalg.norm(x); y=y-x*(x@y)
    if np.linalg.norm(y)<.01: raise ValueError('Axes nearly collinear')
    y/=np.linalg.norm(y); t=np.eye(4);t[:3,:3]=np.column_stack([x,y,np.cross(x,y)]);t[:3,3]=o
    write(a.output,transform(t,a.parent,a.child,status='computed_validation_pending',points_sha256=sha(a.points),source=a.source))
def pivot(a):
    values=read(a.samples)
    if values.get('translation_unit')!='m': raise ValueError('Samples unit must be m')
    samples=values['samples']; train=[se3(s['T_base_flange']) for s in samples if s['split']=='train']; hold=[se3(s['T_base_flange']) for s in samples if s['split']=='holdout']
    if len(train)<6 or len(hold)<3: raise ValueError('Need >=6 train and >=3 held-out actual pivot poses')
    mat=np.vstack([np.column_stack([t[:3,:3],-np.eye(3)]) for t in train]);rhs=np.concatenate([-t[:3,3] for t in train]); x,res,rank,sv=np.linalg.lstsq(mat,rhs,rcond=None)
    if rank<6 or np.linalg.cond(mat)>1e4: raise ValueError('Pivot orientations insufficient')
    errors=[float(np.linalg.norm(t[:3,:3]@x[:3]+t[:3,3]-x[3:])*1000) for t in hold];t=np.eye(4);t[:3,3]=x[:3]; t[:3,:3]=Rotation.from_euler('xyz',a.rpy).as_matrix()
    write(a.output,transform(t,a.parent,a.child,status='passed_provisional_limits' if max(errors)<=a.max_mm else 'failed_provisional_limits',orientation_estimated=False,orientation_source=a.orientation_source,source_sha256=sha(a.samples),training_count=len(train),holdout_count=len(hold),max_holdout_mm=max(errors),holdout_errors_mm=errors,pivot_point_base_m=x[3:].tolist(),condition_number=float(np.linalg.cond(mat))))
    if max(errors)>a.max_mm: raise ValueError('Pivot validation failed; result saved')
def compose(a):
    left,tl=result(a.left); right,tr=result(a.right)
    if a.inverse_right:
        tr=np.linalg.inv(tr); right={'parent_frame':right['child_frame'],'child_frame':right['parent_frame']}
    if left['child_frame']!=right['parent_frame']: raise ValueError('Frame names do not compose')
    write(a.output,transform(tl@tr,left['parent_frame'],right['child_frame'],source_sha256=[sha(a.left),sha(a.right)]))
def compare(a):
    l,tl=result(a.left);r,tr=result(a.right)
    if (l['parent_frame'],l['child_frame'])!=(r['parent_frame'],r['child_frame']): raise ValueError('Frames differ')
    write(a.output,dict(**error(tl,tr),left_sha256=sha(a.left),right_sha256=sha(a.right)))
def normalize(a):
    v=read(a.input); expected='eye_in_hand' if a.parent=='piper_link6' else 'eye_on_base'
    if v.get('synthetic') or v['mode']!=expected: raise ValueError('Synthetic result or hand-eye mode mismatch')
    write(a.output,transform(v.get('matrix',v.get('matrix4x4')),a.parent,a.child,status='computed_validation_pending',source_sha256=sha(a.input),max_holdout_translation_m=v['max_holdout_translation_m'],max_holdout_rotation_deg=v['max_holdout_rotation_deg']))
def validate_handeye(a):
    v=read(a.result);dataset=read(a.dataset)
    if v.get('synthetic') or dataset.get('synthetic') or v['mode']!=dataset['mode']:raise ValueError('Synthetic or mode mismatch')
    for split,key in [('train','training_count'),('holdout','holdout_count')]:
        if sum(s['split']==split for s in dataset['samples'])!=v[key]:raise ValueError('Counts differ from source dataset')
    se3(v.get('matrix',v.get('matrix4x4')))
    trans=float(v['max_holdout_translation_m'])*1000;angle=float(v['max_holdout_rotation_deg'])
    ok=bool(np.isfinite([trans,angle]).all() and trans<=a.max_mm and angle<=a.max_deg)
    write(a.output,dict(status='passed_provisional_limits' if ok else 'failed_provisional_limits',result_sha256=sha(a.result),dataset_sha256=sha(a.dataset),refitted=False,max_holdout_translation_mm=trans,max_holdout_rotation_deg=angle,limits=dict(max_mm=a.max_mm,max_deg=a.max_deg),hardware_deployment_approved=False))
    if not ok:raise ValueError('Hand-eye held-out validation failed; report saved')
def normalize_base(a):
    v=read(a.input)
    if (v['parent_frame'],v['child_frame'])!=('base_link','laser_frame'):raise ValueError('Unexpected Base-LiDAR frames')
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(v['quaternion_xyzw']).as_matrix();t[:3,3]=v['translation_m']
    write(a.output,transform(t,'base_link','laser_frame',status=v['status'],source_sha256=sha(a.input)))
def room_targets(a):
    world=ET.parse(a.world).getroot().find('world')
    if world is None or world.get('name')!='robocup_motion':raise ValueError('Expected robocup_motion room')
    posemsg=read(a.pose);ps=posemsg['pose'];ps=next(p for p in ps if p.get('name')=='robocup') if isinstance(ps,list) else ps
    if ps.get('name')!='robocup':raise ValueError('Expected actual robocup world pose')
    pos=ps.get('position',{});q=ps.get('orientation',{});r=Rotation.from_quat([q.get(k,0 if k!='w' else 1) for k in ('x','y','z','w')]).as_matrix();t=np.array([pos.get(k,0) for k in 'xyz'])
    # Ground z=0 is an explicit independent input; caller must verify this world floor.
    values=[('floor',np.array([0.,0.,1.]),0.)]
    walls=[('wall_east',0,-1,'front'),('wall_north',1,-1,'side')]
    if a.all_walls:walls.extend([('wall_west',0,1,'back'),('wall_south',1,1,'other_side')])
    for name,axis,side,ident in walls:
        m=world.find(f"model[@name='{name}']");link=m.find('link') if m is not None else None;c=link.find('collision') if link is not None else None
        if m is None or m.findtext('static')!='true' or c is None:raise ValueError('Static room wall missing')
        def sdf_pose(e):
            node=e.find('pose')
            if node is None:return np.zeros(6)
            if node.attrib:raise ValueError('Relative SDF pose unsupported')
            v=np.fromstring(node.text,sep=' ')
            if v.shape!=(6,):raise ValueError('Invalid SDF pose')
            return v
        wp=sdf_pose(m)
        if np.linalg.norm(wp[3:])>1e-10 or np.linalg.norm(sdf_pose(link))>1e-10 or np.linalg.norm(sdf_pose(c))>1e-10:raise ValueError('Expected axis-aligned walls without offsets')
        size=np.fromstring(c.findtext('geometry/box/size'),sep=' ');n=np.eye(3)[axis];location=wp[axis]+side*size[axis]/2;values.append((ident,n,-location))
    out=Path(a.output)
    with out.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['plane_id','nx','ny','nz','d','frame','unit','source']);w.writeheader()
        for ident,n,d in values:
            nb=r.T@n;db=d+n@t;w.writerow(dict(plane_id=ident,nx=nb[0],ny=nb[1],nz=nb[2],d=db,frame='base_link',unit='m',source=f'Room inner walls / independently verified world floor z=0; world SHA256 {sha(a.world)}; actual base pose SHA256 {sha(a.pose)}'))
    write(str(out)+'.sources.json',dict(world_sha256=sha(a.world),pose_sha256=sha(a.pose),floor_world_z_m=0,floor_verified_by_user=a.floor_verified,uses_sensor_mount_truth=False))
def tag_plane(a):
    dataset=read(a.dataset);sample=next(s for s in dataset['samples'] if s['id']==a.sample_id);ct=se3(sample['T_camera_target']);offset=se3(read(a.tag_to_plane)['matrix4x4']);t=ct@offset;n=t[:3,2];d=-n@t[:3,3]
    write(a.output,dict(plane_id=a.plane_id,nx=float(n[0]),ny=float(n[1]),nz=float(n[2]),d=float(d),frame=dataset['camera_frame'],unit='m',sample_id=a.sample_id,image_sim_time=sample['image_sim_time'],dataset_sha256=sha(a.dataset),tag_to_plane_sha256=sha(a.tag_to_plane)))
def fk(a):
    from kinematics import fk as measured_fk
    v=read(a.joints);positions={j['name']:float(j.get('axis1',{}).get('position',0.)) for j in v['joint']}
    matrix=measured_fk(a.urdf,positions,a.parent,a.child)
    write(a.output,transform(matrix,a.parent,a.child,source_sha256=sha(a.joints),urdf_sha256=sha(a.urdf),simulation_stamp=v.get('header',{}).get('stamp'),measured_joint_positions=positions))
def patch(a):
    v,t=result(a.result); tree=ET.parse(a.urdf); root=tree.getroot(); child=a.mount_child or v['child_frame'];matches=[j for j in root.findall('joint') if j.find('child').get('link')==child]
    if len(matches)!=1 or matches[0].get('type')!='fixed':raise ValueError('Mount child must have exactly one fixed parent joint; TCP requires a separately added link')
    j=matches[0];parent=j.find('parent').get('link');bp=fixed_fk(root,v['parent_frame'],parent);cs=fixed_fk(root,child,v['child_frame']);local=np.linalg.inv(bp)@t@np.linalg.inv(cs)
    origin=j.find('origin')
    if origin is None:origin=ET.SubElement(j,'origin')
    origin.set('xyz',' '.join(f'{x:.17g}' for x in local[:3,3]));origin.set('rpy',' '.join(f'{x:.17g}' for x in Rotation.from_matrix(local[:3,:3]).as_euler('xyz')))
    rebuilt=fixed_fk(root,v['parent_frame'],v['child_frame'])
    if not np.allclose(rebuilt,t,atol=1e-8):raise ValueError('Composition did not reproduce result')
    for mesh in root.findall('.//mesh'):
        name=mesh.get('filename','')
        if name and not name.startswith(('file:','package:','http:','https:')):
            resolved=(Path(a.urdf).resolve().parent/name).resolve()
            mesh.set('filename',os.path.relpath(resolved,Path(a.output).resolve().parent) if getattr(a,'portable_meshes',False) else resolved.as_uri())
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('xb') as f:tree.write(f,encoding='utf-8',xml_declaration=True)
    write(str(out)+'.application.json',dict(source_urdf_sha256=sha(a.urdf),result_sha256=sha(a.result),runtime_urdf_sha256=sha(out),joint=j.get('name'),origin_xyz=local[:3,3].tolist(),origin_rpy=Rotation.from_matrix(local[:3,:3]).as_euler('xyz').tolist(),composition_verified=True,acceptance='User must review held-out validation before launching'))
def add_tcp(a):
    v,t=result(a.result)
    if v.get('status')!='passed_provisional_limits' or v['parent_frame']!='piper_link6' or v['child_frame']!='tcp':raise ValueError('Expected passed piper_link6<-tcp pivot result')
    tree=ET.parse(a.urdf);root=tree.getroot()
    if root.find("link[@name='tcp']") is not None:raise ValueError('TCP already exists')
    ET.SubElement(root,'link',name='tcp');j=ET.SubElement(root,'joint',name='calibrated_tcp_joint',type='fixed');ET.SubElement(j,'parent',link='piper_link6');ET.SubElement(j,'child',link='tcp');ET.SubElement(j,'origin',xyz=' '.join(map(str,t[:3,3])),rpy=' '.join(map(str,Rotation.from_matrix(t[:3,:3]).as_euler('xyz'))))
    for mesh in root.findall('.//mesh'):
        name=mesh.get('filename','')
        if name and not name.startswith(('file:','package:','http:','https:')):
            resolved=(Path(a.urdf).resolve().parent/name).resolve()
            mesh.set('filename',os.path.relpath(resolved,Path(a.output).resolve().parent) if getattr(a,'portable_meshes',False) else resolved.as_uri())
    with Path(a.output).open('xb') as f:tree.write(f,encoding='utf-8',xml_declaration=True)
    write(str(a.output)+'.application.json',dict(result_sha256=sha(a.result),source_urdf_sha256=sha(a.urdf),runtime_urdf_sha256=sha(a.output),composition_verified=True))

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='cmd',required=True)
    def command(name,fn):q=sub.add_parser(name);q.set_defaults(fn=fn);return q
    def arg(q,name,**kw):q.add_argument('--'+name,required=True,**kw)
    q=command('decode-cloud',cloud);arg(q,'input');arg(q,'output')
    q=command('fit-plane',fit_plane);arg(q,'input');arg(q,'output');arg(q,'bounds',nargs=6,type=float);arg(q,'plane-id');arg(q,'frame');arg(q,'normal-axis',choices=['x','y','z']);q.add_argument('--normal-sign',type=int,choices=[-1,1],default=1);q.add_argument('--threshold',type=float,default=.01)
    q=command('pairs',pairs);arg(q,'targets');arg(q,'observed',nargs='+');arg(q,'output')
    q=command('target-table',targets);arg(q,'planes',nargs='+');arg(q,'output')
    q=command('planes',planes);arg(q,'train');arg(q,'parent');arg(q,'child');arg(q,'output')
    q=command('validate-planes',validate_planes);arg(q,'result');arg(q,'validation');arg(q,'output');q.add_argument('--max-angle-deg',type=float,default=1);q.add_argument('--max-offset-mm',type=float,default=10)
    q=command('mount',mount);arg(q,'points');arg(q,'parent');arg(q,'child');arg(q,'source');arg(q,'output')
    q=command('pivot',pivot);arg(q,'samples');arg(q,'parent');arg(q,'child');arg(q,'rpy',nargs=3,type=float);arg(q,'orientation-source');arg(q,'output');q.add_argument('--max-mm',type=float,default=5)
    q=command('compose',compose);arg(q,'left');arg(q,'right');arg(q,'output');q.add_argument('--inverse-right',action='store_true')
    q=command('compare',compare);arg(q,'left');arg(q,'right');arg(q,'output')
    q=command('normalize-handeye',normalize);arg(q,'input');arg(q,'parent');arg(q,'child');arg(q,'output')
    q=command('validate-handeye',validate_handeye);arg(q,'result');arg(q,'dataset');arg(q,'output');q.add_argument('--max-mm',type=float,default=5);q.add_argument('--max-deg',type=float,default=1)
    q=command('normalize-base-lidar',normalize_base);arg(q,'input');arg(q,'output')
    q=command('room-targets',room_targets);arg(q,'world');arg(q,'pose');arg(q,'output');q.add_argument('--floor-verified',action='store_true',required=True);q.add_argument('--all-walls',action='store_true')
    q=command('tag-plane',tag_plane);arg(q,'dataset');arg(q,'sample-id',type=int);arg(q,'tag-to-plane');arg(q,'plane-id');arg(q,'output')
    q=command('fk',fk);arg(q,'joints');arg(q,'urdf');arg(q,'parent');arg(q,'child');arg(q,'output')
    q=command('pivot-sample',pivot_sample);arg(q,'pose');arg(q,'dataset');arg(q,'split',choices=['train','holdout']);arg(q,'contact-note')
    q=command('patch-urdf',patch);arg(q,'urdf');arg(q,'result');arg(q,'output');q.add_argument('--mount-child');q.add_argument('--portable-meshes',action='store_true',help='Keep mesh paths relative to the output URDF for sharing in Git')
    q=command('add-tcp',add_tcp);arg(q,'urdf');arg(q,'result');arg(q,'output')
    from calibration_auto_planes import add_commands
    add_commands(command,arg)
    a=p.parse_args();a.fn(a)
if __name__=='__main__':
    try:main()
    except (ValueError,KeyError,OSError,StopIteration) as e: print('ERROR:',e,file=sys.stderr);sys.exit(2)
