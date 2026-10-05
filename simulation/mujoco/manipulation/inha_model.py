"""Import the supplied assembly without editing vendor URDF or meshes.

Base/wheel joints are fixed for the stationary manipulation tutorial. Mesh contacts
use MuJoCo convex hulls, not concave CAD interiors; this limitation is recorded.
"""
from pathlib import Path
import hashlib
import json
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT.parent/'robot_description/robocup.urdf'  # 팀 공용 URDF와 HW 메시를 직접 참조한다.
OUT=ROOT/'simulation/generated'
ALIASES={'piper_base_link':'arm_base', **{f'piper_link{i}':f'link{i}' for i in range(1,7)},
         'piper_gripper_link1':'link7','piper_gripper_link2':'link8',
         **{f'piper_joint{i}':f'joint{i}' for i in range(1,7)},
         'piper_gripper_joint1':'joint7','piper_gripper_joint2':'joint8'}

def nums(a):return ' '.join(f'{float(v):.12g}' for v in a)
def vec(s,default='0 0 0'):return np.fromstring(s or default,sep=' ')
def origin(e):
    o=e.find('origin');p=vec(o.get('xyz') if o is not None else None)
    R=Rotation.from_euler('xyz',vec(o.get('rpy') if o is not None else None))
    return p,R

def dae_to_obj(source):
    """Strict triangle-only COLLADA converter; applies scene matrices and SI unit."""
    out=OUT/'meshes'/f'{source.stem}_{hashlib.sha256(str(source).encode()).hexdigest()[:10]}.obj'
    out.parent.mkdir(parents=True,exist_ok=True)
    if out.exists():return out
    root=ET.parse(source).getroot();ns={'c':root.tag.split('}')[0][1:]}
    unit=root.find('c:asset/c:unit',ns);scale=float(unit.get('meter','1')) if unit is not None else 1.
    up=root.find('c:asset/c:up_axis',ns)
    if up is not None and up.text not in ('Z_UP','Y_UP'):raise ValueError('Unsupported COLLADA axis')
    # Preserve the mesh coordinate system used by URDF; up_axis describes the asset,
    # not a requested rotation of the URDF link frame.
    geometries={}
    for g in root.findall('c:library_geometries/c:geometry',ns):
        mesh=g.find('c:mesh',ns);sources={}
        for s in mesh.findall('c:source',ns):
            array=s.find('c:float_array',ns);acc=s.find('c:technique_common/c:accessor',ns)
            if array is not None:sources[s.get('id')]=np.fromstring(array.text,sep=' ').reshape(-1,int(acc.get('stride','1')))[:,:3]
        vertices={v.get('id'):v.find("c:input[@semantic='POSITION']",ns).get('source')[1:] for v in mesh.findall('c:vertices',ns)}
        if mesh.find('c:polylist',ns) is not None:raise ValueError('COLLADA polygon conversion unsupported')
        triangles=[]
        for t in mesh.findall('c:triangles',ns):
            inputs=t.findall('c:input',ns);stride=max(int(i.get('offset','0')) for i in inputs)+1
            item=next(i for i in inputs if i.get('semantic') in ('VERTEX','POSITION'))
            sid=item.get('source')[1:];sid=vertices.get(sid,sid)
            indices=np.fromstring(t.find('c:p',ns).text,sep=' ',dtype=int).reshape(-1,stride)[:,int(item.get('offset','0'))]
            triangles.append(sources[sid][indices].reshape(-1,3,3))
        geometries[g.get('id')]=np.concatenate(triangles)
    transformed=[]
    def visit(node,parent):
        T=parent.copy()
        for x in node:
            tag=x.tag.split('}')[-1]
            if tag=='matrix':T=T@np.fromstring(x.text,sep=' ').reshape(4,4).T
            elif tag in ('translate','rotate','scale'):raise ValueError('Unexpected COLLADA scene transform')
        for x in node.findall('c:instance_geometry',ns):
            a=geometries[x.get('url')[1:]];transformed.append((a@T[:3,:3].T+T[:3,3])*scale)
        for x in node.findall('c:node',ns):visit(x,T)
    scene_id=root.find('c:scene/c:instance_visual_scene',ns).get('url')[1:]
    scene=root.find(f"c:library_visual_scenes/c:visual_scene[@id='{scene_id}']",ns)
    for n in scene.findall('c:node',ns):visit(n,np.eye(4))
    a=np.concatenate(transformed).reshape(-1,3)
    with out.open('w') as f:
        for p in a:f.write('v '+nums(p)+'\n')
        for i in range(0,len(a),3):f.write(f'f {i+1} {i+2} {i+3}\n')
    return out

def stl_vertices(path):
    raw=path.read_bytes();n=struct.unpack('<I',raw[80:84])[0]
    if len(raw)!=84+50*n:raise ValueError(f'Expected binary STL: {path}')
    dt=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attr','<u2')])
    return np.frombuffer(raw,dt,offset=84,count=n)['vertices'].reshape(-1,3).astype(float)

def mesh_file(file):
    if file.suffix.lower()=='.dae':return dae_to_obj(file)
    if file.suffix.lower()!='.stl':return file
    raw=file.read_bytes()
    n=struct.unpack('<I',raw[80:84])[0] if len(raw)>84 else 0
    if len(raw)==84+50*n and n<=200000:return file
    points=stl_vertices(file).tolist() if len(raw)==84+50*n else []
    if not points:
        for line in raw.decode().splitlines():
            tokens=line.split()
            if tokens and tokens[0]=='vertex':points.append([float(x) for x in tokens[1:]])
    if not points or len(points)%3:raise ValueError(f'Invalid STL: {file}')
    out=OUT/'meshes'/f'{file.stem}_{hashlib.sha256(str(file).encode()).hexdigest()[:10]}.obj'
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('w') as f:
        for v in points:f.write('v '+nums(v)+'\n')
        for i in range(0,len(points),3):f.write(f'f {i+1} {i+2} {i+3}\n')
    return out

def build_robot(cfg):
    OUT.mkdir(parents=True,exist_ok=True)
    urdf=ET.parse(SOURCE).getroot()
    for e in urdf.iter():
        for k in ('name','link','joint'):
            if e.get(k) in ALIASES:e.set(k,ALIASES[e.get(k)])
    # Resolve all assets to actual source or converted file, also for MoveIt.
    for mesh in urdf.findall('.//mesh'):
        file=(SOURCE.parent/mesh.get('filename')).resolve()
        if not file.exists():raise FileNotFoundError(file)
        mesh.set('filename',str(mesh_file(file)))
    links={l.get('name'):l for l in urdf.findall('link')};joints=urdf.findall('joint')
    moving=[j for j in joints if j.get('name') in [f'joint{i}' for i in range(1,9)]]
    for j in joints:
        if j not in moving:j.set('type','fixed')
    mimic=ET.SubElement(next(j for j in moving if j.get('name')=='joint8'),'mimic',joint='joint7',multiplier='-1',offset='0')
    root=ET.Element('mujoco',model='inha_robocup_tutorial')
    ET.SubElement(root,'compiler',angle='radian',fusestatic='false')
    # 평평한 손가락 메시의 면 접촉을 여러 점으로 계산하고 연성 접촉의 마찰 드리프트를 보정한다.
    option=ET.SubElement(root,'option',timestep='0.002',gravity='0 0 -9.81',integrator='implicitfast',
                         noslip_iterations=str(cfg.get('contact_noslip_iterations',0)))
    ET.SubElement(option,'flag',multiccd='enable' if cfg.get('contact_multiccd',False) else 'disable')
    assets=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='0 0 3',dir='0 0 -1')
    ET.SubElement(world,'geom',name='floor',type='plane',size='4 4 .1',rgba='.85 .85 .85 1')
    used={};bodies={};mesh_map={}
    def create(name,parent,j=None):
        p,R=origin(j) if j is not None else (np.array(cfg['base_world_m']),Rotation.identity())
        b=ET.SubElement(parent,'body',name=name,pos=nums(p),quat=nums(R.as_quat()[[3,0,1,2]]));bodies[name]=b
        link=links[name]
        inertial=link.find('inertial')
        if inertial is not None:
            ip,iR=origin(inertial);I=inertial.find('inertia').attrib
            M=np.array([[float(I['ixx']),float(I['ixy']),float(I['ixz'])],[float(I['ixy']),float(I['iyy']),float(I['iyz'])],[float(I['ixz']),float(I['iyz']),float(I['izz'])]])
            M=iR.as_matrix()@M@iR.as_matrix().T
            if float(inertial.find('mass').get('value'))>0:
                ET.SubElement(b,'inertial',pos=nums(ip),mass=inertial.find('mass').get('value'),fullinertia=nums([M[0,0],M[1,1],M[2,2],M[0,1],M[0,2],M[1,2]]))
        if j is not None and j in moving:
            lim=j.find('limit');axis=vec(j.find('axis').get('xyz'))
            ET.SubElement(b,'joint',name=j.get('name'),type='slide' if j.get('type')=='prismatic' else 'hinge',axis=nums(axis),range=f"{lim.get('lower')} {lim.get('upper')}",damping='1' if j.get('type')=='prismatic' else '2',armature='.01')
        for tag in ('visual','collision'):
            for idx,g in enumerate(link.findall(tag)):
                gp,gR=origin(g);shape=list(g.find('geometry'))[0]
                a={'name':f'{name}_{tag}_{idx}','pos':nums(gp),'quat':nums(gR.as_quat()[[3,0,1,2]]),'rgba':'.45 .48 .52 1' if tag=='visual' else '.2 .4 .6 .3'}
                if tag=='visual':a.update(contype='0',conaffinity='0',group='2',mass='0')
                else:a.update(group='3',friction='1 .005 .0001')
                if shape.tag=='mesh':
                    file=shape.get('filename');s=shape.get('scale','1 1 1');key=(file,s)
                    if key not in used:
                        m=f'mesh_{len(used)}';used[key]=m;mesh_map[m]=file
                        ET.SubElement(assets,'mesh',name=m,file=file,scale=s)
                    a.update(type='mesh',mesh=used[key])
                elif shape.tag=='box':a.update(type='box',size=nums(vec(shape.get('size'))/2))
                elif shape.tag=='cylinder':a.update(type='cylinder',size=nums([float(shape.get('radius')),float(shape.get('length'))/2]))
                elif shape.tag=='sphere':a.update(type='sphere',size=shape.get('radius'))
                else:raise ValueError(shape.tag)
                ET.SubElement(b,'geom',**a)
        for child in [x for x in joints if x.find('parent').get('link')==name]:create(child.find('child').get('link'),b,child)
    create('base_link',world)
    # Tip position derived from source finger collision mesh in gripper-base frame.
    j=next(j for j in moving if j.get('name')=='joint7');jp,jR=origin(j)
    finger=links['link7'].find('collision');fp,fR=origin(finger)
    vertices=stl_vertices(Path(finger.find('geometry/mesh').get('filename')))
    points=jR.apply(fR.apply(vertices)+fp)+jp
    tip=float(points[:,2].max())
    tipR=Rotation.from_matrix(cfg['tcp_axes_in_link6'])
    ET.SubElement(bodies['piper_gripper_base'],'site',name='tcp',pos=nums([0,0,tip]),quat=nums(tipR.as_quat()[[3,0,1,2]]),size='.006',rgba='0 1 0 1')
    actuator=ET.SubElement(root,'actuator')
    # Tutorial actuators: explicit added position servos, force limited by URDF effort.
    for j in moving:
        if j.get('name')=='joint8':continue
        lim=j.find('limit');arm=j.get('name')!='joint7'
        ET.SubElement(actuator,'position',name=j.get('name') if arm else 'gripper',joint=j.get('name'),kp='1000' if arm else '200',kv='80' if arm else '10',ctrlrange=f"{lim.get('lower')} {lim.get('upper')}",forcerange=f"-{lim.get('effort')} {lim.get('effort')}")
    equality=ET.SubElement(root,'equality');ET.SubElement(equality,'joint',joint1='joint8',joint2='joint7',polycoef='0 -1 0 0 0')
    contact=ET.SubElement(root,'contact')
    # MuJoCo ignores rigidly connected body pairs. Mirror this in MoveIt's SRDF.
    fixed_parent={j.find('child').get('link'):j.find('parent').get('link') for j in joints if j.get('type')=='fixed'}
    def component(n):
        while n in fixed_parent:n=fixed_parent[n]
        return n
    comps={n:component(n) for n in links}
    disabled=set()
    for i,a in enumerate(links):
        for b in list(links)[i+1:]:
            if comps[a]==comps[b]:disabled.add((a,b))
    for j in moving:
        parent=j.find('parent').get('link');child=j.find('child').get('link')
        for a in links:
            for b in links:
                if comps[a]==comps[parent] and comps[b]==comps[child]:disabled.add((a,b))
    for a,b in sorted(disabled):ET.SubElement(contact,'exclude',body1=a,body2=b)
    ET.SubElement(urdf,'link',name='world')
    fixed=ET.SubElement(urdf,'joint',name='world_to_robot',type='fixed');ET.SubElement(fixed,'parent',link='world');ET.SubElement(fixed,'child',link='base_link');ET.SubElement(fixed,'origin',xyz=nums(cfg['base_world_m']),rpy='0 0 0')
    ET.SubElement(urdf,'link',name='tcp');fixed=ET.SubElement(urdf,'joint',name='tcp_fixed',type='fixed');ET.SubElement(fixed,'parent',link='piper_gripper_base');ET.SubElement(fixed,'child',link='tcp');ET.SubElement(fixed,'origin',xyz=nums([0,0,tip]),rpy=nums(tipR.as_euler('xyz')))
    # Reorder joints in parent-before-child order for deterministic FK checking.
    ordered=[];known={'world'};pending=list(urdf.findall('joint'))
    while pending:
        ready=[j for j in pending if j.find('parent').get('link') in known]
        if not ready:raise ValueError('URDF tree disconnected')
        for j in ready:ordered.append(j);known.add(j.find('child').get('link'));pending.remove(j)
    for j in list(urdf.findall('joint')):urdf.remove(j)
    urdf.extend(ordered)
    srdf=ET.Element('robot',name=urdf.get('name'))
    group=ET.SubElement(srdf,'group',name='arm');ET.SubElement(group,'chain',base_link='arm_base',tip_link='tcp')
    group=ET.SubElement(srdf,'group',name='gripper');ET.SubElement(group,'joint',name='joint7');ET.SubElement(group,'joint',name='joint8')
    ET.SubElement(srdf,'end_effector',name='gripper',parent_link='tcp',group='gripper',parent_group='arm')
    for a,b in sorted(disabled):ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='Rigid assembly or adjacent moving component')
    for tag,tree in [('inha_source.urdf',urdf),('piper.srdf',srdf)]:
        ET.indent(tree);(OUT/tag).write_text(ET.tostring(tree,encoding='unicode'))
    report={'source_repository':'https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027','commit':'0e0a28d7866832ecf1e0841bd41760ebde672a79','source_urdf':str(SOURCE),'aliases':ALIASES,'tcp_parent':'piper_gripper_base','tcp_tip_m':[0,0,tip],'robot_links':list(links),'collision_mesh_policy':'MuJoCo convex hull; matching compiled convex hulls exported for MoveIt','modifications':['fixed mobile base and wheels at zero','implicitfast integrator for added damped position servos','added effort-limited tutorial position servos kp=1000 kv=80 arm, kp=200 kv=10 gripper','joint damping 2 arm / 1 gripper, armature .01','added opposite gripper coupling','DAE triangle meshes converted to OBJ including scene matrices; uniform visual color','rigid/adjacent collision exclusions','joint/link aliases for existing tutorial API'],'mesh_files':mesh_map}
    (OUT/'inha_import.json').write_text(json.dumps(report,indent=2)+'\n')
    return root,report
