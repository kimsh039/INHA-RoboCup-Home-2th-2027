"""Use one compiled vendor model for MuJoCo and MoveIt; do not modify vendor files."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
JOINTS = [f"joint{i}" for i in range(1,7)]


def config():
    return json.loads((ROOT/"config/grasp_simulation.json").read_text())


def numbers(values):
    return " ".join(f"{float(x):.12g}" for x in values)


def quat_rotation(q):
    return Rotation.from_quat(np.asarray(q)[[1,2,3,0]])


def transform(position, rotation):
    T = np.eye(4)
    T[:3,:3] = rotation
    T[:3,3] = position
    return T


def target_tcp(grasp, T_world_object, pre_distance=0.0):
    """TCP is physical finger tip; official grasp origin is depth behind finger tip."""
    R=np.asarray(grasp["rotation_matrix"],dtype=float)
    p=np.asarray(grasp["translation"],dtype=float)
    if R.shape!=(3,3) or p.shape!=(3,) or not np.isfinite(R).all() or not np.isfinite(p).all():
        raise ValueError("GRASP_POSE_INVALID")
    if not np.allclose(R.T@R,np.eye(3),atol=1e-5) or abs(np.linalg.det(R)-1)>1e-5:
        raise ValueError("GRASP_ROTATION_INVALID")
    if not np.isfinite(grasp["depth"]) or grasp["depth"]<=0:raise ValueError("GRASP_DEPTH_INVALID")
    T_object_grasp = transform(p,R)
    T_world_grasp = T_world_object @ T_object_grasp
    # T_tcp_grasp maps grasp coords to tutorial tip-TCP coords.
    T_tcp_grasp = transform([-grasp["depth"],0,0], np.eye(3))
    T_world_tcp = T_world_grasp @ np.linalg.inv(T_tcp_grasp)
    T_world_tcp[:3,3] -= pre_distance*T_world_grasp[:3,0]
    return T_world_tcp


def load_model():
    import mujoco
    cfg = config()
    if cfg.get("robot_model")=="inha":
        from manipulation.inha_model import build_robot
        root,_=build_robot(cfg)
    else:
        raise ValueError("ROBOT_MODEL_UNSUPPORTED: use robot_model=inha")
    # Original keyframe has nq=8; adding a free object changes nq. Initialize separately.
    for keyframe in list(root.findall("keyframe")):
        root.remove(keyframe)
    base = root.find('.//body[@name="base_link"]')
    base.set("pos", numbers(cfg["base_world_m"]))
    tip_rotation = Rotation.from_matrix(cfg["tcp_axes_in_link6"])
    tip_quat = tip_rotation.as_quat()[[3,0,1,2]]
    wrist = root.find('.//body[@name="link6"]')
    if cfg.get("robot_model")!="inha":
        ET.SubElement(wrist,"site",name="tcp",pos=numbers(cfg["tcp_tip_link6_m"]),
                      quat=numbers(tip_quat),size="0.006",rgba="0 1 0 1")
    world = root.findall("worldbody")[-1]
    ET.SubElement(world,"geom",name="table",type="box",pos=numbers(cfg["table_center_world_m"]),
                  size=numbers(cfg["table_half_size_m"]),rgba="0.45 0.32 0.20 1")
    # Static table legs reach the floor; identical obstacles are sent to MoveIt.
    center=np.array(cfg["table_center_world_m"]);half=np.array(cfg["table_half_size_m"])
    leg_height=center[2]-half[2]
    for i,(sx,sy) in enumerate(((-1,-1),(-1,1),(1,-1),(1,1))):
        leg_pos=[center[0]+sx*(half[0]-.04),center[1]+sy*(half[1]-.04),leg_height/2]
        ET.SubElement(world,"geom",name=f"table_leg_{i}",type="box",pos=numbers(leg_pos),size=numbers([.025,.025,leg_height/2]),rgba=".3 .3 .3 1")
    cube = ET.SubElement(world,"body",name="cube",pos=numbers(cfg["object_world_m"]))
    ET.SubElement(cube,"freejoint",name="cube_free")
    ET.SubElement(cube,"geom",name="cube_geom",type="box",size=numbers(np.array(json.loads((ROOT/"config/pointclouds.json").read_text())["objects"]["cube"]["dimensions_m"])/2),
                  mass=str(cfg["object_mass_kg"]),friction=numbers(cfg["object_friction"]),rgba="1 0.5 0.1 1")
    for name,color in (("grasp_marker","1 0 0 0.7"),("pregrasp_marker","0 0.5 1 0.7"),("place_marker","0 1 0 0.7")):
        marker = ET.SubElement(world,"body",name=name,mocap="true",pos="0 0 -1")
        ET.SubElement(marker,"geom",type="sphere",size="0.008",rgba=color,contype="0",conaffinity="0")
    ET.indent(root)
    (ROOT/"simulation/generated/inha_scene.xml").write_text(ET.tostring(root,encoding="unicode"))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root,encoding="unicode"))
    data = mujoco.MjData(model)
    for name,q in zip(JOINTS,cfg["initial_joints_rad"]):
        data.qpos[model.joint(name).qposadr[0]]=q
        data.ctrl[model.actuator(name).id]=q
    for name,q in (("joint7",0.035),("joint8",-0.035)):
        data.qpos[model.joint(name).qposadr[0]]=q
    data.ctrl[model.actuator("gripper").id]=0.035
    mujoco.mj_forward(model,data)
    return model,data


def body_transform(model,data,name):
    body = model.body(name).id
    return transform(data.xpos[body],data.xmat[body].reshape(3,3))


def tcp_transform(model,data):
    site = model.site("tcp").id
    return transform(data.site_xpos[site],data.site_xmat[site].reshape(3,3))


def jaw_opening(model,data):
    """Inner surfaces in the final 25 mm of fingers, including source mesh pads."""
    import mujoco
    T=tcp_transform(model,data);axis=T[:3,1];surfaces=[]
    for name in ("link7","link8"):
        points=[]
        for g in range(model.ngeom):
            if model.geom_bodyid[g]!=model.body(name).id or not model.geom_contype[g]:continue
            R=data.geom_xmat[g].reshape(3,3)
            if model.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
                m=int(model.geom_dataid[g]);start=model.mesh_vertadr[m];n=model.mesh_vertnum[m]
                v=model.mesh_vert[start:start+n]@R.T+data.geom_xpos[g]
                along=(v-T[:3,3])@T[:3,0]
                v=v[along>-.025]
            elif model.geom_type[g]==mujoco.mjtGeom.mjGEOM_BOX:
                from itertools import product
                v=np.array(list(product((-1,1),repeat=3)))*model.geom_size[g]
                v=v@R.T+data.geom_xpos[g]
            else:continue
            if len(v):points.extend(v@axis)
        if not points:raise RuntimeError("FINGER_PAD_CALIBRATION_FAILED")
        surfaces.append(np.asarray(points))
    left,right=sorted(surfaces,key=lambda x:float(np.mean(x)))
    return float(right.min()-left.max())


def write_moveit_files(model):
    """Export compiled body/joint frames and actual collision primitives, including capsule caps."""
    import mujoco
    cfg=config()
    if cfg.get("robot_model")=="inha":
        from scipy.spatial import ConvexHull
        out=ROOT/"simulation/generated"
        robot=ET.parse(out/"inha_source.urdf").getroot()
        for link in robot.findall("link"):
            name=link.get("name")
            if name in ("world","tcp"):continue
            for old in list(link.findall("collision")):link.remove(old)
            b=model.body(name).id
            for g in range(model.ngeom):
                if model.geom_bodyid[g]!=b or not model.geom_contype[g]:continue
                elem=ET.SubElement(link,"collision")
                ET.SubElement(elem,"origin",xyz=numbers(model.geom_pos[g]),rpy=numbers(quat_rotation(model.geom_quat[g]).as_euler("xyz")))
                geometry=ET.SubElement(elem,"geometry");kind=int(model.geom_type[g]);size=model.geom_size[g]
                if kind==int(mujoco.mjtGeom.mjGEOM_MESH):
                    m=int(model.geom_dataid[g]);a=model.mesh_vertadr[m];n=model.mesh_vertnum[m]
                    v=model.mesh_vert[a:a+n];hull=ConvexHull(v)
                    file=out/"meshes"/f"collision_hull_{g}.obj"
                    with file.open("w") as f:
                        for point in v:f.write("v "+numbers(point)+"\n")
                        center=np.mean(v[hull.vertices],axis=0)
                        for face in hull.simplices:
                            face=face.copy()
                            if np.dot(np.cross(v[face[1]]-v[face[0]],v[face[2]]-v[face[0]]),v[face[0]]-center)<0:face=face[[0,2,1]]
                            f.write("f "+" ".join(str(int(i)+1) for i in face)+"\n")
                    ET.SubElement(geometry,"mesh",filename=str(file))
                elif kind==int(mujoco.mjtGeom.mjGEOM_BOX):ET.SubElement(geometry,"box",size=numbers(2*size))
                elif kind==int(mujoco.mjtGeom.mjGEOM_CYLINDER):ET.SubElement(geometry,"cylinder",radius=str(size[0]),length=str(2*size[1]))
                elif kind==int(mujoco.mjtGeom.mjGEOM_SPHERE):ET.SubElement(geometry,"sphere",radius=str(size[0]))
                else:raise RuntimeError(f"Unsupported assembly collision {kind}")
        # ROS resource_retriever expects URI paths, including percent-encoded UTF-8.
        for mesh in robot.findall(".//mesh"):
            filename=mesh.get("filename")
            if not filename.startswith("file://"):mesh.set("filename",Path(filename).as_uri())
        ET.indent(robot);(out/"piper.urdf").write_text(ET.tostring(robot,encoding="unicode"))
        return out
    out=ROOT/"simulation/generated"
    out.mkdir(exist_ok=True)
    robot=ET.Element("robot",name="piper_tutorial")
    ET.SubElement(robot,"link",name="world")
    def origin(parent,pos,quat):
        ET.SubElement(parent,"origin",xyz=numbers(pos),rpy=numbers(quat_rotation(quat).as_euler("xyz")))
    robot_ids=[model.body(name).id for name in ("base_link",*[f"link{i}" for i in range(1,9)])]
    for b in robot_ids:
        name=model.body(b).name
        link=ET.SubElement(robot,"link",name=name)
        inertial=ET.SubElement(link,"inertial")
        origin(inertial,model.body_ipos[b],model.body_iquat[b])
        ET.SubElement(inertial,"mass",value=str(float(model.body_mass[b])))
        I=model.body_inertia[b]
        ET.SubElement(inertial,"inertia",ixx=str(I[0]),iyy=str(I[1]),izz=str(I[2]),ixy="0",ixz="0",iyz="0")
        for g in range(model.ngeom):
            if model.geom_bodyid[g]!=b or not model.geom_contype[g]:
                continue
            kind=int(model.geom_type[g]);size=model.geom_size[g]
            parts=[(model.geom_pos[g],kind,size)]
            if kind==int(mujoco.mjtGeom.mjGEOM_CAPSULE):
                parts=[(model.geom_pos[g],int(mujoco.mjtGeom.mjGEOM_CYLINDER),size)]
                direction=quat_rotation(model.geom_quat[g]).apply([0,0,1])
                parts += [(model.geom_pos[g]+s*size[1]*direction,int(mujoco.mjtGeom.mjGEOM_SPHERE),size) for s in (-1,1)]
            for pos,part_kind,sizes in parts:
                for tag in ("collision","visual"):
                    elem=ET.SubElement(link,tag);origin(elem,pos,model.geom_quat[g]);geometry=ET.SubElement(elem,"geometry")
                    if part_kind==int(mujoco.mjtGeom.mjGEOM_BOX):
                        ET.SubElement(geometry,"box",size=numbers(2*sizes))
                    elif part_kind==int(mujoco.mjtGeom.mjGEOM_SPHERE):
                        ET.SubElement(geometry,"sphere",radius=str(sizes[0]))
                    elif part_kind==int(mujoco.mjtGeom.mjGEOM_CYLINDER):
                        ET.SubElement(geometry,"cylinder",radius=str(sizes[0]),length=str(2*sizes[1]))
                    else:
                        raise RuntimeError(f"Unsupported collision geometry {kind}; refusing approximate export")
        jadr=int(model.body_jntadr[b])
        parent=model.body(int(model.body_parentid[b])).name or "world"
        joint=ET.SubElement(robot,"joint",name=model.joint(jadr).name if jadr>=0 else "world_to_base",
                            type=("prismatic" if int(model.jnt_type[jadr])==2 else "revolute") if jadr>=0 else "fixed")
        ET.SubElement(joint,"parent",link=parent);ET.SubElement(joint,"child",link=name)
        origin(joint,model.body_pos[b],model.body_quat[b])
        if jadr>=0:
            if not np.allclose(model.jnt_pos[jadr],0):
                raise RuntimeError("Nonzero joint position needs a split link; refusing incorrect URDF")
            ET.SubElement(joint,"axis",xyz=numbers(model.jnt_axis[jadr]))
            ET.SubElement(joint,"limit",lower=str(model.jnt_range[jadr,0]),upper=str(model.jnt_range[jadr,1]),
                          effort="10" if name in ("link7","link8") else "100",velocity=str(cfg["joint_velocity_limit_rad_s"]))
            if name=="link8": ET.SubElement(joint,"mimic",joint="joint7",multiplier="-1",offset="0")
    ET.SubElement(robot,"link",name="tcp")
    fixed=ET.SubElement(robot,"joint",name="tcp_fixed",type="fixed")
    ET.SubElement(fixed,"parent",link="link6");ET.SubElement(fixed,"child",link="tcp")
    rotation=Rotation.from_matrix(cfg["tcp_axes_in_link6"])
    ET.SubElement(fixed,"origin",xyz=numbers(cfg["tcp_tip_link6_m"]),rpy=numbers(rotation.as_euler("xyz")))
    srdf=ET.Element("robot",name="piper_tutorial")
    arm=ET.SubElement(srdf,"group",name="arm");ET.SubElement(arm,"chain",base_link="base_link",tip_link="tcp")
    gripper=ET.SubElement(srdf,"group",name="gripper")
    ET.SubElement(gripper,"joint",name="joint7");ET.SubElement(gripper,"joint",name="joint8")
    ET.SubElement(srdf,"end_effector",name="gripper",parent_link="tcp",group="gripper",parent_group="arm")
    for b in robot_ids:
        parent=model.body(int(model.body_parentid[b])).name
        if parent and parent!="world":
            ET.SubElement(srdf,"disable_collisions",link1=parent,link2=model.body(b).name,reason="Adjacent: MuJoCo parent filter")
    ET.SubElement(srdf,"disable_collisions",link1="link6",link2="tcp",reason="Adjacent")
    for name,tree in (("piper.urdf",robot),("piper.srdf",srdf)):
        ET.indent(tree);(out/name).write_text(ET.tostring(tree,encoding="unicode"))
    return out


def verify_exported_fk(model,data):
    """Compare exported URDF with MuJoCo FK before allowing trajectory execution."""
    import mujoco
    robot=ET.parse(ROOT/"simulation/generated/piper.urdf").getroot()
    joints=list(robot.findall("joint"))
    scratch=mujoco.MjData(model);scratch.qpos[:]=data.qpos
    rng=np.random.default_rng(42);max_position_error=0.;max_rotation_error=0.
    for sample in range(20):
        values={}
        for name in [*JOINTS,"joint7"]:
            j=model.joint(name)
            values[name]=float(rng.uniform(*j.range))
            scratch.qpos[j.qposadr[0]]=values[name]
        values["joint8"]=-values["joint7"];scratch.qpos[model.joint("joint8").qposadr[0]]=values["joint8"]
        mujoco.mj_forward(model,scratch)
        frames={"world":np.eye(4)}
        for joint in joints:
            parent=joint.find("parent").attrib["link"];child=joint.find("child").attrib["link"]
            origin=joint.find("origin")
            T=transform(np.fromstring(origin.attrib["xyz"],sep=" "),Rotation.from_euler("xyz",np.fromstring(origin.attrib["rpy"],sep=" ")).as_matrix())
            if joint.attrib["type"]!="fixed":
                axis=np.fromstring(joint.find("axis").attrib["xyz"],sep=" ");value=values[joint.attrib["name"]]
                motion=transform(axis*value,np.eye(3)) if joint.attrib["type"]=="prismatic" else transform([0,0,0],Rotation.from_rotvec(axis*value).as_matrix())
                T=T@motion
            frames[child]=frames[parent]@T
        for name in ([model.body(b).name for b in range(1,model.nbody) if model.body(b).name not in ("cube","grasp_marker","pregrasp_marker","place_marker")]+["tcp"]):
            expected=tcp_transform(model,scratch) if name=="tcp" else body_transform(model,scratch,name)
            actual=frames[name]
            max_position_error=max(max_position_error,float(np.max(np.abs(expected[:3,3]-actual[:3,3]))))
            max_rotation_error=max(max_rotation_error,float(np.max(np.abs(expected[:3,:3]-actual[:3,:3]))))
    if max_position_error>1e-7 or max_rotation_error>1e-7:
        raise RuntimeError(f"URDF_MUJOCO_FK_MISMATCH: {max_position_error}, {max_rotation_error}")
    report={"samples":20,"max_position_error_m":max_position_error,"max_rotation_matrix_error":max_rotation_error,
            "result":"FK_PASS","collision_export":"source assembly meshes/primitives; MuJoCo mesh contacts are convex hulls",
            "tcp":"model-derived tip frame; hardware calibration not validated"}
    (ROOT/"reports/moveit_model_check.json").write_text(json.dumps(report,indent=2)+"\n")
    return report
