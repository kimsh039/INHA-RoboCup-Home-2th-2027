#!/usr/bin/env python3
"""Create separate Head fixtures from the existing Wrist world. CAD is setup-only, not solver input."""
import argparse,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from kinematics import fk
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mode',choices=['head-arm','head-mid'],required=True);p.add_argument('--project',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
source=a.project/'cli/world/wrist.world.sdf';urdf=a.project/'robot/simulation/robot_description/robocup.urdf';tree=ET.parse(source);w=tree.getroot().find('world');robot=w.find("model[@name='robocup']")
if a.output.exists():raise SystemExit('Preserve existing scene; use a new output path')
for m in list(w.findall('model')):
    if m.get('name')=='calibration_target':w.remove(m)
for uri in tree.getroot().findall('.//uri')+tree.getroot().findall('.//albedo_map'):
    name=(uri.text or '').strip()
    if name and not name.startswith(('model:','file:','http:','https:','package:')):uri.text=str((source.parent/name).resolve())
positions={j.get('name'):0. for j in ET.parse(urdf).getroot().findall('joint') if j.get('type')!='fixed'}
head=fk(urdf,positions,'base_link','camera_optical_frame');flange=fk(urdf,positions,'base_link','piper_link6');fixture=np.eye(4);fixture[:3,:3]=np.diag([1,-1,-1]);fixture[2,3]=2. if a.mode=='head-mid' else .35;base_fixture=head@fixture
def pose(t):return ' '.join(map(str,[*t[:3,3],*Rotation.from_matrix(t[:3,:3]).as_euler('xyz')]))
if a.mode=='head-arm':
    link=robot.find("link[@name='piper_link6']")
    if link is None:raise SystemExit('PiPER flange link missing')
    local=np.linalg.inv(flange)@base_fixture
else:
    model=ET.SubElement(w,'model',name='calibration_board');ET.SubElement(model,'static').text='true';world_base=np.eye(4);world_base[:3,3]=np.fromstring(robot.findtext('pose'),sep=' ')[:3];ET.SubElement(model,'pose').text=pose(world_base@base_fixture);link=ET.SubElement(model,'link',name='board');local=np.eye(4)
# Plate top is z=0; tag is 0.1mm above top to avoid z-fighting.
plate_local=local.copy();plate_local[:3,3]+=local[:3,:3]@np.array([0,0,-.005]);tag_local=local.copy();tag_local[:3,3]+=local[:3,:3]@np.array([0,0,.0001])
for kind in ('collision','visual'):
    el=ET.SubElement(link,kind,name='calibration_plate_'+kind+'_'+a.mode);ET.SubElement(el,'pose').text=pose(plate_local);box=ET.SubElement(ET.SubElement(el,'geometry'),'box');ET.SubElement(box,'size').text='.8 .8 .01' if a.mode=='head-mid' else '.12 .12 .01'
    if kind=='visual':
        material=ET.SubElement(el,'material');ET.SubElement(material,'ambient').text='.8 .8 .8 1';ET.SubElement(material,'diffuse').text='.8 .8 .8 1'
visual=ET.SubElement(link,'visual',name='calibration_apriltag_'+a.mode);ET.SubElement(visual,'pose').text=pose(tag_local);plane=ET.SubElement(ET.SubElement(visual,'geometry'),'plane');ET.SubElement(plane,'normal').text='0 0 1';ET.SubElement(plane,'size').text='.1 .1';mat=ET.SubElement(visual,'material');ET.SubElement(mat,'ambient').text='1 1 1 1';ET.SubElement(mat,'diffuse').text='1 1 1 1';metal=ET.SubElement(ET.SubElement(mat,'pbr'),'metal');ET.SubElement(metal,'albedo_map').text=str((a.project/'cli/board/apriltag_36h11_id0.png').resolve());ET.SubElement(metal,'metalness').text='0';ET.SubElement(metal,'roughness').text='1'
a.output.parent.mkdir(parents=True,exist_ok=True);ET.indent(tree);tree.write(a.output,encoding='utf-8',xml_declaration=True)
tagplane=np.eye(4);tagplane[2,3]=-.0001
manifest=dict(setup_only=True,mode=a.mode,tag_family='AprilTag 36h11',tag_id=0,tag_black_edge_m=.08,source_world=str(source),camera_mount_cad_used_only_for_fixture_placement=True,solver_must_use_image_pnp_and_measured_joints_or_cloud=True,live_scene_visibility_and_collision_validated=False)
a.output.with_suffix('.setup.json').write_text(json.dumps(manifest,indent=2)+'\n');a.output.with_suffix('.tag_to_plane.json').write_text(json.dumps(dict(matrix4x4=tagplane.tolist(),translation_unit='m',source='Generated fixture: tag visual is 0.1mm above board collision top'),indent=2)+'\n')
print(a.output)
