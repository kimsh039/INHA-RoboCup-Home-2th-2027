import adsk.core,adsk.fusion,json,os,traceback
OUT=r'C:\Users\seung\Documents\INHA\RoboCup\urdf_export\camera_tilt15_20261010'
BASE=[-15.271129415510844,-13.499999999999739,-4.777222609636355]
BASE_ARCHIVE=r'C:\Users\seung\Documents\INHA\RoboCup\urdf_export\update_20261006\rack_update.f3d'
def vec(p):return [p.x,p.y,p.z]
def trans(p):
 m=adsk.core.Matrix3D.create();m.translation=adsk.core.Vector3D.create(*p);return m
def props(comp):
 p=comp.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy);m=p.mass;c=vec(p.centerOfMass);ok,xx,yy,zz,xy,yz,xz=p.getXYZMomentsOfInertia();x,y,z=c
 if not ok:raise RuntimeError('Physical properties unavailable')
 return {'mass_kg':m,'com_m':[a*.01 for a in c],'inertia_kg_m2':[a*.0001 for a in [xx-m*(y*y+z*z),yy-m*(x*x+z*z),zz-m*(x*x+y*y),xy+m*x*y,yz+m*y*z,xz+m*x*z]]}
def run(context):
 app=adsk.core.Application.get();ui=app.userInterface
 os.makedirs(OUT,exist_ok=True)
 log=open(os.path.join(OUT,'mesh_export_progress.txt'),'w',encoding='utf-8')
 def note(s):log.write(s+'\n');log.flush();adsk.doEvents()
 try:
  source=next(d for d in app.documents if d.name=='final_assembly_fix');source.activate();src=adsk.fusion.Design.cast(app.activeProduct)
  camera=next(o for o in src.rootComponent.allOccurrences if o.name.startswith('D435f'))
  mount=next(o for o in src.rootComponent.allOccurrences if o.name.startswith('3D_LiDAR_Camera_mount_15'))
  cam_bodies=[(b.nativeObject or b,camera.transform2.copy(),b.name) for b in camera.bRepBodies]
  mount_bodies=[(b.nativeObject or b,mount.transform2.copy(),b.name) for b in mount.bRepBodies]
  audit={'active_document':source.name,'documents':[]}
  for docname in ('final_assembly','final_assembly_fix'):
   doc=next(d for d in app.documents if d.name==docname);doc.activate();des=adsk.fusion.Design.cast(app.activeProduct)
   entry={'document':doc.name,'occurrences':[]}
   for o in des.rootComponent.allOccurrences:
    item={'name':o.name,'path':o.fullPathName,'component':o.component.name,'transform':o.transform2.asArray(),'bodies':[]}
    for b in o.bRepBodies:
     bb=b.preciseBoundingBox;item['bodies'].append({'name':b.name,'solid':b.isSolid,'bbox_cm':[vec(bb.minPoint),vec(bb.maxPoint)]})
    entry['occurrences'].append(item)
   audit['documents'].append(entry)
  oldcam=next(o for o in audit['documents'][0]['occurrences'] if o['name'].startswith('D435f'))['transform']
  newcam=camera.transform2.asArray()
  R=[[sum(newcam[i*4+k]*oldcam[j*4+k] for k in range(3)) for j in range(3)] for i in range(3)]
  frame_values=[R[0][0],R[0][1],R[0][2],newcam[3],R[1][0],R[1][1],R[1][2],newcam[7],R[2][0],R[2][1],R[2][2],newcam[11],0.,0.,0.,1.]
  json.dump(frame_values,open(os.path.join(OUT,'camera_frame_cm.json'),'w'))
  json.dump(audit,open(os.path.join(OUT,'current_audit.json'),'w',encoding='utf-8'),indent=2,ensure_ascii=False)
  source.activate()
  src.exportManager.execute(src.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'source_current.f3d')))
  note('Import baseline rack archive')
  baseline_doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(BASE_ARCHIVE))
  baseline=adsk.fusion.Design.cast(app.activeProduct)
  baseline_comp=next(o.component for o in baseline.rootComponent.allOccurrences if o.component.name=='rack_base_link')
  if baseline_comp.bRepBodies.count!=51:raise RuntimeError('Expected baseline rack with 51 bodies')
  original_props=props(baseline_comp)
  note('Copy baseline bodies preserving original positions')
  export_doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType);export_doc.name='camera_tilt15_export'
  design=adsk.fusion.Design.cast(app.activeProduct);design.designIntent=adsk.fusion.DesignIntentTypes.HybridDesignIntentType;design.designType=adsk.fusion.DesignTypes.DirectDesignType
  root=design.rootComponent;tmp=adsk.fusion.TemporaryBRepManager.get()
  rack=root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component;rack.name='rack_base_link'
  original_mount=[];preserved=[]
  for b in baseline_comp.bRepBodies:
   if b.name.startswith('3D_LiDAR_Camera_mount'):
    original_mount.append(b);continue
   copy=rack.bRepBodies.add(tmp.copy(b));copy.name=b.name;copy.material=b.material
   bb=copy.preciseBoundingBox;preserved.append({'name':b.name,'bbox_m':[[q*.01 for q in vec(bb.minPoint)],[q*.01 for q in vec(bb.maxPoint)]]})
  if len(original_mount)!=1:raise RuntimeError('Expected exactly one original mount body')
  material=original_mount[0].material
  for b,T,name in mount_bodies:
   clone=tmp.copy(b);tmp.transform(clone,T);tmp.transform(clone,trans([-x for x in BASE]));copy=rack.bRepBodies.add(clone);copy.name='3D_LiDAR_Camera_mount_15_body';copy.material=material
  cam=root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component;cam.name='camera_link'
  frame=adsk.core.Matrix3D.create();frame.setWithArray(json.load(open(os.path.join(OUT,'camera_frame_cm.json'))));frame.invert()
  for b,T,name in cam_bodies:
   clone=tmp.copy(b);tmp.transform(clone,T);tmp.transform(clone,frame);copy=cam.bRepBodies.add(clone);copy.name=name;copy.material=b.material
  def stl(comp,name):
   note('Export '+name)
   options=design.exportManager.createSTLExportOptions(comp,os.path.join(OUT,name));options.unitType=adsk.fusion.DistanceUnits.MeterDistanceUnits;options.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementLow;options.sendToPrintUtility=False;options.isBinaryFormat=True
   if not design.exportManager.execute(options):raise RuntimeError('STL export failed')
  stl(rack,'base_link.stl');stl(cam,'camera_link.stl')
  report={'source_document':source.name,'baseline_archive':BASE_ARCHIVE,'base_source_cm':BASE,'baseline_props':original_props,'rack_props':props(rack),'camera_cad_props':props(cam),'preserved_bodies':preserved,'body_count':rack.bRepBodies.count,'camera_transform_cm':camera.transform2.asArray(),'mount_transform_cm':mount.transform2.asArray()}
  json.dump(report,open(os.path.join(OUT,'mesh_export.json'),'w',encoding='utf-8'),indent=2,ensure_ascii=False)
  design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'rack_camera_tilt15.f3d')))
  note('Done');source.activate();ui.messageBox('Tilted rack and camera STL exported.\n'+OUT)
 except:
  error=traceback.format_exc();note(error);open(os.path.join(OUT,'mesh_error.txt'),'w',encoding='utf-8').write(error);ui.messageBox(error)
 finally:log.close()
