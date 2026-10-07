"""Validate a new Colab ZIP against the current cloud, then derive pre-grasps."""
import argparse
import json
from pathlib import Path
import shutil
import time
import zipfile
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.derive_pregrasps import derive
from manipulation.mid360_cloud import validate_scene_metadata
ROOT=Path(__file__).resolve().parents[1]
def import_output(path, metadata_path=None, output_directory=None):
    local=json.loads((metadata_path or ROOT/'data/pointclouds/cube/metadata.json').read_text())
    validate_scene_metadata(local)
    with zipfile.ZipFile(path) as archive:
        matches=[n for n in archive.namelist() if Path(n).name=='grasps.json']
        if len(matches)!=1:raise ValueError('ZIP must contain exactly one grasps.json')
        source=json.loads(archive.read(matches[0]))
        if source.get('input_metadata')!=local:raise ValueError('GRASP_INPUT_MODEL_MISMATCH: use the latest colab_input.zip')
        if source.get('units')!='meters' or source.get('point_cloud_frame')!='object' or source.get('approach_axis')!='+X_grasp (rotation_matrix first column)':raise ValueError('GRASP_FRAME_INVALID')
        if not source.get('grasps'):raise ValueError('NO_GRASP_CANDIDATE')
        cfg=json.loads((ROOT/'config/pregrasp.json').read_text())
        derived=[]
        for g in source['grasps']:
            if not np.isfinite([g['score'],g['width'],g['depth']]).all() or g['width']<0 or g['depth']<=0:raise ValueError('GRASP_PARAMETERS_INVALID')
            derived.append({**g,**derive(g['rotation_matrix'],g['translation'],cfg['distance_m'],cfg['rotation_tolerance'])})
        directory=output_directory or ROOT/'data/grasps/cube';directory.mkdir(parents=True,exist_ok=True)
        artifacts=('grasps_raw.npy','candidate_frames.png','grasps_camera_raw.npy','grasps_network_raw.npy','network_input_camera.npy','network_input_sensor.npy')
        previous=[directory/n for n in ('grasps.json','pregrasps_object.json',*artifacts) if (directory/n).exists()]
        if previous:
            backup=directory/f'previous_{time.time_ns()}';backup.mkdir()
            for file in previous:shutil.copy2(file,backup/file.name)
        (directory/'grasps.json').write_text(json.dumps(source,indent=2,allow_nan=False)+'\n')
        for filename in artifacts:
            entries=[n for n in archive.namelist() if Path(n).name==filename]
            if len(entries)==1:(directory/filename).write_bytes(archive.read(entries[0]))
        output={**source,'grasps':derived,'pregrasp_distance_m':cfg['distance_m'],'pregrasp_frame':'object','feasibility_checked':False,'tcp_mapping':'runtime: model-derived tip, grasp depth offset','selected_candidate_id':None}
        (directory/'pregrasps_object.json').write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    return len(derived)
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('zip',type=Path)
    parser.add_argument('--metadata',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    print(f'Imported {import_output(args.zip,args.metadata,args.output)} candidates and 0.20 m pre-grasps. Robot feasibility is checked at simulation runtime.')
