"""Export top-down wrist depth returns; default is a planned scratch scene."""
import argparse,hashlib,json,sys,zipfile
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from manipulation.sim_model import config,load_model
from manipulation.observation import top_down_pose,apply_observation
from manipulation.wrist_camera_cloud import scan_cube
from scripts.generate_mid360_cloud import live_status,apply_snapshot


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live',action='store_true')
    parser.add_argument('--height',type=float,default=.30)
    parser.add_argument('--resolution',type=int,default=192)
    parser.add_argument('--output',type=Path,default=ROOT/'data/pointclouds/cube_wrist_camera')
    args=parser.parse_args();m,d=load_model()
    plan=top_down_pose(m,d,args.height)
    if args.live:
        apply_snapshot(m,d,live_status(10))
    else:
        apply_observation(m,d,plan)
    points,camera,network,meta=scan_cube(m,d,config(),args.resolution)
    from manipulation.sim_model import body_transform
    T=body_transform(m,d,'wrist_camera_optical_frame')
    center=body_transform(m,d,'cube')[:3,3]
    if np.linalg.norm(T[:3,2]-[0,0,-1])>.02 or np.linalg.norm((T[:3,3]-center)[:2])>.005:
        raise ValueError('MOVE_TO_TOP_DOWN_OBSERVATION_FIRST')
    meta['snapshot_source']='live bridge' if args.live else 'planned scratch observation; live MoveIt motion pending'
    meta['observation_plan']=plan
    out=args.output;out.mkdir(parents=True,exist_ok=True)
    for name,a in [('points.npy',points),('points_camera.npy',camera),('points_network.npy',network)]:np.save(out/name,a,allow_pickle=False)
    meta['npy_sha256']=hashlib.sha256((out/'points.npy').read_bytes()).hexdigest()
    (out/'metadata.json').write_text(json.dumps(meta,indent=2,allow_nan=False)+'\n')
    with zipfile.ZipFile(out/'cube_wrist_camera_input.zip','w',zipfile.ZIP_DEFLATED) as z:
        for name in ['points.npy','points_camera.npy','points_network.npy','metadata.json']:z.write(out/name,name)
    print('camera world',T[:3,3],'; base-object horizontal',meta['base_to_object_horizontal_distance_m'])
    print('returns',len(points),'first hits',meta['first_hit_counts']);print(out/'cube_wrist_camera_input.zip')


if __name__=='__main__':main()
