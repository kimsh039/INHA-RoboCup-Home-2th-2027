"""Capture one image-based tag pose and timestamp-matched measured robot FK."""
import argparse,base64,json,os,selectors,subprocess,sys,time
from contextlib import closing
from pathlib import Path
import cv2,numpy as np
from kinematics import fk

ROOT=Path(__file__).resolve().parent

def capture_stream(prefix,partition,timeout):
    # Gazebo and OpenCV can link incompatible C++ protobuf libraries on macOS.
    # Isolate Gazebo bindings, including inherited PYTHONPATH, in a fresh worker.
    end=time.monotonic()+timeout
    worker=subprocess.Popen([sys.executable,'-I',str(ROOT/'gazebo_capture_stream.py'),
        '--prefix',prefix,'--partition',partition,'--timeout',str(timeout)],
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    selector=selectors.DefaultSelector();selector.register(worker.stdout,selectors.EVENT_READ)
    try:
        while time.monotonic()<end:
            if worker.poll() is not None:
                raise RuntimeError(f'Gazebo capture worker exited (code {worker.returncode}); see its error above')
            try:worker.stdin.write('next\n');worker.stdin.flush()
            except BrokenPipeError:
                raise RuntimeError('Gazebo capture worker stopped; see its error above') from None
            remaining=end-time.monotonic()
            if remaining<=0 or not selector.select(remaining):return
            line=worker.stdout.readline()
            if not line:
                code=worker.wait()
                raise RuntimeError(f'Gazebo capture worker exited (code {code}); see its error above')
            packet=json.loads(line)
            if 'error' in packet:raise RuntimeError(packet['error'])
            yield packet
    finally:
        selector.close()
        try:worker.stdin.close()
        except BrokenPipeError:pass
        try:worker.wait(timeout=2)
        except subprocess.TimeoutExpired:
            worker.terminate()
            try:worker.wait(timeout=2)
            except subprocess.TimeoutExpired:worker.kill();worker.wait()
        worker.stdout.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--camera',choices=['head','wrist'],default='wrist')
    p.add_argument('--dataset',type=Path,default=ROOT/'data/wrist_samples.json')
    p.add_argument('--split',choices=['train','holdout'],default='train')
    p.add_argument('--corner-refinement',choices=['none','subpix'],default='subpix')
    p.add_argument('--partition',default='robocup_tutorial_mac');p.add_argument('--timeout',type=float,default=25)
    a=p.parse_args();os.environ['GZ_PARTITION']=a.partition;os.environ['GZ_IP']='127.0.0.1'
    prefix='/robocup/wrist_camera' if a.camera=='wrist' else '/robocup/camera'
    camera_frame='wrist_camera_optical_frame' if a.camera=='wrist' else 'camera_optical_frame'
    parameters=cv2.aruco.DetectorParameters()
    if a.corner_refinement=='subpix':
        parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize=5
        parameters.cornerRefinementMaxIterations=50
        parameters.cornerRefinementMinAccuracy=.001
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),parameters)
    size=.08;object_points=np.array([[-size/2,size/2,0],[size/2,size/2,0],[size/2,-size/2,0],[-size/2,-size/2,0]],dtype=np.float64)
    last_reason='Waiting for inputs'
    with closing(capture_stream(prefix,a.partition,a.timeout)) as stream:
        for packet in stream:
            image=packet['image'];info=packet['info'];joint=packet['joints'];ts=packet['image_sim_time']
            joint_time=packet['joint_sim_time']
            if abs(packet['info_sim_time']-ts)>.005 or abs(joint_time-ts)>.005:
                last_reason='Image / CameraInfo / measured joint time mismatch';continue
            frames=image['frame_ids']
            if frames!=[camera_frame]:raise ValueError('Unexpected color optical frame')
            pos={j['name']:j['position'] for j in joint}
            arm=[j for j in joint if j['name'] in [f'piper_joint{i}' for i in range(1,7)]]
            if len(arm)!=6 or max(abs(j['velocity']) for j in arm)>.02:
                last_reason='Wait for all six joints and arm motion to settle';continue
            if image['pixel_format_type']!=3:raise ValueError('This collector expects RGB_INT8 (3)')
            image_bytes=base64.b64decode(image['data'],validate=True)
            rgb=np.ndarray((image['height'],image['width'],3),dtype=np.uint8,buffer=image_bytes,strides=(image['step'],3,1))
            gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);corners,ids,_=detector.detectMarkers(gray)
            if ids is None or 0 not in ids.flatten():last_reason='AprilTag 36h11 ID 0 not visible';continue
            pixels=corners[list(ids.flatten()).index(0)].reshape(4,2).astype(np.float64)
            if info['width']!=image['width'] or info['height']!=image['height']:raise ValueError('Color CameraInfo grid mismatch')
            k=np.array(info['k']).reshape(3,3);d=np.array(info['distortion'])
            if d.size==0:d=np.zeros(5)
            ok,rvec,tvec=cv2.solvePnP(object_points,pixels,k,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or tvec[2,0]<=0:last_reason='Invalid tag pose';continue
            initial_r,initial_t=rvec.copy(),tvec.copy()
            initial_pixels,_=cv2.projectPoints(object_points,initial_r,initial_t,k,d)
            initial_error=float(np.sum((initial_pixels.reshape(4,2)-pixels)**2))
            refined_r,refined_t=cv2.solvePnPRefineLM(object_points,pixels,k,d,rvec,tvec)
            refined_pixels,_=cv2.projectPoints(object_points,refined_r,refined_t,k,d)
            refined_error=float(np.sum((refined_pixels.reshape(4,2)-pixels)**2))
            if refined_t[2,0]>0 and np.isfinite(refined_error) and refined_error<initial_error:
                rvec,tvec=refined_r,refined_t
            else:rvec,tvec=initial_r,initial_t
            projected,_=cv2.projectPoints(object_points,rvec,tvec,k,d)
            reprojection=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-pixels)**2,axis=1))))
            if reprojection>1.:last_reason='Reprojection error exceeds 1 px';continue
            tc=np.eye(4);tc[:3,:3]=cv2.Rodrigues(rvec)[0];tc[:3,3]=tvec.flatten()
            tg=fk(ROOT/'robot/simulation/robot_description/robocup.urdf',pos,'piper_base_link','piper_link6')
            mode='eye_in_hand' if a.camera=='wrist' else 'eye_on_base'
            meta={'mode':mode,'translation_unit':'m','synthetic':False,'source':'Gazebo image AprilTag PnP + measured joints',
                'simulation':True,'pose_estimator':'OpenCV IPPE_SQUARE + RefineLM',
                'corner_refinement':a.corner_refinement,
                'repository_commit':'0e0a28d7866832ecf1e0841bd41760ebde672a79',
                'base_frame':'piper_base_link','flange_frame':'piper_link6','camera_frame':camera_frame,'tag_black_edge_m':size,'samples':[]}
            if a.dataset.exists():
                meta=json.loads(a.dataset.read_text())
                if meta['mode']!=mode or meta.get('camera_frame')!=camera_frame:raise ValueError('Dataset mode/frame mismatch')
            ident=len(meta['samples']);images=a.dataset.parent/'images';images.mkdir(parents=True,exist_ok=True)
            path=images/f'{a.camera}_{ident:03d}.png';bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            if not cv2.imwrite(str(path),bgr):raise IOError('Cannot save raw image')
            cv2.aruco.drawDetectedMarkers(bgr,[pixels.reshape(1,4,2).astype(np.float32)],np.array([[0]]))
            if not cv2.imwrite(str(images/f'{a.camera}_{ident:03d}_preview.png'),bgr):raise IOError('Cannot save preview')
            sample={'id':ident,'split':a.split,'image':str(path),'image_sim_time':ts,'joint_sim_time':joint_time,
                'time_skew_s':abs(ts-joint_time),'reprojection_px':reprojection,'T_base_flange':tg.tolist(),
                'T_camera_target':tc.tolist(),'color_camera_info':info['proto_json'],
                'corner_refinement':a.corner_refinement,
                'image_corners_px':pixels.tolist(),'measured_joint_positions':{j['name']:j['position'] for j in arm}}
            meta['samples'].append(sample);a.dataset.parent.mkdir(parents=True,exist_ok=True)
            temp=a.dataset.with_suffix('.tmp');temp.write_text(json.dumps(meta,indent=2));temp.replace(a.dataset)
            print(json.dumps({'sample':ident,'time_skew_s':sample['time_skew_s'],'reprojection_px':reprojection,'dataset':str(a.dataset)}));return
        raise RuntimeError('Capture timed out: '+last_reason)

if __name__=='__main__':main()
