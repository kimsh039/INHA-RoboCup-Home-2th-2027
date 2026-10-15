#!/usr/bin/env python3
"""Vision environment/weights check; no ROS node, camera, or robot commands."""
import argparse
import importlib
import json
from pathlib import Path
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--vision-config',type=Path,required=True)
    p.add_argument('--load',action='store_true',help='Also load/warm YOLO and SAM on this computer; never opens camera')
    a=p.parse_args()
    import yaml
    v=yaml.safe_load(a.vision_config.expanduser().read_text())
    report={'python':sys.executable,'repo':str(a.repo),'imports':{},'weights':{},'errors':[]}
    for name in ('numpy','cv2','torch','ultralytics','cv_bridge','rclpy')+ (('sam2',) if v['mask_source']=='sam2' else ()):
        try:
            mod=importlib.import_module(name); report['imports'][name]=getattr(mod,'__version__',getattr(mod,'__file__','OK'))
        except Exception as exc: report['errors'].append(name+': '+str(exc))
    for key in ('yolo_model',)+ (('sam_checkpoint',) if v['mask_source']=='sam2' else ()):
        path=Path(v[key]).expanduser(); path=path if path.is_absolute() else a.repo/path
        report['weights'][key]={'path':str(path),'exists':path.is_file()}
        if not path.is_file(): report['errors'].append('MISSING:'+str(path))
    if 'torch' in sys.modules:
        import torch
        report['cuda_available']=torch.cuda.is_available()
        report['bf16_supported']=bool(torch.cuda.is_available() and torch.cuda.is_bf16_supported())
        if str(v['device'])!='cpu' and not torch.cuda.is_available(): report['errors'].append('CUDA_NOT_AVAILABLE')
    if a.load and not report['errors']:
        try:
            sys.path.insert(0,str(a.repo/'SW/detection/head/head_detection_ws/src/robocup_head_detection'))
            from robocup_head_detection.backends import YoloDetector
            detector=YoloDetector(report['weights']['yolo_model']['path'],device=v['device'],task=v['yolo_task'])
            if not detector.ready: raise RuntimeError(detector.error)
            report['yolo_names']=detector.model.names
            if v['mask_source']=='sam2':
                from sam2.build_sam import build_sam2
                build_sam2(v['sam_config'],report['weights']['sam_checkpoint']['path'],device='cpu' if str(v['device'])=='cpu' else 'cuda')
            report['model_load']='OK'
        except Exception as exc: report['errors'].append('MODEL_LOAD:'+str(exc))
    print(json.dumps(report,ensure_ascii=False,indent=2))
    raise SystemExit(1 if report['errors'] else 0)


if __name__=='__main__': main()
