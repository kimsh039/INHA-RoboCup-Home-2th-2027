"""Run both untouched videos; report observations without inventing ground-truth metrics."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import platform
import sys
import time
import zipfile

import cv2
import numpy as np
import torch
import ultralytics
from ultralytics import YOLO

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'head_detection_ws' /
                       'src' / 'robocup_head_detection'))
from robocup_head_detection.class_schema import CLASSES, LEGACY_CLASSES, class_mapping

def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def evaluate(model, source, output, args):
    classes = LEGACY_CLASSES if args.schema == 'legacy' else CLASSES
    mapping = class_mapping(model.names) if args.schema == 'current' else {
        i: (i, name) for i, name in enumerate(LEGACY_CLASSES)}
    output.mkdir(parents=True, exist_ok=False)
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f'Cannot open video: {source}')
    fps = capture.get(cv2.CAP_PROP_FPS)
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    capture.release()
    sample_indices = set(np.linspace(0, max(0, total-1), min(12, total), dtype=int).tolist())
    out_width = min(width, 1280)
    out_height = round(height*out_width/width/2)*2
    writer = cv2.VideoWriter(str(output/'annotated.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), fps, (out_width, out_height))
    if not writer.isOpened():
        raise RuntimeError('Could not create annotated video')
    detection_counts = {name:0 for name in classes}
    frames_with_class = {name:0 for name in classes}
    confidence_sums = {name:0.0 for name in classes}
    speed = {'preprocess':[], 'inference':[], 'postprocess':[]}
    frames_with_any = 0
    count = 0
    started = time.perf_counter()
    samples = []
    with (output/'predictions.jsonl').open('w', encoding='utf-8', newline='\n') as records, (output/'detections.csv').open('w', newline='', encoding='utf-8') as csv_file:
        rows = csv.writer(csv_file, lineterminator='\n')
        rows.writerow(['frame_index','time_s','class_id','class_name','confidence','x1','y1','x2','y2'])
        try:
            for index, result in enumerate(model.predict(source=str(source), stream=True, imgsz=args.imgsz,
                    conf=args.conf, iou=args.iou, device=args.device, vid_stride=1, verbose=False)):
                count += 1
                boxes = []
                seen = set()
                keep = [i for i, box in enumerate(result.boxes)
                        if mapping[int(box.cls.item())] is not None]
                result = result[keep]
                result.boxes.data = result.boxes.data.clone()
                # Adapt IDs before plotting too, so videos and raw records agree.
                original_ids = result.boxes.cls.tolist()
                for row, old_id in enumerate(original_ids):
                    result.boxes.data[row, -1] = mapping[int(old_id)][0]
                result.names = dict(enumerate(classes))
                for box in result.boxes:
                    class_id = int(box.cls.item())
                    name = classes[class_id]
                    confidence = float(box.conf.item())
                    xyxy = [round(float(v), 3) for v in box.xyxy[0].tolist()]
                    boxes.append({'class_id':class_id,'class_name':name,'confidence':confidence,'xyxy':xyxy})
                    rows.writerow([index,index/fps,class_id,name,confidence,*xyxy])
                    detection_counts[name] += 1
                    confidence_sums[name] += confidence
                    seen.add(name)
                for name in seen:
                    frames_with_class[name] += 1
                frames_with_any += bool(seen)
                records.write(json.dumps({'frame_index':index,'time_s':index/fps,'detections':boxes}, ensure_ascii=False)+'\n')
                for key in speed:
                    speed[key].append(float(result.speed[key]))
                annotated = result.plot(line_width=2, font_size=16)
                cv2.putText(annotated, f'{source.stem} | frame {index} | {index/fps:.2f}s',
                            (20,40),cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2,cv2.LINE_AA)
                writer.write(cv2.resize(annotated,(out_width,out_height)))
                if index in sample_indices:
                    filename = f'frame_{index:06d}.jpg'
                    cv2.imwrite(str(output/filename), annotated)
                    samples.append(filename)
        finally:
            writer.release()
    elapsed = time.perf_counter()-started
    if count != total:
        raise RuntimeError(f'Video frame count mismatch: expected {total}, got {count}')
    summary = {
        'video':source.name,'source_sha256':sha256(source),'frames':count,'source_fps':fps,
        'duration_s':count/fps,'width':width,'height':height,
        'frames_with_any_detection':frames_with_any,
        'per_class':{name:{'boxes':detection_counts[name],'frames':frames_with_class[name],
            'frame_fraction':frames_with_class[name]/count,
            'mean_confidence':confidence_sums[name]/detection_counts[name] if detection_counts[name] else None}
            for name in classes},
        'mean_stage_ms':{key:float(np.mean(value)) for key,value in speed.items()},
        'wall_time_s':elapsed,'wall_frames_per_s':count/elapsed,
        'speed_scope':'wall FPS includes decoding, plotting and output; stage times are Ultralytics measurements',
        'ground_truth_available':False,'precision':None,'recall':None,'mAP50':None,'mAP50_95':None,
        'table_assignment_evaluated':False,'sample_images':samples,
    }
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps({'video':source.name,'frames':count,'wall_frames_per_s':count/elapsed,'per_class':summary['per_class']},ensure_ascii=False),flush=True)
    return summary

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--videos',type=Path,nargs=2,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--device',default='0')
    parser.add_argument('--imgsz',type=int,default=960)
    parser.add_argument('--conf',type=float,default=0.25)
    parser.add_argument('--iou',type=float,default=0.7)
    parser.add_argument('--schema', choices=['current', 'legacy'], default='current',
                        help='Current six-class outputs or reproduction of the original seven-class experiment')
    args = parser.parse_args()
    assert args.model.is_file()
    assert all(path.is_file() for path in args.videos)
    args.output.mkdir(parents=True,exist_ok=True)
    model = YOLO(str(args.model))
    source_classes = tuple(model.names[i] for i in range(len(model.names)))
    allowed = (LEGACY_CLASSES,) if args.schema == 'legacy' else (CLASSES, LEGACY_CLASSES)
    if source_classes not in allowed:
        raise ValueError(f'Unexpected Head model schema: {model.names}')
    model.predict(np.zeros((1080,1920,3),dtype=np.uint8),device=args.device,imgsz=args.imgsz,verbose=False)
    environment = {'python':platform.python_version(),'ultralytics':ultralytics.__version__,
        'torch':torch.__version__,'cuda':torch.version.cuda,
        'device':torch.cuda.get_device_name(int(args.device)) if args.device != 'cpu' else 'CPU',
        'model_sha256':sha256(args.model),'model':args.model.name,
        'imgsz':args.imgsz,'conf':args.conf,'iou':args.iou,'vid_stride':1,
        'source_model_classes':source_classes,'output_schema':args.schema,
        'output_classes':LEGACY_CLASSES if args.schema == 'legacy' else CLASSES,
        'legacy_adapter_applied':args.schema == 'current' and source_classes == LEGACY_CLASSES}
    summaries = [evaluate(model,path,args.output/path.stem,args) for path in args.videos]
    (args.output/'test_summary.json').write_text(json.dumps({'environment':environment,'videos':summaries},ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
    lines = ['# Two held-out video inference','',f"Model SHA256: `{environment['model_sha256']}`",'',
        f"Evaluation device: **{environment['device']}**; imgsz={args.imgsz}, conf={args.conf}, IoU={args.iou}, all frames.",'',
        f"Output schema: {environment['output_schema']}; legacy checkpoint adapter applied: {environment['legacy_adapter_applied']}. This adapter does not retrain the model.",'',
        'These videos have no independent bounding-box ground truth. Precision, recall, mAP and table-selection accuracy were not computed. Frame occurrence is observation coverage, not detection recall.','',
        '| Video | Frames | Frames with detections | Mean inference ms | End-to-end FPS |','|---|---:|---:|---:|---:|']
    for item in summaries:
        lines.append(f"| {item['video']} | {item['frames']} | {item['frames_with_any_detection']} | {item['mean_stage_ms']['inference']:.2f} | {item['wall_frames_per_s']:.2f} |")
    for item in summaries:
        lines.extend(['',f"## {item['video']}",'','| Class | Frames | Frame fraction | Boxes | Mean confidence |','|---|---:|---:|---:|---:|'])
        for name,value in item['per_class'].items():
            confidence = '-' if value['mean_confidence'] is None else f"{value['mean_confidence']:.3f}"
            lines.append(f"| {name} | {value['frames']} | {value['frame_fraction']:.3f} | {value['boxes']} | {confidence} |")
        lines.extend(['',f"[Annotated video]({Path(item['video']).stem}/annotated.mp4) · [Raw detections]({Path(item['video']).stem}/detections.csv)",''])
        for name in item['sample_images']:
            lines.append(f"![{name}]({Path(item['video']).stem}/{name})")
    (args.output/'TEST_RESULTS.md').write_text('\n'.join(lines)+'\n',encoding='utf-8',newline='\n')

if __name__ == '__main__':
    main()
