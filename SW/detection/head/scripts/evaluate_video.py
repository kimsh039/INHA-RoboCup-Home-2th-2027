"""Annotate every frame and export a browser-compatible H.264 test video."""
from pathlib import Path
import argparse
import collections
import json
import subprocess
import time


def main():
    import cv2
    import imageio_ffmpeg
    from ultralytics import YOLO

    parser = argparse.ArgumentParser()
    parser.add_argument('video', type=Path)
    root = Path(__file__).resolve().parents[1]
    parser.add_argument('--model', type=Path, default=root / 'models/head_yolo11n_20261013.pt')
    parser.add_argument('--output', type=Path, default=root / 'training/runs/video_tests')
    parser.add_argument('--device', default='0')
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--cup-conf', type=float, default=0.70, help='Higher confidence threshold only for cup; use 0.25 for baseline')
    args = parser.parse_args()
    if not 0 <= args.conf <= 1 or (args.cup_conf is not None and not args.conf <= args.cup_conf <= 1):
        parser.error('Thresholds must satisfy 0 <= conf <= cup-conf <= 1')
    suffix = '' if args.cup_conf is None else f'_cup{round(args.cup_conf * 100):03d}'
    out = args.output / (args.video.stem + suffix)
    out.mkdir(parents=True, exist_ok=False)
    model_path = args.model
    model = YOLO(str(model_path))
    cup_id = next(cls for cls, name in model.names.items() if name == 'cup')
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise RuntimeError('Could not open input video')
    fps = cap.get(cv2.CAP_PROP_FPS)
    width, height = int(cap.get(3)), int(cap.get(4))
    expected = int(cap.get(7))
    if fps <= 0 or expected <= 0:
        cap.release()
        raise RuntimeError('Input has invalid FPS or frame count')
    raw = out / 'annotated_raw.mp4'
    writer = cv2.VideoWriter(str(raw), cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError('Could not create video writer')
    samples = min(12, expected)
    selected = {round(i * (expected - 1) / max(1, samples - 1)) for i in range(samples)}
    previews = []
    count = collections.Counter()
    frame_presence = collections.Counter()
    frames = 0
    start = time.perf_counter()
    try:
        with (out / 'predictions.jsonl').open('w', encoding='utf-8') as log:
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                result = model.predict(frame, device=args.device, imgsz=640, conf=args.conf, iou=0.7, verbose=False)[0]
                if args.cup_conf is not None:
                    keep = (result.boxes.cls != cup_id) | (result.boxes.conf >= args.cup_conf)
                    result.update(boxes=result.boxes.data[keep])
                boxes = []
                found = set()
                for b in result.boxes:
                    cls = int(b.cls.item())
                    name = model.names[cls]
                    count[name] += 1
                    found.add(name)
                    boxes.append({'class_id': cls, 'class': name, 'confidence': float(b.conf.item()), 'xyxy': b.xyxy[0].cpu().tolist()})
                frame_presence.update(found)
                annotated = result.plot(line_width=2, font_size=12)
                caption = f'{frames / fps:.2f}s | conf >= {args.conf:.2f}'
                if args.cup_conf is not None:
                    caption += f' | cup >= {args.cup_conf:.2f}'
                cv2.putText(annotated, caption, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 3)
                cv2.putText(annotated, caption, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
                writer.write(annotated)
                log.write(json.dumps({'frame': frames, 'seconds': frames / fps, 'detections': boxes}) + '\n')
                if frames in selected:
                    previews.append(annotated.copy())
                    cv2.imwrite(str(out / f'frame_{frames:06d}.jpg'), annotated)
                frames += 1
                if frames % 240 == 0:
                    print(f'Processed {frames}/{expected} frames', flush=True)
    finally:
        cap.release()
        writer.release()
    elapsed = time.perf_counter() - start
    if frames != expected:
        raise RuntimeError(f'Incomplete decoding: expected {expected}, decoded {frames}')
    output = out / (args.video.stem + '_yolo11n' + suffix + '_detected.mp4')
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-hide_banner', '-loglevel', 'error', '-i', str(raw), '-i', str(args.video), '-map', '0:v:0', '-map', '1:a?', '-c:v', 'libx264', '-preset', 'fast', '-crf', '20', '-pix_fmt', 'yuv420p', '-c:a', 'aac', '-movflags', '+faststart', str(output)], check=True)
    while len(previews) % 3:
        import numpy as np
        previews.append(np.zeros_like(previews[0]))
    sheet = cv2.vconcat([cv2.hconcat(previews[i:i+3]) for i in range(0, len(previews), 3)])
    cv2.imwrite(str(out / 'contact_sheet.jpg'), sheet)
    summary = {'input': str(args.video.resolve()), 'model': str(model_path.resolve()), 'classes': model.names,
               'device': args.device, 'imgsz': 640, 'nms_iou': 0.7,
               'frames': frames, 'fps': fps, 'duration_seconds': frames / fps,
               'width': width, 'height': height, 'confidence_threshold': args.conf,
               'cup_confidence_threshold': args.cup_conf,
               'detection_count_by_class': dict(count), 'frames_with_class': dict(frame_presence),
               'processing_seconds_before_encoding': elapsed,
               'evaluation': 'Inference only; no ground-truth labels, accuracy metrics unavailable.', 'output': str(output)}
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
