#!/usr/bin/env python3
"""Run the real head pipeline on an image/video/webcam and save annotated output.

Example: python test_video.py --model ../models/yolo11n.pt --source ../demo_assets/bus.jpg
"""
import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'head_detection_ws'/'src'/'robocup_head_detection'))
import cv2
from robocup_head_detection.backends import YoloDetector, make_tracker_factory
from robocup_head_detection.core import Config, HeadPipeline, roi_bounds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--source', required=True, help='Image/video path or camera number')
    parser.add_argument('--class-name', default='bus')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--output', default='detection_test.mp4')
    parser.add_argument('--frames', type=int, default=100)
    args = parser.parse_args()
    detector = YoloDetector(args.model, device=args.device)
    if not detector.ready:
        raise SystemExit(detector.error)
    pipeline = HeadPipeline(detector, make_tracker_factory('CSRT'),
                            Config(target_class=args.class_name, max_frame_gap=10.0))
    still = cv2.imread(args.source) if Path(args.source).suffix.lower() in ('.jpg', '.png', '.jpeg') else None
    capture = None if still is not None else cv2.VideoCapture(int(args.source) if args.source.isdigit() else args.source)
    writer = None
    try:
        for index in range(args.frames):
            ok, frame = (True, still.copy()) if still is not None else capture.read()
            if not ok:
                break
            # Replay timestamp is the synthetic frame time, not CPU inference wall time.
            observation = pipeline.step(frame, 1.0+index/30)
            if writer is None:
                writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*'mp4v'), 30,
                                         (frame.shape[1], frame.shape[0]))
                if not writer.isOpened():
                    raise RuntimeError('Cannot open output video')
            if observation.box:
                box = observation.box
                x, y, w, h = (round(v) for v in (box.x, box.y, box.width, box.height))
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                x0, y0, x1, y1 = roi_bounds(box, frame.shape[1], frame.shape[0], 2.0)
                cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 180, 0), 1)
            cv2.putText(frame, f'{observation.state} YOLO={observation.measured}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
            writer.write(frame)
            if observation.measured or not observation.valid:
                print(index, observation.state, observation.reason, observation.target_id)
    finally:
        if writer:
            writer.release()
        if capture:
            capture.release()
    print(f'Output: {Path(args.output).resolve()}')


if __name__ == '__main__':
    main()
