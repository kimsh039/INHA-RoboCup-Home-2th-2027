#!/usr/bin/env python3
"""Build the current model's FP16 engine on the destination Jetson GPU."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=root / 'models/head_yolo11n_20261013.pt')
    parser.add_argument('--workspace', type=float, default=2)
    args = parser.parse_args()
    if not args.model.is_file() or args.model.suffix != '.pt':
        parser.error('Supply an existing local .pt checkpoint')
    import torch
    import tensorrt
    from ultralytics import YOLO, __version__ as ultralytics_version
    engine = YOLO(str(args.model), task='detect').export(format='engine', imgsz=640,
        batch=1, device=0, half=True, dynamic=False, simplify=False, opset=17,
        workspace=args.workspace)
    metadata = dict(source=str(args.model), source_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
        engine=str(engine), imgsz=640, batch=1, fp16=True, dynamic=False,
        gpu=torch.cuda.get_device_name(0), torch=torch.__version__,
        tensorrt=tensorrt.__version__, ultralytics=ultralytics_version)
    Path(engine).with_suffix('.engine.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
