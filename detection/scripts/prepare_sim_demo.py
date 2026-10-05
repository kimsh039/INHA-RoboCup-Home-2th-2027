#!/usr/bin/env python3
"""Explicitly fetch official pretrained weights and a COCO sample test billboard.

No training or inference environment is installed by this script.
"""
from pathlib import Path
import urllib.request


def download(url, path):
    if path.is_file():
        print(f'Already exists: {path}')
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix+'.part')
    try:
        with urllib.request.urlopen(url, timeout=60) as response, temporary.open('wb') as output:
            while chunk := response.read(1024*1024):
                output.write(chunk)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    print(f'Downloaded: {path}')


def main():
    root = Path(__file__).resolve().parents[1]
    download('https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt', root/'models'/'yolo11n.pt')
    download('https://raw.githubusercontent.com/ultralytics/ultralytics/main/ultralytics/assets/bus.jpg', root/'demo_assets'/'bus.jpg')
    print('Billboard demo class: bus. This tests approach to an image plane, not a 3D bus.')


if __name__ == '__main__':
    main()
