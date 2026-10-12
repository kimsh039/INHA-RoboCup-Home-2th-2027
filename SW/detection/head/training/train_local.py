"""Portable six-class YOLO11n training entry point (Windows or Ubuntu)."""
from pathlib import Path
import argparse
import multiprocessing


def main():
    head = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=head / 'datasets/20261013/data.yaml')
    parser.add_argument('--model', default='yolo11n.pt', help='COCO pretrained weights; downloaded if absent')
    parser.add_argument('--device', default='0')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--batch', type=int, default=4)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--patience', type=int, default=20)
    parser.add_argument('--project', type=Path, default=head / 'training/runs')
    parser.add_argument('--name', default='head_yolo11n_6cls')
    args = parser.parse_args()
    import torch
    from ultralytics import YOLO
    from ultralytics.data.utils import check_det_dataset
    expected = ['green_apple', 'peach', 'tomato', 'fanta_can', 'cup', 'banana']
    dataset = check_det_dataset(str(args.data.resolve()), autodownload=False)
    if list(dataset['names'].values()) != expected:
        raise RuntimeError('Unexpected Head class order')
    if args.device != 'cpu' and not torch.cuda.is_available():
        raise RuntimeError('CUDA is unavailable in this Python environment')
    model = YOLO(args.model)
    model.train(data=str(args.data.resolve()), device=args.device, epochs=args.epochs,
                imgsz=args.imgsz, batch=args.batch, workers=args.workers,
                amp=True, cache=False, patience=args.patience, seed=42,
                project=str(args.project.resolve()), name=args.name,
                exist_ok=False, plots=True, save=True)
    print('Best model:', model.trainer.best)


if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()
