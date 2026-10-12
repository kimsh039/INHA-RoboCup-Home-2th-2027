"""Select the current Head model; Jetson engines stay local to the device."""
import os
from pathlib import Path

MODEL_STEM = 'head_yolo11n_20261013'


def current_model_path(head_root=None):
    override = os.environ.get('ROBOCUP_MODEL')
    if override:
        return Path(override).expanduser()
    root = Path(head_root) if head_root is not None else Path(__file__).resolve().parents[4]
    models = Path(os.environ.get('ROBOCUP_HEAD_MODEL_DIR', str(root / 'models'))).expanduser()
    engine = models / (MODEL_STEM + '.engine')
    return engine if engine.is_file() else models / (MODEL_STEM + '.pt')
