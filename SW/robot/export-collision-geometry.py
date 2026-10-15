#!/usr/bin/env python3
"""Export the existing calibrated geometry; do not maintain a second URDF model."""
import argparse
from pathlib import Path
import yaml
from collision_geometry import RobotGeometry, origin


def matrix(value):
    return dict(rotation=value[:3, :3].reshape(-1).tolist(), translation=value[:3, 3].tolist())


def box(value):
    return dict(name=value.name, half=value.half.tolist(), **matrix(value.transform))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    model = RobotGeometry()
    joints = []
    for child, joint in model.arm_joints.items():
        axis = joint.find('axis')
        limit = joint.find('limit')
        joints.append(dict(child=child, parent=joint.find('parent').get('link'), name=joint.get('name'),
                           kind=joint.get('type'), axis=[float(v) for v in axis.get('xyz').split()] if axis is not None else [1., 0., 0.],
                           lower=float(limit.get('lower', 0)) if limit is not None else 0.,
                           upper=float(limit.get('upper', 0)) if limit is not None else 0.,
                           **matrix(origin(joint.find('origin')))))
    value = dict(fixed=[box(b) for b in model.fixed],
                 tracer=[box(b) for b in model.tracer],
                 self_interiors=[box(b) for b in model.self_interiors], joints=joints,
                 arm_links=[dict(name=name, boxes=[box(b) for b in boxes]) for name, boxes in model.arm_links.items()],
                 required_joints=sorted(model.required_joints))
    Path(args.output).write_text(yaml.safe_dump(value, sort_keys=False))


if __name__ == '__main__':
    main()
