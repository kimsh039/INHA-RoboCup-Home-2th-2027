#!/usr/bin/env python3
"""Bind a backend profile to the selected physical scan front-end's contract."""
import argparse
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('backend', type=Path)
    parser.add_argument('frontend', type=Path)
    parser.add_argument('source', choices=('g2', 'mid360'))
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    backend = yaml.safe_load(args.backend.read_text())
    frontend = yaml.safe_load(args.frontend.read_text())
    settings = dict(frontend.get('/**', {}).get('ros__parameters', {}))
    settings.update(frontend[args.source + '_slam_scan']['ros__parameters'])
    if not isinstance(backend, dict):
        raise ValueError('SLAM backend profile must contain a ROS parameter mapping')
    # Explicit node parameters override a wildcard profile without discarding its
    # solver/graph settings. Keep an existing fully-qualified node key if present.
    key = '/slam_toolbox' if '/slam_toolbox' in backend else 'slam_toolbox'
    params = backend.setdefault(key, {}).setdefault('ros__parameters', {})
    params.update(scan_topic=settings['output_topic'],
                  min_laser_range=settings['min_range'], max_laser_range=settings['max_range'],
                  minimum_time_interval=0.0, minimum_travel_distance=0.0)
    # Installed 2.6.10 has no restamp_tf parameter. Keep its solver/dataset ABI and
    # use one node-remapped broadcaster plus the clock adapter instead of a rebuild.
    params.pop('restamp_tf', None)
    effective = dict(backend.get('/**', {}).get('ros__parameters', {}))
    effective.update(params)
    backend['slam_tf_clock'] = {'ros__parameters': {
        'map_frame': effective.get('map_frame', 'map'),
        'odom_frame': effective.get('odom_frame', 'odom'),
        'transform_timeout': float(effective.get('transform_timeout', 0.2)),
    }}
    args.output.write_text(yaml.safe_dump(backend, sort_keys=False))


if __name__ == '__main__':
    main()
