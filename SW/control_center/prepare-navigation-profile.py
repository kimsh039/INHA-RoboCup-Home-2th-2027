#!/usr/bin/env python3
"""Generate Nav2's footprint from the exact URDF used by the 3D guard."""
import argparse
from pathlib import Path
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'robot'))
from collision_geometry import RobotGeometry, rectangle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--safety', required=True)
    args = parser.parse_args()
    safety = yaml.safe_load(Path(args.safety).read_text())['/**']['ros__parameters']
    profile = yaml.safe_load(Path(args.input).read_text())
    model = RobotGeometry()
    # The guard updates this with measured arm geometry after startup. Start
    # with the static URDF shape and fail closed until arm feedback is available.
    for kind in ('local_costmap', 'global_costmap'):
        settings = profile[kind][kind]['ros__parameters']
        # The 3D guard enforces clearance along the actual motion. Expanding
        # every side here would reject moving away from a close rear obstacle.
        settings['footprint'] = str(rectangle(model.fixed))
        settings['footprint_padding'] = 0.0
        for layer in ('voxel_layer', 'obstacle_layer', 'static_layer'):
            if layer in settings:
                settings[layer]['footprint_clearing_enabled'] = False
        for layer in ('voxel_layer', 'obstacle_layer'):
            if layer in settings:
                # Floor classification is shared with the 3D guard. Ground
                # returns remain available to ray clearing, never marking.
                if 'mid360' in settings[layer] and 'mid360_clear' not in settings[layer]:
                    settings[layer]['mid360_clear'] = dict(settings[layer]['mid360'])
                sources = settings[layer].get('observation_sources', '').split()
                if 'mid360_clear' in settings[layer] and 'mid360_clear' not in sources:
                    sources.append('mid360_clear')
                settings[layer]['observation_sources'] = ' '.join(sources)
                for source in ('mid360', 'mid360_clear'):
                    if source in settings[layer]:
                        sensor = settings[layer][source]
                        sensor['topic'] = safety['obstacle_topic'] if source == 'mid360' else safety['clearing_topic']
                        sensor['max_obstacle_height'] = float(safety['max_obstacle_height'])
                        sensor['min_obstacle_height'] = -0.5
                        sensor['marking'] = source == 'mid360'
                        sensor['clearing'] = source == 'mid360_clear'
    Path(args.output).write_text(yaml.safe_dump(profile, sort_keys=False))


if __name__ == '__main__':
    main()
