#!/usr/bin/env python3
"""Create a six-class config from an existing measured robot config; never launch."""
import argparse
import copy
from pathlib import Path
import yaml


def configure(config, defaults):
    if config.get('model_names') != defaults['model_names']:
        raise ValueError('CLASS_SCHEMA_MISMATCH: verify detector IDs before migration')
    result=copy.deepcopy(config)
    # Replace only object rules, preserving CAN, transforms, table, feedback,
    # execution authorization and physical calibration exactly as supplied.
    result['classes']=copy.deepcopy(defaults['classes'])
    result.pop('class_execution',None)
    result.setdefault('physical_validation',copy.deepcopy(defaults['physical_validation']))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    defaults=Path(__file__).resolve().parents[1]/'manipulation_ws/src/robocup_real_manipulation/config/robot.yaml'
    result=configure(yaml.safe_load(args.config.read_text()),yaml.safe_load(defaults.read_text()))
    with args.output.open('x') as stream:
        stream.write('# Six-class object rules; physical robot calibration preserved.\n')
        yaml.safe_dump(result,stream,sort_keys=False,allow_unicode=True)
    print('Created: '+str(args.output))
    print('Enabled: '+', '.join(result['model_names']))


if __name__=='__main__':
    main()
