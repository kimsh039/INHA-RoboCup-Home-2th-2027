#!/usr/bin/env python3
"""Read-only audit of tracked/new source, local Markdown links and dataset hashes."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-dataset', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    names = subprocess.check_output(
        ['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'], cwd=ROOT
    ).decode('utf-8').split('\0')
    files = sorted({ROOT / name for name in names if name and (ROOT / name).is_file()})
    errors = []
    counts = dict(files=len(files), markdown=0, python=0, json=0, urdf=0, dataset_hashes=0)
    for path in files:
        relative = path.relative_to(ROOT).as_posix()
        if path.suffix not in ('.md', '.py', '.json'):
            continue
        text = path.read_text(encoding='utf-8-sig')
        if path.suffix == '.py':
            counts['python'] += 1
            try:
                ast.parse(text, filename=relative)
            except SyntaxError as exc:
                errors.append(dict(file=relative, kind='python', reason=str(exc)))
        elif path.suffix == '.json':
            counts['json'] += 1
            try:
                json.loads(text)
            except ValueError as exc:
                errors.append(dict(file=relative, kind='json', reason=str(exc)))
        else:
            counts['markdown'] += 1
            # File/dir targets only; anchors and remote URL availability are separate checks.
            for target in re.findall(r'!?\[[^\]\n]*\]\(([^\s)]+)(?:\s+[^)]*)?\)', text):
                if target.startswith(('#', '/', '<')) or re.match(r'^[A-Za-z][\w+.-]*:', target):
                    continue
                target = unquote(target.split('#')[0].split('?')[0])
                if target and not (path.parent / target).exists():
                    errors.append(dict(file=relative, kind='markdown_link', target=target))
    if args.verify_dataset:
        dataset = ROOT / 'SW/detection/head/datasets/20261013'
        hashes = json.loads((dataset / 'file_hashes.json').read_text(encoding='utf-8'))
        for name, expected in hashes.items():
            path = dataset / name
            counts['dataset_hashes'] += 1
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                errors.append(dict(file=name, kind='dataset_hash'))
    for name in ('SW/simulation/robot_description/robocup.urdf',
                 'SW/simulation/robot_description/robocup.calibrated.urdf',
                 'SW/robot/robot.physical.urdf'):
        path = ROOT / name
        robot = ET.parse(path).getroot()
        links = [node.get('name') for node in robot.findall('link')]
        children = [node.find('child').get('link') for node in robot.findall('joint')]
        counts['urdf'] += 1
        if len(links) != len(set(links)) or len(children) != len(set(children)):
            errors.append(dict(file=name, kind='urdf_duplicate_name_or_parent'))
        if set(links) - set(children) != {'base_link'}:
            errors.append(dict(file=name, kind='urdf_root'))
        for mesh in robot.findall('.//mesh'):
            target = mesh.get('filename', '')
            if target and ':' not in target and not (path.parent / target).is_file():
                errors.append(dict(file=name, kind='urdf_mesh', target=target))
    result = dict(ok=not errors, counts=counts, errors=errors)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result['ok'] else 1)


if __name__ == '__main__':
    main()
