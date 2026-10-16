#!/usr/bin/env python3
"""Static contract check of a supplied web source; never connects to a robot."""
import argparse
import ast
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/
    'manipulation_ws/src/robocup_real_manipulation'))
from robocup_real_manipulation.control_center_input import CLASSES, ACTIVE_PHASES, INACTIVE_PHASES


def inspect_source(source):
    tree = ast.parse(source)
    classes = None
    phases = set()
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'CLASSES' for t in node.targets):
            classes = ast.literal_eval(node.value)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        # The new web emits final phase through begin_final_alignment ->
        # advance_heading, so looking only for report('literal') misses it.
        index = {'report': 0, 'begin_final_alignment': 3}.get(node.func.attr)
        if index is not None and len(node.args) > index:
            phase = node.args[index]
            if isinstance(phase, ast.Constant) and isinstance(phase.value, str):
                phases.add(phase.value)
    literals = {n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)}
    required_topics = {'/detection/closed_approach/status', '/detection/closed_approach/target'}
    unknown = sorted(phases-(ACTIVE_PHASES | INACTIVE_PHASES | {'succeeded'}))
    missing = sorted(required_topics-literals)
    return dict(ok=tuple(classes or ()) == CLASSES and not unknown and not missing,
                class_schema_matches=tuple(classes or ()) == CLASSES,
                observed_phases=sorted(phases), unknown_phases=unknown, missing_topics=missing)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    args = parser.parse_args()
    result = inspect_source(args.source.read_text())
    print(json.dumps(result, ensure_ascii=False))
    raise SystemExit(0 if result['ok'] else 1)


if __name__ == '__main__':
    main()
