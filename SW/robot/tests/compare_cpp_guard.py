"""Required C++/Python guard parity checks; no node or actuator is created."""
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collision_guard import CollisionGuard
from collision_distance import PreparedBoxes, build_scene
from collision_geometry import RobotGeometry


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: compare_cpp_guard.py /path/to/guard_reference_probe')
    model = RobotGeometry()
    q = {name: 0. for name in model.required_joints}
    boxes = PreparedBoxes(model.boxes(q))
    node = CollisionGuard.__new__(CollisionGuard)
    node.cfg = dict(prediction_step=.04, linear_deceleration=.5, angular_deceleration=1.)
    node.scene_sources, node.pose, node.nearest_source = None, np.zeros(3), None
    random = np.random.default_rng(20261015)
    clouds = [np.empty((0, 3))]
    for x in (.56, .65, .8, 1.1):
        y, z = np.meshgrid(np.linspace(-1., 1., 21), np.linspace(-.1, 1.5, 12))
        clouds.append(np.column_stack((np.full(y.size, x), y.ravel(), z.ravel())))
    clouds.append(random.uniform(-1.2, 1.2, (1200, 3)))
    capture = Path('/tmp/robocup-guard-optimization-input.npz')
    if capture.exists():
        clouds.append(np.load(capture)['points'])
    fixture = dict(geometry_file='/tmp/robocup-guard-test-geometry.yaml',
                   joints=[q[name] for name in sorted(q)], scenes=[])
    subprocess.run(['/usr/bin/python3', str(Path(__file__).resolve().parents[1]/'export-collision-geometry.py'),
                    '--output', fixture['geometry_file']], check=True)
    expected, elapsed = [], []
    boundary = .1+.05+np.sqrt(3)*.04/2
    for cloud in clouds:
        node.scene = build_scene(cloud)
        queries = []
        for i in range(12):
            pose = np.array([0., 0., (i % 3)*.07])
            command = np.array([(.25, .12, -.15, 0.)[i % 4], (.6, -.4, 0.)[i % 3]])
            if i == 0:
                command[:] = 0.
            measured = np.array([.1, -.1]) if i % 3 == 1 else np.zeros(2)
            fresh, error = i != 2, 'invalid_velocity_command' if i == 3 else ''
            source_age = (.05, .15, .3)[i % 3]
            queries.append(dict(pose=pose.tolist(), measured=measured.tolist(), command=command.tolist(),
                                fresh=fresh, error=error, source_age=source_age))
            began = time.perf_counter()
            deadline = time.monotonic()+5.
            distance, _, _ = node.distance(boxes, pose, boundary+.1, deadline)
            state, reason, scale = 'blocked', '', 0.
            if distance < boundary:
                reason = 'clearance_boundary'
            elif error:
                reason = error
            elif not fresh:
                state, reason = 'idle', 'command_timeout'
            elif not np.any(np.abs(command) > 1e-5):
                state, reason = 'idle', 'zero_command'
            else:
                latency = .3+source_age
                reaction = node.predict(pose, [(measured.copy(), latency, False)], boxes, boundary, boxes.extent, deadline)
                if reaction is not None:
                    for candidate in (1., .75, .5, .25, .1):
                        if node.admissible(command*candidate, reaction, latency, boxes, boundary, boxes.extent, deadline):
                            scale = candidate
                            state, reason = ('clear', 'clear_path') if scale == 1. else ('slowed', 'braking_clearance')
                            break
                if scale == 0.:
                    reason = 'predicted_clearance_violation'
            expected.append(dict(state=state, reason=reason, scale=scale,
                                 distance=float(distance) if np.isfinite(distance) else None))
            elapsed.append((time.perf_counter()-began)*1000.)
        fixture['scenes'].append(dict(points=cloud.tolist(), queries=queries))
    path = Path('/tmp/robocup-guard-parity-fixture.json')
    path.write_text(json.dumps(fixture))
    actual = json.loads(subprocess.check_output([sys.argv[1], str(path)]))
    assert len(expected) == len(actual)
    for i, (a, b) in enumerate(zip(expected, actual)):
        for key in ('state', 'reason', 'scale'):
            assert a[key] == b[key], (i, key, a, b)
        if a['distance'] is None:
            assert b['distance'] is None, (i, a, b)
        else:
            np.testing.assert_allclose(a['distance'], b['distance'], atol=1e-10, rtol=1e-10)
    result = dict(cases=len(actual), states=sorted(set(r['reason'] for r in actual)),
                  python_ms_median=float(np.median(elapsed)),
                  cpp_ms_median=float(np.median([r['processing_ms'] for r in actual])),
                  cpp_ms_p95=float(np.percentile([r['processing_ms'] for r in actual], 95)))
    Path('/tmp/robocup-cpp-guard-parity-result.json').write_text(json.dumps(result))
    print(json.dumps(result))


if __name__ == '__main__':
    main()
