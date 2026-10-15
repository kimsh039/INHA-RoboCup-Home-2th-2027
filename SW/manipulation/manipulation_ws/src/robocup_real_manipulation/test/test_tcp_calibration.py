import copy
import json
import unittest
from pathlib import Path
import numpy as np
from robocup_real_manipulation.core import Fault
from robocup_real_manipulation.tcp_calibration import axis_rotation, solve, apply_result, Model, inspect_cad, stable_window


def dataset():
    rng = np.random.default_rng(7)
    probe = np.array([.012, -.008, .165]); center = np.array([.35, -.1, .2]); samples = []
    for i in range(24):
        R = axis_rotation(rng.normal(size=3), rng.uniform(-1.5, 1.5))
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = center-R@probe
        samples.append(dict(split='train' if i < 18 else 'holdout', base_link6_matrix=T.tolist()))
    return dict(input_kind='synthetic_test', samples=samples), probe


class TCPTests(unittest.TestCase):
    def test_actual_cad_datum_and_fk_against_archived_pose(self):
        repo = Path(__file__).resolve().parents[6]
        model = Model(repo/'SW/simulation/robot_description/robocup.urdf')
        result, _ = inspect_cad(model)
        np.testing.assert_allclose(result['link6_tcp_transform']['xyz'], [0, 0, .1425], atol=1e-8)
        self.assertFalse(result['hardware_accuracy_established'])
        saved = json.loads((repo/'SW/simulation/calibration/records/link6_tcp/train/000/flange_pose.json').read_text())
        T = model.fk('piper_base_link', 'piper_link6', saved['measured_joint_positions'])
        np.testing.assert_allclose(T, saved['matrix4x4'], atol=1e-8)

    def test_capture_stability_rejects_duplicates_gaps_and_motion(self):
        history = [(1+i*.1, np.zeros(6), np.zeros(6)) for i in range(12)]
        self.assertTrue(stable_window(history, 1, .001))
        duplicate = list(history); duplicate[3] = duplicate[2]
        self.assertFalse(stable_window(duplicate, 1, .001))
        gap = [(s[0]+(.5 if i > 5 else 0), s[1], s[2]) for i, s in enumerate(history)]
        self.assertFalse(stable_window(gap, 1, .001))
        moving = list(history); moving[-1] = (moving[-1][0], np.full(6, .002), np.zeros(6))
        self.assertFalse(stable_window(moving, 1, .001))

    def test_pivot_and_rotated_probe_offset(self):
        data, p = dataset(); q = [0, 0, np.sin(.3), np.cos(.3)]
        d = [.01, .02, -.03]
        result = solve(data, q, d)
        expected = p-axis_rotation([0, 0, 1], .6)@d
        np.testing.assert_allclose(result['link6_tcp_transform']['xyz'], expected, atol=1e-12)
        self.assertEqual(result['status'], 'passed')
        self.assertFalse(result['orientation_estimated'])

    def test_holdout_failure_is_not_fitted_away(self):
        data, _ = dataset(); data['samples'][-1]['base_link6_matrix'][0][3] += .01
        result = solve(data, [0, 0, 0, 1], [0, 0, 0])
        self.assertEqual(result['status'], 'failed_residual')
        self.assertLess(max(result['train_errors_m']), 1e-10)

    def test_same_orientation_is_unobservable(self):
        data, _ = dataset()
        for s in data['samples']: s['base_link6_matrix'] = np.eye(4).tolist()
        with self.assertRaises(Fault): solve(data, [0, 0, 0, 1], [0, 0, 0])

    def test_holdout_count_and_bad_pose(self):
        data, _ = dataset(); data['samples'] = data['samples'][:20]
        with self.assertRaises(Fault): solve(data, [0, 0, 0, 1], [0, 0, 0])
        data, _ = dataset(); data['samples'][0]['base_link6_matrix'][0][0] = 2.
        with self.assertRaises(Fault): solve(data, [0, 0, 0, 1], [0, 0, 0])

    def test_apply_rejects_synthetic_unverified_and_preserves_flags(self):
        data, _ = dataset(); result = solve(data, [0, 0, 0, 1], [0, 0, 0])
        cfg = dict(link6_tcp_transform=None, calibration_verified=False, commissioned=False,
                   execution_enabled=False, handeye_transform=None, joint_signs=[1]*6, joint_offsets=[0]*6)
        original = copy.deepcopy(cfg)
        with self.assertRaises(Fault): apply_result(cfg, result, True)
        result.update(input_kind='physical_manual_contact', axes_verified=True, contact_geometry_verified=False)
        with self.assertRaises(Fault): apply_result(cfg, result, True)
        result['contact_geometry_verified'] = True
        result.update(joint_signs=[1]*6, joint_offsets=[0]*6)
        updated = apply_result(cfg, result, True)
        self.assertEqual(cfg, original)
        self.assertFalse(updated['calibration_verified']); self.assertFalse(updated['execution_enabled'])
        self.assertIsNone(updated['handeye_transform'])
        result['joint_offsets'][0] = .01
        with self.assertRaises(Fault): apply_result(cfg, result, True)


if __name__ == '__main__': unittest.main()
