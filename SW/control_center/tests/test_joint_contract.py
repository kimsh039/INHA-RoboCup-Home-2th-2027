import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('joint_contract', Path(__file__).resolve().parents[1]/'joint_contract.py')
joint = importlib.util.module_from_spec(spec)
spec.loader.exec_module(joint)


class JointContractTests(unittest.TestCase):
    def setUp(self):
        self.cache = joint.JointSnapshot()
        self.names = list(joint.MODEL_JOINTS)
        self.values = [-.044, .2, -.3, .07, .66, -.04]

    def test_matches_manipulation_names(self):
        self.assertEqual(self.names, [f'piper_joint{i}' for i in range(1, 7)])

    def test_model_values_are_not_offset_again(self):
        self.assertTrue(self.cache.receive(self.names, self.values, 100., 100.1))
        self.assertEqual(self.cache.positions(100.1), dict(zip(self.names, self.values)))

    def test_order_and_extra_gripper_are_supported(self):
        self.assertTrue(self.cache.receive(self.names[::-1]+['gripper'], self.values[::-1]+[.03], 100., 100.1))
        self.assertEqual(self.cache.positions(100.1), dict(zip(self.names, self.values)))

    def test_raw_names_are_not_silently_renamed(self):
        self.assertFalse(self.cache.receive([f'joint{i}' for i in range(1, 7)], self.values, 100., 100.1))
        self.assertEqual(self.cache.positions(100.1), {})

    def test_malformed_and_nonfinite_are_rejected(self):
        for names, values in [(self.names, self.values[:-1]), (self.names[:-1]+[self.names[0]], self.values),
                              (self.names, [float('nan')]+self.values[1:])]:
            self.assertFalse(self.cache.receive(names, values, 100., 100.1))

    def test_stale_future_and_missing_stamp_are_rejected(self):
        for stamp in [0., 99., 101., float('nan')]:
            self.assertFalse(self.cache.receive(self.names, self.values, stamp, 100.1))

    def test_stale_cached_feedback_cannot_plan(self):
        self.cache.receive(self.names, self.values, 100., 100.1)
        self.assertEqual(self.cache.positions(100.7), {})
        self.assertEqual(self.cache.positions(99.), {})

    def test_older_sample_cannot_overwrite(self):
        self.cache.receive(self.names, self.values, 100., 100.1)
        self.assertFalse(self.cache.receive(self.names, [0.]*6, 99.9, 100.1))
        self.assertEqual(self.cache.positions(100.1)[self.names[0]], self.values[0])

    def test_no_mutable_cache_leak(self):
        self.cache.receive(self.names, self.values, 100., 100.1)
        copy = self.cache.positions(100.1); copy.clear()
        self.assertEqual(len(self.cache.positions(100.1)), 6)


if __name__ == '__main__':
    unittest.main()
