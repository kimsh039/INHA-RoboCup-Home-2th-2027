"""Check imported physical calibration reaches the generated robot description.

No hardware launch, robot commands, or invented hardware certification.
"""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import yaml
from robocup_real_manipulation.core import rotation, transform_point
from robocup_real_manipulation.description import prepare
from robocup_real_manipulation.tcp_calibration import Model

PACKAGE = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[6]
CFG = yaml.safe_load((PACKAGE/'config/robot.yaml').read_text())
SOURCE = yaml.safe_load((REPO/'SW/manipulation/references/wrist_handeye_calibration.source.yaml.txt').read_text().replace('\n  validation:\n', '\nvalidation:\n', 1))


class CalibrationIntegrationTests(unittest.TestCase):
    def test_physical_source_matches_config_and_direction(self):
        self.assertEqual(SOURCE['parent_frame'], 'piper_link6')
        self.assertEqual(SOURCE['child_frame'], CFG['wrist_optical_frame'])
        source = np.asarray(SOURCE['transform_matrix'])
        cfg = CFG['handeye_transform']
        np.testing.assert_allclose(cfg['xyz'], source[:3,3], atol=1e-7)
        np.testing.assert_allclose(rotation(cfg['quaternion_xyzw']), source[:3,:3], atol=2e-6)
        camera_point = np.array([.02, -.01, .3])
        actual = transform_point(camera_point, cfg['xyz'], cfg['quaternion_xyzw'])
        np.testing.assert_allclose(actual, (source @ np.r_[camera_point,1])[:3], atol=1e-6)

    def test_generated_urdf_contains_physical_optical_and_cad_tcp(self):
        with tempfile.TemporaryDirectory() as d:
            # Exercise description export only. Hardware authorization validation
            # is tested separately; this test never starts a launch or driver.
            with patch('robocup_real_manipulation.description.validate'):
                prepare(REPO, CFG, Path(d))
            model = Model(Path(d)/'robot.urdf')
            T = model.fk('piper_link6', CFG['wrist_optical_frame'])
            np.testing.assert_allclose(T, SOURCE['transform_matrix'], atol=2e-6)
            tcp = model.fk('piper_link6', CFG['tcp_frame'])
            np.testing.assert_allclose(tcp[:3,3], [0,0,.1425], atol=1e-9)
            mount = model.fk('base_link', 'piper_base_link')
            np.testing.assert_allclose(mount[:3,3], [-.0195,0,.80611], atol=1e-9)

    def test_import_does_not_claim_all_robot_measurements_complete(self):
        self.assertFalse(CFG['execution_enabled'])
        self.assertFalse(CFG['commissioned'])
        self.assertFalse(CFG['calibration_verified'])
        self.assertEqual(CFG['reference_sources']['physical_handeye'],
                         'feature/wrist-handeye-calibration@0bac1e558075600d21c6b7273f26a6858266ea0d')


if __name__ == '__main__': unittest.main()
