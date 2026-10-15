import importlib.util
import unittest
from pathlib import Path
import yaml

PACKAGE=Path(__file__).resolve().parents[1]
ROOT=PACKAGE.parents[2]
spec=importlib.util.spec_from_file_location('demo_check',ROOT/'tools/check_live_demo.py')
module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
CFG=yaml.safe_load((PACKAGE/'config/robot.yaml').read_text())


class DemoCheckTests(unittest.TestCase):
    def test_all_missing_values_reported_without_claiming_handeye_missing(self):
        report=module.configuration(CFG)
        self.assertIn('can_port',report['missing_values'])
        self.assertIn('table_z_m',report['missing_values'])
        self.assertNotIn('handeye_transform',report['missing_values'])
        self.assertNotIn('link6_tcp_transform',report['missing_values'])
        self.assertFalse(report['hardware_config_ready'])
        self.assertFalse(CFG['commissioned'])

    def test_disabled_banana_does_not_block_other_class_demo(self):
        report=module.configuration(CFG)
        self.assertFalse(any(k.startswith('classes.banana.') for k in report['missing_values']))
        import copy
        cfg=copy.deepcopy(CFG); cfg['classes']['banana']['enabled']=True
        cfg['classes']['banana']['radius_m']=None
        report=module.configuration(cfg)
        self.assertIn('classes.banana.radius_m',report['missing_values'])
        self.assertNotIn('classes.banana.closing_axis_yaw_offset_rad',report['missing_values'])

    def test_invalid_transform_is_reported_even_before_flags_are_set(self):
        import copy
        cfg=copy.deepcopy(CFG); cfg['handeye_transform']['quaternion_xyzw']=[0,0,0,0]
        report=module.configuration(cfg)
        self.assertTrue(any('TRANSFORM:handeye_transform' in issue for issue in report['issues']))


if __name__=='__main__': unittest.main()
