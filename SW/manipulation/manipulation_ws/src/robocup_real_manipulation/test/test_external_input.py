import copy
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import yaml
from robocup_real_manipulation.external_input import check_pair, auto_candidate

CFG=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/robot.yaml').read_text())


class ExternalInputTests(unittest.TestCase):
    def pair(self):
        header=NS(stamp=NS(sec=10,nanosec=50),frame_id='camera_color_optical_frame')
        head=NS(header=header,valid=True,measured=True,reason='',target_id='tracked-object',
                class_id=2,class_name='tomato',confidence=.9)
        point=NS(header=NS(stamp=copy.deepcopy(header.stamp),frame_id='map'),point=NS(x=.5,y=.1,z=.75))
        return head,point

    def test_exact_image_pair_preserves_external_identity(self):
        head,point=self.pair()
        self.assertEqual(check_pair(head,point,CFG,11.,2.),'tracked-object')
        self.assertEqual(point.header.frame_id,'map')

    def test_receipt_order_cannot_pair_different_image_acquisitions(self):
        head,point=self.pair(); point.header.stamp.nanosec+=1
        with self.assertRaisesRegex(ValueError,'CAPTURE_MISMATCH'):
            check_pair(head,point,CFG,11.,2.)

    def test_raw_legacy_class_is_rejected_not_silently_reinterpreted(self):
        head,point=self.pair(); head.class_name='apple'; head.class_id=0
        with self.assertRaisesRegex(ValueError,'CLASS_SCHEMA'):
            check_pair(head,point,CFG,11.,2.)

    def test_stale_tracking_only_and_nonfinite_3d_are_rejected(self):
        head,point=self.pair()
        with self.assertRaisesRegex(ValueError,'STALE'):
            check_pair(head,point,CFG,20.,2.)
        head.measured=False
        with self.assertRaisesRegex(ValueError,'UNVERIFIED'):
            check_pair(head,point,CFG,11.,2.)
        head.measured=True; point.point.z=float('nan')
        with self.assertRaisesRegex(ValueError,'INVALID_3D'):
            check_pair(head,point,CFG,11.,2.)

    def test_auto_handoff_needs_same_job_success_stop_and_no_previous_attempt(self):
        targets={'A':object()}
        self.assertIsNone(auto_candidate(targets,{'B':NS(outcome='SUCCESS',base_stopped=True)},set()))
        self.assertIsNone(auto_candidate(targets,{'A':NS(outcome='FAILED',base_stopped=True)},set()))
        self.assertIsNone(auto_candidate(targets,{'A':NS(outcome='SUCCESS',base_stopped=False)},set()))
        self.assertEqual(auto_candidate(targets,{'A':NS(outcome='SUCCESS',base_stopped=True)},set()),'A')
        self.assertIsNone(auto_candidate(targets,{'A':NS(outcome='SUCCESS',base_stopped=True)},{'A'}))


if __name__=='__main__': unittest.main()
