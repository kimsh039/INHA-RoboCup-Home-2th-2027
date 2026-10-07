"""변경된 하드웨어의 장착 변환, 영점, 모델 출처와 입력 불일치를 검사한다."""
import json,unittest
import numpy as np
from pathlib import Path
from manipulation.mid360_cloud import validate_scene_metadata
ROOT=Path(__file__).resolve().parents[1]
class UpdatedHardwareTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.report=json.loads((ROOT/'reports/hardware_comparison.json').read_text())
 def test_mount_and_fixed_scene(self):
  np.testing.assert_allclose(self.report['mount']['delta_xyz_m'],[-.026251615091,0,-.009],atol=1e-12)
  old=json.loads((ROOT/'tests/fixtures/previous_model/grasp_simulation.json').read_text());new=json.loads((ROOT/'config/grasp_simulation.json').read_text())
  for k in old:
   if k not in ['initial_joints_rad','folded_candidate_rad']:self.assertEqual(old[k],new[k],k)
 def test_new_initial_and_transport_are_separate(self):
  p=self.report['initial_pose'];self.assertEqual(p['new_rad'],[0]*6)
  old=np.array(p['old_success_rad']);old[0]-=1.6
  np.testing.assert_allclose(p['folded_rad'],old)
 def test_camera_and_mesh_integrity(self):
  self.assertTrue(self.report['T_gripper_camera']['unchanged']);self.assertTrue(self.report['sensor_specs_unchanged']);self.assertTrue(self.report['mesh_audit']['all_valid'])
 def test_old_success_metadata_is_rejected(self):
  meta=json.loads((ROOT/'tests/fixtures/previous_model/metadata.json').read_text())
  with self.assertRaisesRegex(ValueError,'SCENE_CHANGED'):validate_scene_metadata(meta)
