"""해법 변경만 허용하고 관측 장면 변경은 거절하는지 검증한다."""
import json
import tempfile
import unittest
from pathlib import Path
from manipulation.mid360_cloud import scan_scene_hash


class ScanHashTest(unittest.TestCase):
    def test_contact_settings_only(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json'
            cfg={'object_world_m':[.5,0,.74]}
            path.write_text(json.dumps(cfg))
            original=scan_scene_hash(path)
            cfg.update(contact_multiccd=True,contact_noslip_iterations=5)
            path.write_text(json.dumps(cfg))
            self.assertEqual(original,scan_scene_hash(path))
            cfg['object_world_m'][0]=.6
            path.write_text(json.dumps(cfg))
            self.assertNotEqual(original,scan_scene_hash(path))
