"""Synthetic parser fixtures; these are not claimed executable grasp candidates."""
import json
import tempfile
from pathlib import Path
from unittest.mock import patch
import unittest
import zipfile
import numpy as np
from scripts import import_graspnet_output as importer

class GraspImportTest(unittest.TestCase):
    def test_matching_import_derives_pregrasp_and_rejects_stale(self):
        project=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(dir=project/'reports') as directory:
            root=Path(directory)
            cloud=root/'data/pointclouds/cube';cloud.mkdir(parents=True)
            meta={'test':'synthetic parser fixture'};(cloud/'metadata.json').write_text(json.dumps(meta))
            (root/'config').mkdir();(root/'config/pregrasp.json').write_text(json.dumps({'distance_m':.20,'rotation_tolerance':1e-5}))
            source={'input_metadata':meta,'units':'meters','point_cloud_frame':'object','approach_axis':'+X_grasp (rotation_matrix first column)','grasps':[{'candidate_id':0,'rotation_matrix':np.eye(3).tolist(),'translation':[0,0,0],'depth':.02,'width':.04,'score':.5}]}
            archive=root/'result.zip'
            with zipfile.ZipFile(archive,'w') as z:z.writestr('grasps.json',json.dumps(source))
            with patch.object(importer,'ROOT',root):
                self.assertEqual(importer.import_output(archive),1)
                out=json.loads((root/'data/grasps/cube/pregrasps_object.json').read_text())
                np.testing.assert_allclose(np.array(out['grasps'][0]['T_object_pregrasp'])[:3,3],[-.20,0,0])
                source['input_metadata']={'test':'stale'}
                with zipfile.ZipFile(archive,'w') as z:z.writestr('grasps.json',json.dumps(source))
                with self.assertRaisesRegex(ValueError,'MODEL_MISMATCH'):importer.import_output(archive)
