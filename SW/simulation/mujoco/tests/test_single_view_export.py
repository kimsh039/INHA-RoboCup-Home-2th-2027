"""Exercise notebook export/import using explicit synthetic parser fixtures.

This executes no CUDA build, checkpoint, neural inference, ROS or physics.
"""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import zipfile

import numpy as np

from scripts.generate_single_view import generate
from scripts.import_graspnet_output import import_output

ROOT = Path(__file__).resolve().parents[1]


class SingleViewExportTest(unittest.TestCase):
    def test_notebook_export_then_import(self):
        notebook = json.loads((ROOT/'tests/fixtures/graspnet_single_view.ipynb').read_text())
        source = ''.join(notebook['cells'][10]['source'])
        # Start AFTER neural inference. These are test rows, never production grasps.
        source = source[source.index('# Preserve original camera-frame decoded output'):]
        points, camera, meta = generate()
        T = np.array(meta['T_camera_object'])
        raw = np.zeros((2, 17))
        raw[:, :4] = [[.5, .04, .02, .03], [.9, 0, .02, .04]]
        raw[:, 4:13] = np.eye(3).reshape(-1)
        raw[:, 13:16] = [[0, 0, .48], [.01, .01, .48]]
        raw[:, 16] = -1
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root/'notebook_output'
            source = source.replace("pathlib.Path('/content/graspnet_output')", 'pathlib.Path(TEST_OUTPUT)')
            env = {'np': np, 'json': json, 'pathlib': __import__('pathlib'),
                   'sys': __import__('sys'), 'TEST_OUTPUT': str(output),
                   'candidates': raw.copy(), 'metadata': meta,
                   'T_camera_object': T, 'model_cloud': camera,
                   'BASELINE_COMMIT': 'synthetic-test-fixture', 'PATCHED_FILES': [],
                   'CHECKPOINT_SHA256': 'no-checkpoint-in-test', 'SEED': 42,
                   'NUM_INPUT_POINTS': 20000,
                   'torch': SimpleNamespace(__version__='not-used', version=SimpleNamespace(cuda='not-used'))}
            exec(compile(source, 'notebook_export_fixture', 'exec'), env)
            exported = json.loads((output/'grasps.json').read_text())
            self.assertEqual([g['candidate_id'] for g in exported['grasps']], [1, 0])
            np.testing.assert_array_equal(np.load(output/'grasps_camera_raw.npy'), raw)
            transformed = np.load(output/'grasps_raw.npy')
            np.testing.assert_array_equal(transformed[:, :4], raw[:, :4])
            np.testing.assert_allclose(transformed[:, 13:16], [[-.02, 0, 0], [-.02, -.01, -.01]])
            meta_path = root/'metadata.json';meta_path.write_text(json.dumps(meta))
            archive = root/'fixture.zip'
            with zipfile.ZipFile(archive, 'w') as z:
                for name in ('grasps.json', 'grasps_raw.npy', 'grasps_camera_raw.npy', 'network_input_camera.npy'):
                    z.write(output/name, name)
            destination = root/'imported'
            self.assertEqual(import_output(archive, meta_path, destination), 2)
            pre = json.loads((destination/'pregrasps_object.json').read_text())
            for g in pre['grasps']:
                grasp = np.array(g['T_object_grasp']);approach = grasp[:3, 0]
                np.testing.assert_allclose(np.array(g['T_object_pregrasp'])[:3, 3]-grasp[:3, 3], -.20*approach)
            self.assertTrue((destination/'network_input_camera.npy').exists())
            import_output(archive, meta_path, destination)
            self.assertEqual(len(list(destination.glob('previous_*'))), 1)
            meta['camera_position_object_m'] = [1, 1, 1]
            meta_path.write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError, 'MODEL_MISMATCH'):
                import_output(archive, meta_path, destination)
