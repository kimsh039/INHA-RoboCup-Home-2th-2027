"""Notebook output/import fixture, no trained model prediction is fabricated."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import zipfile

import numpy as np

from manipulation.wrist_camera_cloud import apply_transform
from scripts.import_graspnet_output import import_output

ROOT = Path(__file__).resolve().parents[1]


class WristCameraExportTest(unittest.TestCase):
    def test_network_output_object_world_pregrasp_import_roundtrip(self):
        meta = json.loads((ROOT/'data/pointclouds/cube_wrist_camera/metadata.json').read_text())
        cloud = np.load(ROOT/'data/pointclouds/cube_wrist_camera/points.npy')
        T = np.array(meta['T_network_object']);W = np.array(meta['T_world_object'])
        raw = np.zeros((2,17))
        raw[:, :4] = [[.6,.05,.02,.03],[.9,0,.02,.04]]
        raw[:,4:13] = np.eye(3).reshape(-1)
        raw[:,13:16] = apply_transform(T, cloud[:2]);raw[:,16] = -1
        nb = json.loads((ROOT/'colab/graspnet_wrist_camera.ipynb').read_text())
        code = ''.join(nb['cells'][10]['source'])
        code = code[code.index('# Preserve original network-frame decoded output'):]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'export'
            code = code.replace("pathlib.Path('/content/graspnet_output')", 'pathlib.Path(TEST_OUTPUT)')
            env = {'np':np,'json':json,'pathlib':__import__('pathlib'),'sys':__import__('sys'),
                   'TEST_OUTPUT':str(output),'candidates':raw.copy(),'metadata':meta,
                   'T_network_object':T,'T_world_object':W,'cloud':cloud,
                   'model_cloud':apply_transform(T,cloud), 'BASELINE_COMMIT':'test-fixture',
                   'PATCHED_FILES':[],'CHECKPOINT_SHA256':'no-checkpoint','SEED':42,'NUM_INPUT_POINTS':20000,
                   'torch':SimpleNamespace(__version__='not-used',version=SimpleNamespace(cuda='not-used'))}
            exec(compile(code,'wrist_camera_export_fixture','exec'),env)
            np.testing.assert_array_equal(np.load(output/'grasps_network_raw.npy'),raw)
            transformed = np.load(output/'grasps_raw.npy')
            np.testing.assert_array_equal(transformed[:,:4],raw[:,:4])
            np.testing.assert_allclose(transformed[:,13:16],cloud[:2],atol=1e-7)
            pre = json.loads((output/'pregrasps_object.json').read_text())
            for g in pre['grasps']:
                object_grasp=np.array(g['T_object_grasp']);world_grasp=np.array(g['T_world_grasp'])
                np.testing.assert_allclose(W@object_grasp,world_grasp)
                np.testing.assert_allclose(world_grasp[:3,3]-np.array(g['T_world_pregrasp'])[:3,3],.20*world_grasp[:3,0],atol=1e-12)
            archive=Path(directory)/'fixture.zip'
            with zipfile.ZipFile(archive,'w') as z:
                for name in ['grasps.json','grasps_raw.npy','grasps_network_raw.npy','network_input_sensor.npy']:
                    z.write(output/name,name)
            dest=Path(directory)/'import'
            self.assertEqual(import_output(archive,ROOT/'data/pointclouds/cube_wrist_camera/metadata.json',dest),2)
            self.assertTrue((dest/'network_input_sensor.npy').exists())
            self.assertFalse(json.loads((dest/'pregrasps_object.json').read_text())['feasibility_checked'])
