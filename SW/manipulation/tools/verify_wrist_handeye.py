#!/usr/bin/env python3
"""Offline audit of pinned physical hand-eye records; never sends robot commands.

Optional verification dependencies: numpy, scipy, opencv-python, PyYAML.
They are not additional runtime requirements of the manipulation package.
"""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def verify(reference_dir):
    source = reference_dir/'wrist_handeye_calibration.source.yaml.txt'
    data_file = reference_dir/'physical_handeye_dataset_best15.json'
    original = source.read_text()
    try:
        doc = yaml.safe_load(original); repair = None
    except yaml.YAMLError:
        # Original branch file has validation indented inside transform_matrix.
        # Preserve the byte content separately; repair this one known indentation.
        assert original.count('\n  validation:\n') == 1
        doc = yaml.safe_load(original.replace('\n  validation:\n', '\nvalidation:\n', 1))
        repair = 'Dedented validation mapping from transform_matrix; original preserved.'
    dataset = json.loads(data_file.read_text())
    assert dataset['mode'] == 'physical_eye_in_hand'
    assert dataset['gripper_frame'] == doc['parent_frame'] == 'piper_link6'
    assert dataset['camera_frame'] == doc['child_frame'] == 'wrist_camera_optical_frame'
    samples = dataset['samples']; assert len(samples) == doc['sample_count'] == 15
    poses = [np.asarray(s['T_base_link6'], float) for s in samples]
    board = [np.asarray(s['T_camera_target'], float) for s in samples]
    for T in poses+board:
        assert T.shape == (4,4) and np.isfinite(T).all()
        assert np.allclose(T[3], [0,0,0,1])
        assert np.allclose(T[:3,:3].T@T[:3,:3], np.eye(3), atol=1e-6)
    q = np.array([doc['quaternion_xyzw'][k] for k in 'xyzw'])
    assert abs(np.linalg.norm(q)-1) < 1e-5
    q /= np.linalg.norm(q); R = Rotation.from_quat(q).as_matrix()
    xyz = np.array([doc['translation_m'][k] for k in 'xyz']); matrix=np.asarray(doc['transform_matrix'])
    assert np.allclose(xyz, matrix[:3,3], atol=1e-6)
    assert np.allclose(R, matrix[:3,:3], atol=2e-6)
    r, t = cv2.calibrateHandEye([T[:3,:3] for T in poses], [T[:3,3] for T in poses],
                              [T[:3,:3] for T in board], [T[:3,3] for T in board],
                              method=cv2.CALIB_HAND_EYE_PARK)
    delta_mm = float(np.linalg.norm(t.ravel()-xyz)*1000)
    delta_deg = float(np.degrees(Rotation.from_matrix(r@R.T).magnitude()))
    assert delta_mm < .01 and delta_deg < .001, 'Published result does not reproduce'
    return dict(source_repository='kimsh039/INHA-RoboCup-Home-2th-2027',
                source_branch='feature/wrist-handeye-calibration',
                source_commit='0bac1e558075600d21c6b7273f26a6858266ea0d',
                source_path='SW/simulation/calibration/wrist_handeye/wrist_handeye_calibration.yaml',
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                dataset_sha256=hashlib.sha256(data_file.read_bytes()).hexdigest(),
                source_yaml_repair=repair, parent_frame=doc['parent_frame'], child_frame=doc['child_frame'],
                camera_serial=dataset['camera_serial'], sample_count=len(samples),
                handeye_transform={'xyz':xyz.tolist(),'quaternion_xyzw':q.tolist()},
                offline_reproduction={'translation_difference_mm':delta_mm,'rotation_difference_deg':delta_deg},
                source_reported_validation=doc['validation'],
                hardware_revalidated_in_this_task=False)


if __name__ == '__main__':
    import argparse
    p=argparse.ArgumentParser(); p.add_argument('--references',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True); a=p.parse_args()
    result=verify(a.references)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result['offline_reproduction']))
