import importlib.util
from pathlib import Path
import tempfile
import yaml

SCRIPT = Path(__file__).resolve().parents[4]/'tools/patch_control_center_joints.py'
spec = importlib.util.spec_from_file_location('joint_patch', SCRIPT)
patch = importlib.util.module_from_spec(spec); spec.loader.exec_module(patch)

SOURCE = """class Approach:
    def __init__(self):
        self.planner = GraspPlanner(settings)
    def plan(self):
        with self.state.condition:
            joints = self.state.values.get('joints') or {}
        positions = dict(zip(joints.get('names', []), joints.get('positions', [])))
        keep_other_developer_change = 42
"""


def test_patch_preserves_other_code_and_handles_two_planning_sites():
    source = SOURCE+SOURCE.split('    def plan(self):')[1].join(['    def alternate(self):', ''])
    result, count = patch.rewrite(source)
    assert count == 2
    assert result.count('self.arm_feedback.positions()') == 2
    assert 'keep_other_developer_change = 42' in result
    assert patch.rewrite(result) == (result, 0)


def test_apply_is_backed_up_check_only_and_idempotent():
    with tempfile.TemporaryDirectory() as td:
        folder = Path(td); target = folder/'detection_approach.py'; target.write_text(SOURCE)
        helper = folder/'helper.py'; helper.write_text('class ModelJointFeedback: pass\n')
        assert patch.apply(folder, helper, check=True) == (1, None)
        assert target.read_text() == SOURCE
        count, backup = patch.apply(folder, helper)
        assert count == 1 and (backup/target.name).read_text() == SOURCE
        assert (folder/'joint_contract.py').read_bytes() == helper.read_bytes()
        assert patch.apply(folder, helper) == (0, None)


def test_model_names_match_actual_manipulation_config():
    root = Path(__file__).resolve().parents[5]
    helper = root/'control_center/joint_contract.py'
    spec = importlib.util.spec_from_file_location('model_joint_contract', helper)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    config = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/robot.yaml').read_text())
    assert config['joint_names'] == list(module.MODEL_JOINTS)
    assert config['driver_joint_names'] == [f'joint{i}' for i in range(1, 7)]
