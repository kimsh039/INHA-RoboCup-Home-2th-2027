"""Six-class migration must preserve measured robot calibration and authorization."""
import copy
import importlib.util
from pathlib import Path
import pytest
import yaml
from robocup_real_manipulation.core import rule_for
from robocup_real_manipulation.tuning import overlay

PACKAGE=Path(__file__).resolve().parents[1]
MANIP=PACKAGE.parents[2]
CFG=yaml.safe_load((PACKAGE/'config/robot.yaml').read_text())
spec=importlib.util.spec_from_file_location('configure_multiclass',MANIP/'tools/configure_multiclass.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_migrate_green_apple_trial_to_all_six_preserves_robot_settings():
    old=copy.deepcopy(CFG)
    for name,rule in old['classes'].items():
        rule['enabled']=name=='green_apple'
    old['class_execution']={name:{'ready':name=='green_apple'} for name in old['classes']}
    old.update(can_port='measured-port',table_z_m=.5775,
               joint_offsets=[.1,.2,.3,.4,.5,.6],width_mapping_mode='nominal_trial')
    before=copy.deepcopy(old)
    result=module.configure(old,CFG)
    assert old==before
    assert result['classes']==CFG['classes'] and 'class_execution' not in result
    assert all(rule_for(result,i)['enabled'] for i in range(6))
    for key,value in old.items():
        if key not in ('classes','class_execution'):
            assert result[key]==value


def test_migration_rejects_wrong_detector_schema():
    old=copy.deepcopy(CFG)
    old['model_names']=['apple','banana']
    with pytest.raises(ValueError,match='CLASS_SCHEMA_MISMATCH'):
        module.configure(old,CFG)


def test_default_integrated_tuning_enables_all_classes_on_old_config():
    old=copy.deepcopy(CFG)
    for rule in old['classes'].values():
        rule['enabled']=False
    result=overlay(old,yaml.safe_load((PACKAGE/'config/tuning.yaml').read_text()))
    assert [rule_for(result,i)['class_id'] for i in range(6)]==list(range(6))
    assert result['classes']['banana']['axis_required'] is True
    assert result['physical_validation']['banana']['pick_place_success_recorded'] is False
