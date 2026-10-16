"""Regression evidence: real success telemetry + isolated synthetic boundary cases."""
import copy
import json
import math
from pathlib import Path
import tempfile
import threading
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
import pytest
import yaml
from robocup_real_manipulation.core import Fault, grasp_geometry, support, EventJoin, rule_for
from robocup_real_manipulation.integration import (grasp_contact,hold_contact,lease_evidence,
    full_view_distance,scene_at_arrival,ProcessOwner,TerminalIDs)
from robocup_real_manipulation.motion import Motion
from robocup_real_manipulation.piper_bridge import PiperBridge
from moveit_msgs.msg import AllowedCollisionMatrix,AllowedCollisionEntry,PlanningScene

PKG=Path(__file__).resolve().parents[1]
CFG=yaml.safe_load((PKG/'config/robot.yaml').read_text())

def configured():
    c=copy.deepcopy(CFG);c.update(grasp_min_effort=.2,camera_min_depth_m=.17,coarse_z_error_bound_m=.03,
                                table_z_m=.5775,table_bounds_xy=[0.,2.,-1.,1.],obstacles=[])
    return c

def test_real_success_contact_width_and_negative_effort():
    records=json.loads((PKG/'test/fixtures/physical_success.json').read_text())['records']
    close=next(r for r in records if r['stage']=='CLOSE_GRIPPER' and r.get('event')=='GRIPPER_FEEDBACK')
    c=configured(); rule=c['classes']['green_apple']
    assert close['effort']<0 and abs(close['measured_width_m']-rule['close_m'])>c['gripper_tolerance_m']
    grasp_contact(close['measured_width_m'],close['effort'],rule,c)
    hold_contact(close['measured_width_m']+.002,close['effort'],close['measured_width_m'],c)
    with pytest.raises(Fault,match='HOLD_LOST'): hold_contact(close['measured_width_m']+.006,close['effort'],close['measured_width_m'],c)
    assert records[-1]['outcome']=='SUCCESS' and records[-1]['return_success']

def test_empty_or_still_open_and_nonfinite_grasp_rejected():
    c=configured();r=c['classes']['green_apple']
    for width,effort in [(0.,-.4),(.1,-.4),(.0806,0.),(.0806,float('nan'))]:
        with pytest.raises(Fault): grasp_contact(width,effort,r,c)

def test_lease_requires_post_request_evidence_not_docked_or_velocity_command():
    c=configured();lease=NS(task_id='A',locked=True,stopped=True,header=NS(stamp=NS(sec=10,nanosec=0)))
    lease_evidence(lease,'A',10.1,c,9.9)
    for task,now,requested in [('B',10.1,9.9),('A',10.1,10.05),('A',11.,9.9)]:
        with pytest.raises(Fault):lease_evidence(lease,task,now,c,requested)
    lease.stopped=False
    with pytest.raises(Fault):lease_evidence(lease,'A',10.1,c,9.9)

def test_actual_jetson_factory_fov_fits_error_bound():
    report=json.loads((PKG/'test/fixtures/jetson_intrinsics.json').read_text())
    info=NS(**report['intrinsics']);c=configured();r=c['classes']['green_apple']
    distance=full_view_distance(info,r,c)
    assert distance>c['observation_surface_clearance_m']+r['center_height_m']
    assert distance<c['observation_max_distance_m']
    c['observation_max_distance_m']=.3
    with pytest.raises(Fault,match='OUT_OF_RANGE'): full_view_distance(info,r,c)

def test_all_six_rules_use_observed_xy_and_table_relative_height_once():
    c=configured(); c['tcp_to_grip_center_m']=[0.,0.,0.]
    widths=[(.1,.079),(.1,.079),(.1,.082),(.08,.064),(.1,.09),(.04,.026)]
    for index in range(6):
        r=rule_for(c,index)
        p=np.array([.412,-.123,c['table_z_m']+r['height_m']])
        center,tcp,q=grasp_geometry(p,'SURFACE',r,c,.3 if index==5 else None)
        np.testing.assert_allclose(center[:2],p[:2]); np.testing.assert_allclose(tcp[:2],p[:2])
        assert tcp[2]==pytest.approx(c['table_z_m']+r['grip_height_m'])
        assert (r['open_m'],r['close_m'])==widths[index]

@pytest.mark.parametrize('class_id', range(6))
def test_real_coordinator_accepts_every_enabled_class_without_success_history(class_id):
    from robocup_real_manipulation.node import Coordinator
    c=configured()
    # Older measured configs may still carry the old green-apple-only gate.
    # Historical success/commissioning metadata must not discard valid targets.
    c['class_execution']={name: {'ready':False} for name in c['model_names']}
    warnings=[]
    holder=NS(c=c,now=lambda:100.,join=EventJoin(),mutex=threading.RLock(),
              input_errors={},get_logger=lambda:NS(warning=warnings.append))
    message=NS(task_id='class-'+str(class_id),class_id=class_id,
        header=NS(frame_id='odom',stamp=NS(sec=100,nanosec=0)),
        coarse_position=NS(x=.4,y=.1,z=.65))
    Coordinator.event(holder,'targets',message)
    assert holder.join.targets[message.task_id] is message
    assert not warnings and not holder.input_errors
    holder.c['classes'][c['model_names'][class_id]]['enabled']=False
    message.task_id+='-disabled'
    Coordinator.event(holder,'targets',message)
    assert message.task_id not in holder.join.targets
    assert holder.input_errors[message.task_id]=='UNSUPPORTED_CLASS'

def test_scene_rotates_at_arrival_and_support_excludes_bounding_box_corner():
    c=configured();c['table_bounds_xy']=[-1,1,-1,1]
    q=[0.,0.,math.sin(math.pi/8),math.cos(math.pi/8)]
    arrived=scene_at_arrival(c,[2.,0.,0.],q)
    assert support([2,0,1],.05,arrived)
    assert not support([3.3,1.3,1],.05,arrived)
    assert c['table_bounds_xy']==[-1,1,-1,1] and 'table_polygon_xy' not in c

def test_durable_exact_replay_guard_and_process_exclusion():
    with tempfile.TemporaryDirectory() as td:
        owner=ProcessOwner(td+'/owner')
        with pytest.raises(RuntimeError,match='DUPLICATE_PROCESS'): ProcessOwner(td+'/owner')
        ids=TerminalIDs(Path(td)/'tasks.sqlite3',['old']);ids.add('new')
        reloaded=TerminalIDs(Path(td)/'tasks.sqlite3')
        assert 'old' in reloaded and 'new' in reloaded and 'other' not in reloaded
        join=EventJoin();join.completed=reloaded
        with pytest.raises(Fault,match='DUPLICATE_TASK'):join.begin('new')
        owner.file.close()

def test_acm_retains_unrelated_pairs_and_revokes_temporary_table_contact():
    acm=AllowedCollisionMatrix(entry_names=['a','b'],entry_values=[AllowedCollisionEntry(enabled=[False,True]),AllowedCollisionEntry(enabled=[True,False])])
    holder=NS(acm=acm)
    def call(request):
        f=Future();f.set_result(NS(scene=NS(allowed_collision_matrix=holder.acm)));return f
    def apply(scene,guard):holder.acm=scene.allowed_collision_matrix
    node=NS(get_scene=NS(wait_for_service=lambda **k:True,call_async=call),apply=apply,allowed=set())
    Motion.contacts(node,['finger','manip_table'],lambda:None)
    Motion.contacts(node,['finger'],lambda:None)
    acm=holder.acm;names=list(acm.entry_names)
    assert acm.entry_values[names.index('a')].enabled[names.index('b')]
    assert not acm.entry_values[names.index('manip_target')].enabled[names.index('manip_table')]
    assert acm.entry_values[names.index('manip_target')].enabled[names.index('finger')]

def test_attachment_does_not_double_remove_world_object():
    scenes=[];c=configured()
    node=NS(c=c,box=lambda *args:NS(),apply=lambda scene,g:scenes.append(scene))
    # Use a real CollisionObject from the implementation helper.
    node.box=lambda *args:Motion.box(NS(c=c,pose=Motion.pose.__get__(NS())),*args)
    Motion.attach(node,[.3,0.,.6],c['classes']['green_apple'],lambda:None)
    assert not scenes[0].world.collision_objects
    assert scenes[0].robot_state.attached_collision_objects[0].object.id=='manip_target'

def test_hold_reuses_close_command_instead_of_measured_width():
    sent=[];node=NS(mutex=threading.RLock(),q=np.zeros(6),width=.0806,grip_command=.079,publish=lambda q,w,**k:sent.append(w))
    PiperBridge.hold(node);assert sent==[.079]

@pytest.mark.parametrize('opening,duplicate,expected',[(False,False,True),(True,False,False),(False,True,False)])
def test_gripper_contact_requires_fresh_distinct_samples_and_closing(opening,duplicate,expected):
    c=configured();sent=[];state={'iterations':0,'ok':False}
    node=NS(c=c,q=np.zeros(6),width=.08 if opening else .1,effort=-.358,grip_stamp=100.,grip_command=None,
        publish=lambda q,w:sent.append(w),confirm_stop=lambda:True)
    def guard(h,d):
        state['iterations']+=1
        if state['iterations']>20: raise Fault('MOTION_TIMEOUT')
        node.width=.0806
        if not duplicate:node.grip_stamp+=.1
    node.guard=guard
    handle=NS(request=NS(timeout_s=1.,width_m=.1 if opening else .079),is_cancel_requested=False,
        publish_feedback=lambda f:None,succeed=lambda:state.update(ok=True),abort=lambda:None)
    with patch('robocup_real_manipulation.piper_bridge.time.sleep',lambda _:None):
        result=PiperBridge.execute_gripper(node,handle)
    assert result.success==expected and state['ok']==expected
    assert node.grip_command==handle.request.width_m

def test_measured_mask_scene_includes_neighbor_and_excludes_target_and_table():
    import cv2
    from robocup_real_manipulation.wrist_geometry import mask_scene
    c=configured(); info=NS(width=160,height=120,k=[200.,0.,80.,0.,200.,60.,0.,0.,1.],d=[])
    depth=np.full((120,160),1.-c['table_z_m']);mask=np.zeros(depth.shape,np.uint8)
    cv2.circle(mask,(80,60),20,255,-1);depth[mask!=0]=1.-c['table_z_m']-.068
    R=np.diag([1.,-1.,-1.]);translation=np.array([.3,0.,1.]);center=[.3,0.,.6115]
    r=c['classes']['green_apple']
    assert mask_scene(depth,mask,info,R,translation,center,r,c)==[]
    depth[45:65,120:140]=.3
    obstacles=mask_scene(depth,mask,info,R,translation,center,r,c)
    assert obstacles and all(o['center'][2]>c['table_z_m'] for o in obstacles)
    c['scene_max_voxels']=1
    with pytest.raises(Fault,match='OCCUPANCY_LIMIT'):mask_scene(depth,mask,info,R,translation,center,r,c)

def test_cup_body_radius_does_not_certify_handle_extent():
    import cv2
    from robocup_real_manipulation.wrist_geometry import mask_scene
    c=configured();info=NS(width=160,height=120,k=[200.,0.,80.,0.,200.,60.,0.,0.,1.],d=[])
    mask=np.zeros((120,160),np.uint8);cv2.rectangle(mask,(50,45),(145,75),255,-1)
    depth=np.full(mask.shape,.35)
    with pytest.raises(Fault,match='MASK_EXCEEDS_COLLISION_EXTENT'):
        mask_scene(depth,mask,info,np.diag([1.,-1.,-1.]),np.array([.3,0.,1.]),[.3,0.,.6115],c['classes']['cup'],c)

def test_integration_rejects_stationary_scene_and_unreviewed_success_exceptions():
    from robocup_real_manipulation.integration import validate_integration
    c=configured();c['scene_frame']='odom';validate_integration(c)
    c['coarse_fixed_frame']='base_link'
    with pytest.raises(Fault,match='FIXED_FRAME'):validate_integration(c)
    c['coarse_fixed_frame']='odom';c['startup_prefix_allowed_contacts']=[['piper_link1','piper_link5',.008]]
    with pytest.raises(Fault,match='COLLISION_EXCEPTIONS'):validate_integration(c)
