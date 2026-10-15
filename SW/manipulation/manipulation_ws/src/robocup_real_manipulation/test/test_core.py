from pathlib import Path
import copy
import unittest
import numpy as np
import yaml
from robocup_real_manipulation.core import (Fault,EventJoin,validate,rotation,transform_point,
    robust_depth,associate,grasp_geometry,rule_for,width_to_driver,fresh,support,STAGES,observation_geometry)
from robocup_real_manipulation.sequence import Sequence
from robocup_real_manipulation.tuning import overlay

CFG=yaml.safe_load((Path(__file__).parents[1]/'config/robot.yaml').read_text())

class CoreTests(unittest.TestCase):
    def test_nominal_width_trial_preserves_other_interlocks(self):
        c=copy.deepcopy(CFG); c['width_mapping_mode']='nominal_trial'
        with self.assertRaisesRegex(Fault,'NOT_VERIFIED:execution_enabled'):
            validate(c,True)
        for key in ('execution_enabled','commissioned','calibration_verified','scene_verified',
                    'model_names_verified','gravity_frame_verified'):
            c[key]=True
        with self.assertRaisesRegex(Fault,'WIDTH_TRIAL_GREEN_APPLE_ONLY'):
            validate(c,True)
        for name,r in c['classes'].items(): r['enabled']=name=='green_apple'
        with self.assertRaisesRegex(Fault,'UNSET:joint_signs'):
            validate(c,True)
        c['velocity_scale']=.1
        with self.assertRaisesRegex(Fault,'WIDTH_TRIAL_SPEED'):
            validate(c,True)
        c['velocity_scale']=.05; c['classes']['green_apple']['close_m']=.06
        with self.assertRaisesRegex(Fault,'WIDTH_TRIAL_APERTURE'):
            validate(c,True)
    def test_both_arrival_orders_and_missing_half(self):
        for order in ('targets','approaches'),('approaches','targets'):
            j=EventJoin();j.put(order[0],'a',object());j.begin('a');self.assertFalse(j.ready())
            j.put(order[1],'a',object());self.assertTrue(j.ready());j.finish()
            with self.assertRaises(Fault):j.begin('a')
            with self.assertRaises(Fault):j.put('targets','a',object())
    def test_duplicate_and_busy(self):
        j=EventJoin();j.begin('a');j.put('targets','a',1)
        for f in (lambda:j.put('targets','a',2),lambda:j.begin('b'),lambda:j.put('targets','b',1)):
            with self.assertRaises(Fault):f()
    def test_persistent_replay_and_bounded_cache(self):
        j=EventJoin(['completed'])
        with self.assertRaises(Fault):j.begin('completed')
        for i in range(64):j.put('targets',str(i),1)
        with self.assertRaises(Fault):j.put('targets','overflow',1)
    def test_defaults_are_buildable_but_cannot_drive(self):
        validate(CFG)
        with self.assertRaisesRegex(Fault,'NOT_VERIFIED'):validate(CFG,True)
        c=copy.deepcopy(CFG);c['fixture_only']=True
        with self.assertRaisesRegex(Fault,'TEST_FIXTURE'):validate(c,True)
    def test_class_map_and_widths(self):
        self.assertEqual(rule_for(CFG,0)['close_m'],.079)
        self.assertEqual(rule_for(CFG,2)['close_m'],.082)
        self.assertEqual(rule_for(CFG,4)['close_m'],.090)
        for i in (5,6,99):
            with self.assertRaises(Fault):rule_for(CFG,i)
        c=copy.deepcopy(CFG);c.update(width_scale=1.25,width_bias_m=0.,driver_width_max=.08)
        self.assertAlmostEqual(width_to_driver(.1,c),.08)
        c['width_scale']=1
        with self.assertRaises(Fault):width_to_driver(.1,c)
        for v in (float('nan'),-.01,.101):
            with self.assertRaises(Fault):width_to_driver(v,c)
    def test_published_detector_adapter_matches_manipulation_schema(self):
        import importlib.util
        path=Path(__file__).resolve().parents[4]/'references/head_class_schema.source.py'
        spec=importlib.util.spec_from_file_location('head_class_schema',path)
        schema=importlib.util.module_from_spec(spec);spec.loader.exec_module(schema)
        self.assertEqual(tuple(CFG['model_names']),schema.CLASSES)
        mapped=schema.class_mapping(dict(enumerate(schema.LEGACY_CLASSES)))
        self.assertEqual(mapped[0],(2,'tomato'))
        self.assertIsNone(mapped[6])
        for identity in mapped.values():
            if identity is None: continue
            canonical_id,name=identity
            self.assertEqual(CFG['classes'][name]['class_id'],canonical_id)
            if CFG['classes'][name]['enabled']:
                self.assertEqual(rule_for(CFG,canonical_id),CFG['classes'][name])
        self.assertEqual(schema.class_mapping(dict(enumerate(schema.CLASSES)))[0],(0,'green_apple'))
    def test_tuning_does_not_modify_calibration_or_source(self):
        tuned=overlay(CFG,{'observation_surface_clearance_m':.23,'classes':{'green_apple':{'close_m':.078}}})
        self.assertEqual(tuned['classes']['green_apple']['close_m'],.078)
        self.assertEqual(CFG['classes']['green_apple']['close_m'],.079)
        validate(tuned)
        with self.assertRaisesRegex(Fault,'NON_HEURISTIC'):overlay(CFG,{'handeye_transform':{}})
        with self.assertRaises(Fault):overlay(CFG,{'classes':{'green_apple':{'class_id':9}}})
        with self.assertRaises(Fault):validate(overlay(CFG,{'classes':{'green_apple':{'open_m':.101}}}))
    def test_class_name_mismatch(self):
        c=copy.deepcopy(CFG);c['model_names'][3]='tomato'
        with self.assertRaisesRegex(Fault,'CLASS_NAME'):validate(c)
    def test_time_validation(self):
        fresh(10,10.2,.3)
        for stamp in (9.,11.,0.,float('nan')):
            with self.assertRaises(Fault):fresh(stamp,10,.3)
    def test_rigid_transform_and_quaternion(self):
        p=transform_point([1,0,0],[1,2,3],[0,0,np.sqrt(.5),np.sqrt(.5)])
        np.testing.assert_allclose(p,[1,3,3],atol=1e-12)
        with self.assertRaises(Fault):rotation([0,0,0,0])
    def test_depth_ignores_single_bad_pixel_and_background(self):
        d=np.full((50,50),1.2);mask=np.zeros((50,50),bool);mask[10:40,10:40]=1;d[10:40,10:40]=.5;d[25,25]=0
        p,std=robust_depth(d,mask,[100,0,25,0,100,25,0,0,1],CFG)
        self.assertAlmostEqual(p[2],.5);self.assertLess(std,.002)
        d[10:40,10:40]=0
        with self.assertRaisesRegex(Fault,'INVALID_DEPTH'):robust_depth(d,mask,[100,0,25,0,100,25,0,0,1],CFG)
    def test_unstable_depth_and_empty_mask(self):
        d=np.indices((40,40))[0]*.003+.2;mask=np.ones((40,40))
        c=copy.deepcopy(CFG);c['depth_max_mad_m']=.002
        with self.assertRaisesRegex(Fault,'UNSTABLE_DEPTH'):robust_depth(d,mask,[100,0,20,0,100,20,0,0,1],c)
        with self.assertRaises(Fault):robust_depth(d,np.zeros((40,40)),[100,0,20,0,100,20,0,0,1],c)
    def test_same_class_multiple_instances(self):
        candidates=[(np.array([.04,0,0]),'b','B'),(np.array([.01,0,0]),'a','A')]
        self.assertEqual(associate(candidates,np.zeros(3),'a',CFG)[0],'A')
        self.assertEqual(associate(candidates,np.zeros(3),'',CFG)[0],'A')
        candidates[0]=(np.array([.015,0,0]),'b','B')
        with self.assertRaisesRegex(Fault,'AMBIGUOUS'):associate(candidates,np.zeros(3),'',CFG)
        with self.assertRaises(Fault):associate(candidates,np.zeros(3),'unknown',CFG)
    def test_observation_follows_coarse_and_measured_camera_offset(self):
        c=copy.deepcopy(CFG);c.update(topdown_quaternion=[1,0,0,0],observation_camera_distance_m=.25,
            handeye_transform={'xyz':[.03,0,0],'quaternion_xyzw':[0,0,0,1]},
            link6_tcp_transform={'xyz':[0,0,.1],'quaternion_xyzw':[0,0,0,1]})
        p,q=observation_geometry([.4,.1,.74],c)
        np.testing.assert_allclose(p,[.37,.1,.89])
        p2,_=observation_geometry([.6,-.1,.74],c)
        np.testing.assert_allclose(p2-p,[.2,-.2,0])
        c['handeye_transform']['quaternion_xyzw']=[0,np.sqrt(.5),0,np.sqrt(.5)]
        p,q=observation_geometry([.4,0,.74],c)
        Ropt=rotation(q)@rotation(c['link6_tcp_transform']['quaternion_xyzw']).T@rotation(c['handeye_transform']['quaternion_xyzw'])
        np.testing.assert_allclose(Ropt[:,2],[0,0,-1],atol=1e-12)
        c['observation_camera_quaternion']=[0,0,0,1]
        with self.assertRaisesRegex(Fault,'CAMERA_NOT_TOPDOWN'):observation_geometry([.4,0,.74],c)
    def test_center_surface_tcp_and_support(self):
        c=copy.deepcopy(CFG);c.update(table_z_m=.7,table_bounds_xy=[.1,.9,-.4,.4],topdown_quaternion=[1,0,0,0],tcp_to_grip_center_m=[0,0,.02])
        r=copy.deepcopy(rule_for(c,0));r.update(center_height_m=.04,grip_height_m=.035)
        center,tcp,q=grasp_geometry([.4,0,.78],'SURFACE',r,c)
        np.testing.assert_allclose(center,[.4,0,.74]);self.assertAlmostEqual(tcp[2],.755)
        self.assertTrue(support(center,.04,c));self.assertFalse(support([.1,0,.74],.04,c))
        r.update(axis_required=True,closing_axis_yaw_offset_rad=0)
        with self.assertRaisesRegex(Fault,'BANANA_AXIS_REQUIRED'):grasp_geometry(center,'CENTER',r,c)
        grasp_geometry(center,'CENTER',r,c,0.)

class FakeBackend:
    def __init__(self,fail=None,cancel=None):self.fail=fail;self.cancel=cancel;self.performed=[];self.statuses=[];self.stops=0;self.stage=''
    def status(self,seq,outcome,reason):self.stage=seq.stage;self.statuses.append((seq.stage,outcome,seq.holding,seq.place_success,seq.return_success))
    def guard(self):
        if self.stage==self.cancel:raise Fault('CANCELED')
    def perform(self,stage,seq):
        if stage==self.fail:raise Fault('INJECTED_FAILURE')
        self.performed.append(stage)
    def stop(self):self.stops+=1;return True

class SequenceTests(unittest.TestCase):
    def test_success_requires_return_completion(self):
        b=FakeBackend();s=Sequence(b);self.assertEqual(s.run()[0],'SUCCESS');self.assertEqual(b.performed,list(STAGES));self.assertTrue(s.return_success)
    def test_every_motion_failure_and_cancel_stops_before_next_stage(self):
        for stage in STAGES:
            for cancel in (False,True):
                b=FakeBackend(fail=None if cancel else stage,cancel=stage if cancel else None);s=Sequence(b)
                self.assertEqual(s.run()[0],'CANCELED' if cancel else 'FAILED');self.assertEqual(b.performed,list(STAGES[:STAGES.index(stage)]));self.assertEqual(b.stops,1)
    def test_place_success_return_failure_reported_separately(self):
        s=Sequence(FakeBackend(fail='RETURN_ARM'));self.assertEqual(s.run()[0],'FAILED');self.assertTrue(s.place_success);self.assertFalse(s.return_success);self.assertEqual(s.holding,'EMPTY')
    def test_uncertain_close_and_held_place_failure(self):
        s=Sequence(FakeBackend(fail='CLOSE_GRIPPER'));s.run();self.assertEqual(s.holding,'UNKNOWN')
        s=Sequence(FakeBackend(fail='MOVE_TO_PLACE'));s.run();self.assertEqual(s.holding,'HELD')

if __name__=='__main__':unittest.main()
