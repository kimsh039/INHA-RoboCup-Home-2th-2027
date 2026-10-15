#!/usr/bin/env python3
"""Real ROS actions/topics, synthetic robot backend. Run with isolated domain."""
import os
import time
import tempfile
from pathlib import Path
import yaml
import rclpy
from rclpy.executors import MultiThreadedExecutor
from robocup_real_manipulation.mock_node import MockCoordinator
from robocup_real_manipulation.test_sender import Sender

assert os.environ.get('ROS_DOMAIN_ID')=='187','Use ROS_DOMAIN_ID=187 for test traffic, never the robot domain'
base=Path(__file__).parent/'manipulation_ws/src/robocup_real_manipulation/config/test_fixture.yaml'
cases=[('target-first',{},'SUCCESS','COMPLETE'),
       ('approach-first',{'approach_first':'true'},'SUCCESS','COMPLETE'),
       ('right-blocked',{'right_blocked':'true'},'SUCCESS','COMPLETE'),
       ('approach-failed',{'approach_outcome':'FAILED'},'FAILED','WAIT_FOR_APPROACH'),
       ('base-moving',{'base_stopped':'false'},'FAILED','WAIT_FOR_APPROACH'),
       ('wrist-timeout',{'scenario':'wrist_timeout'},'FAILED','WRIST_REDETECT'),
       ('stale-wrist',{'scenario':'stale_observation'},'FAILED','WRIST_REDETECT'),
       ('invalid-depth',{'scenario':'invalid_depth'},'FAILED','WRIST_REDETECT'),
       ('missing-tf',{'scenario':'missing_tf'},'FAILED','WRIST_REDETECT'),
       ('ambiguous',{'scenario':'ambiguous'},'FAILED','WRIST_REDETECT'),
       ('cancel',{'scenario':'cancel'},'CANCELED',None),
       ('unlock-missing',{'scenario':'unlock_missing'},'FAILED','COMPLETE'),
       ('hold-success',{},'SUCCESS','COMPLETE'),
       ('hold-unlock-missing',{'scenario':'unlock_missing'},'SUCCESS','COMPLETE'),
       ('return-failed',{'fail_stage':'RETURN_ARM'},'FAILED','RETURN_ARM')]
for name,params,outcome,stage in cases:
    with tempfile.TemporaryDirectory(prefix='real_manip_contract_') as td:
        cfg=yaml.safe_load(base.read_text()); cfg['log_path']=td+'/events.jsonl';cfg['journal_path']=td+'/completed.json'
        if name.startswith('hold-'):cfg['base_completion_policy']='hold'
        path=Path(td)/'test.yaml';path.write_text(yaml.safe_dump(cfg))
        args=['--ros-args','-p','config:='+str(path)]
        for k,v in params.items():args+=['-p',k+':='+v]
        rclpy.init(args=args);robot=MockCoordinator();sender=Sender();ex=MultiThreadedExecutor(num_threads=8);ex.add_node(robot);ex.add_node(sender)
        deadline=time.monotonic()+12
        try:
            while sender.result is None and time.monotonic()<deadline:ex.spin_once(timeout_sec=.05)
            assert sender.result is not None,(name,'result timeout')
            s=sender.result;assert s.outcome==outcome,(name,s.outcome,s.reason)
            if stage:assert s.stage==stage,(name,s.stage)
            if name=='right-blocked': assert robot.place_tcp[1]>0,'Must fall back to left'
            if name=='target-first': assert robot.place_tcp[1]<0,'Must prefer right'
            if name=='return-failed':assert s.place_success and not s.return_success and s.holding=='EMPTY'
            if name.startswith('hold-'):
                assert s.reason=='BASE_LOCK_RETAINED'
                assert robot.locked_task==s.task_id and robot.lock_requested
                assert __import__('json').loads(Path(cfg['journal_path']).read_text())['locked_task']==s.task_id
            # No robot command topic may be created by this mock.
            assert '/joint_ctrl_single' not in [n for n,t in robot.get_topic_names_and_types()]
            print('PASS',name,s.stage,s.outcome,s.reason,flush=True)
        finally:
            # Drain action feedback/result callbacks before destroying their entities.
            settle=time.monotonic()+.15
            while time.monotonic()<settle: ex.spin_once(timeout_sec=.01)
            ex.remove_node(robot);ex.remove_node(sender);ex.shutdown(timeout_sec=3)
            # Installed Humble MultiThreadedExecutor does not join queued pool jobs.
            # Finish them before destroying DDS handles.
            pool=getattr(ex,'_executor',None)
            if pool is not None: pool.shutdown(wait=True)
            robot.destroy_node();sender.destroy_node();rclpy.shutdown()
