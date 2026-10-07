"""시간 초기화 뒤 이전 계획을 적용하지 않고 실패를 전파하는지 검사한다."""
import threading,unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from run_mujoco_moveit_bridge import Bridge
class ClockResetTest(unittest.TestCase):
 def bridge(self):
  pending={'t0':206.162,'event':threading.Event(),'error':None}
  b=SimpleNamespace(data=SimpleNamespace(time=.066,qpos=np.zeros(6)),jq=list(range(6)),
    arm_target=np.ones(6),last_simulation_time=230.,clock_discontinuity=None,
    failure=None,active=pending,lock=threading.RLock(),incidents=[],messages=[])
  b.get_logger=lambda:SimpleNamespace(error=b.messages.append)
  b.save_runtime_incident=lambda kind,details:b.incidents.append((kind,details))
  b.check_simulation_clock=lambda where:Bridge.check_simulation_clock(b,where)
  return b
 def test_rewind_aborts_before_physics_or_old_target_application(self):
  b=self.bridge()
  with patch('run_mujoco_moveit_bridge.mujoco.mj_step') as step:
   Bridge.step(b);step.assert_not_called()
  self.assertTrue(b.active['event'].is_set());self.assertIn('SIMULATION_TIME_RESET',b.active['error'])
  np.testing.assert_array_equal(b.arm_target,np.zeros(6))
  self.assertEqual(b.incidents[0][1]['location'],'before_physics_step')
  with patch('run_mujoco_moveit_bridge.mujoco.mj_step') as step:
   Bridge.step(b);step.assert_not_called()
  self.assertEqual(len(b.incidents),1)
 def test_normal_clock_advances_without_failure(self):
  b=self.bridge();b.data.time=230.002
  self.assertTrue(Bridge.check_simulation_clock(b,'inside_physics_step'))
  self.assertIsNone(b.failure);self.assertFalse(b.active['event'].is_set())
 def test_reset_inside_physics_step_has_distinct_location(self):
  b=self.bridge();Bridge.check_simulation_clock(b,'inside_physics_step')
  self.assertEqual(b.incidents[0][1]['location'],'inside_physics_step')
