"""Exercise the real MuJoCo/ROS status serializer without network or a viewer."""
import json
import threading
from types import SimpleNamespace
import unittest

try:
    import mujoco
    from rclpy.clock import Clock
    from run_mujoco_moveit_bridge import Bridge
    from manipulation.sim_model import load_model,config,jaw_opening
    AVAILABLE=True
except ImportError:
    AVAILABLE=False


@unittest.skipUnless(AVAILABLE,"Requires MuJoCo and sourced ROS Humble")
class StatusTest(unittest.TestCase):
    def test_ros_array_serializes_with_actual_model(self):
        model,data=load_model()
        published={}
        js=SimpleNamespace(publish=lambda message:published.update(joints=message))
        status=SimpleNamespace(publish=lambda message:published.update(status=message))
        bridge=SimpleNamespace(model=model,data=data,cfg=config(),lock=threading.RLock(),
                               stage="INIT",base_state="STOPPED",failure=None,
                               js=js,status_pub=status,get_clock=lambda:Clock(),max_width=jaw_opening(model,data),
                               gripper=model.actuator("gripper").id)
        bridge.contacts=lambda:Bridge.contacts(bridge)
        Bridge.publish(bridge)
        result=json.loads(published["status"].data)
        self.assertEqual(len(result["joint_positions"]),8)
        self.assertEqual(result["joint_positions"],list(published["joints"].position))
        self.assertEqual(len(result["T_world_tcp"]),4)
        self.assertAlmostEqual(result["max_jaw_opening_m"],0.070,places=5)


if __name__=="__main__":unittest.main()
