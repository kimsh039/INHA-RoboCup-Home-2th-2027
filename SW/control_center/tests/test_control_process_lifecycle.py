"""Exercise control IPC lifetime with a fake ROS executor; no ROS node starts."""
from collections import deque
import multiprocessing
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control_process import run_controller


class ControlProcessLifecycleTests(unittest.TestCase):
    def run_offline(self, disconnect=False):
        created = threading.Event()
        stopped = threading.Event()
        executed = threading.Event()

        class OfflineController:
            def __init__(self, state, config):
                self.state = state
                self.nav_requests = deque()
                self.nav_status = dict(state='idle')
                self.front_retreat = SimpleNamespace(cancel=Mock())
                self.detection_approach = SimpleNamespace(requests=deque(), active=False, cancel=Mock())
                created.set()

            def cancel_goal(self):
                self.nav_status['state'] = 'idle'

            def stop_control(self):
                stopped.set()

            def destroy_node(self):
                pass

        def spin_once(node, timeout_sec):
            if node.detection_approach.requests:
                node.detection_approach.requests.clear()
                node.detection_approach.active = True
                node.state.put('detection_control', dict(active=True, phase='approaching'))
                executed.set()
            time.sleep(.005)

        fake_ros = ModuleType('rclpy')
        fake_ros.init = Mock(); fake_ros.shutdown = Mock(); fake_ros.ok = lambda: True
        fake_ros.spin_once = spin_once
        fake_signals = ModuleType('rclpy.signals')
        fake_signals.SignalHandlerOptions = SimpleNamespace(NO=0)
        fake_navigation = ModuleType('navigation_controller')
        fake_navigation.NavigationController = OfflineController
        connection, remote = multiprocessing.Pipe()
        thread = threading.Thread(target=run_controller, args=(remote, {}, False), daemon=True)
        with patch.dict(sys.modules, {'rclpy': fake_ros, 'rclpy.signals': fake_signals,
                                      'navigation_controller': fake_navigation}):
            try:
                thread.start()
                self.assertTrue(created.wait(1.))
                connection.send(('heartbeat', time.monotonic(), {}, False))
                connection.send(('detection', dict(action='find_and_approach', label='cup'), time.monotonic()))
                self.assertTrue(executed.wait(1.))
                if disconnect:
                    connection.close()
                else:
                    connection.send(('shutdown',))
                thread.join(timeout=2.)
                self.assertFalse(thread.is_alive())
                self.assertTrue(stopped.is_set())
                fake_ros.shutdown.assert_called_once()
            finally:
                connection.close()
                thread.join(timeout=2.)

    def test_ordered_shutdown_stops_active_control(self):
        self.run_offline()

    def test_pipe_loss_stops_active_control(self):
        self.run_offline(disconnect=True)


if __name__ == '__main__':
    unittest.main()
