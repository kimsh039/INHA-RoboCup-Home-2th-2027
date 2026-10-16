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


class ControlSensorContractTests(unittest.TestCase):
    def connect(self, service):
        from control_process import ControlSensors, Outbox
        from detection_approach import DetectionApproach
        from server import State
        state = State('live', {})
        state.values['service_control'] = {'navigation': service}
        outgoing = Outbox()
        state.sensor_manager = ControlSensors(state, outgoing, Mock())
        approach = SimpleNamespace(state=state, report=Mock())
        DetectionApproach.connect_navigation(approach)
        approach.report.assert_called_once()
        self.assertEqual(approach.report.call_args.args[0], 'waiting_nav')
        return outgoing.drain()

    def test_missing_navigation_requests_start_through_parent(self):
        self.assertEqual(self.connect({}), [('service', 'navigation', 'start')])

    def test_running_or_starting_navigation_is_not_restarted(self):
        for service in ({'managed': True}, {'publishers': 1}, {'busy': True}):
            with self.subTest(service=service):
                self.assertEqual(self.connect(service), [])

    def test_updated_service_snapshot_controls_availability(self):
        from control_process import ControlSensors, Outbox
        from server import State
        state = State('live', {})
        ready = Mock()
        sensors = ControlSensors(state, Outbox(), ready)
        self.assertFalse(sensors.navigation_available())
        state.values['service_control'] = {'navigation': {'managed': True}}
        self.assertTrue(sensors.navigation_available())
        state.values['service_control'] = {'navigation': {'managed': False, 'publishers': 0}}
        self.assertFalse(sensors.navigation_available())
        sensors.require_collision_ready()
        ready.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
