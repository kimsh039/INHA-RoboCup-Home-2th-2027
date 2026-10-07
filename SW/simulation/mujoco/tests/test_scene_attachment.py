"""ROS 노드를 시작하지 않고 장면 메시지 구성과 연결 유지 규약을 검증한다."""
import unittest
import numpy as np
from moveit_msgs.msg import PlanningScene
from run_grasp_moveit import Demo


class SceneAttachmentTest(unittest.TestCase):
    def test_wait_uses_simulation_time(self):
        node = object.__new__(Demo)
        node.status = {'simulation_time_s': 10.0}
        ticks = []
        def tick(seconds):
            ticks.append(seconds)
            node.status['simulation_time_s'] += .5
        node.tick = tick
        node.wait_simulation(3.0)
        self.assertEqual(len(ticks), 6)
        self.assertGreaterEqual(node.status['simulation_time_s'], 13.0)

    def test_attach_does_not_remove_world_twice(self):
        node = object.__new__(Demo)
        messages = []
        node.apply_scene = messages.append
        node.status = {'T_world_tcp': np.eye(4).tolist(), 'T_world_object': np.eye(4).tolist()}
        node.attach_planning_object()
        scene = messages[0]
        self.assertEqual(len(scene.world.collision_objects), 0)
        self.assertEqual(scene.robot_state.attached_collision_objects[0].object.id, 'cube')
        self.assertTrue(scene.robot_state.is_diff)
        self.assertIsNotNone(node.attached)

    def test_scene_diff_preserves_attached_objects(self):
        node = object.__new__(Demo)
        requests = []
        class Client:
            def call_async(self, request):
                requests.append(request)
                return None
        node.scene = Client()
        node.wait_future = lambda future: type('Response', (), {'success': True})()
        scene = PlanningScene()
        scene.is_diff = True
        node.apply_scene(scene)
        self.assertTrue(requests[0].scene.robot_state.is_diff)
