"""Offline geometry and consumer lifecycle checks; never send robot commands."""
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_msgs.msg import TFMessage

spec = importlib.util.spec_from_file_location('slam_scan', Path(__file__).resolve().parents[1] / 'slam_scan.py')
scan_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scan_module)
tf_spec = importlib.util.spec_from_file_location('slam_tf_clock',
    Path(__file__).resolve().parents[2] / 'control_center/slam-tf-clock.py')
tf_module = importlib.util.module_from_spec(tf_spec)
tf_spec.loader.exec_module(tf_module)


def transform(pose):
    x, y, angle = pose
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])


class SlamScanChecks(unittest.TestCase):
    def test_yaw_interpolation_crosses_pi_without_turning_through_zero(self):
        track = scan_module.Trajectory(3., .08)
        track.add(1_000_000_000, np.array([0., 0., np.deg2rad(179.)]))
        track.add(1_020_000_000, np.array([0., 0., np.deg2rad(-179.)]))
        self.assertAlmostEqual(track.at([1_010_000_000])[0, 2], np.pi)

    def test_pose_history_never_extrapolates(self):
        track = scan_module.Trajectory(3., .08)
        track.add(1_000_000_000, np.zeros(3))
        track.add(1_020_000_000, np.zeros(3))
        with self.assertRaises(scan_module.AwaitOdometry):
            track.at([1_030_000_000])
        with self.assertRaises(ValueError):
            track.at([990_000_000])

    def test_missing_odometry_interval_is_rejected(self):
        track = scan_module.Trajectory(3., .08)
        track.add(1_000_000_000, np.zeros(3))
        track.add(1_200_000_000, np.zeros(3))
        with self.assertRaises(ValueError):
            track.at([1_000_000_000, 1_200_000_000])

    def test_moving_scan_recovers_known_wall_with_sensor_lever_arm(self):
        origin = 1_000_000_000
        track = scan_module.Trajectory(3., .08)
        for dt in np.arange(0., .121, .02):
            track.add(origin + round(dt * 1e9), np.array([.3 * dt, -.1 * dt, .6 * dt]))
        times = np.arange(0., .101, .01)
        wall = np.c_[np.full(len(times), 5.), np.linspace(-2., 2., len(times)), np.zeros(len(times)), np.ones(len(times))]
        mount = transform([.18, -.09, .23])
        # Include the measured mounting tilt as well as planar offset/yaw.
        tilt = .02
        pitch = np.array([[np.cos(tilt), 0, np.sin(tilt)], [0, 1, 0], [-np.sin(tilt), 0, np.cos(tilt)]])
        mount[:3, :3] = mount[:3, :3] @ pitch
        distorted = np.array([(np.linalg.inv(transform([.3*t, -.1*t, .6*t]) @ mount) @ p)[:3]
                              for t, p in zip(times, wall)])
        reference = origin + 50_000_000
        node = SimpleNamespace(trajectory=track)
        corrected, _ = scan_module.SlamScan.common_time_points(
            node, distorted, origin + np.rint(times * 1e9).astype(np.int64), reference,
            mount[:3, :3], mount[:3, 3], True)
        expected = (np.linalg.inv(transform([.015, -.005, .03]) @ mount) @ wall.T).T[:, :3]
        np.testing.assert_allclose(corrected, expected, atol=1e-10)

    def test_empty_bins_remain_unknown_and_nearest_return_wins(self):
        scan = LaserScan()
        scan.angle_min, scan.angle_increment = -1., .5
        scan.range_min, scan.range_max = .35, 8.
        scan.ranges = [0.] * 5
        scan_module.SlamScan.bin_points(scan, np.array([[2., 0., 0.], [1., 0., 0.], [.1, 0., 0.]]))
        self.assertEqual(scan.ranges[2], 1.)
        self.assertTrue(np.isnan(scan.ranges[0]))
        self.assertFalse(any(r == 0 for r in scan.ranges))

    def test_stationary_mapping_can_attach_again_and_pure_rotation_is_kept(self):
        published, endpoints = [], []
        node = SimpleNamespace(settings=dict(minimum_valid_beams=1, output_topic='/scan_slam',
                                             minimum_time_interval=.2, minimum_travel_distance=.04,
                                             minimum_travel_heading=.02, stationary_refresh_interval=0.,
                                             subscriber_warmup_sec=0., slam_node_name='slam_toolbox', initial_slam_scans=3),
                               last_pose=None, last_stamp=None, last_frame=None, subscribers=set(),
                               subscriber_ready_at=0., awaiting_first_pose=False, first_slam_stamp=None,
                               initial_pending=set(), initial_confirmed=0,
                               counts=dict(published=0), reason='',
                               get_subscriptions_info_by_topic=lambda _: endpoints,
                               pub=SimpleNamespace(publish=lambda s: published.append(copy.deepcopy(s))))
        scan = LaserScan()
        scan.header.frame_id = 'laser_frame'
        scan.header.stamp.sec = 10
        scan.ranges = [1.]
        pose = np.zeros(3)
        select = scan_module.SlamScan.select
        self.assertFalse(select(node, scan, pose))  # no consumer: do not consume the first scan
        endpoints.append(SimpleNamespace(endpoint_gid=b'first'))
        self.assertTrue(select(node, scan, pose))
        scan.header.stamp.sec = 11
        self.assertFalse(select(node, scan, pose))
        self.assertTrue(select(node, scan, np.array([0., 0., .03])))
        endpoints[:] = [SimpleNamespace(endpoint_gid=b'restarted')]
        scan.header.stamp.sec = 12
        self.assertTrue(select(node, scan, np.array([0., 0., .03])))
        self.assertEqual(len(published), 3)

    def test_initial_constraints_require_distinct_slam_confirmations(self):
        sent = []
        endpoint = SimpleNamespace(endpoint_gid=b'slam', node_name='slam_toolbox')
        endpoints = [endpoint]
        node = SimpleNamespace(settings=dict(minimum_valid_beams=1, output_topic='/scan_slam',
                                             minimum_time_interval=.2, minimum_travel_distance=.04,
                                             minimum_travel_heading=.02, stationary_refresh_interval=0.,
                                             subscriber_warmup_sec=0., slam_node_name='slam_toolbox', initial_slam_scans=3),
                               last_pose=None, last_stamp=None, last_frame=None, subscribers=set(),
                               subscriber_ready_at=0., awaiting_first_pose=False, first_slam_stamp=None,
                               initial_pending=set(), initial_confirmed=0,
                               counts=dict(published=0), reason='',
                               get_subscriptions_info_by_topic=lambda _: endpoints,
                               pub=SimpleNamespace(publish=lambda s: sent.append(copy.deepcopy(s))))
        scan = LaserScan(); scan.header.frame_id = 'laser_frame'; scan.ranges = [1.]
        scan.header.stamp.sec = 10
        self.assertTrue(scan_module.SlamScan.select(node, scan, np.zeros(3)))
        self.assertTrue(node.awaiting_first_pose)
        scan.header.stamp.sec = 11
        self.assertTrue(scan_module.SlamScan.select(node, scan, np.zeros(3)))
        pose = PoseWithCovarianceStamped(); pose.header.stamp.sec = 11
        scan_module.SlamScan.on_slam_pose(node, pose)
        scan_module.SlamScan.on_slam_pose(node, pose)  # a duplicate acknowledgement does not count
        self.assertEqual(node.initial_confirmed, 1)
        scan.header.stamp.sec = 12
        self.assertTrue(scan_module.SlamScan.select(node, scan, np.zeros(3)))
        pose.header.stamp.sec = 12
        scan_module.SlamScan.on_slam_pose(node, pose)
        self.assertTrue(node.awaiting_first_pose)
        scan.header.stamp.sec = 13
        self.assertTrue(scan_module.SlamScan.select(node, scan, np.zeros(3)))
        pose.header.stamp.sec = 13
        scan_module.SlamScan.on_slam_pose(node, pose)
        self.assertFalse(node.awaiting_first_pose)
        self.assertEqual(node.initial_confirmed, 3)
        scan.header.stamp.sec = 14
        endpoints.append(SimpleNamespace(endpoint_gid=b'diagnostic', node_name='topic_echo'))
        self.assertFalse(scan_module.SlamScan.select(node, scan, np.zeros(3)))
        self.assertEqual(len(sent), 4)

    def test_tf_adapter_preserves_pose_and_never_forwards_odom_base(self):
        correction = TransformStamped(); correction.header.frame_id = 'map'; correction.child_frame_id = 'odom'
        correction.header.stamp.sec = 10; correction.transform.translation.x = 1.25
        correction.transform.rotation.w = 1.
        body = TransformStamped(); body.header.frame_id = 'odom'; body.child_frame_id = 'base_link'
        body.transform.rotation.w = 1.
        original = TFMessage(transforms=[correction, body])
        stamp = copy.deepcopy(correction.header.stamp); stamp.sec = 20
        result = tf_module.map_transforms(original, 'map', 'odom', stamp)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].transform, correction.transform)
        self.assertEqual(result[0].header.stamp.sec, 20)
        self.assertEqual(correction.header.stamp.sec, 10)

    def test_initial_pose_reopens_stationary_gate_only_for_localization(self):
        endpoint = SimpleNamespace(endpoint_gid=b'slam', node_name='slam_toolbox')
        initial_endpoints = []
        sent = []
        node = SimpleNamespace(
            settings=dict(map_frame='map', initial_pose_topic='/initialpose', output_topic='/scan_slam',
                          slam_node_name='slam_toolbox', subscriber_warmup_sec=0.,
                          minimum_valid_beams=1, minimum_time_interval=.2, minimum_travel_distance=.04,
                          minimum_travel_heading=.02, stationary_refresh_interval=0., initial_slam_scans=3),
            pending={1: object()}, last_pose=np.zeros(3), last_stamp=10_000_000_000,
            last_frame='laser_frame', subscribers={b'slam'}, subscriber_ready_at=0.,
            awaiting_first_pose=False, first_slam_stamp=None, initial_pending=set(), initial_confirmed=3,
            last_initial_pose_stamp=None, counts=dict(published=0), reason='',
            get_subscriptions_info_by_topic=lambda topic: [endpoint] if topic == '/scan_slam' else initial_endpoints,
            get_logger=lambda: SimpleNamespace(info=lambda _: None),
            pub=SimpleNamespace(publish=lambda s: sent.append(copy.deepcopy(s))))
        request = PoseWithCovarianceStamped()
        request.header.frame_id = 'map'; request.header.stamp.sec = 10
        request.pose.pose.orientation.w = 1.
        scan_module.SlamScan.on_initial_pose(node, request)
        self.assertFalse(node.awaiting_first_pose)  # mapping has no initialpose backend consumer
        initial_endpoints.append(endpoint)
        scan_module.SlamScan.on_initial_pose(node, request)
        self.assertTrue(node.awaiting_first_pose)
        self.assertEqual(node.pending, {})
        self.assertEqual(node.initial_confirmed, 0)
        scan = LaserScan(); scan.header.frame_id = 'laser_frame'; scan.ranges = [1.]
        for second in (11, 12, 13):
            scan.header.stamp.sec = second
            self.assertTrue(scan_module.SlamScan.select(node, scan, np.zeros(3)))
            ack = PoseWithCovarianceStamped(); ack.header.stamp.sec = second
            scan_module.SlamScan.on_slam_pose(node, ack)
        self.assertFalse(node.awaiting_first_pose)
        scan.header.stamp.sec = 14
        self.assertFalse(scan_module.SlamScan.select(node, scan, np.zeros(3)))
        self.assertEqual(len(sent), 3)


if __name__ == '__main__':
    unittest.main()
