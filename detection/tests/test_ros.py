"""Optional real ROS pub/sub smoke test with a synthetic detector, no weights.

Source ROS Humble and the built workspace before running this test.
"""
import time
import unittest

try:
    import numpy as np
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from robocup_detection_msgs.msg import HeadTarget
    from robocup_head_detection.core import Box, Config, HeadPipeline
    from robocup_head_detection.node import HeadDetectionNode
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "Requires ROS Humble, built messages and OpenCV")
class RosSmokeTests(unittest.TestCase):
    def test_waiting_model_tracking_masks_stamps_and_input_timeout(self):
        rclpy.init(args=["--ros-args", "-p", "input_timeout:=0.3", "-p", "max_input_age:=2.0"])
        head = peer = executor = None
        try:
            head, peer = HeadDetectionNode(), Node("head_test_peer")
            executor = SingleThreadedExecutor()
            executor.add_node(head)
            executor.add_node(peer)
            targets, masks = [], []
            peer.create_subscription(HeadTarget, "/detection/head/target", targets.append, 10)
            peer.create_subscription(Image, "/detection/head/target_mask", masks.append, 10)
            publisher = peer.create_publisher(Image, "/head_camera/color/image_raw", qos_profile_sensor_data)

            def until(predicate, timeout=3.0):
                deadline = time.monotonic()+timeout
                while not predicate() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                self.assertTrue(predicate(), "ROS event timed out")

            until(lambda: publisher.get_subscription_count() > 0)
            image = np.zeros((100, 200, 3), np.uint8)

            def send():
                header = peer.get_clock().now().to_msg()
                msg = Image()
                msg.header.stamp, msg.header.frame_id = header, "head_color_optical_frame"
                msg.height, msg.width, msg.step = 100, 200, 600
                msg.encoding, msg.data = "bgr8", image.tobytes()
                publisher.publish(msg)
                until(lambda: any(t.header.stamp == header and t.state != "WAITING_IMAGE" for t in targets))
                return next(t for t in reversed(targets) if t.header.stamp == header and t.state != "WAITING_IMAGE")

            self.assertEqual(send().state, "WAITING_MODEL")

            class Detector:
                ready = True

                def detect(self, frame, roi=None):
                    mask = np.zeros(frame.shape[:2], np.uint8)
                    mask[30:60, 50:90] = 255
                    return [Box(50, 30, 40, 30, 0, "cup", 0.8, mask)]

            class Tracker:
                def __init__(self, box):
                    self.box = box

                def update(self, frame):
                    return self.box

            head.pipeline = HeadPipeline(Detector(), lambda frame, box: Tracker(box),
                                         Config(confirm_frames=1, redetect_interval=2))
            measured = send()
            self.assertTrue(measured.valid and measured.measured and measured.mask_available)
            until(lambda: any(m.header.stamp == measured.header.stamp for m in masks))
            mask = next(m for m in masks if m.header.stamp == measured.header.stamp)
            self.assertEqual((mask.width, mask.height, mask.encoding), (200, 100, "mono8"))
            self.assertEqual(mask.header.frame_id, "head_color_optical_frame")
            tracked = send()
            self.assertTrue(tracked.valid)
            self.assertFalse(tracked.measured or tracked.mask_available)
            self.assertEqual(tracked.last_verified_stamp, measured.header.stamp)
            verified = send()
            self.assertTrue(verified.measured)
            self.assertEqual(verified.target_id, measured.target_id)
            until(lambda: targets[-1].state == "WAITING_IMAGE")
            self.assertFalse(targets[-1].valid)
        finally:
            if executor:
                executor.shutdown()
            for node in (head, peer):
                if node:
                    node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
