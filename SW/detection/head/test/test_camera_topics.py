import unittest
from camera_topics import PAIRS, input_status, select_topics


class TopicTests(unittest.TestCase):
    def test_named_head_pair_selected(self):
        self.assertEqual(select_topics(set(PAIRS[1])), PAIRS[1])

    def test_never_automatically_uses_wrist_or_generic_camera(self):
        topics = {'/wrist/wrist/color/image_raw', '/wrist/wrist/aligned_depth_to_color/image_raw',
                  '/camera/camera/color/image_raw', '/camera/camera/aligned_depth_to_color/image_raw'}
        self.assertEqual(select_topics(topics), PAIRS[0])

    def test_rgb_only_does_not_get_paired_with_other_camera_depth(self):
        self.assertEqual(select_topics({PAIRS[1][0], PAIRS[2][1]}), PAIRS[0])

    def test_explicit_pair_preserved_and_half_pair_rejected(self):
        self.assertEqual(select_topics(set(), '/custom/rgb', '/custom/depth'), ('/custom/rgb', '/custom/depth'))
        with self.assertRaises(ValueError):
            select_topics(set(), '/custom/rgb')

    def test_absent_publisher_waiting_frame_and_missing_depth_distinguished(self):
        self.assertTrue(input_status(0, 0, 0).startswith('NO_RGB_PUBLISHER'))
        self.assertTrue(input_status(1, 1, 0).startswith('WAITING_RGB_FRAMES'))
        self.assertTrue(input_status(1, 0, 10).startswith('NO_DEPTH_PUBLISHER'))
        self.assertEqual(input_status(1, 1, 10), '')
