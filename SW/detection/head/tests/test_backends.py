"""CPU adapter checks with synthetic frames; no weights or ROS required."""
import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from robocup_head_detection.backends import YoloDetector, make_tracker_factory
from robocup_head_detection.core import Box
from robocup_head_detection.class_schema import CLASSES, LEGACY_CLASSES, class_mapping
from robocup_head_detection.images import decode_raw, decode_compressed


class AdapterTests(unittest.TestCase):
    def test_legacy_head_outputs_use_six_current_ids_and_exclude_plate(self):
        items = [SimpleNamespace(xyxy=np.array([[i, 3, i+12, 13]]), cls=[i], conf=[0.8])
                 for i in range(7)]
        result = SimpleNamespace(boxes=items, names=dict(enumerate(LEGACY_CLASSES)), masks=None)
        detector = YoloDetector("")
        detector.model = SimpleNamespace(predict=lambda *args, **kwargs: [result])
        boxes = detector.detect(np.zeros((100, 200, 3), np.uint8))
        self.assertEqual([(b.class_id, b.class_name) for b in boxes],
                         [(2, "tomato"), (5, "banana"), (3, "fanta_can"),
                          (0, "green_apple"), (4, "cup"), (1, "peach")])
        self.assertEqual([b.x for b in boxes], list(range(6)))

    def test_current_schema_and_unrelated_models_preserve_identity(self):
        self.assertEqual(class_mapping(dict(enumerate(CLASSES))),
                         {i: (i, name) for i, name in enumerate(CLASSES)})
        self.assertEqual(class_mapping({0: "apple", 1: "cup", 2: "plate"}),
                         {0: (0, "apple"), 1: (1, "cup"), 2: (2, "plate")})

    def test_missing_model_does_not_import_yolo_or_download(self):
        self.assertFalse(YoloDetector("").ready)
        self.assertFalse(YoloDetector("/does/not/exist/model.pt").ready)

    def test_roi_boxes_and_masks_return_to_original_grid(self):
        item = SimpleNamespace(xyxy=np.array([[2, 3, 12, 13]]), cls=[0], conf=[0.8])
        result = SimpleNamespace(boxes=[item], names={0: "cup"}, masks=SimpleNamespace(
            xy=[np.array([[2, 3], [12, 3], [12, 13], [2, 13]])]))
        detector = YoloDetector("")
        detector.model = SimpleNamespace(predict=lambda *args, **kwargs: [result])
        frame = np.zeros((100, 200, 3), np.uint8)
        box = detector.detect(frame, (50, 20, 90, 60))[0]
        self.assertEqual((box.x, box.y, box.width, box.height), (52, 23, 10, 10))
        self.assertEqual(box.mask.shape, (100, 200))
        self.assertEqual(box.mask[25, 55], 255)
        self.assertEqual(box.mask[5, 5], 0)

    def test_padded_rgb_and_mono_conversion(self):
        msg = SimpleNamespace(width=1, height=2, step=4, encoding="rgb8",
                              data=bytes([255, 0, 0, 99, 0, 255, 0, 99]))
        image = decode_raw(msg)
        self.assertEqual(image[0, 0].tolist(), [0, 0, 255])
        self.assertEqual(image[1, 0].tolist(), [0, 255, 0])
        mono = SimpleNamespace(width=1, height=1, step=1, encoding="mono8", data=b"\x40")
        self.assertEqual(decode_raw(mono)[0, 0].tolist(), [64, 64, 64])

    def test_malformed_images_are_rejected(self):
        msg = SimpleNamespace(width=2, height=1, step=6, encoding="bgr8", data=b"bad")
        with self.assertRaises(ValueError):
            decode_raw(msg)
        with self.assertRaises(ValueError):
            decode_compressed(SimpleNamespace(data=b"not jpeg"))

    def test_csrt_and_kcf_initialize_and_update(self):
        rng = np.random.default_rng(123)
        frame = np.zeros((160, 200, 3), np.uint8)
        frame[40:90, 60:120] = rng.integers(0, 255, (50, 60, 3), dtype=np.uint8)
        box = Box(60, 40, 60, 50, 0, "cup", 0.8)
        for kind in ("CSRT", "KCF"):
            tracker = make_tracker_factory(kind)(frame, box)
            result = tracker.update(frame.copy())
            self.assertIsNotNone(result, kind)
            self.assertLess(abs(result.x-box.x), 5, kind)


if __name__ == "__main__":
    unittest.main()
