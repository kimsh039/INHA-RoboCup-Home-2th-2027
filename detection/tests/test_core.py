"""No ROS, model weights, GPU or third-party packages needed."""
import unittest
from dataclasses import replace

from robocup_head_detection.core import Box, Config, HeadPipeline, iou, roi_bounds


class Frame:
    shape = (100, 200, 3)


BOX = Box(50, 30, 40, 30, 0, "cup", 0.8, "fresh-mask")


class Detector:
    ready = True

    def __init__(self, batches):
        self.batches = iter(batches)
        self.rois = []

    def detect(self, frame, roi=None):
        self.rois.append(roi)
        value = next(self.batches)
        if isinstance(value, Exception):
            raise value
        return value


class Tracker:
    def __init__(self, box):
        self.box = box

    def update(self, frame):
        return self.box


class PipelineTests(unittest.TestCase):
    def pipeline(self, batches, **kwargs):
        detector = Detector(batches)
        pipeline = HeadPipeline(detector, lambda frame, box: Tracker(box), Config(**kwargs))
        return pipeline, detector

    def test_no_model_is_not_a_fake_detection(self):
        pipeline, detector = self.pipeline([])
        detector.ready = False
        result = pipeline.step(Frame(), 1.0)
        self.assertEqual(result.state, "WAITING_MODEL")
        self.assertFalse(result.valid)
        self.assertEqual(detector.rois, [])

    def test_confirmation_periodic_roi_and_no_stale_mask(self):
        corrected = replace(BOX, x=52, confidence=0.9)
        pipeline, detector = self.pipeline([[BOX], [BOX], [BOX], [corrected]],
                                           redetect_interval=2)
        first = pipeline.step(Frame(), 1.0)
        self.assertEqual(first.state, "CONFIRMING")
        self.assertFalse(first.valid)
        pipeline.step(Frame(), 1.03)
        acquired = pipeline.step(Frame(), 1.06)
        self.assertTrue(acquired.valid)
        self.assertTrue(acquired.measured)
        tracked = pipeline.step(Frame(), 1.09)
        self.assertFalse(tracked.measured)
        self.assertIsNone(tracked.box.mask)
        self.assertEqual(tracked.last_verified, 1.06)
        verified = pipeline.step(Frame(), 1.12)
        self.assertTrue(verified.measured)
        self.assertEqual(verified.target_id, acquired.target_id)
        self.assertEqual(verified.box.x, 52)
        self.assertEqual(detector.rois, [None, None, None, (30, 15, 110, 75)])

    def test_same_class_neighbour_does_not_win_by_confidence(self):
        neighbour = replace(BOX, x=130, confidence=0.99)
        same = replace(BOX, x=51, confidence=0.5)
        pipeline, _ = self.pipeline([[BOX], [neighbour, same]],
                                    confirm_frames=1, redetect_interval=1)
        pipeline.step(Frame(), 1.0)
        result = pipeline.step(Frame(), 1.03)
        self.assertEqual(result.box.x, 51)

    def test_wrong_class_and_roi_failures_trigger_full_search(self):
        wrong = replace(BOX, class_id=1, class_name="bottle")
        pipeline, detector = self.pipeline([[BOX], [wrong], [], [BOX]],
                                           confirm_frames=1, redetect_interval=1)
        acquired = pipeline.step(Frame(), 1.0)
        interim = pipeline.step(Frame(), 1.03)
        self.assertEqual(interim.reason, "ROI_UNCONFIRMED")
        self.assertFalse(interim.measured)
        lost = pipeline.step(Frame(), 1.06)
        self.assertFalse(lost.valid)
        self.assertEqual(lost.state, "LOST")
        reacquired = pipeline.step(Frame(), 1.09)
        self.assertNotEqual(acquired.target_id, reacquired.target_id)
        self.assertIsNone(detector.rois[-1])

    def test_target_class_filters_initial_candidates(self):
        bottle = replace(BOX, class_id=1, class_name="bottle", confidence=0.99)
        pipeline, _ = self.pipeline([[bottle, BOX]], confirm_frames=1, target_class="cup")
        self.assertEqual(pipeline.step(Frame(), 1.0).box.class_name, "cup")

    def test_verification_age_forces_early_check_and_invalidates_on_failure(self):
        pipeline, detector = self.pipeline([[BOX], []], confirm_frames=1,
                                           redetect_interval=100, max_verification_age=0.1)
        pipeline.step(Frame(), 1.0)
        result = pipeline.step(Frame(), 1.11)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "YOLO_VERIFICATION_FAILED")
        self.assertIsNotNone(detector.rois[-1])

    def test_tracker_failure_loses_without_waiting_for_verify_interval(self):
        pipeline, _ = self.pipeline([[BOX]], confirm_frames=1)
        pipeline.step(Frame(), 1.0)
        pipeline.tracker.box = None
        self.assertEqual(pipeline.step(Frame(), 1.03).reason, "TRACKER_FAILED")

    def test_model_runtime_error_invalidates_target(self):
        pipeline, _ = self.pipeline([[BOX], RuntimeError("CUDA unavailable")],
                                    confirm_frames=1, redetect_interval=1)
        pipeline.step(Frame(), 1.0)
        result = pipeline.step(Frame(), 1.03)
        self.assertFalse(result.valid)
        self.assertTrue(result.reason.startswith("PROCESSING_ERROR"))

    def test_non_monotonic_timestamp_and_large_gap(self):
        pipeline, detector = self.pipeline([[BOX], [BOX]], confirm_frames=1)
        pipeline.step(Frame(), 1.0)
        self.assertEqual(pipeline.step(Frame(), 0.9).reason, "NON_MONOTONIC_TIMESTAMP")
        pipeline.step(Frame(), 2.0)
        self.assertEqual(detector.rois, [None, None])
        pipeline, detector = self.pipeline([[BOX], [BOX]], confirm_frames=1)
        old = pipeline.step(Frame(), 1.0)
        new = pipeline.step(Frame(), 2.0)
        self.assertNotEqual(old.target_id, new.target_id)
        self.assertIsNone(detector.rois[-1])

    def test_resolution_change_restarts_search(self):
        pipeline, detector = self.pipeline([[BOX], [BOX]], confirm_frames=1)
        pipeline.step(Frame(), 1.0)
        changed = Frame()
        changed.shape = (200, 400, 3)
        pipeline.step(changed, 1.03)
        self.assertIsNone(detector.rois[-1])

    def test_invalid_boxes_and_edge_roi(self):
        self.assertIsNone(replace(BOX, width=-1).clipped(200, 100))
        self.assertIsNone(replace(BOX, x=float("nan")).clipped(200, 100))
        self.assertEqual(roi_bounds(replace(BOX, x=0, y=0), 200, 100, 2), (0, 0, 60, 45))
        self.assertAlmostEqual(iou(BOX, BOX), 1.0)

    def test_invalid_settings_fail_early(self):
        for values in ({"redetect_interval": 0}, {"roi_margin": 0.5},
                       {"match_iou": 0}, {"max_frame_gap": float("nan")}):
            with self.assertRaises(ValueError):
                Config(**values)


if __name__ == "__main__":
    unittest.main()
