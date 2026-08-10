import importlib.util
import sys
import types
import unittest
from pathlib import Path


CAR_ROOT = Path(__file__).resolve().parents[1]
VISION_MODEL_PATH = CAR_ROOT / "主摄像头" / "vision_model.py"


def load_vision_model():
    tf_stub = types.ModuleType("tf")
    tf_stub.load = lambda path: object()
    image_stub = types.ModuleType("image")

    old_tf = sys.modules.get("tf")
    old_image = sys.modules.get("image")
    sys.modules["tf"] = tf_stub
    sys.modules["image"] = image_stub
    try:
        spec = importlib.util.spec_from_file_location(
            "main_camera_same_class_priority", VISION_MODEL_PATH
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        if old_tf is None:
            sys.modules.pop("tf", None)
        else:
            sys.modules["tf"] = old_tf
        if old_image is None:
            sys.modules.pop("image", None)
        else:
            sys.modules["image"] = old_image

    return module


vision_model = load_vision_model()


def candidate(name, cx, cy, score, size=12):
    return {
        "name": name,
        "score": score,
        "x": int(cx - size / 2),
        "y": int(cy - size / 2),
        "w": size,
        "h": size,
        "cx": float(cx),
        "cy": float(cy),
    }


class SameClassPriorityTests(unittest.TestCase):
    def test_detection_keeps_all_same_class_candidates(self):
        class InferenceImage:
            def draw_image(self, *args, **kwargs):
                pass

        class SourceImage:
            def width(self):
                return 160

            def height(self):
                return 120

            def format(self):
                return 0

            def copy(self, **kwargs):
                return type("SmallImage", (), {"height": lambda self: 84})()

        def detection(x1, y1, x2, y2, score):
            scale = vision_model.MODEL_SIZE / 160.0
            offset_y = (vision_model.MODEL_SIZE - 84) // 2
            return [
                x1 / 160.0,
                (y1 * scale + offset_y) / vision_model.MODEL_SIZE,
                x2 / 160.0,
                (y2 * scale + offset_y) / vision_model.MODEL_SIZE,
                0,
                score,
            ]

        vision_model.image.Image = lambda *args, **kwargs: InferenceImage()
        vision_model.tf.detect = lambda net, image: [
            detection(10, 10, 30, 30, 0.95),
            detection(70, 70, 90, 90, 0.65),
        ]
        tracker = vision_model.Tracker()

        candidates_by_class, _ = tracker.get_candidates(SourceImage(), False)

        self.assertEqual(len(candidates_by_class["redbag"]), 2)

    def test_acquire_filters_score_before_distance(self):
        tracker = vision_model.Tracker()
        closest_but_low_score = candidate("redbag", 80, 80, 0.59)
        nearest_eligible = candidate("redbag", 70, 80, 0.61)
        farther_high_score = candidate("redbag", 20, 20, 0.99)

        selected = tracker.choose_acquire_candidate(
            {
                "redbag": [
                    closest_but_low_score,
                    farther_high_score,
                    nearest_eligible,
                ]
            },
            target_filter_name="redbag",
        )

        self.assertIs(selected, nearest_eligible)

    def test_tracking_keeps_original_same_class_object(self):
        tracker = vision_model.Tracker()
        original = candidate("redbag", 20, 20, 0.80)
        original_next_frame = candidate("redbag", 22, 20, 0.75)
        closer_to_priority_point = candidate("redbag", 80, 80, 0.95)
        tracker.start_lock(original)
        tracker.get_candidates = lambda img, enabled: (
            {"redbag": [closer_to_priority_point, original_next_frame]},
            None,
        )

        result = tracker.process_frame(object(), target_filter_id=1)

        self.assertEqual(tracker.status, "DETECT")
        self.assertIs(result[2], original_next_frame)
        self.assertLess(tracker.smooth_x, 30.0)

    def test_reacquire_uses_priority_point_after_target_is_lost(self):
        tracker = vision_model.Tracker()
        tracker.start_lock(candidate("redbag", 20, 20, 0.80))
        tracker.get_candidates = lambda img, enabled: ({}, None)

        for _ in range(vision_model.MAX_HOLD_FRAMES + 1):
            tracker.process_frame(object(), target_filter_id=1)

        self.assertIsNone(tracker.lock_name)
        self.assertEqual(tracker.status, "LOST")

        nearer = candidate("redbag", 75, 80, 0.65)
        farther = candidate("redbag", 20, 20, 0.95)
        tracker.get_candidates = lambda img, enabled: (
            {"redbag": [farther, nearer]},
            None,
        )

        result = tracker.process_frame(object(), target_filter_id=1)

        self.assertEqual(tracker.status, "ACQUIRE")
        self.assertIs(result[2], nearer)
        self.assertEqual((tracker.smooth_x, tracker.smooth_y), (75.0, 80.0))


if __name__ == "__main__":
    unittest.main()
