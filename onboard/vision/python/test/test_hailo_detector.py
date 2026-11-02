import tempfile
import unittest
from pathlib import Path

import numpy as np
from argus_vision.hailo_detector import parse_nms_output, read_labels
from argus_vision.model_input import LetterboxTransform

LABELS = ("tank", "person", "car")
SAME_SIZE = LetterboxTransform(640, 480, 640, 480, 1.0, 0, 0)


def rows(*values: tuple[float, ...]) -> np.ndarray:
    return np.array(values, dtype=np.float32).reshape(-1, 5)


class ParseNmsOutputTest(unittest.TestCase):
    def test_converts_classes_and_sorts_by_confidence(self) -> None:
        result = [
            rows((0.25, 0.5, 0.5, 0.75, 0.8)),
            rows(),
            rows((0.0, 0.0, 0.5, 0.25, 0.95), (0.1, 0.1, 0.2, 0.2, 0.71)),
        ]
        detections = parse_nms_output(result, SAME_SIZE, LABELS, 0.7)
        self.assertEqual(
            [(d.classification.class_id, d.classification.label) for d in detections],
            [(2, "car"), (0, "tank"), (2, "car")],
        )
        tank = detections[1]
        self.assertAlmostEqual(tank.classification.confidence, 0.8, places=6)
        self.assertEqual(
            (tank.box.x1, tank.box.y1, tank.box.x2, tank.box.y2),
            (320.0, 120.0, 480.0, 240.0),
        )

    def test_drops_boxes_below_threshold(self) -> None:
        # NMS 출력은 float32 라 0.7 은 0.69999999 로 경계값 0.7 아래다 (RPi 와 같음).
        result = [
            rows(
                (0.1, 0.1, 0.2, 0.2, 0.69),
                (0.1, 0.1, 0.2, 0.2, 0.7),
                (0.1, 0.1, 0.2, 0.2, 0.71),
            )
        ]
        detections = parse_nms_output(result, SAME_SIZE, LABELS, 0.7)
        self.assertEqual(len(detections), 1)

    def test_uses_class_id_when_label_is_missing(self) -> None:
        result = [rows(), rows(), rows(), rows((0.1, 0.1, 0.2, 0.2, 0.9))]
        detections = parse_nms_output(result, SAME_SIZE, LABELS, 0.5)
        self.assertEqual(detections[0].classification.label, "3")

    def test_skips_short_rows(self) -> None:
        detections = parse_nms_output([[[0.1, 0.1, 0.2]]], SAME_SIZE, LABELS, 0.0)
        self.assertEqual(detections, [])

    def test_letterboxed_square_model_maps_back(self) -> None:
        transform = LetterboxTransform(640, 480, 640, 640, 1.0, 0, 80)
        result = [rows((0.125, 0.0, 0.875, 1.0, 0.9))]
        box = parse_nms_output(result, transform, LABELS, 0.5)[0].box
        self.assertEqual((box.x1, box.y1, box.x2, box.y2), (0.0, 0.0, 639.0, 479.0))


class ReadLabelsTest(unittest.TestCase):
    def test_skips_blank_lines(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "labels.txt"
            path.write_text("tank\n\n person \ncar\n", encoding="utf-8")
            self.assertEqual(read_labels(path), ["tank", "person", "car"])


if __name__ == "__main__":
    unittest.main()
