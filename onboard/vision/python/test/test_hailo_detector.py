import tempfile
import unittest
from pathlib import Path

import numpy as np
from argus_vision.hailo_detector import (
    SuppressRule,
    box_iou,
    parse_class_score_thresholds,
    parse_class_suppress,
    parse_nms_output,
    read_labels,
)
from argus_vision.model_input import LetterboxTransform
from argus_vision.types import PixelBox

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


class ClassThresholdTest(unittest.TestCase):
    def test_class_threshold_replaces_score_threshold(self) -> None:
        # 탱크 0.5, 사람 0.45, 차 0.35 (실기체 3클래스 설정)
        result = [
            rows((0.1, 0.1, 0.2, 0.2, 0.55), (0.3, 0.3, 0.4, 0.4, 0.45)),
            rows((0.5, 0.5, 0.6, 0.6, 0.46)),
            rows((0.7, 0.7, 0.8, 0.8, 0.36), (0.7, 0.1, 0.8, 0.2, 0.30)),
        ]
        detections = parse_nms_output(result, SAME_SIZE, LABELS, 0.7, (0.5, 0.45, 0.35))
        self.assertEqual(
            sorted(round(d.classification.confidence, 2) for d in detections),
            [0.36, 0.46, 0.55],
        )

    def test_empty_class_thresholds_keep_score_threshold(self) -> None:
        result = [rows((0.1, 0.1, 0.2, 0.2, 0.6))]
        self.assertEqual(parse_nms_output(result, SAME_SIZE, LABELS, 0.7, ()), [])

    def test_parse_thresholds(self) -> None:
        self.assertEqual(
            parse_class_score_thresholds("0.5, 0.45,0.35", LABELS), (0.5, 0.45, 0.35)
        )
        self.assertEqual(parse_class_score_thresholds("  ", LABELS), ())
        with self.assertRaises(ValueError):
            parse_class_score_thresholds("0.5,0.45", LABELS)


class ClassSuppressTest(unittest.TestCase):
    def test_parse_rules(self) -> None:
        self.assertEqual(
            parse_class_suppress("tank:car:0.3", LABELS), (SuppressRule(0, 2, 0.3),)
        )
        self.assertEqual(parse_class_suppress("", LABELS), ())
        for bad in ("tank:car", "tank:dog:0.3", "tank:tank:0.3"):
            with self.assertRaises(ValueError):
                parse_class_suppress(bad, LABELS)

    def test_box_iou(self) -> None:
        a = PixelBox(0.0, 0.0, 10.0, 10.0)
        self.assertAlmostEqual(box_iou(a, a), 1.0)
        self.assertAlmostEqual(box_iou(a, PixelBox(5.0, 0.0, 15.0, 10.0)), 50.0 / 150.0)
        self.assertEqual(box_iou(a, PixelBox(20.0, 20.0, 30.0, 30.0)), 0.0)

    def test_drops_car_overlapping_tank(self) -> None:
        # 같은 물체를 탱크와 차로 함께 잡은 경우 차 박스를 지우고, 떨어진 차는 남긴다
        result = [
            rows((0.1, 0.1, 0.3, 0.3, 0.9)),
            rows(),
            rows((0.1, 0.12, 0.3, 0.32, 0.8), (0.6, 0.6, 0.8, 0.8, 0.7)),
        ]
        rules = parse_class_suppress("tank:car:0.3", LABELS)
        detections = parse_nms_output(result, SAME_SIZE, LABELS, 0.5, (), rules)
        self.assertEqual(
            [
                (d.classification.label, round(d.classification.confidence, 2))
                for d in detections
            ],
            [("tank", 0.9), ("car", 0.7)],
        )

    def test_keeps_both_without_rules(self) -> None:
        result = [
            rows((0.1, 0.1, 0.3, 0.3, 0.9)),
            rows(),
            rows((0.1, 0.12, 0.3, 0.32, 0.8)),
        ]
        self.assertEqual(len(parse_nms_output(result, SAME_SIZE, LABELS, 0.5)), 2)


if __name__ == "__main__":
    unittest.main()
