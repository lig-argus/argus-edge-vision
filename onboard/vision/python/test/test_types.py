import unittest

from argus_vision.types import (
    Classification,
    Detection,
    Gray8Frame,
    PixelBox,
    RawFrame,
)


def make_raw(width: int = 4, height: int = 2, stride: int = 8) -> RawFrame:
    return RawFrame(
        source="ir0",
        session_id="s1",
        frame_id=7,
        unix_ns=1_000,
        received_monotonic_ns=2_000,
        width=width,
        height=height,
        stride=stride,
        pixels=bytes(stride * height),
    )


class RawFrameTest(unittest.TestCase):
    def test_accepts_matching_layout(self) -> None:
        frame = make_raw()
        self.assertEqual(frame.frame_id, 7)
        self.assertIsNone(frame.exposure_monotonic_ns)

    def test_accepts_padded_stride(self) -> None:
        frame = make_raw(stride=10)
        self.assertEqual(len(frame.pixels), 20)

    def test_rejects_stride_smaller_than_row(self) -> None:
        with self.assertRaises(ValueError):
            make_raw(stride=6)

    def test_rejects_wrong_pixel_size(self) -> None:
        with self.assertRaises(ValueError):
            RawFrame(
                source="ir0",
                session_id="s1",
                frame_id=0,
                unix_ns=0,
                received_monotonic_ns=0,
                width=4,
                height=2,
                stride=8,
                pixels=bytes(15),
            )


class Gray8FrameTest(unittest.TestCase):
    def test_rejects_zero_size(self) -> None:
        with self.assertRaises(ValueError):
            Gray8Frame(
                source="ir0",
                session_id="s1",
                frame_id=0,
                unix_ns=0,
                received_monotonic_ns=0,
                width=0,
                height=2,
                stride=0,
                preprocess="fake",
                pixels=b"",
            )


class DetectionTest(unittest.TestCase):
    def test_box_size(self) -> None:
        box = PixelBox(10.0, 20.0, 50.0, 35.0)
        self.assertEqual(box.width, 40.0)
        self.assertEqual(box.height, 15.0)

    def test_rejects_reversed_box(self) -> None:
        with self.assertRaises(ValueError):
            PixelBox(50.0, 20.0, 10.0, 35.0)

    def test_rejects_confidence_out_of_range(self) -> None:
        with self.assertRaises(ValueError):
            Classification(0, "tank", 1.5)

    def test_detection_is_value(self) -> None:
        a = Detection(Classification(0, "tank", 0.9), PixelBox(1.0, 2.0, 3.0, 4.0))
        b = Detection(Classification(0, "tank", 0.9), PixelBox(1.0, 2.0, 3.0, 4.0))
        self.assertEqual(a, b)


if __name__ == "__main__":
    unittest.main()
