import unittest

from argus_vision.types import Classification, Detection, PixelBox, RawFrame
from argus_vision_testing.fakes import FakeDetector, FakePreprocessor, FakePreviewSink


def make_raw() -> RawFrame:
    # 2×2, 픽셀 값 0x0100, 0x0200, 0x0300, 0x0400 (리틀 엔디언)
    return RawFrame(
        source="ir0",
        session_id="s1",
        frame_id=3,
        unix_ns=10,
        received_monotonic_ns=20,
        width=2,
        height=2,
        stride=4,
        pixels=bytes([0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04]),
    )


TANK = Detection(Classification(0, "tank", 0.8), PixelBox(1.0, 1.0, 5.0, 4.0))


class FakePreprocessorTest(unittest.TestCase):
    def test_keeps_frame_key_and_size(self) -> None:
        pre = FakePreprocessor()
        gray = pre.process(make_raw())
        self.assertEqual(
            (gray.source, gray.session_id, gray.frame_id), ("ir0", "s1", 3)
        )
        self.assertEqual((gray.width, gray.height, gray.stride), (2, 2, 2))
        self.assertEqual(gray.pixels, bytes([1, 2, 3, 4]))
        self.assertEqual(gray.preprocess, pre.name)

    def test_counts_reset(self) -> None:
        pre = FakePreprocessor()
        pre.reset()
        pre.reset()
        self.assertEqual(pre.reset_count, 2)


class FakeDetectorTest(unittest.TestCase):
    def test_returns_set_detections(self) -> None:
        detector = FakeDetector()
        gray = FakePreprocessor().process(make_raw())
        self.assertEqual(detector.detect(gray), [])
        detector.set_detections([TANK])
        self.assertEqual(detector.detect(gray), [TANK])
        self.assertEqual(len(detector.frames), 2)

    def test_error_and_close(self) -> None:
        detector = FakeDetector()
        gray = FakePreprocessor().process(make_raw())
        detector.set_error("npu timeout")
        with self.assertRaises(RuntimeError):
            detector.detect(gray)
        detector.set_error(None)
        detector.close()
        detector.close()
        with self.assertRaises(RuntimeError):
            detector.detect(gray)

    def test_model_and_labels(self) -> None:
        detector = FakeDetector("m1", ["tank", "person"])
        self.assertEqual(detector.model, "m1")
        self.assertEqual(list(detector.labels), ["tank", "person"])


class FakePreviewSinkTest(unittest.TestCase):
    def test_records_and_drops(self) -> None:
        sink = FakePreviewSink()
        gray = FakePreprocessor().process(make_raw())
        self.assertTrue(sink.offer(gray, [TANK]))
        sink.set_accept(False)
        self.assertFalse(sink.offer(gray, []))
        self.assertEqual(len(sink.offered), 1)
        self.assertEqual(sink.offered[0][1], (TANK,))
        self.assertEqual(sink.dropped, 1)

    def test_closed_sink_drops(self) -> None:
        sink = FakePreviewSink()
        sink.close()
        gray = FakePreprocessor().process(make_raw())
        self.assertFalse(sink.offer(gray, []))


if __name__ == "__main__":
    unittest.main()
