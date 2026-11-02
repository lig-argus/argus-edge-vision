import unittest
from collections.abc import Iterator

from argus_vision.types import Classification, Detection, PixelBox, RawFrame
from argus_vision.worker import PythonVisionWorker
from argus_vision_testing.fakes import FakeDetector, FakePreprocessor, FakePreviewSink

TANK = Detection(Classification(0, "tank", 0.8), PixelBox(1.0, 1.0, 5.0, 4.0))


def make_raw(frame_id: int = 3) -> RawFrame:
    return RawFrame(
        source="ir0",
        session_id="s1",
        frame_id=frame_id,
        unix_ns=10,
        received_monotonic_ns=20,
        width=2,
        height=1,
        stride=4,
        pixels=bytes([0x00, 0x01, 0x00, 0x02]),
    )


def ticking_clock(step_ns: int) -> Iterator[int]:
    now = 1_000_000
    while True:
        yield now
        now += step_ns


class PythonVisionWorkerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.pre = FakePreprocessor()
        self.detector = FakeDetector()
        self.preview = FakePreviewSink()
        clock = ticking_clock(2_000_000)
        self.worker = PythonVisionWorker(
            self.pre, self.detector, self.preview, clock_ns=lambda: next(clock)
        )

    def test_runs_preprocess_detect_and_preview_in_order(self) -> None:
        self.detector.set_detections([TANK])
        result = self.worker.handle(make_raw())
        self.assertEqual(result.detections, [TANK])
        self.assertEqual(self.detector.frames, [result.gray])
        self.assertEqual(self.preview.offered, [(result.gray, (TANK,))])
        self.assertEqual(result.gray.frame_id, 3)

    def test_reports_times(self) -> None:
        result = self.worker.handle(make_raw())
        self.assertEqual(result.preprocess_ms, 2.0)
        self.assertEqual(result.inference_ms, 2.0)
        self.assertEqual(result.infer_done_monotonic_ns, 5_000_000)

    def test_preview_drop_does_not_fail(self) -> None:
        self.preview.set_accept(False)
        self.detector.set_detections([TANK])
        self.assertEqual(self.worker.handle(make_raw()).detections, [TANK])
        self.assertEqual(self.preview.dropped, 1)

    def test_detector_error_propagates(self) -> None:
        self.detector.set_error("npu")
        with self.assertRaises(RuntimeError):
            self.worker.handle(make_raw())
        self.assertEqual(self.preview.offered, [])

    def test_start_stream_resets_preprocessor(self) -> None:
        self.worker.start_stream()
        self.assertEqual(self.pre.reset_count, 1)

    def test_close_closes_preview_and_detector(self) -> None:
        self.worker.close()
        self.assertTrue(self.preview.closed)
        self.assertTrue(self.detector.closed)

    def test_works_without_preview(self) -> None:
        worker = PythonVisionWorker(FakePreprocessor(), self.detector)
        self.assertEqual(worker.handle(make_raw()).detections, [])
        worker.close()
        self.assertTrue(self.detector.closed)


if __name__ == "__main__":
    unittest.main()
