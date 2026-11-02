import unittest
from typing import Any

from argus_vision.messagepack import pack_message
from argus_vision.types import Classification, Detection, PixelBox
from argus_vision.worker import PythonVisionWorker
from argus_vision.zmq_worker_server import SCHEMA, ModelInfo, RequestHandler
from argus_vision_testing.fakes import FakeDetector, FakePreprocessor

TANK = Detection(Classification(0, "tank", 0.8), PixelBox(1.0, 2.0, 5.0, 4.0))
PIXELS = bytes([0x00, 0x01, 0x00, 0x02])


def header(op: str, **fields: Any) -> bytes:
    return pack_message({"schema": SCHEMA, "session_id": "s1", "op": op, **fields})


def infer_header(frame_id: int, **overrides: Any) -> bytes:
    fields: dict[str, Any] = {
        "frame_id": frame_id,
        "width": 2,
        "height": 1,
        "stride": 4,
        "pixel_format": "RAW14_LE16",
        "received_unix_ns": 100,
        "received_monotonic_ns": 200,
        "clock_domain": "host_monotonic",
        "exposure_time_valid": False,
        "exposure_monotonic_ns": None,
        "source_receive_monotonic_ns": None,
    }
    fields.update(overrides)
    return header("infer", **fields)


class RequestHandlerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.pre = FakePreprocessor()
        self.detector = FakeDetector(model="m.hef")
        worker = PythonVisionWorker(self.pre, self.detector)
        self.handler = RequestHandler(
            worker, ModelInfo("m.hef", 640, 480), "ir_sdk:0", "s1"
        )

    def test_ready_resets_stream_and_reports_model(self) -> None:
        reply = self.handler.handle([header("ready")])
        self.assertTrue(reply["ok"], reply)
        self.assertEqual(
            (reply["model"], reply["input_width"], reply["input_height"]),
            ("m.hef", 640, 480),
        )
        self.assertEqual(self.pre.reset_count, 1)

    def test_infer_returns_integer_boxes(self) -> None:
        self.detector.set_detections([TANK])
        reply = self.handler.handle([infer_header(7), PIXELS])
        self.assertTrue(reply["ok"], reply)
        self.assertEqual(reply["frame_id"], 7)
        self.assertEqual(reply["coordinate_frame"], "camera_pixel")
        self.assertEqual(
            reply["detections"],
            [
                {
                    "class_id": 0,
                    "label": "tank",
                    "confidence": 0.8,
                    "x1": 1,
                    "y1": 2,
                    "x2": 5,
                    "y2": 4,
                }
            ],
        )
        raw = self.pre.processed[0]
        self.assertEqual(
            (raw.source, raw.session_id, raw.frame_id), ("ir_sdk:0", "s1", 7)
        )
        self.assertEqual((raw.unix_ns, raw.received_monotonic_ns), (100, 200))

    def test_frame_id_must_advance(self) -> None:
        self.assertTrue(self.handler.handle([infer_header(7), PIXELS])["ok"])
        reply = self.handler.handle([infer_header(7), PIXELS])
        self.assertFalse(reply["ok"])
        self.assertIn("advance", reply["error"])

    def test_rejects_other_session(self) -> None:
        bad = pack_message({"schema": SCHEMA, "session_id": "other", "op": "ready"})
        self.assertFalse(self.handler.handle([bad])["ok"])

    def test_rejects_inconsistent_exposure(self) -> None:
        reply = self.handler.handle([infer_header(1, exposure_time_valid=True), PIXELS])
        self.assertFalse(reply["ok"])

    def test_rejects_other_pixel_format(self) -> None:
        reply = self.handler.handle([infer_header(1, pixel_format="BGR8"), PIXELS])
        self.assertFalse(reply["ok"])

    def test_rejects_wrong_pixel_count(self) -> None:
        reply = self.handler.handle([infer_header(1), PIXELS[:2]])
        self.assertFalse(reply["ok"])

    def test_detector_error_becomes_error_reply(self) -> None:
        self.detector.set_error("npu down")
        reply = self.handler.handle([infer_header(1), PIXELS])
        self.assertEqual((reply["ok"], reply["error"]), (False, "npu down"))

    def test_shutdown_stops(self) -> None:
        reply = self.handler.handle([header("shutdown")])
        self.assertTrue(reply["ok"])
        self.assertTrue(self.handler.stopping)

    def test_unknown_operation_is_rejected(self) -> None:
        self.assertFalse(self.handler.handle([header("ready"), PIXELS])["ok"])


if __name__ == "__main__":
    unittest.main()
