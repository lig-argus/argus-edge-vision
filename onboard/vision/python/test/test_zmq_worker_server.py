import unittest
from typing import Any

from argus_msg import perception_ipc_pb2
from argus_msg.perception_ipc_pb2 import PerceptionReply, PerceptionRequest
from argus_vision.types import Classification, Detection, PixelBox
from argus_vision.worker import PythonVisionWorker
from argus_vision.zmq_worker_server import ModelInfo, RequestHandler
from argus_vision_testing.fakes import FakeDetector, FakePreprocessor

TANK = Detection(Classification(0, "tank", 0.8), PixelBox(1.0, 2.0, 5.0, 4.0))
PIXELS = bytes([0x00, 0x01, 0x00, 0x02])
READY = perception_ipc_pb2.PERCEPTION_OP_READY
INFER = perception_ipc_pb2.PERCEPTION_OP_INFER
SHUTDOWN = perception_ipc_pb2.PERCEPTION_OP_SHUTDOWN


def request(
    op: perception_ipc_pb2.PerceptionOp, session_id: str = "s1", **fields: Any
) -> bytes:
    message = PerceptionRequest(session_id=session_id, op=op, **fields)
    return message.SerializeToString()


def infer_request(frame_id: int, **overrides: Any) -> bytes:
    fields: dict[str, Any] = {
        "frame_id": frame_id,
        "width": 2,
        "height": 1,
        "stride": 4,
        "pixel_format": "RAW14_LE16",
        "received_unix_ns": 100,
        "received_monotonic_ns": 200,
    }
    fields.update(overrides)
    return request(INFER, **fields)


class RequestHandlerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.pre = FakePreprocessor()
        self.detector = FakeDetector(model="m.hef")
        worker = PythonVisionWorker(self.pre, self.detector)
        self.handler = RequestHandler(
            worker, ModelInfo("m.hef", 640, 480), "ir_sdk:0", "s1"
        )

    def handle(self, *parts: bytes) -> PerceptionReply:
        # 소켓으로 보낸 것과 같게 직렬화한 뒤 다시 읽는다
        reply = PerceptionReply()
        reply.ParseFromString(self.handler.handle(list(parts)).SerializeToString())
        return reply

    def test_ready_resets_stream_and_reports_model(self) -> None:
        reply = self.handle(request(READY))
        self.assertTrue(reply.ok, reply)
        self.assertEqual(
            (reply.session_id, reply.model, reply.input_width, reply.input_height),
            ("s1", "m.hef", 640, 480),
        )
        self.assertEqual(self.pre.reset_count, 1)

    def test_infer_returns_integer_boxes(self) -> None:
        self.detector.set_detections([TANK])
        reply = self.handle(infer_request(7), PIXELS)
        self.assertTrue(reply.ok, reply)
        self.assertEqual((reply.frame_id, reply.model), (7, "m.hef"))
        self.assertEqual(len(reply.detections), 1)
        d = reply.detections[0]
        self.assertEqual((d.class_id, d.label), (0, "tank"))
        self.assertAlmostEqual(d.confidence, 0.8)
        self.assertEqual((d.x1, d.y1, d.x2, d.y2), (1, 2, 5, 4))
        raw = self.pre.processed[0]
        self.assertEqual(
            (raw.source, raw.session_id, raw.frame_id), ("ir_sdk:0", "s1", 7)
        )
        self.assertEqual((raw.unix_ns, raw.received_monotonic_ns), (100, 200))
        self.assertIsNone(raw.exposure_monotonic_ns)

    def test_exposure_time_is_optional(self) -> None:
        self.handle(infer_request(1, exposure_monotonic_ns=150), PIXELS)
        self.assertEqual(self.pre.processed[0].exposure_monotonic_ns, 150)

    def test_frame_id_must_advance(self) -> None:
        self.assertTrue(self.handle(infer_request(7), PIXELS).ok)
        reply = self.handle(infer_request(7), PIXELS)
        self.assertFalse(reply.ok)
        self.assertIn("advance", reply.error)

    def test_rejects_other_session(self) -> None:
        reply = self.handle(request(READY, session_id="other"))
        self.assertFalse(reply.ok)
        self.assertEqual(reply.session_id, "s1")

    def test_rejects_bytes_that_are_not_a_request(self) -> None:
        reply = self.handle(b"\xff\xff\xff")
        self.assertFalse(reply.ok)
        self.assertIn("PerceptionRequest", reply.error)

    def test_rejects_exposure_after_receive(self) -> None:
        reply = self.handle(infer_request(1, exposure_monotonic_ns=300), PIXELS)
        self.assertFalse(reply.ok)

    def test_rejects_other_pixel_format(self) -> None:
        reply = self.handle(infer_request(1, pixel_format="BGR8"), PIXELS)
        self.assertFalse(reply.ok)

    def test_rejects_wrong_pixel_count(self) -> None:
        reply = self.handle(infer_request(1), PIXELS[:2])
        self.assertFalse(reply.ok)

    def test_detector_error_becomes_error_reply(self) -> None:
        self.detector.set_error("npu down")
        reply = self.handle(infer_request(1), PIXELS)
        self.assertEqual((reply.ok, reply.error), (False, "npu down"))

    def test_shutdown_stops(self) -> None:
        reply = self.handle(request(SHUTDOWN))
        self.assertTrue(reply.ok)
        self.assertTrue(self.handler.stopping)

    def test_unknown_operation_is_rejected(self) -> None:
        self.assertFalse(self.handle(request(READY), PIXELS).ok)
        unspecified = request(perception_ipc_pb2.PERCEPTION_OP_UNSPECIFIED)
        self.assertFalse(self.handle(unspecified).ok)


if __name__ == "__main__":
    unittest.main()
