import unittest
from typing import Any

import cv2
import numpy as np
import zmq
from argus_vision.messagepack import unpack_message
from argus_vision.types import Classification, Detection, Gray8Frame, PixelBox
from argus_vision.zmq_preview_sink import (
    GRAY8_TOPIC,
    NPU_INPUT_TOPIC,
    OVERLAY_TOPIC,
    FpsPacer,
    PreviewHeaderInfo,
    PreviewOptions,
    ZmqPreviewSink,
)

TANK = Detection(Classification(0, "tank", 0.8), PixelBox(10.0, 12.0, 30.0, 40.0))
INFO = PreviewHeaderInfo(
    source="ir_sdk:0",
    session_id="s1",
    backend="hailo",
    model="m.hef",
    model_width=64,
    model_height=64,
    model_channels=1,
)


def make_gray(frame_id: int = 5) -> Gray8Frame:
    pixels = np.random.default_rng(frame_id).integers(0, 256, (48, 64), dtype=np.uint8)
    return Gray8Frame(
        source="ir_sdk:0",
        session_id="s1",
        frame_id=frame_id,
        unix_ns=111,
        received_monotonic_ns=222,
        width=64,
        height=48,
        stride=64,
        preprocess="double_plateau_adaptive",
        pixels=pixels.tobytes(),
    )


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


class FpsPacerTest(unittest.TestCase):
    def test_allows_one_frame_of_jitter_then_limits(self) -> None:
        pacer = FpsPacer(10.0)
        self.assertEqual(
            [pacer.due(0.0), pacer.due(0.0), pacer.due(0.0)], [True, True, False]
        )
        self.assertFalse(pacer.due(0.05))
        self.assertTrue(pacer.due(0.1))

    def test_rejects_non_positive_rate(self) -> None:
        with self.assertRaises(ValueError):
            FpsPacer(0.0)


class ZmqPreviewSinkTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context = zmq.Context()
        self.pull = self.context.socket(zmq.PULL)
        self.pull.setsockopt(zmq.LINGER, 0)
        self.pull.setsockopt(zmq.RCVTIMEO, 3000)
        port = self.pull.bind_to_random_port("tcp://127.0.0.1")
        self.endpoint = f"tcp://127.0.0.1:{port}"
        self.clock = FakeClock()

    def tearDown(self) -> None:
        self.pull.close(linger=0)
        self.context.term()

    def receive(self) -> tuple[str, dict[str, Any], bytes]:
        topic, header, payload = self.pull.recv_multipart()
        return topic.decode(), unpack_message(header), payload

    def wait_connected(self, sink: ZmqPreviewSink, frame: Gray8Frame) -> None:
        # IMMEDIATE 라 연결 전에 보낸 것은 버려진다. 메시지가 올 때까지 보내고,
        # 연결된 뒤 받은 것은 비운다.
        for _ in range(30):
            self.clock.now += 1.0
            self.assertTrue(sink.offer(frame, [TANK]))
            if self.pull.poll(100):
                while self.pull.poll(300):
                    self.pull.recv_multipart()
                self.clock.now += 1.0
                return
        self.fail("preview sink did not connect")

    def test_sends_enabled_topics_with_headers(self) -> None:
        options = PreviewOptions(gray8=True, overlay=True, npu_input=True)
        sink = ZmqPreviewSink(self.endpoint, INFO, options, clock=self.clock)
        frame = make_gray()
        try:
            self.wait_connected(sink, frame)
            self.assertTrue(sink.offer(frame, [TANK]))
            received = [self.receive() for _ in range(3)]
        finally:
            sink.close()
        messages = {topic: (header, payload) for topic, header, payload in received}
        self.assertEqual(set(messages), {GRAY8_TOPIC, OVERLAY_TOPIC, NPU_INPUT_TOPIC})

        gray_header, gray_payload = messages[GRAY8_TOPIC]
        self.assertEqual(gray_payload, frame.pixels)
        self.assertEqual(
            gray_header,
            {
                "schema": "argus.vision.frame.v1",
                "source": "ir_sdk:0",
                "session_id": "s1",
                "frame_id": 5,
                "seq": 5,
                "unix_ns": 111,
                "received_monotonic_ns": 222,
                "width": 64,
                "height": 48,
                "stride": 64,
                "encoding": "gray8",
                "preprocess": "double_plateau_adaptive",
            },
        )

        overlay_header, overlay_jpeg = messages[OVERLAY_TOPIC]
        self.assertEqual(overlay_header["schema"], "argus.vision.overlay.v1")
        self.assertEqual(
            (overlay_header["model"], overlay_header["backend"]), ("m.hef", "hailo")
        )
        self.assertEqual(
            overlay_header["detections"],
            [
                {
                    "label": "tank",
                    "confidence": 0.8,
                    "x1": 10,
                    "y1": 12,
                    "x2": 30,
                    "y2": 40,
                }
            ],
        )
        image = cv2.imdecode(np.frombuffer(overlay_jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(image.shape, (48, 64, 3))

        npu_header, npu_jpeg = messages[NPU_INPUT_TOPIC]
        self.assertEqual(npu_header["schema"], "argus.vision.npu_input.v1")
        self.assertEqual(
            (npu_header["lb_scale"], npu_header["lb_pad_x"], npu_header["lb_pad_y"]),
            (1.0, 0, 8),
        )
        npu_image = cv2.imdecode(
            np.frombuffer(npu_jpeg, np.uint8), cv2.IMREAD_UNCHANGED
        )
        self.assertEqual(npu_image.shape, (64, 64))

    def test_fps_limit_skips_frames(self) -> None:
        options = PreviewOptions(overlay=True, overlay_max_fps=10.0)
        sink = ZmqPreviewSink(self.endpoint, INFO, options, clock=self.clock)
        try:
            results = [sink.offer(make_gray(i), []) for i in (1, 2, 3)]
        finally:
            sink.close()
        self.assertEqual(results, [True, True, False])

    def test_nothing_enabled_sends_nothing(self) -> None:
        sink = ZmqPreviewSink(self.endpoint, INFO, PreviewOptions(), clock=self.clock)
        try:
            self.assertFalse(sink.offer(make_gray(), [TANK]))
        finally:
            sink.close()

    def test_close_twice_and_offer_after_close(self) -> None:
        sink = ZmqPreviewSink(
            self.endpoint, INFO, PreviewOptions(overlay=True), clock=self.clock
        )
        sink.close()
        sink.close()
        self.assertFalse(sink.offer(make_gray(), []))


if __name__ == "__main__":
    unittest.main()
