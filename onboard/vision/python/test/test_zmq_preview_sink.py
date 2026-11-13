import random
import unittest
from dataclasses import replace
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


def picks(fps: float, times: list[float]) -> list[int]:
    pacer = FpsPacer(fps)
    return [i for i, t in enumerate(times) if pacer.due(t)]


class FpsPacerTest(unittest.TestCase):
    # 실기체 tests/preview/test_preview_pacer.py (2026-11-12) 와 같은 시험

    def test_half_rate_takes_every_second_frame(self) -> None:
        chosen = picks(15, [i / 30.0 for i in range(300)])
        self.assertEqual({b - a for a, b in zip(chosen, chosen[1:], strict=False)}, {2})
        self.assertEqual(len(chosen), 150)

    def test_capture_jitter_keeps_even_steps(self) -> None:
        rng = random.Random(7)
        times = [i / 30.0 + rng.uniform(-0.008, 0.008) for i in range(3000)]
        chosen = picks(15, times)
        steps = [b - a for a, b in zip(chosen, chosen[1:], strict=False)]
        self.assertGreaterEqual(steps.count(2) / len(steps), 0.99)
        self.assertNotIn(1, steps)

    def test_same_rate_takes_every_frame(self) -> None:
        rng = random.Random(3)
        times = [i / 30.0 + rng.uniform(-0.005, 0.005) for i in range(600)]
        self.assertEqual(len(picks(30, times)), 600)

    def test_stall_restarts_grid(self) -> None:
        times = [i / 30.0 for i in range(60)] + [2.5 + i / 30.0 for i in range(60)]
        chosen = picks(15, times)
        steps = [b - a for a, b in zip(chosen, chosen[1:], strict=False)]
        self.assertEqual(steps.count(1), 0)
        self.assertIn(60, chosen)  # 멈춘 뒤 첫 프레임은 바로 보낸다

    def test_rejects_bad_fps(self) -> None:
        for fps in (0.0, -1.0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                FpsPacer(fps)


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

    def at_now(self, frame: Gray8Frame) -> Gray8Frame:
        # 미리보기는 카메라 수신 시각으로 간격을 고르므로 시계에 맞춘 프레임을 만든다
        return replace(frame, received_monotonic_ns=int(self.clock.now * 1e9))

    def receive(self) -> tuple[str, dict[str, Any], bytes]:
        topic, header, payload = self.pull.recv_multipart()
        return topic.decode(), unpack_message(header), payload

    def wait_connected(self, sink: ZmqPreviewSink, frame: Gray8Frame) -> None:
        # IMMEDIATE 라 연결 전에 보낸 것은 버려진다. 메시지가 올 때까지 보내고,
        # 연결된 뒤 받은 것은 비운다.
        for _ in range(30):
            self.clock.now += 1.0
            self.assertTrue(sink.offer(self.at_now(frame), [TANK]))
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
            frame = self.at_now(frame)
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
                "received_monotonic_ns": frame.received_monotonic_ns,
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

    def test_fps_limit_uses_capture_time(self) -> None:
        # 30 Hz 카메라를 10 FPS 로: 세 장에 한 장.
        # 시계는 움직이지 않아도 수신 시각으로 고른다
        options = PreviewOptions(overlay=True, overlay_max_fps=10.0)
        sink = ZmqPreviewSink(self.endpoint, INFO, options, clock=self.clock)
        frames = [
            replace(make_gray(i), received_monotonic_ns=1_000_000_000 + i * 33_333_333)
            for i in range(7)
        ]
        try:
            results = [sink.offer(frame, []) for frame in frames]
        finally:
            sink.close()
        self.assertEqual(results, [True, False, False, True, False, False, True])

    def test_frame_without_capture_time_uses_clock(self) -> None:
        options = PreviewOptions(overlay=True, overlay_max_fps=10.0)
        sink = ZmqPreviewSink(self.endpoint, INFO, options, clock=self.clock)
        frame = replace(make_gray(), received_monotonic_ns=0)
        try:
            first = sink.offer(frame, [])
            same_time = sink.offer(frame, [])
            self.clock.now += 0.1
            later = sink.offer(frame, [])
        finally:
            sink.close()
        self.assertEqual([first, same_time, later], [True, False, True])

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
