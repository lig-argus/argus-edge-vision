# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Pi-side box drawing/encoding on a separate worker; no public bus sockets."""

import math
import os
import queue
import threading
import time

import cv2
import zmq

from argus.transport.protocol import pack_message

_STOP = object()


class _PreviewPacer:
    """Pick frames on an even grid of camera capture times (2026-11-12).

    The old credit pacer used the post-inference time and allowed a 2-frame burst, so a
    30 Hz camera limited to 15 fps produced 33/66/100 ms steps. Capture times keep the grid
    even; the tolerance is half the measured input interval so a neighbouring frame is
    never taken. After a stall (e.g. camera NUC) the grid restarts at the next frame.
    """

    def __init__(self, fps):
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("Preview FPS must be finite and positive")
        self.fps = fps
        self.period = 1.0 / fps
        self.next = None
        self.last = None
        self.input_dt = None

    def due(self, now):
        if self.last is not None:
            dt = now - self.last
            if 0.0 < dt < 1.0:
                self.input_dt = dt if self.input_dt is None else 0.9 * self.input_dt + 0.1 * dt
        self.last = now
        tolerance = 0.5 * min(self.period, self.input_dt or self.period)
        if self.next is not None and now < self.next - tolerance:
            return False
        if self.next is None or now - self.next >= self.period:
            self.next = now + self.period
        else:
            self.next += self.period
        return True


class OverlayEncoder:
    def __init__(
        self,
        endpoint,
        source,
        session_id,
        overlay,
        npu_input,
        max_fps=10,
        quality=75,
        npu_fps=15,
        npu_quality=80,
    ):
        self.endpoint, self.source, self.session_id = endpoint, source, session_id
        self.overlay, self.npu_input = overlay, npu_input
        self.overlay_pacer = _PreviewPacer(max_fps)
        self.npu_pacer = _PreviewPacer(npu_fps)
        self.max_fps = max_fps
        self.quality, self.npu_quality = quality, npu_quality
        self.queue = queue.Queue(
            maxsize=1
        )  # Preview-only queue; perception queue remains C++ capacity 2.
        self.stop_event = threading.Event()
        self.error = None
        self.h264 = None
        self.codec = os.environ.get("ARGUS_PREVIEW_CODEC", "jpeg")
        if self.codec not in ("jpeg", "h264"):
            raise ValueError("Unsupported preview codec")
        self.published = self.dropped = 0
        self.thread = threading.Thread(target=self._run, name="OverlayEncoder")
        self.thread.start()

    def offer(self, header, bgr, detections, model_rgb, transform, t_pre_us):
        if self.error:
            raise RuntimeError("OverlayEncoder failed") from self.error
        # Camera receive time, not post-inference time: keeps the published frames evenly spaced.
        captured = header.get("received_monotonic_ns")
        now = captured / 1e9 if isinstance(captured, int) and captured > 0 else time.monotonic()
        overlay = self.overlay and self.overlay_pacer.due(now)
        npu = self.npu_input and self.npu_pacer.due(now)
        if not overlay and not npu:
            return
        item = (dict(header), bgr, tuple(detections), model_rgb, transform, t_pre_us, overlay, npu)
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            try:
                self.queue.get_nowait()
                self.dropped += 1
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(item)
            except queue.Full:
                self.dropped += 1

    def _send(self, socket, topic, header, image, quality):
        if topic == "vision/overlay/jpeg" and self.codec == "h264":
            from argus.transport.video.i3u1.udp_video import H264Encoder

            if self.h264 is None:
                self.h264 = H264Encoder(
                    image.shape[1], image.shape[0], fps=max(1, round(self.max_fps))
                )
            began = time.perf_counter()
            payload = self.h264.encode(image)
            topic = "vision/overlay/h264"
            header = dict(
                header,
                encoding="h264",
                keyframe=True,
                encoder="FFmpeg libx264 ultrafast",
                encode_ms=(time.perf_counter() - began) * 1000,
            )
        else:
            # Keep baseline/non-optimized JPEG explicit across OpenCV versions.
            # RFC 2435 packetization compatibility still requires payloader validation.
            ok, encoded = cv2.imencode(
                ".jpg",
                image,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    quality,
                    cv2.IMWRITE_JPEG_PROGRESSIVE,
                    0,
                    cv2.IMWRITE_JPEG_OPTIMIZE,
                    0,
                ],
            )
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            payload = encoded.tobytes()
        try:
            socket.send_multipart(
                [topic.encode(), pack_message(header), payload], flags=zmq.NOBLOCK
            )
            self.published += 1
        except zmq.Again:
            self.dropped += 1

    def _run(self):
        context = zmq.Context()
        socket = context.socket(zmq.PUSH)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.SNDHWM, 1)
        socket.setsockopt(zmq.IMMEDIATE, 1)
        try:
            socket.connect(self.endpoint)
            while not self.stop_event.is_set():
                try:
                    item = self.queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                if item is _STOP:
                    break
                header, bgr, detections, rgb, transform, pre_us, overlay, npu = item
                common = {
                    "source": self.source,
                    "session_id": self.session_id,
                    "frame_id": header["frame_id"],
                    "seq": header["frame_id"],
                    "encoding": "jpeg",
                    "backend": getattr(self, "backend", None),
                    "model": getattr(self, "model", None),
                    "preprocess": getattr(self, "preprocess", None),
                    "detections": [
                        {
                            "label": d.label,
                            "confidence": float(d.confidence),
                            "x1": int(d.x1),
                            "y1": int(d.y1),
                            "x2": int(d.x2),
                            "y2": int(d.y2),
                        }
                        for d in detections
                    ],
                }
                if overlay:
                    image = bgr.copy()  # Never draw into inference buffers/shared source frames.
                    for d in detections:
                        cv2.rectangle(image, (d.x1, d.y1), (d.x2, d.y2), (0, 255, 0), 2)
                        cv2.putText(
                            image,
                            f"{d.label} {d.confidence:.2f}",
                            (d.x1, max(18, d.y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 255, 0),
                            2,
                            cv2.LINE_AA,
                        )
                    overlay_header = dict(
                        common,
                        schema="argus.vision.overlay.v1",
                        unix_ns=header["received_unix_ns"],
                        width=header["width"],
                        height=header["height"],
                        received_monotonic_ns=header["received_monotonic_ns"],
                    )
                    self._send(socket, "vision/overlay/jpeg", overlay_header, image, self.quality)
                if npu:
                    npu_header = dict(
                        common,
                        schema="argus.vision.npu_input.v1",
                        t_capture_us=header["received_monotonic_ns"] // 1000,
                        t_pre_us=pre_us,
                        src_w=transform.source_width,
                        src_h=transform.source_height,
                        lb_scale=transform.scale,
                        lb_pad_x=transform.pad_x,
                        lb_pad_y=transform.pad_y,
                        preprocess="letterbox_bgr_to_rgb",
                    )
                    # ARGUS 2026-10-21 prep1ch: a 1-channel HEF input is already gray
                    npu_image = (
                        rgb[:, :, 0]
                        if rgb.ndim == 3 and rgb.shape[2] == 1
                        else cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    )
                    self._send(
                        socket, "vision/npu_input/jpeg", npu_header, npu_image, self.npu_quality
                    )
        except BaseException as exc:
            self.error = exc
        finally:
            socket.close(linger=0)
            context.term()

    def close(self):
        self.stop_event.set()
        try:
            self.queue.put_nowait(_STOP)
        except queue.Full:
            pass
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise RuntimeError("OverlayEncoder did not stop")
        if self.error:
            raise RuntimeError("OverlayEncoder failed") from self.error
