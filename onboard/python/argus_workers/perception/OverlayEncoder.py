"""Pi-side full-box drawing/JPEG on a separate worker; no public bus sockets."""
import queue
import threading
import time

import cv2
import zmq

from ..transport.Protocol import pack_message

_STOP = object()


class OverlayEncoder:
    def __init__(self, endpoint, source, session_id, overlay, npu_input,
                 max_fps=10, quality=75, npu_fps=15, npu_quality=80):
        self.endpoint, self.source, self.session_id = endpoint, source, session_id
        self.overlay, self.npu_input = overlay, npu_input
        self.period, self.npu_period = 1 / max_fps, 1 / npu_fps
        self.quality, self.npu_quality = quality, npu_quality
        self.next_overlay = self.next_npu = 0.0
        self.queue = queue.Queue(maxsize=1)  # Preview-only queue; perception queue remains C++ capacity 2.
        self.stop_event = threading.Event()
        self.error = None
        self.published = self.dropped = 0
        self.thread = threading.Thread(target=self._run, name="OverlayEncoder")
        self.thread.start()

    def offer(self, header, bgr, detections, model_rgb, transform, t_pre_us):
        if self.error:
            raise RuntimeError("OverlayEncoder failed") from self.error
        now = time.monotonic()
        overlay = self.overlay and now >= self.next_overlay
        npu = self.npu_input and now >= self.next_npu
        if not overlay and not npu:
            return
        if overlay:
            self.next_overlay = now + self.period
        if npu:
            self.next_npu = now + self.npu_period
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
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        try:
            socket.send_multipart([topic.encode(), pack_message(header), encoded.tobytes()], flags=zmq.NOBLOCK)
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
                common = {"source": self.source, "session_id": self.session_id,
                          "frame_id": header["frame_id"], "seq": header["frame_id"],
                          "encoding": "jpeg"}
                if overlay:
                    image = bgr.copy()  # Never draw into inference buffers/shared source frames.
                    for d in detections:
                        cv2.rectangle(image, (d.x1, d.y1), (d.x2, d.y2), (0, 255, 0), 2)
                        cv2.putText(image, f"{d.label} {d.confidence:.2f}", (d.x1, max(18, d.y1 - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
                    overlay_header = dict(common, schema="argus.vision.overlay.v1",
                        unix_ns=header["received_unix_ns"], width=header["width"], height=header["height"],
                        received_monotonic_ns=header["received_monotonic_ns"])
                    self._send(socket, "vision/overlay/jpeg", overlay_header, image, self.quality)
                if npu:
                    npu_header = dict(common, schema="argus.vision.npu_input.v1",
                        t_capture_us=header["received_monotonic_ns"] // 1000, t_pre_us=pre_us,
                        src_w=transform.source_width, src_h=transform.source_height,
                        lb_scale=transform.scale, lb_pad_x=transform.pad_x, lb_pad_y=transform.pad_y,
                        preprocess="letterbox_bgr_to_rgb")
                    self._send(socket, "vision/npu_input/jpeg", npu_header,
                               cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), self.npu_quality)
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
