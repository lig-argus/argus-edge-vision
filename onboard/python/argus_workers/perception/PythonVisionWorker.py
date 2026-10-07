"""Python image executor only. C++ owns frame source, queue, public observations and lifecycle.

No camera opening, tracking, FC access, flight command generation, or public PUB sockets here.
"""
import argparse
import dataclasses
import os
from pathlib import Path
import select
import signal
import sys
import time

import zmq

from .OverlayEncoder import OverlayEncoder
from .Preprocessor import decode_source_frame, letterbox_bgr_to_rgb
from .YOLOXDetector import HailoNmsDetector, HailoNmsDetectorSync, MockDetector
from ..transport.Protocol import pack_message, unpack_message

SCHEMA = "argus.ipc.perception.v1"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--endpoint", required=True)
    p.add_argument("--preview-endpoint", required=True)
    p.add_argument("--session-id", required=True)
    p.add_argument("--source", required=True)
    p.add_argument("--backend", choices=("mock", "hailo", "hailo-sync"), required=True)
    p.add_argument("--hef")
    p.add_argument("--labels", required=True)
    p.add_argument("--score-threshold", type=float, default=0.35)
    p.add_argument("--ir-preprocess", choices=("minmax", "fixed14"), default="minmax")
    p.add_argument("--publish-preview", action="store_true")
    p.add_argument("--publish-npu-input", action="store_true")
    p.add_argument("--preview-max-fps", type=float, default=10)
    p.add_argument("--jpeg-quality", type=int, default=75)
    p.add_argument("--npu-input-max-fps", type=float, default=15)
    p.add_argument("--npu-input-jpeg-quality", type=int, default=80)
    args = p.parse_args()
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.setsockopt(zmq.MAXMSGSIZE, 32 * 1024 * 1024)
    detector = overlay = None
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    last_frame = None
    try:
        socket.bind(args.endpoint)
        labels = [s.strip() for s in Path(args.labels).read_text().splitlines() if s.strip()]
        if args.backend == "mock":
            detector = MockDetector(labels)
        else:
            if not args.hef:
                raise ValueError("Hailo requires a HEF")
            detector = (HailoNmsDetectorSync if args.backend == "hailo-sync" else HailoNmsDetector)(
                args.hef, labels, args.score_threshold)
        if args.publish_preview or args.publish_npu_input:
            overlay = OverlayEncoder(args.preview_endpoint, args.source, args.session_id,
                args.publish_preview, args.publish_npu_input, args.preview_max_fps, args.jpeg_quality,
                args.npu_input_max_fps, args.npu_input_jpeg_quality)
        while not stopping:
            # Parent-owned stdin EOF also stops an idle worker after parent crash/exit.
            if select.select([sys.stdin], [], [], 0)[0] and not os.read(sys.stdin.fileno(), 1):
                break
            if not socket.poll(100, zmq.POLLIN):
                continue
            parts = socket.recv_multipart()
            reply = {"schema": SCHEMA, "session_id": args.session_id, "ok": False}
            try:
                h = unpack_message(parts[0])
                if h.get("schema") != SCHEMA or h.get("session_id") != args.session_id:
                    raise ValueError("request schema/session mismatch")
                op = h["op"]
                if op == "ready" and len(parts) == 1:
                    reply.update(ok=True, model=detector.model_name,
                                 input_width=detector.input_width, input_height=detector.input_height)
                elif op == "shutdown" and len(parts) == 1:
                    reply.update(ok=True)
                    stopping = True
                elif op == "infer" and len(parts) == 2:
                    seq = h["frame_id"]
                    if not isinstance(seq, int) or seq < 0 or (last_frame is not None and seq <= last_frame):
                        raise ValueError("frame sequence must advance")
                    if h["clock_domain"] != "host_monotonic":
                        raise ValueError("unmapped source clock")
                    exposure = h["exposure_monotonic_ns"]
                    if h["exposure_time_valid"] != (exposure is not None):
                        raise ValueError("inconsistent exposure validity")
                    if exposure is not None and not 0 <= exposure <= h["received_monotonic_ns"]:
                        raise ValueError("invalid exposure timestamp")
                    t0 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                    bgr = decode_source_frame(h, parts[1], args.ir_preprocess)
                    model_rgb, transform = letterbox_bgr_to_rgb(bgr, detector.input_width, detector.input_height)
                    t1 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                    detections = detector.infer(model_rgb, transform)
                    t2 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                    if overlay:
                        overlay.offer(h, bgr, detections, model_rgb, transform, t1 // 1000)
                    reply.update(ok=True, frame_id=seq, model=detector.model_name,
                        coordinate_frame="camera_pixel", infer_done_monotonic_ns=t2,
                        preprocess_ms=(t1 - t0) / 1e6, inference_ms=(t2 - t1) / 1e6,
                        detections=[dataclasses.asdict(d) for d in detections])
                    last_frame = seq
                else:
                    raise ValueError("invalid IPC operation/multipart")
            except Exception as exc:
                reply["error"] = str(exc)
            socket.send(pack_message(reply))
    finally:
        # Cleanup all owned resources even if one close fails.
        failures = []
        for resource in (overlay, detector):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    failures.append(exc)
        socket.close(linger=0)
        context.term()
        if failures:
            raise RuntimeError("worker cleanup failed") from failures[0]


if __name__ == "__main__":
    main()
