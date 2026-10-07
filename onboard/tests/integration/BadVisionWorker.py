"""Fault-injection test peer; never a registered production backend."""
import argparse
import time
import msgpack
import zmq

p = argparse.ArgumentParser()
p.add_argument("--endpoint", required=True)
p.add_argument("--session-id", required=True)
args, _ = p.parse_known_args()
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.setsockopt(zmq.LINGER, 0)
socket.bind(args.endpoint)
try:
    while True:
        request = msgpack.unpackb(socket.recv_multipart()[0], raw=False)
        reply = dict(schema="argus.ipc.perception.v1", session_id=args.session_id, ok=True)
        op = request["op"]
        if op == "ready":
            reply.update(input_width=640, input_height=640)
        elif op == "infer":
            reply.update(frame_id=request["frame_id"] + 1, model="bad-test-peer", coordinate_frame="camera_pixel",
                infer_done_monotonic_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC),
                preprocess_ms=0.0, inference_ms=0.0, detections=[])
        socket.send(msgpack.packb(reply, use_bin_type=True))
        if op == "shutdown":
            break
finally:
    socket.close(linger=0)
    context.term()
