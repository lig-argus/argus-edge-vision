"""Unresponsive test peer; never a registered production backend."""
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
while True:
    request = msgpack.unpackb(socket.recv_multipart()[0], raw=False)
    if request["op"] == "infer":
        time.sleep(30)
    reply = dict(schema="argus.ipc.perception.v1", session_id=args.session_id, ok=True,
                 input_width=640, input_height=640)
    socket.send(msgpack.packb(reply, use_bin_type=True))
    if request["op"] == "shutdown":
        break
socket.close(linger=0)
context.term()
