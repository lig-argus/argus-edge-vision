from __future__ import annotations

import argparse
import signal
import threading

import zmq


def run(frontend_endpoint: str, backend_endpoint: str) -> None:
    context = zmq.Context()
    frontend = context.socket(zmq.XSUB)
    backend = context.socket(zmq.XPUB)
    frontend.setsockopt(zmq.LINGER, 0)
    backend.setsockopt(zmq.LINGER, 0)
    stopped = threading.Event()

    def stop(_signum: int, _frame: object) -> None:
        stopped.set()

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        frontend.bind(frontend_endpoint)
        backend.bind(backend_endpoint)
        print(f"ARGUS bus: publishers -> {frontend.getsockopt_string(zmq.LAST_ENDPOINT)}, "
              f"subscribers <- {backend.getsockopt_string(zmq.LAST_ENDPOINT)}", flush=True)
        poller = zmq.Poller()
        poller.register(frontend, zmq.POLLIN)
        poller.register(backend, zmq.POLLIN)
        while not stopped.is_set():
            events = dict(poller.poll(50))
            if frontend in events:
                backend.send_multipart(frontend.recv_multipart())
            if backend in events:
                frontend.send_multipart(backend.recv_multipart())
    finally:
        frontend.close(linger=0)
        backend.close(linger=0)
        context.term()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main() -> None:
    parser = argparse.ArgumentParser(description="ARGUS ZeroMQ topic bus")
    parser.add_argument("--publish-bind", default="tcp://127.0.0.1:5555")
    parser.add_argument("--subscribe-bind", default="tcp://*:5556")
    args = parser.parse_args()
    run(args.publish_bind, args.subscribe_bind)


if __name__ == "__main__":
    main()
