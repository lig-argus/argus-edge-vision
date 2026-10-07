from __future__ import annotations

from collections.abc import Iterable

import zmq

from .Protocol import pack_message, unpack_message


DEFAULT_PUBLISH_ENDPOINT = "tcp://127.0.0.1:5555"
DEFAULT_SUBSCRIBE_ENDPOINT = "tcp://127.0.0.1:5556"


class Publisher:
    """Non-blocking PUB; delivery and newest-only consumption are not guaranteed."""

    def __init__(self, endpoint: str = DEFAULT_PUBLISH_ENDPOINT, high_water_mark: int = 4):
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.setsockopt(zmq.SNDHWM, high_water_mark)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(endpoint)

    def publish(self, topic: str, message: dict, payload: bytes | None = None) -> bool:
        frames = [topic.encode("utf-8"), pack_message(message)]
        if payload is not None:
            frames.append(payload)
        try:
            self._socket.send_multipart(frames, flags=zmq.NOBLOCK)
            return True
        except zmq.Again:
            return False

    def close(self) -> None:
        self._socket.close(linger=0)

    def __enter__(self) -> "Publisher":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


class Subscriber:
    def __init__(
        self,
        topics: Iterable[str],
        endpoint: str = DEFAULT_SUBSCRIBE_ENDPOINT,
        high_water_mark: int = 8,
    ):
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.RCVHWM, high_water_mark)
        self._socket.setsockopt(zmq.LINGER, 0)
        for topic in topics:
            self._socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self._socket.connect(endpoint)

    def receive(self, timeout_ms: int | None = None) -> tuple[str, dict, bytes | None]:
        if timeout_ms is not None and not self._socket.poll(timeout_ms, zmq.POLLIN):
            raise TimeoutError("no bus message received")
        frames = self._socket.recv_multipart()
        if len(frames) not in (2, 3):
            raise ValueError(f"expected 2 or 3 message frames, got {len(frames)}")
        topic = frames[0].decode("utf-8")
        header = unpack_message(frames[1])
        payload = frames[2] if len(frames) == 3 else None
        return topic, header, payload

    def close(self) -> None:
        self._socket.close(linger=0)

    def __enter__(self) -> "Subscriber":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
