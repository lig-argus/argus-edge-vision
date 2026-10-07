"""MessagePack encoding for worker IPC and retained bus tools.

The authoritative public contract is cpp/contracts + docs/BUS_CONTRACT.md.
This module must not become a second perception/controller implementation.
"""
import msgpack


def pack_message(data: dict) -> bytes:
    return msgpack.packb(data, use_bin_type=True)


def unpack_message(payload: bytes) -> dict:
    value = msgpack.unpackb(payload, raw=False, strict_map_key=True)
    if not isinstance(value, dict):
        raise ValueError("message must be a map")
    return value
