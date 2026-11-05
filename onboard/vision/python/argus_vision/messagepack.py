# C++ 어댑터와 주고받는 MessagePack map 의 인코딩.
# 출처: 실기체 RPi `src/argus/transport/protocol.py`.

from typing import Any

import msgpack


def pack_message(data: dict[str, Any]) -> bytes:
    """문자열 키의 map 하나를 MessagePack 으로 만든다 (bytes 는 bin 형).

    Args:
        data: 문자열 키의 map.

    Returns:
        MessagePack 바이트.
    """
    return bytes(msgpack.packb(data, use_bin_type=True))


def unpack_message(payload: bytes) -> dict[str, Any]:
    """MessagePack map 하나를 읽는다.

    Args:
        payload: MessagePack 바이트.

    Returns:
        문자열 키의 map.

    Raises:
        ValueError: map 이 아니거나 형식이 틀렸을 때.
    """
    value = msgpack.unpackb(payload, raw=False, strict_map_key=True)
    if not isinstance(value, dict):
        raise ValueError("message must be a map")
    return value
