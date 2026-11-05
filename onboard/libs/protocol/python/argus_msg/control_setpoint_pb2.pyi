from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class ControlSetpoint(_message.Message):
    __slots__ = ["down_m_s", "forward_m_s", "right_m_s", "seq", "valid_until_monotonic_ns", "yaw_rate_rad_s"]
    DOWN_M_S_FIELD_NUMBER: _ClassVar[int]
    FORWARD_M_S_FIELD_NUMBER: _ClassVar[int]
    RIGHT_M_S_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    VALID_UNTIL_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    YAW_RATE_RAD_S_FIELD_NUMBER: _ClassVar[int]
    down_m_s: float
    forward_m_s: float
    right_m_s: float
    seq: int
    valid_until_monotonic_ns: int
    yaw_rate_rad_s: float
    def __init__(self, seq: _Optional[int] = ..., forward_m_s: _Optional[float] = ..., right_m_s: _Optional[float] = ..., down_m_s: _Optional[float] = ..., yaw_rate_rad_s: _Optional[float] = ..., valid_until_monotonic_ns: _Optional[int] = ...) -> None: ...
