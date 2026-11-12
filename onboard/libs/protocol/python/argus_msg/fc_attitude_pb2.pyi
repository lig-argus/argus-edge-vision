from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class FcAttitude(_message.Message):
    __slots__ = ["fc_epoch", "fc_time_boot_ms", "fc_time_monotonic_ns", "pitch_rad", "pitch_rate_rad_s", "received_monotonic_ns", "roll_rad", "roll_rate_rad_s", "seq", "timesync_valid", "yaw_rad", "yaw_rate_rad_s"]
    FC_EPOCH_FIELD_NUMBER: _ClassVar[int]
    FC_TIME_BOOT_MS_FIELD_NUMBER: _ClassVar[int]
    FC_TIME_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    PITCH_RAD_FIELD_NUMBER: _ClassVar[int]
    PITCH_RATE_RAD_S_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    ROLL_RAD_FIELD_NUMBER: _ClassVar[int]
    ROLL_RATE_RAD_S_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    TIMESYNC_VALID_FIELD_NUMBER: _ClassVar[int]
    YAW_RAD_FIELD_NUMBER: _ClassVar[int]
    YAW_RATE_RAD_S_FIELD_NUMBER: _ClassVar[int]
    fc_epoch: int
    fc_time_boot_ms: int
    fc_time_monotonic_ns: int
    pitch_rad: float
    pitch_rate_rad_s: float
    received_monotonic_ns: int
    roll_rad: float
    roll_rate_rad_s: float
    seq: int
    timesync_valid: bool
    yaw_rad: float
    yaw_rate_rad_s: float
    def __init__(self, seq: _Optional[int] = ..., roll_rad: _Optional[float] = ..., pitch_rad: _Optional[float] = ..., yaw_rad: _Optional[float] = ..., roll_rate_rad_s: _Optional[float] = ..., pitch_rate_rad_s: _Optional[float] = ..., yaw_rate_rad_s: _Optional[float] = ..., fc_time_boot_ms: _Optional[int] = ..., received_monotonic_ns: _Optional[int] = ..., fc_time_monotonic_ns: _Optional[int] = ..., timesync_valid: bool = ..., fc_epoch: _Optional[int] = ...) -> None: ...
