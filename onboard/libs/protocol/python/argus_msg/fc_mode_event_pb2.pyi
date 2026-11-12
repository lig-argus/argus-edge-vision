from argus_msg import common_pb2 as _common_pb2
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
OFFBOARD_EXIT_CAUSE_FAILSAFE: OffboardExitCause
OFFBOARD_EXIT_CAUSE_NONE: OffboardExitCause
OFFBOARD_EXIT_CAUSE_PILOT_SWITCH: OffboardExitCause
OFFBOARD_EXIT_CAUSE_UNKNOWN: OffboardExitCause

class FcModeEvent(_message.Message):
    __slots__ = ["exit_cause", "mode", "received_monotonic_ns", "request_command_id", "seq"]
    EXIT_CAUSE_FIELD_NUMBER: _ClassVar[int]
    MODE_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    REQUEST_COMMAND_ID_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    exit_cause: OffboardExitCause
    mode: _common_pb2.FlightMode
    received_monotonic_ns: int
    request_command_id: int
    seq: int
    def __init__(self, seq: _Optional[int] = ..., mode: _Optional[_Union[_common_pb2.FlightMode, str]] = ..., request_command_id: _Optional[int] = ..., exit_cause: _Optional[_Union[OffboardExitCause, str]] = ..., received_monotonic_ns: _Optional[int] = ...) -> None: ...

class OffboardExitCause(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
