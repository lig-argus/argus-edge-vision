from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
FLIGHT_MODE_ALTITUDE: FlightMode
FLIGHT_MODE_HOLD: FlightMode
FLIGHT_MODE_LAND: FlightMode
FLIGHT_MODE_MANUAL: FlightMode
FLIGHT_MODE_MISSION: FlightMode
FLIGHT_MODE_OFFBOARD: FlightMode
FLIGHT_MODE_POSITION: FlightMode
FLIGHT_MODE_RETURN: FlightMode
FLIGHT_MODE_STABILIZED: FlightMode
FLIGHT_MODE_TAKEOFF: FlightMode
FLIGHT_MODE_UNKNOWN: FlightMode

class RequestHeader(_message.Message):
    __slots__ = ["id", "valid_until_monotonic_ns"]
    ID_FIELD_NUMBER: _ClassVar[int]
    VALID_UNTIL_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    id: RequestId
    valid_until_monotonic_ns: int
    def __init__(self, id: _Optional[_Union[RequestId, _Mapping]] = ..., valid_until_monotonic_ns: _Optional[int] = ...) -> None: ...

class RequestId(_message.Message):
    __slots__ = ["command_id", "fc_epoch", "seq", "session_id"]
    COMMAND_ID_FIELD_NUMBER: _ClassVar[int]
    FC_EPOCH_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    command_id: int
    fc_epoch: int
    seq: int
    session_id: str
    def __init__(self, command_id: _Optional[int] = ..., session_id: _Optional[str] = ..., seq: _Optional[int] = ..., fc_epoch: _Optional[int] = ...) -> None: ...

class FlightMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
