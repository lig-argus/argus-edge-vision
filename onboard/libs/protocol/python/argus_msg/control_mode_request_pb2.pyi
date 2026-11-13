from argus_msg import common_pb2 as _common_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class ControlModeReply(_message.Message):
    __slots__ = ["accepted", "id", "reason"]
    ACCEPTED_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    REASON_FIELD_NUMBER: _ClassVar[int]
    accepted: bool
    id: _common_pb2.RequestId
    reason: str
    def __init__(self, id: _Optional[_Union[_common_pb2.RequestId, _Mapping]] = ..., accepted: bool = ..., reason: _Optional[str] = ...) -> None: ...

class ControlModeRequest(_message.Message):
    __slots__ = ["header", "hold", "offboard"]
    class Hold(_message.Message):
        __slots__ = []
        def __init__(self) -> None: ...
    class Offboard(_message.Message):
        __slots__ = []
        def __init__(self) -> None: ...
    HEADER_FIELD_NUMBER: _ClassVar[int]
    HOLD_FIELD_NUMBER: _ClassVar[int]
    OFFBOARD_FIELD_NUMBER: _ClassVar[int]
    header: _common_pb2.RequestHeader
    hold: ControlModeRequest.Hold
    offboard: ControlModeRequest.Offboard
    def __init__(self, header: _Optional[_Union[_common_pb2.RequestHeader, _Mapping]] = ..., offboard: _Optional[_Union[ControlModeRequest.Offboard, _Mapping]] = ..., hold: _Optional[_Union[ControlModeRequest.Hold, _Mapping]] = ...) -> None: ...
