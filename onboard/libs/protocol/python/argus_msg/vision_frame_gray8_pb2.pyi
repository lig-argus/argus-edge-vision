from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class VisionFrameGray8(_message.Message):
    __slots__ = ["frame_id", "height", "preprocess", "received_monotonic_ns", "seq", "session_id", "source", "stride", "unix_ns", "width"]
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    PREPROCESS_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    STRIDE_FIELD_NUMBER: _ClassVar[int]
    UNIX_NS_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    frame_id: int
    height: int
    preprocess: str
    received_monotonic_ns: int
    seq: int
    session_id: str
    source: str
    stride: int
    unix_ns: int
    width: int
    def __init__(self, source: _Optional[str] = ..., session_id: _Optional[str] = ..., frame_id: _Optional[int] = ..., seq: _Optional[int] = ..., unix_ns: _Optional[int] = ..., received_monotonic_ns: _Optional[int] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., stride: _Optional[int] = ..., preprocess: _Optional[str] = ...) -> None: ...
