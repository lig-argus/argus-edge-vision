from argus_msg import detection_pb2 as _detection_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class VisionOverlayJpeg(_message.Message):
    __slots__ = ["backend", "detections", "frame_id", "height", "model", "preprocess", "received_monotonic_ns", "seq", "session_id", "source", "unix_ns", "width"]
    BACKEND_FIELD_NUMBER: _ClassVar[int]
    DETECTIONS_FIELD_NUMBER: _ClassVar[int]
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    MODEL_FIELD_NUMBER: _ClassVar[int]
    PREPROCESS_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    UNIX_NS_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    backend: str
    detections: _containers.RepeatedCompositeFieldContainer[_detection_pb2.Detection]
    frame_id: int
    height: int
    model: str
    preprocess: str
    received_monotonic_ns: int
    seq: int
    session_id: str
    source: str
    unix_ns: int
    width: int
    def __init__(self, source: _Optional[str] = ..., session_id: _Optional[str] = ..., frame_id: _Optional[int] = ..., seq: _Optional[int] = ..., unix_ns: _Optional[int] = ..., received_monotonic_ns: _Optional[int] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., backend: _Optional[str] = ..., model: _Optional[str] = ..., preprocess: _Optional[str] = ..., detections: _Optional[_Iterable[_Union[_detection_pb2.Detection, _Mapping]]] = ...) -> None: ...
