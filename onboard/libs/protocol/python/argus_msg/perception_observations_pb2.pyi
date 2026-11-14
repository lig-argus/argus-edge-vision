from argus_msg import detection_pb2 as _detection_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class PerceptionObservations(_message.Message):
    __slots__ = ["detections", "exposure_monotonic_ns", "frame_id", "image_height", "image_width", "inference_ms", "model", "published_monotonic_ns", "received_monotonic_ns", "received_unix_ns", "seq", "session_id", "source", "source_receive_monotonic_ns"]
    DETECTIONS_FIELD_NUMBER: _ClassVar[int]
    EXPOSURE_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    IMAGE_HEIGHT_FIELD_NUMBER: _ClassVar[int]
    IMAGE_WIDTH_FIELD_NUMBER: _ClassVar[int]
    INFERENCE_MS_FIELD_NUMBER: _ClassVar[int]
    MODEL_FIELD_NUMBER: _ClassVar[int]
    PUBLISHED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_UNIX_NS_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    SOURCE_RECEIVE_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    detections: _containers.RepeatedCompositeFieldContainer[_detection_pb2.Detection]
    exposure_monotonic_ns: int
    frame_id: int
    image_height: int
    image_width: int
    inference_ms: float
    model: str
    published_monotonic_ns: int
    received_monotonic_ns: int
    received_unix_ns: int
    seq: int
    session_id: str
    source: str
    source_receive_monotonic_ns: int
    def __init__(self, source: _Optional[str] = ..., session_id: _Optional[str] = ..., frame_id: _Optional[int] = ..., seq: _Optional[int] = ..., image_width: _Optional[int] = ..., image_height: _Optional[int] = ..., model: _Optional[str] = ..., inference_ms: _Optional[float] = ..., received_unix_ns: _Optional[int] = ..., received_monotonic_ns: _Optional[int] = ..., source_receive_monotonic_ns: _Optional[int] = ..., exposure_monotonic_ns: _Optional[int] = ..., published_monotonic_ns: _Optional[int] = ..., detections: _Optional[_Iterable[_Union[_detection_pb2.Detection, _Mapping]]] = ...) -> None: ...
