from argus_msg import detection_pb2 as _detection_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
PERCEPTION_OP_INFER: PerceptionOp
PERCEPTION_OP_READY: PerceptionOp
PERCEPTION_OP_SHUTDOWN: PerceptionOp
PERCEPTION_OP_UNSPECIFIED: PerceptionOp

class PerceptionReply(_message.Message):
    __slots__ = ["detections", "error", "frame_id", "infer_done_monotonic_ns", "inference_ms", "input_height", "input_width", "model", "ok", "preprocess_ms", "session_id"]
    DETECTIONS_FIELD_NUMBER: _ClassVar[int]
    ERROR_FIELD_NUMBER: _ClassVar[int]
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    INFERENCE_MS_FIELD_NUMBER: _ClassVar[int]
    INFER_DONE_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    INPUT_HEIGHT_FIELD_NUMBER: _ClassVar[int]
    INPUT_WIDTH_FIELD_NUMBER: _ClassVar[int]
    MODEL_FIELD_NUMBER: _ClassVar[int]
    OK_FIELD_NUMBER: _ClassVar[int]
    PREPROCESS_MS_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    detections: _containers.RepeatedCompositeFieldContainer[_detection_pb2.Detection]
    error: str
    frame_id: int
    infer_done_monotonic_ns: int
    inference_ms: float
    input_height: int
    input_width: int
    model: str
    ok: bool
    preprocess_ms: float
    session_id: str
    def __init__(self, session_id: _Optional[str] = ..., ok: bool = ..., error: _Optional[str] = ..., model: _Optional[str] = ..., input_width: _Optional[int] = ..., input_height: _Optional[int] = ..., frame_id: _Optional[int] = ..., infer_done_monotonic_ns: _Optional[int] = ..., preprocess_ms: _Optional[float] = ..., inference_ms: _Optional[float] = ..., detections: _Optional[_Iterable[_Union[_detection_pb2.Detection, _Mapping]]] = ...) -> None: ...

class PerceptionRequest(_message.Message):
    __slots__ = ["exposure_monotonic_ns", "frame_id", "height", "op", "pixel_format", "received_monotonic_ns", "received_unix_ns", "session_id", "stride", "width"]
    EXPOSURE_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    OP_FIELD_NUMBER: _ClassVar[int]
    PIXEL_FORMAT_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_UNIX_NS_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    STRIDE_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    exposure_monotonic_ns: int
    frame_id: int
    height: int
    op: PerceptionOp
    pixel_format: str
    received_monotonic_ns: int
    received_unix_ns: int
    session_id: str
    stride: int
    width: int
    def __init__(self, session_id: _Optional[str] = ..., op: _Optional[_Union[PerceptionOp, str]] = ..., frame_id: _Optional[int] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., stride: _Optional[int] = ..., pixel_format: _Optional[str] = ..., received_unix_ns: _Optional[int] = ..., received_monotonic_ns: _Optional[int] = ..., exposure_monotonic_ns: _Optional[int] = ...) -> None: ...

class PerceptionOp(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
