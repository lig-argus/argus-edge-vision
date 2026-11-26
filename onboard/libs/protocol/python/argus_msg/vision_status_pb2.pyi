from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class VisionStatus(_message.Message):
    __slots__ = ["camera_ok", "image_height", "image_width", "infer_fps", "inference_ok", "model_ver", "published_monotonic_ns", "seq", "source"]
    CAMERA_OK_FIELD_NUMBER: _ClassVar[int]
    IMAGE_HEIGHT_FIELD_NUMBER: _ClassVar[int]
    IMAGE_WIDTH_FIELD_NUMBER: _ClassVar[int]
    INFERENCE_OK_FIELD_NUMBER: _ClassVar[int]
    INFER_FPS_FIELD_NUMBER: _ClassVar[int]
    MODEL_VER_FIELD_NUMBER: _ClassVar[int]
    PUBLISHED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    camera_ok: bool
    image_height: int
    image_width: int
    infer_fps: float
    inference_ok: bool
    model_ver: int
    published_monotonic_ns: int
    seq: int
    source: str
    def __init__(self, source: _Optional[str] = ..., seq: _Optional[int] = ..., published_monotonic_ns: _Optional[int] = ..., camera_ok: bool = ..., inference_ok: bool = ..., infer_fps: _Optional[float] = ..., model_ver: _Optional[int] = ..., image_width: _Optional[int] = ..., image_height: _Optional[int] = ...) -> None: ...
