from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class TrackingStatus(_message.Message):
    __slots__ = ["cmc_valid", "fc_fresh", "fps", "generation", "published_monotonic_ns", "seq", "source"]
    CMC_VALID_FIELD_NUMBER: _ClassVar[int]
    FC_FRESH_FIELD_NUMBER: _ClassVar[int]
    FPS_FIELD_NUMBER: _ClassVar[int]
    GENERATION_FIELD_NUMBER: _ClassVar[int]
    PUBLISHED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    cmc_valid: bool
    fc_fresh: bool
    fps: float
    generation: int
    published_monotonic_ns: int
    seq: int
    source: str
    def __init__(self, source: _Optional[str] = ..., seq: _Optional[int] = ..., published_monotonic_ns: _Optional[int] = ..., generation: _Optional[int] = ..., fc_fresh: bool = ..., cmc_valid: bool = ..., fps: _Optional[float] = ...) -> None: ...
