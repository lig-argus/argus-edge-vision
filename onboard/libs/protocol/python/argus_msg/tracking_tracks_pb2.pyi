from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
TRACK_STATE_CONFIRMED: TrackState
TRACK_STATE_PREDICTED: TrackState
TRACK_STATE_UNSPECIFIED: TrackState

class TrackingTracks(_message.Message):
    __slots__ = ["fc_epoch", "frame_id", "generation", "ground_pose", "image_height", "image_width", "motion_sample_ns", "sample_monotonic_ns", "seq", "session_id", "source", "tracks"]
    class Classification(_message.Message):
        __slots__ = ["class_id", "confidence", "label"]
        CLASS_ID_FIELD_NUMBER: _ClassVar[int]
        CONFIDENCE_FIELD_NUMBER: _ClassVar[int]
        LABEL_FIELD_NUMBER: _ClassVar[int]
        class_id: int
        confidence: float
        label: str
        def __init__(self, class_id: _Optional[int] = ..., label: _Optional[str] = ..., confidence: _Optional[float] = ...) -> None: ...
    class GroundAnchor(_message.Message):
        __slots__ = ["anchor_y_ratio", "height_m"]
        ANCHOR_Y_RATIO_FIELD_NUMBER: _ClassVar[int]
        HEIGHT_M_FIELD_NUMBER: _ClassVar[int]
        anchor_y_ratio: float
        height_m: float
        def __init__(self, anchor_y_ratio: _Optional[float] = ..., height_m: _Optional[float] = ...) -> None: ...
    class GroundEstimate(_message.Message):
        __slots__ = ["alignment_s", "cov_north_east_m2", "east_m", "north_m", "var_east_m2", "var_north_m2"]
        ALIGNMENT_S_FIELD_NUMBER: _ClassVar[int]
        COV_NORTH_EAST_M2_FIELD_NUMBER: _ClassVar[int]
        EAST_M_FIELD_NUMBER: _ClassVar[int]
        NORTH_M_FIELD_NUMBER: _ClassVar[int]
        VAR_EAST_M2_FIELD_NUMBER: _ClassVar[int]
        VAR_NORTH_M2_FIELD_NUMBER: _ClassVar[int]
        alignment_s: float
        cov_north_east_m2: float
        east_m: float
        north_m: float
        var_east_m2: float
        var_north_m2: float
        def __init__(self, north_m: _Optional[float] = ..., east_m: _Optional[float] = ..., var_north_m2: _Optional[float] = ..., cov_north_east_m2: _Optional[float] = ..., var_east_m2: _Optional[float] = ..., alignment_s: _Optional[float] = ...) -> None: ...
    class GroundPose(_message.Message):
        __slots__ = ["down_m", "east_m", "north_m", "pitch_rad", "roll_rad", "yaw_rad"]
        DOWN_M_FIELD_NUMBER: _ClassVar[int]
        EAST_M_FIELD_NUMBER: _ClassVar[int]
        NORTH_M_FIELD_NUMBER: _ClassVar[int]
        PITCH_RAD_FIELD_NUMBER: _ClassVar[int]
        ROLL_RAD_FIELD_NUMBER: _ClassVar[int]
        YAW_RAD_FIELD_NUMBER: _ClassVar[int]
        down_m: float
        east_m: float
        north_m: float
        pitch_rad: float
        roll_rad: float
        yaw_rad: float
        def __init__(self, north_m: _Optional[float] = ..., east_m: _Optional[float] = ..., down_m: _Optional[float] = ..., roll_rad: _Optional[float] = ..., pitch_rad: _Optional[float] = ..., yaw_rad: _Optional[float] = ...) -> None: ...
    class PixelBox(_message.Message):
        __slots__ = ["x1", "x2", "y1", "y2"]
        X1_FIELD_NUMBER: _ClassVar[int]
        X2_FIELD_NUMBER: _ClassVar[int]
        Y1_FIELD_NUMBER: _ClassVar[int]
        Y2_FIELD_NUMBER: _ClassVar[int]
        x1: float
        x2: float
        y1: float
        y2: float
        def __init__(self, x1: _Optional[float] = ..., y1: _Optional[float] = ..., x2: _Optional[float] = ..., y2: _Optional[float] = ...) -> None: ...
    class Track(_message.Message):
        __slots__ = ["box", "classification", "ground", "ground_anchor", "last_real_detection_monotonic_ns", "real_detection", "state", "track_id"]
        BOX_FIELD_NUMBER: _ClassVar[int]
        CLASSIFICATION_FIELD_NUMBER: _ClassVar[int]
        GROUND_ANCHOR_FIELD_NUMBER: _ClassVar[int]
        GROUND_FIELD_NUMBER: _ClassVar[int]
        LAST_REAL_DETECTION_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
        REAL_DETECTION_FIELD_NUMBER: _ClassVar[int]
        STATE_FIELD_NUMBER: _ClassVar[int]
        TRACK_ID_FIELD_NUMBER: _ClassVar[int]
        box: TrackingTracks.PixelBox
        classification: TrackingTracks.Classification
        ground: TrackingTracks.GroundEstimate
        ground_anchor: TrackingTracks.GroundAnchor
        last_real_detection_monotonic_ns: int
        real_detection: bool
        state: TrackState
        track_id: int
        def __init__(self, track_id: _Optional[int] = ..., classification: _Optional[_Union[TrackingTracks.Classification, _Mapping]] = ..., box: _Optional[_Union[TrackingTracks.PixelBox, _Mapping]] = ..., state: _Optional[_Union[TrackState, str]] = ..., real_detection: bool = ..., last_real_detection_monotonic_ns: _Optional[int] = ..., ground: _Optional[_Union[TrackingTracks.GroundEstimate, _Mapping]] = ..., ground_anchor: _Optional[_Union[TrackingTracks.GroundAnchor, _Mapping]] = ...) -> None: ...
    FC_EPOCH_FIELD_NUMBER: _ClassVar[int]
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    GENERATION_FIELD_NUMBER: _ClassVar[int]
    GROUND_POSE_FIELD_NUMBER: _ClassVar[int]
    IMAGE_HEIGHT_FIELD_NUMBER: _ClassVar[int]
    IMAGE_WIDTH_FIELD_NUMBER: _ClassVar[int]
    MOTION_SAMPLE_NS_FIELD_NUMBER: _ClassVar[int]
    SAMPLE_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    TRACKS_FIELD_NUMBER: _ClassVar[int]
    fc_epoch: int
    frame_id: int
    generation: int
    ground_pose: TrackingTracks.GroundPose
    image_height: int
    image_width: int
    motion_sample_ns: int
    sample_monotonic_ns: int
    seq: int
    session_id: str
    source: str
    tracks: _containers.RepeatedCompositeFieldContainer[TrackingTracks.Track]
    def __init__(self, source: _Optional[str] = ..., session_id: _Optional[str] = ..., frame_id: _Optional[int] = ..., seq: _Optional[int] = ..., fc_epoch: _Optional[int] = ..., generation: _Optional[int] = ..., sample_monotonic_ns: _Optional[int] = ..., motion_sample_ns: _Optional[int] = ..., image_width: _Optional[int] = ..., image_height: _Optional[int] = ..., ground_pose: _Optional[_Union[TrackingTracks.GroundPose, _Mapping]] = ..., tracks: _Optional[_Iterable[_Union[TrackingTracks.Track, _Mapping]]] = ...) -> None: ...

class TrackState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
