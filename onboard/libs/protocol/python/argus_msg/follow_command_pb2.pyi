from argus_msg import common_pb2 as _common_pb2
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
FOLLOW_OUTCOME_FAILED: FollowOutcome
FOLLOW_OUTCOME_IN_PROGRESS: FollowOutcome
FOLLOW_OUTCOME_REJECTED: FollowOutcome
FOLLOW_OUTCOME_SUCCEEDED: FollowOutcome
FOLLOW_OUTCOME_UNSPECIFIED: FollowOutcome
SENSOR_EO: Sensor
SENSOR_IR: Sensor
SENSOR_UNKNOWN: Sensor
STOP_REASON_LINK_LOSS: StopReason
STOP_REASON_ONBOARD: StopReason
STOP_REASON_OPERATOR: StopReason
STOP_REASON_UNSPECIFIED: StopReason

class FollowCommand(_message.Message):
    __slots__ = ["header", "search", "stop", "track_start"]
    class Search(_message.Message):
        __slots__ = ["timeout_s", "yaw_range_rad", "yaw_rate_rad_s"]
        TIMEOUT_S_FIELD_NUMBER: _ClassVar[int]
        YAW_RANGE_RAD_FIELD_NUMBER: _ClassVar[int]
        YAW_RATE_RAD_S_FIELD_NUMBER: _ClassVar[int]
        timeout_s: float
        yaw_range_rad: float
        yaw_rate_rad_s: float
        def __init__(self, yaw_range_rad: _Optional[float] = ..., yaw_rate_rad_s: _Optional[float] = ..., timeout_s: _Optional[float] = ...) -> None: ...
    class Stop(_message.Message):
        __slots__ = ["reason"]
        REASON_FIELD_NUMBER: _ClassVar[int]
        reason: StopReason
        def __init__(self, reason: _Optional[_Union[StopReason, str]] = ...) -> None: ...
    class TrackStart(_message.Message):
        __slots__ = ["class_id", "generation", "label", "sensor", "track_id"]
        CLASS_ID_FIELD_NUMBER: _ClassVar[int]
        GENERATION_FIELD_NUMBER: _ClassVar[int]
        LABEL_FIELD_NUMBER: _ClassVar[int]
        SENSOR_FIELD_NUMBER: _ClassVar[int]
        TRACK_ID_FIELD_NUMBER: _ClassVar[int]
        class_id: int
        generation: int
        label: str
        sensor: Sensor
        track_id: int
        def __init__(self, sensor: _Optional[_Union[Sensor, str]] = ..., generation: _Optional[int] = ..., class_id: _Optional[int] = ..., track_id: _Optional[int] = ..., label: _Optional[str] = ...) -> None: ...
    HEADER_FIELD_NUMBER: _ClassVar[int]
    SEARCH_FIELD_NUMBER: _ClassVar[int]
    STOP_FIELD_NUMBER: _ClassVar[int]
    TRACK_START_FIELD_NUMBER: _ClassVar[int]
    header: _common_pb2.RequestHeader
    search: FollowCommand.Search
    stop: FollowCommand.Stop
    track_start: FollowCommand.TrackStart
    def __init__(self, header: _Optional[_Union[_common_pb2.RequestHeader, _Mapping]] = ..., track_start: _Optional[_Union[FollowCommand.TrackStart, _Mapping]] = ..., stop: _Optional[_Union[FollowCommand.Stop, _Mapping]] = ..., search: _Optional[_Union[FollowCommand.Search, _Mapping]] = ...) -> None: ...

class FollowResult(_message.Message):
    __slots__ = ["id", "outcome", "reason", "request_accepted"]
    ID_FIELD_NUMBER: _ClassVar[int]
    OUTCOME_FIELD_NUMBER: _ClassVar[int]
    REASON_FIELD_NUMBER: _ClassVar[int]
    REQUEST_ACCEPTED_FIELD_NUMBER: _ClassVar[int]
    id: _common_pb2.RequestId
    outcome: FollowOutcome
    reason: str
    request_accepted: bool
    def __init__(self, id: _Optional[_Union[_common_pb2.RequestId, _Mapping]] = ..., outcome: _Optional[_Union[FollowOutcome, str]] = ..., request_accepted: bool = ..., reason: _Optional[str] = ...) -> None: ...

class Sensor(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class StopReason(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class FollowOutcome(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
