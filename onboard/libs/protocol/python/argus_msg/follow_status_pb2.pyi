from argus_msg import follow_command_pb2 as _follow_command_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
FOLLOW_ACTION_SEARCH: FollowAction
FOLLOW_ACTION_STOP: FollowAction
FOLLOW_ACTION_TRACK_START: FollowAction
FOLLOW_ACTION_UNSPECIFIED: FollowAction
MISSION_STATE_FAULT: MissionState
MISSION_STATE_FOLLOW_ACTIVE: MissionState
MISSION_STATE_HOLD_TRANSITION: MissionState
MISSION_STATE_HOLD_WAIT: MissionState
MISSION_STATE_IDLE: MissionState
MISSION_STATE_MANUAL: MissionState
MISSION_STATE_PREPARING: MissionState
MISSION_STATE_SCAN_ACTIVE: MissionState
MISSION_STATE_UNSPECIFIED: MissionState
TARGET_STATE_CONFIRMED: TargetState
TARGET_STATE_LOST: TargetState
TARGET_STATE_NONE: TargetState
TARGET_STATE_PREDICTED: TargetState
TARGET_STATE_UNSPECIFIED: TargetState

class FollowStatus(_message.Message):
    __slots__ = ["client_session", "command_epoch", "command_results", "command_sent", "controller_session", "fc_epoch", "mission_state", "onboard_session", "received_monotonic_ns", "scan", "selected", "seq", "session_id", "source", "target_state", "uid"]
    class CommandResult(_message.Message):
        __slots__ = ["action", "command_id", "outcome", "reason", "request_accepted", "target"]
        ACTION_FIELD_NUMBER: _ClassVar[int]
        COMMAND_ID_FIELD_NUMBER: _ClassVar[int]
        OUTCOME_FIELD_NUMBER: _ClassVar[int]
        REASON_FIELD_NUMBER: _ClassVar[int]
        REQUEST_ACCEPTED_FIELD_NUMBER: _ClassVar[int]
        TARGET_FIELD_NUMBER: _ClassVar[int]
        action: FollowAction
        command_id: int
        outcome: _follow_command_pb2.FollowOutcome
        reason: str
        request_accepted: bool
        target: FollowStatus.Target
        def __init__(self, command_id: _Optional[int] = ..., action: _Optional[_Union[FollowAction, str]] = ..., outcome: _Optional[_Union[_follow_command_pb2.FollowOutcome, str]] = ..., request_accepted: bool = ..., reason: _Optional[str] = ..., target: _Optional[_Union[FollowStatus.Target, _Mapping]] = ...) -> None: ...
    class Scan(_message.Message):
        __slots__ = ["progress_deg", "remaining_s", "sweep_rad", "timeout_s", "yaw_rate_rad_s"]
        PROGRESS_DEG_FIELD_NUMBER: _ClassVar[int]
        REMAINING_S_FIELD_NUMBER: _ClassVar[int]
        SWEEP_RAD_FIELD_NUMBER: _ClassVar[int]
        TIMEOUT_S_FIELD_NUMBER: _ClassVar[int]
        YAW_RATE_RAD_S_FIELD_NUMBER: _ClassVar[int]
        progress_deg: float
        remaining_s: float
        sweep_rad: float
        timeout_s: float
        yaw_rate_rad_s: float
        def __init__(self, yaw_rate_rad_s: _Optional[float] = ..., sweep_rad: _Optional[float] = ..., timeout_s: _Optional[float] = ..., progress_deg: _Optional[float] = ..., remaining_s: _Optional[float] = ...) -> None: ...
    class Target(_message.Message):
        __slots__ = ["class_id", "generation", "label", "sensor", "track_id"]
        CLASS_ID_FIELD_NUMBER: _ClassVar[int]
        GENERATION_FIELD_NUMBER: _ClassVar[int]
        LABEL_FIELD_NUMBER: _ClassVar[int]
        SENSOR_FIELD_NUMBER: _ClassVar[int]
        TRACK_ID_FIELD_NUMBER: _ClassVar[int]
        class_id: int
        generation: int
        label: str
        sensor: _follow_command_pb2.Sensor
        track_id: int
        def __init__(self, generation: _Optional[int] = ..., track_id: _Optional[int] = ..., class_id: _Optional[int] = ..., label: _Optional[str] = ..., sensor: _Optional[_Union[_follow_command_pb2.Sensor, str]] = ...) -> None: ...
    CLIENT_SESSION_FIELD_NUMBER: _ClassVar[int]
    COMMAND_EPOCH_FIELD_NUMBER: _ClassVar[int]
    COMMAND_RESULTS_FIELD_NUMBER: _ClassVar[int]
    COMMAND_SENT_FIELD_NUMBER: _ClassVar[int]
    CONTROLLER_SESSION_FIELD_NUMBER: _ClassVar[int]
    FC_EPOCH_FIELD_NUMBER: _ClassVar[int]
    MISSION_STATE_FIELD_NUMBER: _ClassVar[int]
    ONBOARD_SESSION_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    SCAN_FIELD_NUMBER: _ClassVar[int]
    SELECTED_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    TARGET_STATE_FIELD_NUMBER: _ClassVar[int]
    UID_FIELD_NUMBER: _ClassVar[int]
    client_session: str
    command_epoch: int
    command_results: _containers.RepeatedCompositeFieldContainer[FollowStatus.CommandResult]
    command_sent: bool
    controller_session: str
    fc_epoch: int
    mission_state: MissionState
    onboard_session: str
    received_monotonic_ns: int
    scan: FollowStatus.Scan
    selected: FollowStatus.Target
    seq: int
    session_id: str
    source: str
    target_state: TargetState
    uid: str
    def __init__(self, source: _Optional[str] = ..., seq: _Optional[int] = ..., session_id: _Optional[str] = ..., controller_session: _Optional[str] = ..., client_session: _Optional[str] = ..., onboard_session: _Optional[str] = ..., uid: _Optional[str] = ..., fc_epoch: _Optional[int] = ..., received_monotonic_ns: _Optional[int] = ..., mission_state: _Optional[_Union[MissionState, str]] = ..., target_state: _Optional[_Union[TargetState, str]] = ..., selected: _Optional[_Union[FollowStatus.Target, _Mapping]] = ..., command_results: _Optional[_Iterable[_Union[FollowStatus.CommandResult, _Mapping]]] = ..., command_epoch: _Optional[int] = ..., command_sent: bool = ..., scan: _Optional[_Union[FollowStatus.Scan, _Mapping]] = ...) -> None: ...

class MissionState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class TargetState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class FollowAction(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
