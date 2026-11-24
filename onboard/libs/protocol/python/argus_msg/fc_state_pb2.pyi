from argus_msg import common_pb2 as _common_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class FcState(_message.Message):
    __slots__ = ["downward_distance", "local", "seq", "status"]
    class DownwardDistance(_message.Message):
        __slots__ = ["distance_m", "fc_epoch", "received_monotonic_ns"]
        DISTANCE_M_FIELD_NUMBER: _ClassVar[int]
        FC_EPOCH_FIELD_NUMBER: _ClassVar[int]
        RECEIVED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
        distance_m: float
        fc_epoch: int
        received_monotonic_ns: int
        def __init__(self, distance_m: _Optional[float] = ..., received_monotonic_ns: _Optional[int] = ..., fc_epoch: _Optional[int] = ...) -> None: ...
    class FlightStatus(_message.Message):
        __slots__ = ["armed", "fc_epoch", "in_air", "mode", "offboard_hover", "received_monotonic_ns"]
        ARMED_FIELD_NUMBER: _ClassVar[int]
        FC_EPOCH_FIELD_NUMBER: _ClassVar[int]
        IN_AIR_FIELD_NUMBER: _ClassVar[int]
        MODE_FIELD_NUMBER: _ClassVar[int]
        OFFBOARD_HOVER_FIELD_NUMBER: _ClassVar[int]
        RECEIVED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
        armed: bool
        fc_epoch: int
        in_air: bool
        mode: _common_pb2.FlightMode
        offboard_hover: bool
        received_monotonic_ns: int
        def __init__(self, mode: _Optional[_Union[_common_pb2.FlightMode, str]] = ..., armed: bool = ..., in_air: bool = ..., received_monotonic_ns: _Optional[int] = ..., fc_epoch: _Optional[int] = ..., offboard_hover: bool = ...) -> None: ...
    class GlobalPosition(_message.Message):
        __slots__ = ["alt_m", "fc_epoch", "lat_deg", "lon_deg", "received_monotonic_ns"]
        ALT_M_FIELD_NUMBER: _ClassVar[int]
        FC_EPOCH_FIELD_NUMBER: _ClassVar[int]
        LAT_DEG_FIELD_NUMBER: _ClassVar[int]
        LON_DEG_FIELD_NUMBER: _ClassVar[int]
        RECEIVED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
        alt_m: float
        fc_epoch: int
        lat_deg: float
        lon_deg: float
        received_monotonic_ns: int
        def __init__(self, lat_deg: _Optional[float] = ..., lon_deg: _Optional[float] = ..., alt_m: _Optional[float] = ..., received_monotonic_ns: _Optional[int] = ..., fc_epoch: _Optional[int] = ...) -> None: ...
    class LocalPosition(_message.Message):
        __slots__ = ["down_m", "east_m", "fc_epoch", "north_m", "received_monotonic_ns", "vel_down_m_s", "vel_east_m_s", "vel_north_m_s"]
        DOWN_M_FIELD_NUMBER: _ClassVar[int]
        EAST_M_FIELD_NUMBER: _ClassVar[int]
        FC_EPOCH_FIELD_NUMBER: _ClassVar[int]
        NORTH_M_FIELD_NUMBER: _ClassVar[int]
        RECEIVED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
        VEL_DOWN_M_S_FIELD_NUMBER: _ClassVar[int]
        VEL_EAST_M_S_FIELD_NUMBER: _ClassVar[int]
        VEL_NORTH_M_S_FIELD_NUMBER: _ClassVar[int]
        down_m: float
        east_m: float
        fc_epoch: int
        north_m: float
        received_monotonic_ns: int
        vel_down_m_s: float
        vel_east_m_s: float
        vel_north_m_s: float
        def __init__(self, north_m: _Optional[float] = ..., east_m: _Optional[float] = ..., down_m: _Optional[float] = ..., vel_north_m_s: _Optional[float] = ..., vel_east_m_s: _Optional[float] = ..., vel_down_m_s: _Optional[float] = ..., received_monotonic_ns: _Optional[int] = ..., fc_epoch: _Optional[int] = ...) -> None: ...
    DOWNWARD_DISTANCE_FIELD_NUMBER: _ClassVar[int]
    GLOBAL_FIELD_NUMBER: _ClassVar[int]
    LOCAL_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    downward_distance: FcState.DownwardDistance
    local: FcState.LocalPosition
    seq: int
    status: FcState.FlightStatus
    def __init__(self, seq: _Optional[int] = ..., local: _Optional[_Union[FcState.LocalPosition, _Mapping]] = ..., downward_distance: _Optional[_Union[FcState.DownwardDistance, _Mapping]] = ..., status: _Optional[_Union[FcState.FlightStatus, _Mapping]] = ..., **kwargs) -> None: ...
