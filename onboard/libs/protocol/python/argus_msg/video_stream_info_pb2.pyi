from argus_msg import follow_command_pb2 as _follow_command_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
VIDEO_CODEC_H264: VideoCodec
VIDEO_CODEC_JPEG: VideoCodec
VIDEO_CODEC_UNSPECIFIED: VideoCodec

class VideoStreamInfo(_message.Message):
    __slots__ = ["published_monotonic_ns", "seq", "source", "streams"]
    class Stream(_message.Message):
        __slots__ = ["bitrate_bps", "codec", "framerate_hz", "height", "hfov_deg", "name", "rtsp_path", "rtsp_port", "running", "sensor", "stream_id", "width"]
        BITRATE_BPS_FIELD_NUMBER: _ClassVar[int]
        CODEC_FIELD_NUMBER: _ClassVar[int]
        FRAMERATE_HZ_FIELD_NUMBER: _ClassVar[int]
        HEIGHT_FIELD_NUMBER: _ClassVar[int]
        HFOV_DEG_FIELD_NUMBER: _ClassVar[int]
        NAME_FIELD_NUMBER: _ClassVar[int]
        RTSP_PATH_FIELD_NUMBER: _ClassVar[int]
        RTSP_PORT_FIELD_NUMBER: _ClassVar[int]
        RUNNING_FIELD_NUMBER: _ClassVar[int]
        SENSOR_FIELD_NUMBER: _ClassVar[int]
        STREAM_ID_FIELD_NUMBER: _ClassVar[int]
        WIDTH_FIELD_NUMBER: _ClassVar[int]
        bitrate_bps: int
        codec: VideoCodec
        framerate_hz: float
        height: int
        hfov_deg: float
        name: str
        rtsp_path: str
        rtsp_port: int
        running: bool
        sensor: _follow_command_pb2.Sensor
        stream_id: int
        width: int
        def __init__(self, stream_id: _Optional[int] = ..., sensor: _Optional[_Union[_follow_command_pb2.Sensor, str]] = ..., name: _Optional[str] = ..., rtsp_port: _Optional[int] = ..., rtsp_path: _Optional[str] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., framerate_hz: _Optional[float] = ..., codec: _Optional[_Union[VideoCodec, str]] = ..., bitrate_bps: _Optional[int] = ..., hfov_deg: _Optional[float] = ..., running: bool = ...) -> None: ...
    PUBLISHED_MONOTONIC_NS_FIELD_NUMBER: _ClassVar[int]
    SEQ_FIELD_NUMBER: _ClassVar[int]
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    STREAMS_FIELD_NUMBER: _ClassVar[int]
    published_monotonic_ns: int
    seq: int
    source: str
    streams: _containers.RepeatedCompositeFieldContainer[VideoStreamInfo.Stream]
    def __init__(self, source: _Optional[str] = ..., seq: _Optional[int] = ..., published_monotonic_ns: _Optional[int] = ..., streams: _Optional[_Iterable[_Union[VideoStreamInfo.Stream, _Mapping]]] = ...) -> None: ...

class VideoCodec(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
