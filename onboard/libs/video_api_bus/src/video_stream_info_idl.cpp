#include "video_stream_info_idl.hpp"

namespace argus::video_api_bus {
namespace {

// Sensor 는 follow_command.idl 에 정의되어 있다. vehicle_api_bus 의 변환 함수는 그 라이브러리
// 안에서만 쓰는 헤더(src/)에 있으므로 이 라이브러리에 따로 둔다
idl::Sensor sensor_to_idl(vehicle_api::Sensor sensor) {
    switch (sensor) {
        case vehicle_api::Sensor::Ir:
            return idl::Sensor::SENSOR_IR;
        case vehicle_api::Sensor::Eo:
            return idl::Sensor::SENSOR_EO;
        case vehicle_api::Sensor::Unknown:
            return idl::Sensor::SENSOR_UNKNOWN;
    }
    return idl::Sensor::SENSOR_UNKNOWN;
}

vehicle_api::Sensor sensor_from_idl(idl::Sensor sensor) {
    switch (sensor) {
        case idl::Sensor::SENSOR_IR:
            return vehicle_api::Sensor::Ir;
        case idl::Sensor::SENSOR_EO:
            return vehicle_api::Sensor::Eo;
        case idl::Sensor::SENSOR_UNKNOWN:
            return vehicle_api::Sensor::Unknown;
    }
    return vehicle_api::Sensor::Unknown;
}

idl::VideoCodec codec_to_idl(video_api::VideoCodec codec) {
    switch (codec) {
        case video_api::VideoCodec::H264:
            return idl::VideoCodec::VIDEO_CODEC_H264;
        case video_api::VideoCodec::Jpeg:
            return idl::VideoCodec::VIDEO_CODEC_JPEG;
        case video_api::VideoCodec::Unknown:
            return idl::VideoCodec::VIDEO_CODEC_UNSPECIFIED;
    }
    return idl::VideoCodec::VIDEO_CODEC_UNSPECIFIED;
}

video_api::VideoCodec codec_from_idl(idl::VideoCodec codec) {
    switch (codec) {
        case idl::VideoCodec::VIDEO_CODEC_H264:
            return video_api::VideoCodec::H264;
        case idl::VideoCodec::VIDEO_CODEC_JPEG:
            return video_api::VideoCodec::Jpeg;
        case idl::VideoCodec::VIDEO_CODEC_UNSPECIFIED:
            return video_api::VideoCodec::Unknown;
    }
    return video_api::VideoCodec::Unknown;
}

idl::VideoStreamInfo_Stream stream_to_idl(const video_api::VideoStream& stream) {
    idl::VideoStreamInfo_Stream sample;
    sample.stream_id(stream.stream_id);
    sample.sensor(sensor_to_idl(stream.sensor));
    sample.name(stream.name);
    sample.rtsp_port(stream.rtsp_port);
    sample.rtsp_path(stream.rtsp_path);
    sample.width(stream.width);
    sample.height(stream.height);
    sample.framerate_hz(stream.framerate_hz);
    sample.codec(codec_to_idl(stream.codec));
    sample.bitrate_bps(stream.bitrate_bps);
    sample.hfov_deg(stream.hfov_deg);
    sample.running(stream.running);
    return sample;
}

video_api::VideoStream stream_from_idl(const idl::VideoStreamInfo_Stream& sample) {
    video_api::VideoStream stream;
    stream.stream_id = sample.stream_id();
    stream.sensor = sensor_from_idl(sample.sensor());
    stream.name = sample.name();
    stream.rtsp_port = sample.rtsp_port();
    stream.rtsp_path = sample.rtsp_path();
    stream.width = sample.width();
    stream.height = sample.height();
    stream.framerate_hz = sample.framerate_hz();
    stream.codec = codec_from_idl(sample.codec());
    stream.bitrate_bps = sample.bitrate_bps();
    stream.hfov_deg = sample.hfov_deg();
    stream.running = sample.running();
    return stream;
}

}  // namespace

void to_idl_video_stream_info(const video_api::VideoStreamInfo& info,
                              idl::VideoStreamInfo& sample) {
    sample.source(info.source);
    sample.published_monotonic_ns(info.published_monotonic_ns);
    sample.streams().clear();
    for (const video_api::VideoStream& stream : info.streams) {
        sample.streams().push_back(stream_to_idl(stream));
    }
}

bool from_idl_video_stream_info(const idl::VideoStreamInfo& sample,
                                video_api::VideoStreamInfo& info) {
    info = video_api::VideoStreamInfo{};
    info.source = sample.source();
    info.published_monotonic_ns = sample.published_monotonic_ns();
    for (const idl::VideoStreamInfo_Stream& stream : sample.streams()) {
        info.streams.push_back(stream_from_idl(stream));
    }
    return true;
}

}  // namespace argus::video_api_bus
