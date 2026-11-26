#include "video_stream_info_codec.hpp"

#include <cstdint>
#include <optional>

#include "argus_msg/video_stream_info.pb.h"

namespace argus::video_api_bus {
namespace {

// Sensor 는 follow_command.proto 에 정의되어 있다. vehicle_api_bus 의 변환 함수는 그 라이브러리
// 안에서만 쓰는 헤더(src/)에 있으므로 이 라이브러리에 따로 둔다
msg::Sensor sensor_to_message(vehicle_api::Sensor sensor) {
    switch (sensor) {
        case vehicle_api::Sensor::Ir:
            return msg::SENSOR_IR;
        case vehicle_api::Sensor::Eo:
            return msg::SENSOR_EO;
        case vehicle_api::Sensor::Unknown:
            return msg::SENSOR_UNKNOWN;
    }
    return msg::SENSOR_UNKNOWN;
}

vehicle_api::Sensor sensor_from_message(msg::Sensor sensor) {
    switch (sensor) {
        case msg::SENSOR_IR:
            return vehicle_api::Sensor::Ir;
        case msg::SENSOR_EO:
            return vehicle_api::Sensor::Eo;
        default:
            return vehicle_api::Sensor::Unknown;
    }
}

msg::VideoCodec codec_to_message(video_api::VideoCodec codec) {
    switch (codec) {
        case video_api::VideoCodec::H264:
            return msg::VIDEO_CODEC_H264;
        case video_api::VideoCodec::Jpeg:
            return msg::VIDEO_CODEC_JPEG;
        case video_api::VideoCodec::Unknown:
            return msg::VIDEO_CODEC_UNSPECIFIED;
    }
    return msg::VIDEO_CODEC_UNSPECIFIED;
}

video_api::VideoCodec codec_from_message(msg::VideoCodec codec) {
    switch (codec) {
        case msg::VIDEO_CODEC_H264:
            return video_api::VideoCodec::H264;
        case msg::VIDEO_CODEC_JPEG:
            return video_api::VideoCodec::Jpeg;
        default:
            return video_api::VideoCodec::Unknown;
    }
}

void stream_to_message(const video_api::VideoStream& stream,
                       msg::VideoStreamInfo::Stream& message) {
    message.set_stream_id(stream.stream_id);
    message.set_sensor(sensor_to_message(stream.sensor));
    message.set_name(stream.name);
    message.set_rtsp_port(stream.rtsp_port);
    message.set_rtsp_path(stream.rtsp_path);
    message.set_width(stream.width);
    message.set_height(stream.height);
    message.set_framerate_hz(stream.framerate_hz);
    message.set_codec(codec_to_message(stream.codec));
    message.set_bitrate_bps(stream.bitrate_bps);
    message.set_hfov_deg(stream.hfov_deg);
    message.set_running(stream.running);
}

video_api::VideoStream stream_from_message(const msg::VideoStreamInfo::Stream& message) {
    video_api::VideoStream stream;
    stream.stream_id = message.stream_id();
    stream.sensor = sensor_from_message(message.sensor());
    stream.name = message.name();
    stream.rtsp_port = message.rtsp_port();
    stream.rtsp_path = message.rtsp_path();
    stream.width = message.width();
    stream.height = message.height();
    stream.framerate_hz = message.framerate_hz();
    stream.codec = codec_from_message(message.codec());
    stream.bitrate_bps = message.bitrate_bps();
    stream.hfov_deg = message.hfov_deg();
    stream.running = message.running();
    return stream;
}

}  // namespace

bus_zmq::Bytes encode_video_stream_info(const video_api::VideoStreamInfo& info, std::uint64_t seq) {
    msg::VideoStreamInfo message;
    message.set_source(info.source);
    message.set_seq(seq);
    message.set_published_monotonic_ns(info.published_monotonic_ns);
    for (const video_api::VideoStream& stream : info.streams) {
        stream_to_message(stream, *message.add_streams());
    }
    // 직렬화에 실패하는 경우는 메시지 크기가 2 GB 를 넘을 때뿐이며 이 메시지에서는 생기지 않는다.
    // 실패하면 빈 바이트를 보낸다
    bus_zmq::Bytes body(message.ByteSizeLong());
    if (!message.SerializeToArray(body.data(), static_cast<std::int32_t>(body.size()))) {
        body.clear();
    }
    return body;
}

std::optional<DecodedVideoStreamInfo> decode_video_stream_info(const bus_zmq::Bytes& body) {
    msg::VideoStreamInfo message;
    if (!message.ParseFromArray(body.data(), static_cast<std::int32_t>(body.size()))) {
        return std::nullopt;
    }
    DecodedVideoStreamInfo decoded;
    decoded.seq = message.seq();
    decoded.value.source = message.source();
    decoded.value.published_monotonic_ns = message.published_monotonic_ns();
    for (const msg::VideoStreamInfo::Stream& stream : message.streams()) {
        decoded.value.streams.push_back(stream_from_message(stream));
    }
    return decoded;
}

}  // namespace argus::video_api_bus
