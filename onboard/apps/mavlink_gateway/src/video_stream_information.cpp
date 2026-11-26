#include "argus/mavlink_gateway/video_stream_information.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/mavlink_ids.hpp"

namespace argus::mavlink_gateway {
namespace {

using NameField = std::array<char, MAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_NAME_LEN>;
using UriField = std::array<char, MAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_URI_LEN>;

// MAVLink 의 stream id 는 uint8 이다
constexpr float kMaxStreamId = 255.0F;

std::optional<std::uint32_t> stream_id_of(float param2) {
    if (!std::isfinite(param2) || param2 < 0.0F || param2 > kMaxStreamId ||
        std::trunc(param2) != param2) {
        return std::nullopt;
    }
    return static_cast<std::uint32_t>(param2);
}

std::uint16_t to_uint16(std::uint32_t value) {
    return static_cast<std::uint16_t>(
        std::min<std::uint32_t>(value, std::numeric_limits<std::uint16_t>::max()));
}

// 화각(deg)을 정수로 반올림한다. 유한하지 않거나 음수면 0 (모름)
std::uint16_t hfov_of(double hfov_deg) {
    if (!std::isfinite(hfov_deg) || hfov_deg <= 0.0) {
        return 0;
    }
    const double clamped = std::min(std::round(hfov_deg),
                                    static_cast<double>(std::numeric_limits<std::uint16_t>::max()));
    return static_cast<std::uint16_t>(clamped);
}

std::uint16_t flags_of(const video_api::VideoStream& stream) {
    std::uint16_t flags = 0;
    if (stream.running) {
        flags |= VIDEO_STREAM_STATUS_FLAGS_RUNNING;
    }
    if (stream.sensor == vehicle_api::Sensor::Ir) {
        flags |= VIDEO_STREAM_STATUS_FLAGS_THERMAL;
    }
    return flags;
}

// MAVLink 에 JPEG 값이 없으므로 H.264 가 아니면 UNKNOWN (video.md 7장)
VIDEO_STREAM_ENCODING encoding_of(video_api::VideoCodec codec) {
    return codec == video_api::VideoCodec::H264 ? VIDEO_STREAM_ENCODING_H264
                                                : VIDEO_STREAM_ENCODING_UNKNOWN;
}

// 필드보다 긴 이름은 필드 길이에서 자른다. 필드를 다 채우면 끝의 0 이 없어도 된다 (MAVLink 문자열
// 규칙)
NameField name_field(const std::string& name) {
    NameField field{};
    name.copy(field.data(), std::min(name.size(), field.size()));
    return field;
}

// uri 가 필드보다 길면 nullopt. 잘린 주소로는 접속할 수 없기 때문이다
std::optional<UriField> uri_field(const std::string& host, const video_api::VideoStream& stream) {
    const std::string uri =
        "rtsp://" + host + ":" + std::to_string(stream.rtsp_port) + stream.rtsp_path;
    if (uri.size() > UriField{}.size()) {
        return std::nullopt;
    }
    UriField field{};
    uri.copy(field.data(), uri.size());
    return field;
}

RequestedMessages rejected(MAV_RESULT result, std::string reason) {
    RequestedMessages rejected;
    rejected.result = result;
    rejected.reason = std::move(reason);
    return rejected;
}

}  // namespace

RequestedMessages build_video_stream_information(const CommandRequest& request,
                                                 const video_api::IVideoStreamInfoSource& source,
                                                 const IGcsLink& link) {
    const std::optional<video_api::ReceivedVideoStreamInfo> received = source.latest();
    if (!received.has_value() || received->info.streams.empty()) {
        return rejected(MAV_RESULT_DENIED, "no video stream info");
    }
    const std::optional<std::uint32_t> stream_id = stream_id_of(request.command.param2);
    if (!stream_id.has_value()) {
        return rejected(MAV_RESULT_DENIED, "p2 must be stream id");
    }
    const std::vector<video_api::VideoStream>& streams = received->info.streams;
    std::vector<const video_api::VideoStream*> selected;
    for (const video_api::VideoStream& stream : streams) {
        if (*stream_id == 0 || stream.stream_id == *stream_id) {
            selected.push_back(&stream);
        }
    }
    if (selected.empty()) {
        return rejected(MAV_RESULT_DENIED, "no such video stream");
    }
    const std::optional<std::string> host = link.local_address(request.sender);
    if (!host.has_value()) {
        return rejected(MAV_RESULT_FAILED, "no local address");
    }

    RequestedMessages built;
    built.result = MAV_RESULT_ACCEPTED;
    const auto count = static_cast<std::uint8_t>(
        std::min<std::size_t>(streams.size(), std::numeric_limits<std::uint8_t>::max()));
    for (const video_api::VideoStream* stream : selected) {
        const std::optional<UriField> uri = uri_field(*host, *stream);
        if (!uri.has_value()) {
            return rejected(MAV_RESULT_FAILED, "video uri too long");
        }
        const NameField name = name_field(stream->name);
        mavlink_message_t message{};
        mavlink_msg_video_stream_information_pack_chan(
            kVehicleSysid, kOnboardCommandCompid, kCommandChannel, &message,
            static_cast<std::uint8_t>(stream->stream_id), count, VIDEO_STREAM_TYPE_RTSP,
            flags_of(*stream), static_cast<float>(stream->framerate_hz), to_uint16(stream->width),
            to_uint16(stream->height), stream->bitrate_bps, 0, hfov_of(stream->hfov_deg),
            name.data(), uri->data(), encoding_of(stream->codec), 0);
        built.messages.push_back(message);
    }
    return built;
}

}  // namespace argus::mavlink_gateway
