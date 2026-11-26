#include "argus/gcs/link/video_stream_request.hpp"

#include <QString>
#include <algorithm>
#include <cstddef>
#include <iterator>
#include <utility>

namespace argus::gcs::link {

namespace {

// REQUEST_MESSAGE 의 param2: 모든 스트림
constexpr float kAllStreams = 0.0F;

// 생성된 구조체의 char 배열을 글로 바꾼다. 배열을 다 채우면 끝에 NUL 이 없다
template <std::size_t N>
QString fieldText(
    const char (&chars)[N]) {  // NOLINT(modernize-avoid-c-arrays): 생성된 구조체의 필드
    const char* end = std::find(std::begin(chars), std::end(chars), '\0');
    return QString::fromUtf8(chars, std::distance(std::begin(chars), end));
}

bool fromGateway(const mavlink_message_t& message) {
    return message.sysid == kVehicleSysid && message.compid == kGatewayCompid;
}

}  // namespace

std::optional<VideoStreamUri> decodeVideoStream(const mavlink_video_stream_information_t& info) {
    if (info.type != VIDEO_STREAM_TYPE_RTSP) {
        return std::nullopt;
    }
    QString uri = fieldText(info.uri);
    if (!uri.startsWith(QStringLiteral("rtsp://"))) {
        return std::nullopt;
    }
    const bool thermal = (info.flags & VIDEO_STREAM_STATUS_FLAGS_THERMAL) != 0;
    const auto sensor = static_cast<std::uint8_t>(thermal ? ARGUS_SENSOR_IR : ARGUS_SENSOR_EO);
    return VideoStreamUri{sensor, std::move(uri), fieldText(info.name), info.encoding};
}

void VideoStreamRequest::begin(std::int64_t nowNs) {
    waiting_ = true;
    sends_ = 0;
    lastSendNs_ = nowNs - kRetryIntervalNs;  // 바로 보낼 수 있게
    received_.clear();
}

std::optional<mavlink_message_t> VideoStreamRequest::sendDue(std::int64_t nowNs) {
    if (!waiting_ || sends_ >= kMaxSends || nowNs - lastSendNs_ < kRetryIntervalNs) {
        return std::nullopt;
    }
    ++sends_;
    lastSendNs_ = nowNs;
    return requestMessage();
}

mavlink_message_t VideoStreamRequest::requestMessage() const {
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(sysid_, kGcsCompid, &message, kVehicleSysid, kGatewayCompid,
                                  MAV_CMD_REQUEST_MESSAGE, 0,
                                  static_cast<float>(MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION),
                                  kAllStreams, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    return message;
}

VideoStreamRequest::Outcome VideoStreamRequest::check(std::int64_t nowNs) {
    if (!waiting_ || sends_ < kMaxSends || nowNs - lastSendNs_ < kRetryIntervalNs) {
        return Outcome::None;
    }
    waiting_ = false;
    return Outcome::NoReply;
}

VideoStreamRequest::Outcome VideoStreamRequest::onCommandAck(const mavlink_message_t& message) {
    if (message.msgid != MAVLINK_MSG_ID_COMMAND_ACK || !fromGateway(message)) {
        return Outcome::None;
    }
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message, &ack);
    // target_system 0 은 대상을 정하지 않은 응답이다
    const bool forOther = ack.target_system != 0 && ack.target_system != sysid_;
    if (!waiting_ || forOther || ack.command != MAV_CMD_REQUEST_MESSAGE ||
        ack.result == MAV_RESULT_ACCEPTED || ack.result == MAV_RESULT_IN_PROGRESS) {
        return Outcome::None;
    }
    waiting_ = false;
    return Outcome::Denied;
}

std::optional<VideoStreams> VideoStreamRequest::onStreamInformation(
    const mavlink_message_t& message) {
    if (message.msgid != MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION || !fromGateway(message)) {
        return std::nullopt;
    }
    mavlink_video_stream_information_t info{};
    mavlink_msg_video_stream_information_decode(&message, &info);
    if (!waiting_ || info.stream_id == 0 || info.count == 0) {
        return std::nullopt;
    }
    received_[info.stream_id] = decodeVideoStream(info);
    if (received_.size() < info.count) {
        return std::nullopt;
    }
    waiting_ = false;
    return collect();
}

VideoStreams VideoStreamRequest::collect() const {
    VideoStreams out;
    for (const auto& entry : received_) {
        const std::optional<VideoStreamUri>& stream = entry.second;
        if (!stream) {
            continue;
        }
        const std::uint8_t sensor = stream->sensor;
        const bool known = std::any_of(out.begin(), out.end(), [sensor](const VideoStreamUri& v) {
            return v.sensor == sensor;
        });
        if (!known) {
            out.push_back(*stream);  // stream_id 순서로 돌므로 같은 센서는 작은 번호가 남는다
        }
    }
    std::sort(out.begin(), out.end(),
              [](const VideoStreamUri& a, const VideoStreamUri& b) { return a.sensor < b.sensor; });
    return out;
}

}  // namespace argus::gcs::link
