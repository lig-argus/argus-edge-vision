#include "argus/video_sender/video_stream_info_publisher.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace argus::video_sender {
namespace {

std::int64_t to_ns(std::chrono::milliseconds duration) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

video_api::VideoCodec to_message_codec(VideoCodec codec) {
    switch (codec) {
        case VideoCodec::H264:
            return video_api::VideoCodec::H264;
        case VideoCodec::Jpeg:
            return video_api::VideoCodec::Jpeg;
    }
    return video_api::VideoCodec::Unknown;
}

// 설정이 없거나 bit/s 로 uint32 범위를 넘으면 0 (모름)
std::uint32_t to_bitrate_bps(const std::optional<std::int32_t>& bitrate_kbps) {
    constexpr std::int64_t kBitsPerKilobit = 1000;
    if (!bitrate_kbps.has_value() || bitrate_kbps.value() <= 0) {
        return 0;
    }
    const std::int64_t bps = static_cast<std::int64_t>(bitrate_kbps.value()) * kBitsPerKilobit;
    if (bps > std::numeric_limits<std::uint32_t>::max()) {
        return 0;
    }
    return static_cast<std::uint32_t>(bps);
}

// RtspServer 가 크기가 맞는 프레임으로 받은 수. 크기가 맞지 않는 프레임은 송신할 수 없으므로 세지
// 않는다
std::uint64_t accepted_frames(const RtspServerStats& stats) {
    return stats.sent + stats.dropped_no_client + stats.dropped_not_playing +
           stats.dropped_send_failed;
}

}  // namespace

VideoStreamInfoPublisher::VideoStreamInfoPublisher(
    video_api_bus::BusVideoStreamInfoPublisher& publisher, const core::IClock& clock,
    std::string source, VideoStreamDescription description, std::string rtsp_path,
    const VideoEncodeConfig& encode)
    : publisher_(publisher),
      clock_(clock),
      source_(std::move(source)),
      description_(std::move(description)),
      rtsp_path_(std::move(rtsp_path)),
      codec_(to_message_codec(encode.codec)),
      bitrate_bps_(to_bitrate_bps(encode.bitrate_kbps)) {}

void VideoStreamInfoPublisher::tick(std::uint16_t port, const RtspServerStats& stats) {
    const std::int64_t now_ns = clock_.now_monotonic_ns();
    const std::uint64_t frames = accepted_frames(stats);
    if (frames != accepted_frames_) {
        accepted_frames_ = frames;
        last_frame_seen_ns_ = now_ns;
    }
    if (port == 0) {
        return;
    }
    const bool running = last_frame_seen_ns_.has_value() &&
                         now_ns - last_frame_seen_ns_.value() < to_ns(kRunningWindow);
    const video_api::VideoStream stream = make_stream(port, running);
    const bool changed = !last_published_.has_value() || last_published_.value() != stream;
    if (!changed && now_ns - last_published_ns_ < to_ns(kRepublishPeriod)) {
        return;
    }
    video_api::VideoStreamInfo info;
    info.source = source_;
    info.published_monotonic_ns = now_ns;
    info.streams.push_back(stream);
    publisher_.publish(info);
    last_published_ = stream;
    last_published_ns_ = now_ns;
}

video_api::VideoStream VideoStreamInfoPublisher::make_stream(std::uint16_t port,
                                                             bool running) const {
    video_api::VideoStream stream;
    stream.stream_id = description_.stream_id;
    stream.sensor = description_.sensor;
    stream.name = description_.name;
    stream.rtsp_port = port;
    stream.rtsp_path = rtsp_path_;
    stream.width = description_.width;
    stream.height = description_.height;
    stream.framerate_hz = description_.framerate_hz;
    stream.codec = codec_;
    stream.bitrate_bps = bitrate_bps_;
    stream.hfov_deg = description_.hfov_deg;
    stream.running = running;
    return stream;
}

}  // namespace argus::video_sender
