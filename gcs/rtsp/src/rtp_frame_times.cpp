#include "argus/gcs/rtsp/rtp_frame_times.hpp"

namespace argus::gcs::rtsp {

void RtpFrameTimes::recordPacket(std::optional<std::uint64_t> pts, std::uint32_t ssrc,
                                 std::uint32_t rtpTime) {
    if (!records_.empty() && records_.back().time.rtpTime == rtpTime &&
        records_.back().time.ssrc == ssrc) {
        return;  // 같은 프레임의 다음 패킷
    }
    records_.push_back(Record{pts, RtpFrameTime{ssrc, rtpTime}});
    while (records_.size() > kMaxRecords) {
        records_.pop_front();
    }
}

std::optional<RtpFrameTime> RtpFrameTimes::takeFrame(std::uint64_t pts) {
    while (!records_.empty() && records_.front().pts != pts) {
        if (records_.front().pts.value_or(0) > pts) {
            return std::nullopt;  // 기록보다 앞선 프레임 (기록이 넘쳐 이미 지워졌다)
        }
        records_.pop_front();
    }
    if (records_.empty()) {
        return std::nullopt;
    }
    const RtpFrameTime time = records_.front().time;
    records_.pop_front();
    return time;
}

}  // namespace argus::gcs::rtsp
