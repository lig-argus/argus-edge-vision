#include "argus/gcs/rtsp/rtcp_clock.hpp"

namespace argus::gcs::rtsp {

namespace {

// 1900-01-01(NTP)과 1970-01-01(Unix) 사이 초 (RFC 868)
constexpr std::int64_t kNtpToUnixSeconds = 2'208'988'800;
constexpr std::int64_t kNsPerSecond = 1'000'000'000;
constexpr std::uint32_t kFractionBits = 32U;
constexpr std::uint64_t kFractionMask = 0xFFFF'FFFFULL;

}  // namespace

std::int64_t RtcpClock::ntpToUnixNs(std::uint64_t ntpTime) {
    const auto seconds = static_cast<std::int64_t>(ntpTime >> kFractionBits);
    const std::uint64_t fraction = ntpTime & kFractionMask;
    // 분수 × 10^9 / 2^32 (곱이 64 비트를 넘지 않는다: 2^32 × 10^9 < 2^63)
    const auto fractionNs = static_cast<std::int64_t>(
        (fraction * static_cast<std::uint64_t>(kNsPerSecond)) >> kFractionBits);
    return (seconds - kNtpToUnixSeconds) * kNsPerSecond + fractionNs;
}

void RtcpClock::onSenderReport(std::uint32_t ssrc, std::uint64_t ntpTime, std::uint32_t rtpTime) {
    reports_[ssrc] = Report{ntpToUnixNs(ntpTime), rtpTime};
}

std::optional<std::int64_t> RtcpClock::utcNs(std::uint32_t ssrc, std::uint32_t rtpTime,
                                             std::int64_t clockRate) const {
    const auto found = reports_.find(ssrc);
    if (found == reports_.end() || clockRate <= 0) {
        return std::nullopt;
    }
    // RTP 타임스탬프는 32 비트라 한 바퀴 돈다: 차이를 부호 있는 32 비트로 본다 (90 kHz 에서 ±약 6.6
    // 시간)
    const auto ticks = static_cast<std::int32_t>(rtpTime - found->second.rtpTime);
    return found->second.unixNs + static_cast<std::int64_t>(ticks) * kNsPerSecond / clockRate;
}

}  // namespace argus::gcs::rtsp
