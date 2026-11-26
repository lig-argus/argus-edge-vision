// RTP 타임스탬프 → UTC (docs/spec/video.md 4장). 송신부의 RTCP Sender Report 는 같은 순간의 NTP
// 시각과 RTP 타임스탬프를 함께 알린다. 송신자(SSRC)마다 마지막 SR 을 기억해 두었다가 영상 프레임의
// RTP 타임스탬프를 UTC(ns)로 바꾼다. 첫 SR 을 받기 전에는 바꿀 수 없다 (video.md 6장: 그동안 상자
// 없이)
#ifndef ARGUS_GCS_RTSP_RTCP_CLOCK_HPP_
#define ARGUS_GCS_RTSP_RTCP_CLOCK_HPP_

#include <cstdint>
#include <map>
#include <optional>

namespace argus::gcs::rtsp {

class RtcpClock {
public:
    static constexpr std::int64_t kVideoClockRate = 90'000;  // 영상 RTP 클럭 (Hz, RFC 3551)

    // RTCP SR 하나: NTP 64 비트(1900년 기준, 상위 32 비트 초 + 하위 32 비트 분수)와 RTP 타임스탬프
    void onSenderReport(std::uint32_t ssrc, std::uint64_t ntpTime, std::uint32_t rtpTime);

    // 그 송신자의 RTP 타임스탬프 → UTC (1970년 기준 ns). SR 을 받기 전이면 없음
    std::optional<std::int64_t> utcNs(std::uint32_t ssrc, std::uint32_t rtpTime,
                                      std::int64_t clockRate = kVideoClockRate) const;

    // NTP 64 비트 → 1970년 기준 ns
    static std::int64_t ntpToUnixNs(std::uint64_t ntpTime);

private:
    struct Report {
        std::int64_t unixNs{0};
        std::uint32_t rtpTime{0};
    };
    std::map<std::uint32_t, Report> reports_;  // SSRC → 마지막 SR
};

}  // namespace argus::gcs::rtsp

#endif  // ARGUS_GCS_RTSP_RTCP_CLOCK_HPP_
