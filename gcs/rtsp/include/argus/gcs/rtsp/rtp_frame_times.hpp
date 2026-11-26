// 해독한 영상 프레임의 PTS(수신기가 버퍼에 붙인 표시 시각) → 그 프레임의 RTP 타임스탬프.
// 해독기는 RTP 헤더를 버리므로, 해독 전 영상 RTP 패킷마다 (PTS, SSRC, RTP 타임스탬프)를 기록해 두고
// 해독한 프레임의 PTS 로 찾는다. 찾은 RTP 타임스탬프는 RtcpClock 으로 UTC 가 된다
// (docs/spec/video.md 4장)
#ifndef ARGUS_GCS_RTSP_RTP_FRAME_TIMES_HPP_
#define ARGUS_GCS_RTSP_RTP_FRAME_TIMES_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace argus::gcs::rtsp {

struct RtpFrameTime {
    std::uint32_t ssrc{0};
    std::uint32_t rtpTime{0};
};

class RtpFrameTimes {
public:
    // 보관하는 프레임 기록 수. 지터 버퍼(100 ms)와 해독 지연보다 넉넉하게
    static constexpr std::size_t kMaxRecords = 256;

    // 영상 RTP 패킷 하나. pts 가 없으면(수신기가 시각을 붙이지 않음) 그 프레임은 찾을 수 없다.
    // 한 프레임의 패킷은 모두 같은 RTP 타임스탬프라 프레임마다 한 번 기록한다. PTS 로 프레임을
    // 가르지 않는다: 수신 시작 직후 지터 버퍼가 몰린 프레임 여럿에 같은 PTS 를 붙이기 때문이다
    void recordPacket(std::optional<std::uint64_t> pts, std::uint32_t ssrc, std::uint32_t rtpTime);

    // 해독한 프레임의 PTS → 그 프레임의 RTP 시각. 앞에서부터 같은 PTS 의 기록을 하나씩 꺼낸다
    // (같은 PTS 의 프레임이 여럿이어도 들어온 차례대로). 그보다 앞선 기록은 해독되지 않은
    // 프레임이라 버린다
    std::optional<RtpFrameTime> takeFrame(std::uint64_t pts);

    void clear() { records_.clear(); }

private:
    struct Record {
        std::optional<std::uint64_t> pts;
        RtpFrameTime time;
    };
    std::deque<Record> records_;  // 받은 순서
};

}  // namespace argus::gcs::rtsp

#endif  // ARGUS_GCS_RTSP_RTP_FRAME_TIMES_HPP_
