// RTP 순번(seq, 16 비트)으로 패킷 손실, 순서 바뀜, 중복을 센다 (RFC 3550 A.1 의 순번 진행 판단을
// 줄인 것). 통신 통계 화면이 읽는다. 수신기의 지터 버퍼를 지난 뒤에 세므로 늦게 와서 버린 패킷도
// 손실이다
#ifndef ARGUS_GCS_RTSP_RTP_SEQUENCE_HPP_
#define ARGUS_GCS_RTSP_RTP_SEQUENCE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>

namespace argus::gcs::rtsp {

// 손실 기록 하나: 순번이 건너뛴 자리
struct RtpLossRecord {
    std::int64_t detectedNs{0};  // 건너뛴 뒤 첫 패킷을 받은 시각 (IClock monotonic ns)
    std::uint16_t lastSeq{0};    // 건너뛰기 전에 받은 순번
    std::uint16_t nextSeq{0};    // 건너뛴 뒤 받은 순번
    std::int32_t missing{0};     // 빠진 수
};

struct RtpStats {
    static constexpr std::size_t kMaxLossRecords = 50;  // 보관하는 최근 손실 기록 수

    std::uint64_t received{0};
    std::uint64_t lost{0};
    std::uint64_t reordered{0};          // 이미 손실로 센 앞 순번이 늦게 도착한 수
    std::uint64_t duplicated{0};         // 같은 순번이 다시 도착한 수
    std::deque<RtpLossRecord> losses{};  // 오래된 것부터, 최대 kMaxLossRecords
};

class RtpSequenceCounter {
public:
    // 앞으로 이보다 크게 건너뛰면 송신자가 다시 시작한 것으로 보고 손실로 세지 않는다
    static constexpr std::uint16_t kMaxGap = 3000;

    // 받은 패킷 하나. 송신자(SSRC)가 바뀌면 그 패킷부터 다시 센다
    void push(std::uint32_t ssrc, std::uint16_t seq, std::int64_t nowNs);

    const RtpStats& stats() const { return stats_; }

private:
    RtpStats stats_;
    bool started_{false};
    std::uint32_t ssrc_{0};
    std::uint16_t lastSeq_{0};
};

}  // namespace argus::gcs::rtsp

#endif  // ARGUS_GCS_RTSP_RTP_SEQUENCE_HPP_
