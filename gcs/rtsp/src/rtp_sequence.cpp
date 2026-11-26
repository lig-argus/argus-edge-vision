#include "argus/gcs/rtsp/rtp_sequence.hpp"

namespace argus::gcs::rtsp {

namespace {

constexpr std::uint16_t kHalfRange = 0x8000U;  // 순번 공간의 절반: 이보다 크면 뒤로 간 순번

}  // namespace

void RtpSequenceCounter::push(std::uint32_t ssrc, std::uint16_t seq, std::int64_t nowNs) {
    ++stats_.received;
    if (!started_ || ssrc != ssrc_) {
        started_ = true;
        ssrc_ = ssrc;
        lastSeq_ = seq;
        return;
    }
    const auto forward = static_cast<std::uint16_t>(seq - lastSeq_);
    if (forward == 0U) {
        ++stats_.duplicated;
        return;
    }
    if (forward > kHalfRange) {
        ++stats_.reordered;  // 뒤로 간 순번: 이미 손실로 센 패킷이 늦게 왔다
        if (stats_.lost > 0U) {
            --stats_.lost;
        }
        return;
    }
    if (forward > kMaxGap) {
        lastSeq_ = seq;  // 송신자 재시작으로 본다
        return;
    }
    if (forward > 1U) {
        const auto missing = static_cast<std::int32_t>(forward - 1U);
        stats_.lost += static_cast<std::uint64_t>(missing);
        stats_.losses.push_back(RtpLossRecord{nowNs, lastSeq_, seq, missing});
        while (stats_.losses.size() > RtpStats::kMaxLossRecords) {
            stats_.losses.pop_front();
        }
    }
    lastSeq_ = seq;
}

}  // namespace argus::gcs::rtsp
