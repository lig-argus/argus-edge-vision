// 영상 프레임과 메타데이터(추적 결과)를 UTC 시각으로 짝짓는다 (docs/spec/video.md 4장, 6장).
// 영상 프레임의 UTC 는 RTCP Sender Report 로 RTP 타임스탬프에서 구하고(RtcpClock), 메타데이터는
// 문서 안 UtcTime 을 쓴다. 둘 다 같은 RPi 시계의 프레임 수신 시각이라 오차는 ±반 프레임 안이다.
// - 메타데이터가 아직 오지 않은 프레임은 짝이 없다 (상자 없이 표시)
// - 이미 지나간 프레임의 메타데이터가 늦게 오면 버린다
#ifndef ARGUS_GCS_RTSP_FRAME_SYNCHRONIZER_HPP_
#define ARGUS_GCS_RTSP_FRAME_SYNCHRONIZER_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

#include "argus/gcs/rtsp/onvif_metadata.hpp"

namespace argus::gcs::rtsp {

class FrameSynchronizer {
public:
    static constexpr std::int64_t kDefaultToleranceNs = 16'666'667;  // ±반 프레임 (30 FPS)

    // 짝을 기다리며 보관하는 메타데이터 수 (30 FPS 에서 약 1 초)
    static constexpr std::size_t kMaxHeld = 32;

    explicit FrameSynchronizer(std::int64_t toleranceNs = kDefaultToleranceNs);

    void addMetadata(FrameMetadata metadata);

    // 프레임 UTC 에 가장 가까운 메타데이터를 꺼낸다 (허용 오차 안). 짝보다 오래된 것은 버린다
    std::optional<FrameMetadata> takeFor(std::int64_t frameUtcNs);

    std::uint64_t matched() const { return matched_; }
    std::uint64_t unmatchedFrames() const { return unmatchedFrames_; }

    // 짝지을 프레임이 이미 지나가 버린 메타데이터 (늦게 왔거나, 그 프레임이 오지 않았음)
    std::uint64_t droppedMetadata() const { return droppedMetadata_; }

private:
    std::int64_t toleranceNs_;
    std::deque<FrameMetadata> held_;  // UtcTime 오름차순
    std::optional<std::int64_t> lastFrameUtcNs_;
    std::uint64_t matched_{0};
    std::uint64_t unmatchedFrames_{0};
    std::uint64_t droppedMetadata_{0};
};

}  // namespace argus::gcs::rtsp

#endif  // ARGUS_GCS_RTSP_FRAME_SYNCHRONIZER_HPP_
