// 영상 출처가 받기 시작한 뒤부터 센 누적 값 (모든 센서의 합). 녹화가 1 초마다 차이를 기록한다.
// 화면과 녹화가 수신기 내부를 알 필요가 없게 IVideoSource 가 이 모양으로 낸다
#ifndef ARGUS_GCS_VIDEO_VIDEO_COUNTERS_HPP_
#define ARGUS_GCS_VIDEO_VIDEO_COUNTERS_HPP_

#include <cstdint>
#include <optional>

namespace argus::gcs::video {

struct VideoCounters {
    std::uint64_t rtpPackets{0};       // 영상 트랙 RTP 패킷
    std::uint64_t rtpLost{0};          // 영상 트랙 RTP 순번이 빠진 수
    std::uint64_t rtpBytes{0};         // 받은 RTP 바이트 (영상 트랙과 메타데이터 트랙, 헤더 포함)
    std::uint64_t metadataPackets{0};  // 메타데이터 트랙 RTP 패킷
    std::uint64_t metadataLost{0};     // 메타데이터 트랙 RTP 순번이 빠진 수
    std::uint64_t frames{0};           // 해독한 영상 프레임
    std::uint64_t displayed{0};        // 화면으로 넘긴 프레임
    std::optional<std::int64_t> lastDisplayNs;  // 마지막으로 화면에 넘긴 시각 (IClock monotonic ns)
    bool receiving{false};                      // 최근 1 초 안에 프레임을 받음
};

}  // namespace argus::gcs::video

#endif  // ARGUS_GCS_VIDEO_VIDEO_COUNTERS_HPP_
