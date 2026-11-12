// 영상 트랙과 메타데이터 트랙의 시각 계산
// 규약: docs/spec/video.md 2장 (RTP 타임스탬프), 3.1, 4장 (동기화)
// 사용: RtspServer
//
// 이 헤더의 함수는 GStreamer 와 core::IClock 을 호출하지 않고 인자만으로 결과를 계산한다.
// 그래서 GStreamer 없이 gtest 만으로 테스트한다
#ifndef ARGUS_VIDEO_SENDER_STREAM_TIMING_HPP_
#define ARGUS_VIDEO_SENDER_STREAM_TIMING_HPP_

#include <cstdint>

namespace argus::video_sender {

// 영상 트랙과 메타데이터 트랙의 RTP 클럭 (Hz). 영상 RTP 는 코덱과 무관하게 90 kHz 이고
// (video.md 2장) 메타데이터 트랙도 같은 값이다 (video.md 3.1)
inline constexpr std::int64_t kRtpClockRate = 90'000;

// unix_ns 를 kRtpClockRate 의 RTP 타임스탬프로 변환한다. RTP 타임스탬프는 32 비트이므로 하위
// 32 비트만 남는다
constexpr std::uint32_t rtp_timestamp_from_unix_ns(std::int64_t unix_ns) {
    constexpr std::int64_t kNsPerSecond = 1'000'000'000;
    // unix_ns * kRtpClockRate 는 64 비트를 초과하므로 초와 ns 로 나누어 계산한다
    const std::int64_t seconds = unix_ns / kNsPerSecond;
    const std::int64_t nanoseconds = unix_ns % kNsPerSecond;
    const std::int64_t ticks = seconds * kRtpClockRate + nanoseconds * kRtpClockRate / kNsPerSecond;
    return static_cast<std::uint32_t>(static_cast<std::uint64_t>(ticks));
}

// 프레임을 RPi 가 수신한 시각에 해당하는 파이프라인의 running time (ns).
// 현재의 running time 에서, 프레임을 수신한 뒤 현재까지 경과한 시간을 뺀 값이다.
// - running_time_now_ns: 파이프라인의 현재 running time
// - now_monotonic_ns, frame_received_monotonic_ns: 같은 monotonic clock 의 현재 시각과 프레임 수신
//   시각 (core::IClock, Frame.received_monotonic_ns)
// 수신 시각이 현재보다 뒤이면 경과 시간을 0 으로, 경과 시간이 running time 보다 길면 결과를 0 으로
// 처리한다 (running time 은 음수가 될 수 없다)
constexpr std::uint64_t frame_running_time_ns(std::uint64_t running_time_now_ns,
                                              std::int64_t now_monotonic_ns,
                                              std::int64_t frame_received_monotonic_ns) {
    const std::int64_t elapsed_ns = now_monotonic_ns - frame_received_monotonic_ns;
    if (elapsed_ns <= 0) {
        return running_time_now_ns;
    }
    const auto elapsed = static_cast<std::uint64_t>(elapsed_ns);
    return running_time_now_ns > elapsed ? running_time_now_ns - elapsed : 0;
}

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_STREAM_TIMING_HPP_
