#include "argus/video_sender/stream_timing.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace argus::video_sender {
namespace {

constexpr std::int64_t kUnixNs = 1'792'048'527'321'000'000;  // 2026-10-15T07:15:27.321Z
constexpr std::uint64_t kNsPerSecond = 1'000'000'000;

TEST(RtpTimestampFromUnixNsTest, ConvertsTo90kHzTicks) {
    EXPECT_EQ(rtp_timestamp_from_unix_ns(0), 0U);
    EXPECT_EQ(rtp_timestamp_from_unix_ns(1'000'000'000), 90'000U);
    EXPECT_EQ(rtp_timestamp_from_unix_ns(33'333'334), 3000U);  // 30 FPS 의 프레임 간격
}

TEST(RtpTimestampFromUnixNsTest, KeepsLower32BitsForCurrentTime) {
    // 1'792'048'527.321 s * 90000 = 161'284'367'458'890 = 0x92AF'F16E'224A → 하위 32 비트
    EXPECT_EQ(rtp_timestamp_from_unix_ns(kUnixNs), 0xF16E224AU);
}

TEST(FrameRunningTimeTest, SubtractsTimeElapsedSinceFrameWasReceived) {
    // 현재 running time 10 s, 프레임을 수신한 뒤 100 ms 경과
    EXPECT_EQ(frame_running_time_ns(10 * kNsPerSecond, 5'100'000'000, 5'000'000'000),
              9'900'000'000U);
}

TEST(FrameRunningTimeTest, FrameReceivedNowHasCurrentRunningTime) {
    EXPECT_EQ(frame_running_time_ns(10 * kNsPerSecond, 5'000'000'000, 5'000'000'000),
              10 * kNsPerSecond);
}

TEST(FrameRunningTimeTest, ReceiveTimeAfterNowIsHandledAsNow) {
    EXPECT_EQ(frame_running_time_ns(10 * kNsPerSecond, 5'000'000'000, 6'000'000'000),
              10 * kNsPerSecond);
}

TEST(FrameRunningTimeTest, ElapsedLongerThanRunningTimeGivesZero) {
    // 파이프라인이 시작되기 전에 수신한 프레임
    EXPECT_EQ(frame_running_time_ns(50'000'000, 5'000'000'000, 4'000'000'000), 0U);
}

TEST(FrameRunningTimeTest, FramesReceivedAtSameTimeGetSameRunningTime) {
    // 영상과 메타데이터에 같은 값을 부여하는 근거: 결과가 인자만으로 정해진다
    const std::uint64_t first = frame_running_time_ns(7 * kNsPerSecond, 900, 400);
    const std::uint64_t second = frame_running_time_ns(7 * kNsPerSecond, 900, 400);
    EXPECT_EQ(first, second);
}

}  // namespace
}  // namespace argus::video_sender
