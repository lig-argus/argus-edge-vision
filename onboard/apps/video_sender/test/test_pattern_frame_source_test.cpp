#include "argus/video_sender/test_pattern_frame_source.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/video_sender/test_pattern_scene.hpp"

namespace argus::video_sender {
namespace {

using perception_api::Frame;

constexpr std::int64_t kUnixNs = 1'792'048'527'321'000'000;
constexpr std::int64_t kMonotonicNs = 5'000'000'000;

class TestPatternFrameSourceTest : public ::testing::Test {
protected:
    TestPatternConfig config_;
    core_testing::FakeClock clock_{kMonotonicNs};
    TestPatternFrameSource source_{clock_, [] { return kUnixNs; }, config_};
    std::vector<Frame> frames_;
    core::Subscription subscription_ =
        source_.subscribe([this](const Frame& frame) { frames_.push_back(frame); });
};

TEST_F(TestPatternFrameSourceTest, EmitsFrameWithConfiguredSizeAndTimes) {
    source_.emit_frame();

    ASSERT_EQ(frames_.size(), 1U);
    const Frame& frame = frames_[0];
    EXPECT_EQ(frame.source, config_.source);
    EXPECT_EQ(frame.session_id, config_.session_id);
    EXPECT_EQ(frame.width, 640U);
    EXPECT_EQ(frame.height, 480U);
    EXPECT_EQ(frame.stride, 640U);
    EXPECT_EQ(frame.pixels.size(), frame.expected_pixel_bytes());
    EXPECT_EQ(frame.unix_ns, kUnixNs);
    EXPECT_EQ(frame.received_monotonic_ns, kMonotonicNs);
}

TEST_F(TestPatternFrameSourceTest, FrameIdIncreasesFromZeroAndPixelsFollowIt) {
    source_.emit_frame();
    source_.emit_frame();
    source_.emit_frame();

    ASSERT_EQ(frames_.size(), 3U);
    for (std::uint64_t frame_id = 0; frame_id < 3; ++frame_id) {
        EXPECT_EQ(frames_[frame_id].frame_id, frame_id);
        EXPECT_EQ(frames_[frame_id].pixels, test_pattern_pixels(config_, frame_id));
    }
    EXPECT_NE(frames_[0].pixels, frames_[2].pixels);
}

TEST_F(TestPatternFrameSourceTest, EveryHandlerReceivesFrame) {
    std::int32_t second_handler_calls = 0;
    const core::Subscription second =
        source_.subscribe([&](const Frame&) { ++second_handler_calls; });

    source_.emit_frame();

    EXPECT_EQ(frames_.size(), 1U);
    EXPECT_EQ(second_handler_calls, 1);
}

TEST_F(TestPatternFrameSourceTest, ReleasedSubscriptionStopsDelivery) {
    subscription_.reset();

    source_.emit_frame();

    EXPECT_TRUE(frames_.empty());
}

}  // namespace
}  // namespace argus::video_sender
