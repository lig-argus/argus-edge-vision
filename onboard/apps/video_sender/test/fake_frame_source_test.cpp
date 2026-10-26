#include "fake_frame_source.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/perception_api/types.hpp"
// 인터페이스·값 타입 헤더가 ZeroMQ, GStreamer 없이 컴파일되는지 함께 확인한다
#include "argus/video_sender/collected_frame.hpp"
#include "argus/video_sender/video_encode_config.hpp"

namespace argus::video_sender {
namespace {

using perception_api::Frame;

Frame frame_with_id(std::uint64_t frame_id) {
    Frame frame;
    frame.source = "camera-ir";
    frame.frame_id = frame_id;
    frame.width = 640;
    frame.height = 480;
    frame.stride = 640;
    frame.pixels.assign(frame.expected_pixel_bytes(), 0);
    return frame;
}

class FakeFrameSourceTest : public ::testing::Test {
protected:
    void process() {
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    FakeFrameSource source_{poller_};
};

TEST_F(FakeFrameSourceTest, DeliversPushedFramesInsideProcessEventsInOrder) {
    std::vector<std::uint64_t> received;
    const core::Subscription subscription =
        source_.subscribe([&](const Frame& frame) { received.push_back(frame.frame_id); });

    source_.push(frame_with_id(1));
    source_.push(frame_with_id(2));
    EXPECT_TRUE(received.empty());  // process_events() 전에는 전달되지 않는다

    process();
    EXPECT_EQ(received, (std::vector<std::uint64_t>{1, 2}));
}

TEST_F(FakeFrameSourceTest, EveryHandlerReceivesAndPixelsArrive) {
    int first = 0;
    int second = 0;
    std::size_t pixel_bytes = 0;
    const core::Subscription a = source_.subscribe([&](const Frame&) { ++first; });
    const core::Subscription b = source_.subscribe([&](const Frame& frame) {
        ++second;
        pixel_bytes = frame.pixels.size();
    });

    source_.push(frame_with_id(7));
    process();

    EXPECT_EQ(first, 1);
    EXPECT_EQ(second, 1);
    EXPECT_EQ(pixel_bytes, 640U * 480U);
}

TEST_F(FakeFrameSourceTest, ReleasedSubscriptionStopsDelivery) {
    int received = 0;
    core::Subscription subscription = source_.subscribe([&](const Frame&) { ++received; });

    source_.push(frame_with_id(1));
    subscription.reset();  // 전달 전에 해제
    process();

    EXPECT_EQ(received, 0);
}

TEST(CollectedFrameTest, ObservationsAndTracksAreEmptyUntilCollected) {
    CollectedFrame collected;
    collected.frame = frame_with_id(3);

    EXPECT_EQ(collected.frame.frame_id, 3U);
    EXPECT_FALSE(collected.observations.has_value());
    EXPECT_FALSE(collected.tracks.has_value());
}

TEST(VideoEncodeConfigTest, DefaultsToRtpJpegWithoutFixedQuality) {
    const VideoEncodeConfig config;

    EXPECT_EQ(config.codec, VideoCodec::Jpeg);
    EXPECT_EQ(config.payload_type, 26);
    EXPECT_FALSE(config.quality.has_value());
    EXPECT_NE(config.gstreamer_elements.find("rtpjpegpay"), std::string::npos);
}

}  // namespace
}  // namespace argus::video_sender
