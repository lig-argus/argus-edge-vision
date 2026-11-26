#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>

#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/video_api/types.hpp"
#include "argus/video_api_bus/bus_video_stream_info_publisher.hpp"
#include "argus/video_api_bus/bus_video_stream_info_subscriber.hpp"

namespace argus::video_api_bus {
namespace {

using video_api::ReceivedVideoStreamInfo;
using video_api::VideoStream;
using video_api::VideoStreamInfo;

constexpr std::int64_t kMs = 1'000'000;

VideoStreamInfo info_with_port(std::uint32_t port) {
    VideoStream stream;
    stream.stream_id = 1;
    stream.sensor = vehicle_api::Sensor::Ir;
    stream.name = "IR";
    stream.rtsp_port = port;
    stream.rtsp_path = "/argus/ir";
    stream.running = true;
    VideoStreamInfo info;
    info.source = "argus_video_sender";
    info.streams.push_back(stream);
    return info;
}

class VideoStreamInfoTest : public ::testing::Test {
protected:
    void SetUp() override {
        publisher_ = BusVideoStreamInfoPublisher::create(bus_);
        subscriber_ = BusVideoStreamInfoSubscriber::create(bus_, clock_);
        ASSERT_NE(publisher_, nullptr);
        ASSERT_NE(subscriber_, nullptr);
    }

    // 시각을 at_ns 로 옮기고 info 를 발행해 구독자에게 전달한다
    void deliver_at(std::int64_t at_ns, const VideoStreamInfo& info) {
        clock_.advance_ns(at_ns - clock_.now_monotonic_ns());
        ASSERT_TRUE(publisher_->publish(info));
        poller_.process_events(std::chrono::milliseconds{0}, kBudget);
    }

    static constexpr std::chrono::milliseconds kBudget{5};

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
    std::unique_ptr<BusVideoStreamInfoPublisher> publisher_;
    std::unique_ptr<BusVideoStreamInfoSubscriber> subscriber_;
};

TEST_F(VideoStreamInfoTest, LatestIsEmptyUntilFirstInfo) {
    EXPECT_FALSE(subscriber_->latest().has_value());
}

TEST_F(VideoStreamInfoTest, LatestKeepsLastInfoWithArrivalTimeFromClock) {
    VideoStreamInfo first = info_with_port(8554);
    first.published_monotonic_ns = 1;  // 발행자가 채운 값은 받은 시각으로 사용하지 않는다
    deliver_at(100 * kMs, first);
    deliver_at(250 * kMs, info_with_port(8555));

    ASSERT_TRUE(subscriber_->latest().has_value());
    const ReceivedVideoStreamInfo latest =
        subscriber_->latest().value_or(ReceivedVideoStreamInfo{});
    EXPECT_EQ(latest.arrived_monotonic_ns, 250 * kMs);
    EXPECT_EQ(latest.info.source, "argus_video_sender");
    ASSERT_EQ(latest.info.streams.size(), 1U);
    EXPECT_EQ(latest.info.streams.front(), info_with_port(8555).streams.front());
}

TEST_F(VideoStreamInfoTest, MessageArrivingAfterSubscriberIsDestroyedIsIgnored) {
    ASSERT_TRUE(publisher_->publish(info_with_port(8554)));
    subscriber_.reset();

    // 해제된 handler 는 불리지 않는다
    poller_.process_events(std::chrono::milliseconds{0}, kBudget);
}

}  // namespace
}  // namespace argus::video_api_bus
