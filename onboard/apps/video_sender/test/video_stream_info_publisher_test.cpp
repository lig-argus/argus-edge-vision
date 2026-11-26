#include "argus/video_sender/video_stream_info_publisher.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "argus/bus/typed.hpp"
#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/video_api/types.hpp"
#include "argus/video_api_bus/bus_video_stream_info_publisher.hpp"
#include "argus/video_api_bus/message_types.hpp"
#include "argus/video_sender/rtsp_server.hpp"
#include "argus/video_sender/video_encode_config.hpp"

namespace argus::video_sender {
namespace {

using video_api::VideoStream;
using video_api::VideoStreamInfo;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::uint16_t kPort = 8554;

VideoStreamDescription ir_description() {
    VideoStreamDescription description;
    description.stream_id = 1;
    description.sensor = vehicle_api::Sensor::Ir;
    description.name = "IR";
    description.width = 640;
    description.height = 480;
    description.framerate_hz = 30.0;
    return description;
}

RtspServerStats stats_with_sent(std::uint64_t sent) {
    RtspServerStats stats;
    stats.sent = sent;
    return stats;
}

class VideoStreamInfoPublisherTest : public ::testing::Test {
protected:
    void SetUp() override {
        bus_publisher_ = video_api_bus::BusVideoStreamInfoPublisher::create(bus_);
        ASSERT_NE(bus_publisher_, nullptr);
        // mavlink_gateway 의 구독 클래스 대신 typed 구독자로 받는다
        subscriber_ = bus::make_subscriber<VideoStreamInfo>(bus_, protocol::kVideoStreamInfo,
                                                            video_api_bus::kVideoStreamInfoType,
                                                            protocol::kVideoStreamInfoQueueDepth);
        ASSERT_NE(subscriber_, nullptr);
        subscription_ = subscriber_->subscribe(
            [this](const VideoStreamInfo& info, const bus::MessageInfo& /*info*/) {
                received_.push_back(info);
            });
    }

    std::unique_ptr<VideoStreamInfoPublisher> make_publisher(
        const VideoEncodeConfig& encode = make_video_encode_config(VideoCodec::H264)) {
        return std::make_unique<VideoStreamInfoPublisher>(
            *bus_publisher_, clock_, "argus_video_sender", ir_description(), "/argus/ir", encode);
    }

    // 시각을 at_ms 로 옮기고 tick() 을 부른 뒤 발행된 메시지를 전달한다
    void tick_at(VideoStreamInfoPublisher& publisher, std::int64_t at_ms, std::uint16_t port,
                 const RtspServerStats& stats) {
        clock_.advance_ns(at_ms * kMs - clock_.now_monotonic_ns());
        publisher.tick(port, stats);
        poller_.process_events(std::chrono::milliseconds{0}, kBudget);
    }

    // 마지막으로 받은 메시지의 스트림
    VideoStream last_stream() const {
        if (received_.empty() || received_.back().streams.size() != 1) {
            ADD_FAILURE() << "스트림이 하나인 메시지를 받지 못함";
            return {};
        }
        return received_.back().streams.front();
    }

    static constexpr std::chrono::milliseconds kBudget{5};

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
    std::unique_ptr<video_api_bus::BusVideoStreamInfoPublisher> bus_publisher_;
    std::unique_ptr<bus::Subscriber<VideoStreamInfo>> subscriber_;
    core::Subscription subscription_;
    std::vector<VideoStreamInfo> received_;
};

TEST_F(VideoStreamInfoPublisherTest, FirstTickPublishesEveryField) {
    VideoEncodeConfig encode = make_video_encode_config(VideoCodec::H264);
    encode.bitrate_kbps = 2000;
    const auto publisher = make_publisher(encode);

    tick_at(*publisher, 10, kPort, stats_with_sent(1));

    ASSERT_EQ(received_.size(), 1U);
    EXPECT_EQ(received_.back().source, "argus_video_sender");
    EXPECT_EQ(received_.back().published_monotonic_ns, 10 * kMs);
    const VideoStream stream = last_stream();
    EXPECT_EQ(stream.stream_id, 1U);
    EXPECT_EQ(stream.sensor, vehicle_api::Sensor::Ir);
    EXPECT_EQ(stream.name, "IR");
    EXPECT_EQ(stream.rtsp_port, kPort);
    EXPECT_EQ(stream.rtsp_path, "/argus/ir");
    EXPECT_EQ(stream.width, 640U);
    EXPECT_EQ(stream.height, 480U);
    EXPECT_DOUBLE_EQ(stream.framerate_hz, 30.0);
    EXPECT_EQ(stream.codec, video_api::VideoCodec::H264);
    EXPECT_EQ(stream.bitrate_bps, 2'000'000U);
    EXPECT_DOUBLE_EQ(stream.hfov_deg, 0.0);
    EXPECT_TRUE(stream.running);
}

TEST_F(VideoStreamInfoPublisherTest, BitrateIsZeroWithoutEncoderSetting) {
    const auto publisher = make_publisher(make_video_encode_config(VideoCodec::Jpeg));

    tick_at(*publisher, 10, kPort, stats_with_sent(1));

    EXPECT_EQ(last_stream().codec, video_api::VideoCodec::Jpeg);
    EXPECT_EQ(last_stream().bitrate_bps, 0U);
}

TEST_F(VideoStreamInfoPublisherTest, SameValueIsRepublishedOnlyAfterRepublishPeriod) {
    const auto publisher = make_publisher();
    const RtspServerStats stats = stats_with_sent(1);
    tick_at(*publisher, 0, kPort, stats);  // 첫 발행

    // 프레임이 계속 늘어 값이 같으면 1 s 가 지날 때까지 발행하지 않는다
    tick_at(*publisher, 100, kPort, stats_with_sent(2));
    tick_at(*publisher, 900, kPort, stats_with_sent(3));
    EXPECT_EQ(received_.size(), 1U);

    tick_at(*publisher, 1000, kPort, stats_with_sent(4));
    EXPECT_EQ(received_.size(), 2U);
    EXPECT_EQ(received_.back().published_monotonic_ns, 1000 * kMs);
}

TEST_F(VideoStreamInfoPublisherTest, ChangedPortIsPublishedAtOnce) {
    const auto publisher = make_publisher();
    tick_at(*publisher, 0, kPort, stats_with_sent(1));

    tick_at(*publisher, 100, kPort + 1, stats_with_sent(2));

    ASSERT_EQ(received_.size(), 2U);
    EXPECT_EQ(last_stream().rtsp_port, kPort + 1U);
}

TEST_F(VideoStreamInfoPublisherTest, RunningIsFalseWithoutAnyFrame) {
    const auto publisher = make_publisher();

    tick_at(*publisher, 10, kPort, RtspServerStats{});

    EXPECT_FALSE(last_stream().running);
}

TEST_F(VideoStreamInfoPublisherTest, RunningBecomesFalseOneSecondAfterLastNewFrame) {
    const auto publisher = make_publisher();
    tick_at(*publisher, 0, kPort, stats_with_sent(5));
    ASSERT_TRUE(last_stream().running);

    tick_at(*publisher, 999, kPort, stats_with_sent(5));  // 1 s 가 지나지 않음
    EXPECT_EQ(received_.size(), 1U);

    tick_at(*publisher, 1000, kPort, stats_with_sent(5));
    ASSERT_EQ(received_.size(), 2U);
    EXPECT_FALSE(last_stream().running);
}

TEST_F(VideoStreamInfoPublisherTest, RunningBecomesTrueAtOnceWhenFramesResume) {
    const auto publisher = make_publisher();
    tick_at(*publisher, 0, kPort, RtspServerStats{});
    ASSERT_FALSE(last_stream().running);

    tick_at(*publisher, 100, kPort, stats_with_sent(1));

    ASSERT_EQ(received_.size(), 2U);
    EXPECT_TRUE(last_stream().running);
}

TEST_F(VideoStreamInfoPublisherTest, FramesDroppedForNoClientCountAsRunning) {
    const auto publisher = make_publisher();
    RtspServerStats stats;
    stats.dropped_no_client = 3;

    tick_at(*publisher, 10, kPort, stats);

    EXPECT_TRUE(last_stream().running);
}

TEST_F(VideoStreamInfoPublisherTest, InvalidFramesDoNotCountAsRunning) {
    const auto publisher = make_publisher();
    RtspServerStats stats;
    stats.dropped_invalid_frame = 3;

    tick_at(*publisher, 10, kPort, stats);

    EXPECT_FALSE(last_stream().running);
}

TEST_F(VideoStreamInfoPublisherTest, NothingIsPublishedWhileServerIsClosed) {
    const auto publisher = make_publisher();

    tick_at(*publisher, 10, 0, stats_with_sent(1));
    tick_at(*publisher, 2000, 0, stats_with_sent(2));

    EXPECT_TRUE(received_.empty());
}

}  // namespace
}  // namespace argus::video_sender
