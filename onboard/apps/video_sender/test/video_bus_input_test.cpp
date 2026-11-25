// VideoBusInput 이 bus 의 세 토픽을 FrameCollector 에 연결하는지 FakeBus 로 확인한다. 결합 규칙의
// 세부 동작은 frame_collector_test 가 확인한다
#include "argus/video_sender/video_bus_input.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/video_sender/collected_frame.hpp"
#include "argus/video_sender/frame_collector.hpp"

namespace argus::video_sender {
namespace {

constexpr const char* kSource = "ir";
constexpr const char* kSession = "session-a";

class VideoBusInputTest : public ::testing::Test {
protected:
    void SetUp() override {
        input_ = VideoBusInput::create(bus_, collector_);
        ASSERT_NE(input_, nullptr);
    }

    void receive_frame(std::uint64_t frame_id) {
        perception_api::Frame frame;
        frame.source = kSource;
        frame.session_id = kSession;
        frame.frame_id = frame_id;
        bus_.simulate_message_received(protocol::kVisionFrameGray8, frame);
    }

    void receive_observations(std::uint64_t frame_id) {
        perception_api::Observations observations;
        observations.source = kSource;
        observations.session_id = std::string{kSession};
        observations.frame_id = frame_id;
        observations.detections.emplace_back().classification.label = "tank";
        bus_.simulate_message_received(protocol::kPerceptionObservations, observations);
    }

    void receive_tracks(std::uint64_t frame_id) {
        perception_api::Tracks tracks;
        tracks.source = kSource;
        tracks.session_id = std::string{kSession};
        tracks.frame_id = frame_id;
        tracks.tracks.emplace_back().track_id = 17;
        bus_.simulate_message_received(protocol::kTrackingTracks, tracks);
    }

    void deliver() {
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
    FrameCollector collector_{clock_};
    std::vector<CollectedFrame> collected_;
    core::Subscription collected_subscription_ = collector_.subscribe(
        [this](const CollectedFrame& collected) { collected_.push_back(collected); });
    std::unique_ptr<VideoBusInput> input_;
};

TEST_F(VideoBusInputTest, CollectsFrameObservationsAndTracksOfSameFrameFromBus) {
    receive_tracks(1);
    receive_observations(1);
    receive_frame(1);
    deliver();

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_EQ(collected_[0].frame.frame_id, 1U);
    ASSERT_TRUE(collected_[0].observations.has_value());
    ASSERT_TRUE(collected_[0].tracks.has_value());
    EXPECT_EQ(collected_[0].tracks.value_or(perception_api::Tracks{}).tracks.size(), 1U);
}

TEST_F(VideoBusInputTest, StopsDeliveringAfterDestruction) {
    input_.reset();
    receive_frame(1);
    receive_observations(1);
    receive_tracks(1);
    deliver();

    clock_.advance_ns(std::chrono::nanoseconds{FrameCollectorConfig::kDefaultMaxWait}.count());
    collector_.process_timeouts();
    EXPECT_TRUE(collected_.empty());
}

}  // namespace
}  // namespace argus::video_sender
