// 구독자가 정해진 토픽의 값만 받는지 FakeBus 로 확인한다
#include "argus/perception_api_bus/subscribers.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <vector>

#include "argus/bus/message_type.hpp"
#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/protocol/topics.hpp"

namespace argus::perception_api_bus {
namespace {

class SubscribersTest : public ::testing::Test {
protected:
    void deliver() {
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
};

TEST_F(SubscribersTest, FrameSubscriberReceivesVisionFrameGray8) {
    const auto subscriber = create_frame_subscriber(bus_);
    ASSERT_NE(subscriber, nullptr);
    std::vector<perception_api::Frame> received;
    const core::Subscription subscription =
        subscriber->subscribe([&](const perception_api::Frame& frame,
                                  const bus::MessageInfo& /*info*/) { received.push_back(frame); });
    perception_api::Frame frame;
    frame.source = "ir";
    frame.frame_id = 7;
    frame.width = 2;
    frame.height = 1;
    frame.stride = 2;
    frame.pixels = {1, 2};

    bus_.simulate_message_received(protocol::kVisionFrameGray8, frame);
    bus_.simulate_message_received(protocol::kPerceptionObservations, frame);  // 다른 토픽
    deliver();

    ASSERT_EQ(received.size(), 1U);
    EXPECT_EQ(received[0].frame_id, 7U);
    EXPECT_EQ(received[0].pixels, frame.pixels);
}

TEST_F(SubscribersTest, ObservationsSubscriberReceivesPerceptionObservations) {
    const auto subscriber = create_observations_subscriber(bus_);
    ASSERT_NE(subscriber, nullptr);
    std::vector<std::uint64_t> frame_ids;
    const core::Subscription subscription = subscriber->subscribe(
        [&](const perception_api::Observations& observations, const bus::MessageInfo& /*info*/) {
            frame_ids.push_back(observations.frame_id);
        });
    perception_api::Observations observations;
    observations.frame_id = 8;

    bus_.simulate_message_received(protocol::kPerceptionObservations, observations);
    bus_.simulate_message_received(protocol::kTrackingTracks, observations);  // 다른 토픽
    deliver();

    EXPECT_EQ(frame_ids, std::vector<std::uint64_t>{8});
}

TEST_F(SubscribersTest, TracksSubscriberReceivesTrackingTracks) {
    const auto subscriber = create_tracks_subscriber(bus_);
    ASSERT_NE(subscriber, nullptr);
    std::vector<std::uint64_t> frame_ids;
    const core::Subscription subscription = subscriber->subscribe(
        [&](const perception_api::Tracks& tracks, const bus::MessageInfo& /*info*/) {
            frame_ids.push_back(tracks.frame_id);
        });
    perception_api::Tracks tracks;
    tracks.frame_id = 9;

    bus_.simulate_message_received(protocol::kTrackingTracks, tracks);
    bus_.simulate_message_received(protocol::kVisionFrameGray8, tracks);  // 다른 토픽
    deliver();

    EXPECT_EQ(frame_ids, std::vector<std::uint64_t>{9});
}

}  // namespace
}  // namespace argus::perception_api_bus
