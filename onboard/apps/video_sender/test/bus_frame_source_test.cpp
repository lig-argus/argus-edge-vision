#include "argus/video_sender/bus_frame_source.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <vector>

#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/protocol/topics.hpp"

namespace argus::video_sender {
namespace {

class BusFrameSourceTest : public ::testing::Test {
protected:
    void SetUp() override {
        source_ = BusFrameSource::create(bus_);
        ASSERT_NE(source_, nullptr);
    }

    void receive_frame(std::uint64_t frame_id) {
        perception_api::Frame frame;
        frame.frame_id = frame_id;
        bus_.simulate_message_received(protocol::kVisionFrameGray8, frame);
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
    std::unique_ptr<BusFrameSource> source_;
};

TEST_F(BusFrameSourceTest, PassesFramesFromVisionFrameGray8ToEveryHandler) {
    std::vector<std::uint64_t> first;
    std::vector<std::uint64_t> second;
    const core::Subscription first_subscription = source_->subscribe(
        [&](const perception_api::Frame& frame) { first.push_back(frame.frame_id); });
    const core::Subscription second_subscription = source_->subscribe(
        [&](const perception_api::Frame& frame) { second.push_back(frame.frame_id); });

    receive_frame(1);

    EXPECT_EQ(first, std::vector<std::uint64_t>{1});
    EXPECT_EQ(second, std::vector<std::uint64_t>{1});
}

TEST_F(BusFrameSourceTest, ReleasedSubscriptionStopsDelivery) {
    std::vector<std::uint64_t> received;
    core::Subscription subscription = source_->subscribe(
        [&](const perception_api::Frame& frame) { received.push_back(frame.frame_id); });

    subscription.reset();
    receive_frame(1);

    EXPECT_TRUE(received.empty());
}

TEST_F(BusFrameSourceTest, RejectsEmptyHandler) {
    EXPECT_FALSE(source_->subscribe(nullptr).active());
}

}  // namespace
}  // namespace argus::video_sender
