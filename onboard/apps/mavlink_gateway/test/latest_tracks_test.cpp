#include "argus/mavlink_gateway/latest_tracks.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/protocol/topics.hpp"

namespace argus::mavlink_gateway {
namespace {

class LatestTracksTest : public ::testing::Test {
protected:
    void receive(const std::string& source, std::uint64_t frame_id) {
        perception_api::Tracks tracks;
        tracks.source = source;
        tracks.frame_id = frame_id;
        bus_.simulate_message_received(protocol::kTrackingTracks, tracks);
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{0});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
    std::unique_ptr<LatestTracks> tracks_ = LatestTracks::create(bus_);
};

TEST_F(LatestTracksTest, NothingBeforeFirstMessage) {
    ASSERT_NE(tracks_, nullptr);

    EXPECT_FALSE(tracks_->latest("camera-ir").has_value());
}

TEST_F(LatestTracksTest, KeepsLatestPerSource) {
    receive("camera-ir", 1);
    receive("camera-eo", 2);
    receive("camera-ir", 3);

    ASSERT_TRUE(tracks_->latest("camera-ir").has_value());
    EXPECT_EQ(tracks_->latest("camera-ir").value_or(perception_api::Tracks{}).frame_id, 3U);
    ASSERT_TRUE(tracks_->latest("camera-eo").has_value());
    EXPECT_EQ(tracks_->latest("camera-eo").value_or(perception_api::Tracks{}).frame_id, 2U);
}

}  // namespace
}  // namespace argus::mavlink_gateway
