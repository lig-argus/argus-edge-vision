#include "argus/mavlink_gateway/control_authority.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/command_activity.hpp"
#include "argus/mavlink_gateway/control_requests.hpp"
#include "argus/mavlink_gateway/fc_command_status.hpp"
#include "argus/mavlink_gateway/gcs_registry.hpp"
#include "argus/mavlink_gateway/landing_lock_state.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_follow_status_source.hpp"
#include "fake_fc_state_source.hpp"
#include "fake_gcs_link.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway {
namespace {

using test_messages::gcs_peer;
using test_messages::heartbeat_from;

constexpr std::int64_t kSecond = 1'000'000'000;

class FakeFcCommandStatus : public IFcCommandStatus {
public:
    bool sending_setpoints() const override { return sending; }
    bool entering() const override { return false; }

    bool sending = false;
};

class FakeLandingLock : public ILandingLockState {
public:
    bool active() const override { return locked; }

    bool locked = false;
};

class ControlAuthorityTest : public ::testing::Test {
protected:
    ControlAuthorityTest() { clock_.advance_ns(100 * kSecond); }

    void heartbeat(const GcsPeer& gcs) {
        link_.simulate_message_received(heartbeat_from(gcs), gcs);
    }

    void set_flying(bool flying) {
        vehicle_api::FlightStatus status;
        status.in_air = flying;
        fc_state_.state.status = status;
    }

    void set_mission_state(vehicle_api::MissionState state) {
        vehicle_api::FollowStatus status;
        status.mission_state = state;
        follow_status_.simulate_status(status, clock_.now_monotonic_ns());
    }

    const GcsPeer first_ = gcs_peer(250, "192.168.0.10");
    const GcsPeer second_ = gcs_peer(251, "192.168.0.11");

    core_testing::FakeClock clock_;
    FakeGcsLink link_;
    GcsRegistry registry_{link_, clock_};
    FakeFcStateSource fc_state_;
    FakeFcCommandStatus command_status_;
    CommandActivity activity_;
    vehicle_api_testing::FakeFollowStatusSource follow_status_;
    FakeLandingLock landing_lock_;
    ControlAuthority authority_{ControlAuthorityInputs{registry_, registry_, clock_, fc_state_,
                                                       command_status_, activity_, follow_status_,
                                                       landing_lock_}};
};

TEST_F(ControlAuthorityTest, FirstGcsHeartbeatGetsControl) {
    heartbeat(first_);
    heartbeat(second_);

    EXPECT_EQ(authority_.controller(), std::optional<std::uint8_t>{250});
    EXPECT_TRUE(authority_.is_controller(250));
    EXPECT_FALSE(authority_.is_controller(251));
}

TEST_F(ControlAuthorityTest, NoControlIsGrantedWhileFollowing) {
    set_mission_state(vehicle_api::MissionState::FollowActive);

    heartbeat(first_);

    EXPECT_EQ(authority_.controller(), std::nullopt);
}

TEST_F(ControlAuthorityTest, OldFollowStatusDoesNotBlockGrant) {
    set_mission_state(vehicle_api::MissionState::FollowActive);
    clock_.advance_ns(3 * kSecond + 1);  // follow/status 가 끊김: 추종 없음으로 본다

    heartbeat(first_);

    EXPECT_EQ(authority_.controller(), std::optional<std::uint8_t>{250});
}

TEST_F(ControlAuthorityTest, NoControlIsGrantedWhileSendingSetpoints) {
    command_status_.sending = true;

    heartbeat(first_);

    EXPECT_EQ(authority_.controller(), std::nullopt);
}

TEST_F(ControlAuthorityTest, RequestIsRejectedWhileAliveControllerIsFlying) {
    heartbeat(first_);
    heartbeat(second_);
    set_flying(true);

    EXPECT_EQ(authority_.request(second_, ControlAction::Request), MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(authority_.controller(), std::optional<std::uint8_t>{250});
}

TEST_F(ControlAuthorityTest, RequestTakesControlAfterControllerHeartbeatStops) {
    heartbeat(first_);
    set_flying(true);
    clock_.advance_ns(3 * kSecond + 1);
    heartbeat(second_);

    EXPECT_EQ(authority_.request(second_, ControlAction::Request), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(authority_.controller(), std::optional<std::uint8_t>{251});
}

TEST_F(ControlAuthorityTest, RequestIsRejectedWhileCommandInProgressOrLandingLock) {
    heartbeat(first_);
    activity_.set_command_in_progress(true);
    EXPECT_EQ(authority_.request(second_, ControlAction::Request), MAV_RESULT_TEMPORARILY_REJECTED);

    activity_.set_command_in_progress(false);
    landing_lock_.locked = true;
    EXPECT_EQ(authority_.request(second_, ControlAction::Request), MAV_RESULT_TEMPORARILY_REJECTED);
}

TEST_F(ControlAuthorityTest, ReleaseByControllerLetsNextHeartbeatTakeControl) {
    heartbeat(first_);

    EXPECT_EQ(authority_.request(second_, ControlAction::Release), MAV_RESULT_DENIED);
    EXPECT_EQ(authority_.request(first_, ControlAction::Release), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(authority_.controller(), std::nullopt);

    heartbeat(second_);
    EXPECT_EQ(authority_.controller(), std::optional<std::uint8_t>{251});
}

}  // namespace
}  // namespace argus::mavlink_gateway
