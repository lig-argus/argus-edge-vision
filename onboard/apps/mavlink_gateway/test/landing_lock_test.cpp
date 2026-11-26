#include "argus/mavlink_gateway/landing_lock.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/fc_landing_release.hpp"
#include "argus/mavlink_gateway/request_header_factory.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_follow_control.hpp"
#include "argus/vehicle_api_testing/fake_follow_control_client.hpp"
#include "argus/vehicle_api_testing/fake_follow_status_source.hpp"
#include "fake_command_restrictions.hpp"
#include "fake_fc_epoch_source.hpp"
#include "fake_fc_state_source.hpp"
#include "fake_forwarded_command_events.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::FlightMode;

constexpr std::int64_t kSecond = 1'000'000'000;

class FakeLandingRelease : public IFcLandingRelease {
public:
    void release_for_landing() override { ++calls; }

    std::int32_t calls = 0;
};

class LandingLockTest : public ::testing::Test {
protected:
    LandingLockTest() {
        clock_.advance_ns(100 * kSecond);
        fc_state_.set_status(FlightMode::Offboard, clock_.now_monotonic_ns());
        started_ = lock_.subscribe_started([this] { ++started_count_; });
    }

    void set_mission_state(vehicle_api::MissionState state) {
        vehicle_api::FollowStatus status;
        status.mission_state = state;
        follow_status_.simulate_status(status, clock_.now_monotonic_ns());
    }

    void forward_land() { forwarded_.simulate_forwarded(MAV_CMD_NAV_LAND); }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    FakeForwardedCommandEvents forwarded_;
    FakeFcStateSource fc_state_;
    FakeRcOverride rc_override_;
    FakeLandingRelease release_;
    vehicle_api_testing::FakeFollowStatusSource follow_status_;
    vehicle_api_testing::FakeFollowControlClient follow_control_{poller_, clock_};
    FakeFcEpochSource epoch_;
    RequestHeaderFactory headers_{"session-l", clock_, epoch_};
    LandingLock lock_{LandingLockInputs{forwarded_, fc_state_, rc_override_, clock_, release_,
                                        follow_status_, follow_control_, headers_}};
    std::int32_t started_count_ = 0;
    core::Subscription started_;
};

TEST_F(LandingLockTest, ForwardedLandStartsLockAndReleasesFcControl) {
    EXPECT_FALSE(lock_.active());

    forward_land();

    EXPECT_TRUE(lock_.active());
    EXPECT_EQ(release_.calls, 1);
    EXPECT_EQ(started_count_, 1);
}

TEST_F(LandingLockTest, OtherForwardedCommandsDoNotStartLock) {
    forwarded_.simulate_forwarded(MAV_CMD_DO_SET_MODE, FlightMode::Position);
    forwarded_.simulate_forwarded(MAV_CMD_NAV_RETURN_TO_LAUNCH);

    EXPECT_FALSE(lock_.active());
    EXPECT_EQ(release_.calls, 0);
}

TEST_F(LandingLockTest, RepeatedLandDoesNotRepeatStartActions) {
    forward_land();
    forward_land();

    EXPECT_EQ(release_.calls, 1);
    EXPECT_EQ(started_count_, 1);
}

TEST_F(LandingLockTest, RequestsOnboardStopOnlyWhileFollowing) {
    set_mission_state(vehicle_api::MissionState::FollowActive);
    forward_land();

    ASSERT_EQ(follow_control_.commands().size(), 1U);
    EXPECT_EQ(follow_control_.commands()[0].kind, vehicle_api_testing::FollowCommandKind::Stop);
    EXPECT_EQ(follow_control_.commands()[0].reason,
              std::optional<vehicle_api::StopReason>{vehicle_api::StopReason::Onboard});
}

TEST_F(LandingLockTest, NoStopRequestWhenNotFollowing) {
    set_mission_state(vehicle_api::MissionState::HoldWait);
    forward_land();

    EXPECT_TRUE(follow_control_.commands().empty());
}

TEST_F(LandingLockTest, FcRejectionReleasesLockButAcceptanceKeepsIt) {
    forward_land();
    forwarded_.simulate_answered(MAV_CMD_NAV_LAND, MAV_RESULT_IN_PROGRESS);
    forwarded_.simulate_answered(MAV_CMD_NAV_LAND, MAV_RESULT_ACCEPTED);
    EXPECT_TRUE(lock_.active());

    forwarded_.simulate_answered(MAV_CMD_NAV_LAND, MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_FALSE(lock_.active());
}

TEST_F(LandingLockTest, DisarmAfterLockReleasesIt) {
    forward_land();
    lock_.tick();
    EXPECT_TRUE(lock_.active());

    clock_.advance_ns(kSecond);
    fc_state_.set_status(FlightMode::Land, clock_.now_monotonic_ns(), false);
    lock_.tick();

    EXPECT_FALSE(lock_.active());
}

TEST_F(LandingLockTest, StatusOfEarlierFcEpochIsNotUsed) {
    forward_land();
    clock_.advance_ns(kSecond);
    fc_state_.set_status(FlightMode::Land, clock_.now_monotonic_ns(), false);
    fc_state_.epoch = 2;  // 비행 상태는 이전 fc_epoch 의 값

    lock_.tick();

    EXPECT_TRUE(lock_.active());
}

TEST_F(LandingLockTest, ReturnAfterPilotTakeoverReleasesLock) {
    forward_land();
    rc_override_.pilot = true;
    lock_.tick();
    EXPECT_TRUE(lock_.active());

    rc_override_.pilot = false;  // 통제기의 복귀 명령이 성공함
    lock_.tick();

    EXPECT_FALSE(lock_.active());
}

}  // namespace
}  // namespace argus::mavlink_gateway
