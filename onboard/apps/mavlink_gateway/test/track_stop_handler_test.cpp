#include "argus/mavlink_gateway/track_stop_handler.hpp"

#include <gtest/gtest.h>

#include <cstdint>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/fc_command_lock_release.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_follow_control_client.hpp"
#include "follow_command_handler_rig.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway {
namespace {

using testing::kSecond;

class FakeLockRelease : public IFcCommandLockRelease {
public:
    bool release_lock() override {
        ++calls;
        return result;
    }

    std::int32_t calls = 0;
    bool result = true;
};

mavlink_command_long_t track_stop() {
    mavlink_command_long_t command{};
    command.command = MAV_CMD_ARGUS_TRACK_STOP;
    command.param1 = test_messages::kNan;
    command.param7 = 1;
    return command;
}

class TrackStopHandlerTest : public ::testing::Test {
protected:
    testing::FollowCommandHandlerRig rig_;
    FakeLockRelease lock_;
    TrackStopHandler handler_{TrackStopHandlerInputs{
        rig_.clock, lock_, rig_.follow_status, rig_.headers, rig_.follow_control, rig_.tracker}};
};

TEST_F(TrackStopHandlerTest, RulesAreNumberedControlledStop) {
    EXPECT_EQ(handler_.command(), MAV_CMD_ARGUS_TRACK_STOP);
    EXPECT_TRUE(TrackStopHandler::kRules.numbered);
    EXPECT_TRUE(TrackStopHandler::kRules.requires_control);
    EXPECT_EQ(TrackStopHandler::kRules.kind, CommandKind::Stop);
}

TEST_F(TrackStopHandlerTest, IdleReleasesLockAndAcceptsAtOnce) {
    rig_.set_mission_state(vehicle_api::MissionState::HoldWait);

    rig_.handle(handler_, track_stop());

    EXPECT_EQ(lock_.calls, 1);
    ASSERT_EQ(rig_.acks.size(), 1U);
    EXPECT_EQ(rig_.acks[0].result, MAV_RESULT_ACCEPTED);
    EXPECT_TRUE(rig_.follow_control.commands().empty());
}

TEST_F(TrackStopHandlerTest, FollowingSendsInProgressAndOperatorStop) {
    rig_.set_mission_state(vehicle_api::MissionState::FollowActive);

    rig_.handle(handler_, track_stop());

    EXPECT_EQ(lock_.calls, 1);
    ASSERT_EQ(rig_.acks.size(), 1U);
    EXPECT_EQ(rig_.acks[0].result, MAV_RESULT_IN_PROGRESS);
    ASSERT_EQ(rig_.follow_control.commands().size(), 1U);
    const auto& record = rig_.follow_control.commands()[0];
    EXPECT_EQ(record.kind, vehicle_api_testing::FollowCommandKind::Stop);
    EXPECT_EQ(record.reason, vehicle_api::StopReason::Operator);
}

TEST_F(TrackStopHandlerTest, FaultSendsStopForRecovery) {
    rig_.set_mission_state(vehicle_api::MissionState::Fault);

    rig_.handle(handler_, track_stop());

    ASSERT_EQ(rig_.follow_control.commands().size(), 1U);
    EXPECT_EQ(rig_.acks[0].result, MAV_RESULT_IN_PROGRESS);
}

TEST_F(TrackStopHandlerTest, StaleFollowStatusAcceptsAtOnce) {
    rig_.set_mission_state(vehicle_api::MissionState::FollowActive);
    rig_.clock.advance_ns(3 * kSecond + 1);

    rig_.handle(handler_, track_stop());

    ASSERT_EQ(rig_.acks.size(), 1U);
    EXPECT_EQ(rig_.acks[0].result, MAV_RESULT_ACCEPTED);
    EXPECT_TRUE(rig_.follow_control.commands().empty());
}

TEST_F(TrackStopHandlerTest, LockReleaseFailureStillHandlesStop) {
    lock_.result = false;
    rig_.set_mission_state(vehicle_api::MissionState::Idle);

    rig_.handle(handler_, track_stop());

    EXPECT_EQ(lock_.calls, 1);
    ASSERT_EQ(rig_.acks.size(), 1U);
    EXPECT_EQ(rig_.acks[0].result, MAV_RESULT_ACCEPTED);
}

}  // namespace
}  // namespace argus::mavlink_gateway
