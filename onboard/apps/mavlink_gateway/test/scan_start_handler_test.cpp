#include "argus/mavlink_gateway/scan_start_handler.hpp"

#include <gtest/gtest.h>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/follow_command_policy.hpp"
#include "argus/vehicle_api_testing/fake_follow_control_client.hpp"
#include "follow_command_handler_rig.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway {
namespace {

mavlink_command_long_t scan_start(float p1, float p2, float p3) {
    mavlink_command_long_t command{};
    command.command = MAV_CMD_ARGUS_SCAN_START;
    command.param1 = p1;
    command.param2 = p2;
    command.param3 = p3;
    command.param4 = test_messages::kNan;
    command.param5 = test_messages::kNan;
    command.param6 = test_messages::kNan;
    command.param7 = 1;
    return command;
}

class ScanStartHandlerTest : public ::testing::Test {
protected:
    testing::FollowCommandHandlerRig rig_;
    ScanStartHandler handler_{
        ScanStartHandlerInputs{rig_.headers, rig_.follow_control, rig_.tracker}};
};

TEST_F(ScanStartHandlerTest, RulesAreNumberedControlledStart) {
    EXPECT_EQ(handler_.command(), MAV_CMD_ARGUS_SCAN_START);
    EXPECT_TRUE(ScanStartHandler::kRules.numbered);
    EXPECT_TRUE(ScanStartHandler::kRules.requires_control);
    EXPECT_EQ(ScanStartHandler::kRules.kind, CommandKind::Start);
}

TEST_F(ScanStartHandlerTest, NanParamsAreSentAsDefaults) {
    const float nan = test_messages::kNan;

    rig_.handle(handler_, scan_start(nan, nan, nan));

    ASSERT_EQ(rig_.acks.size(), 1U);
    EXPECT_EQ(rig_.acks[0].result, MAV_RESULT_IN_PROGRESS);
    EXPECT_EQ(rig_.acks[0].progress, 0);
    ASSERT_EQ(rig_.follow_control.commands().size(), 1U);
    const auto& record = rig_.follow_control.commands()[0];
    EXPECT_EQ(record.kind, vehicle_api_testing::FollowCommandKind::ScanStart);
    ASSERT_TRUE(record.scan.has_value());
    const vehicle_api::ScanRequest scan = record.scan.value_or(vehicle_api::ScanRequest{});
    EXPECT_DOUBLE_EQ(scan.yaw_range_rad.value_or(0.0), kDefaultScanYawRangeRad);
    EXPECT_DOUBLE_EQ(scan.yaw_rate_rad_s.value_or(0.0), kDefaultScanYawRateRadS);
    EXPECT_DOUBLE_EQ(scan.timeout_s.value_or(0.0), kDefaultScanTimeoutS);
}

TEST_F(ScanStartHandlerTest, OutOfRangeIsDeniedWithoutRequest) {
    rig_.handle(handler_, scan_start(1.0F, 0.0F, 10.0F));

    ASSERT_EQ(rig_.acks.size(), 1U);
    EXPECT_EQ(rig_.acks[0].result, MAV_RESULT_DENIED);
    EXPECT_FALSE(rig_.acks[0].reason.empty());
    EXPECT_TRUE(rig_.follow_control.commands().empty());
}

}  // namespace
}  // namespace argus::mavlink_gateway
