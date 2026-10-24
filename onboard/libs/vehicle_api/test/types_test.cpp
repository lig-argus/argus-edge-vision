#include "argus/vehicle_api/types.hpp"

#include <gtest/gtest.h>

#include "argus/mavlink.h"
// 인터페이스 헤더가 ZeroMQ·MessagePack 없이 각각 컴파일되는지 함께 확인한다
#include "argus/vehicle_api/follow_command.hpp"
#include "argus/vehicle_api/mode_control.hpp"
#include "argus/vehicle_api/vehicle_state.hpp"
#include "argus/vehicle_api/velocity_setpoint_sender.hpp"

namespace argus::vehicle_api {
namespace {

TEST(CommandIdTest, ComparesByValue) {
    EXPECT_EQ(CommandId{42}, CommandId{42});
    EXPECT_NE(CommandId{42}, CommandId{43});
    EXPECT_EQ(CommandId{}.value, 0U);
}

TEST(ScanRequestTest, DefaultsToEmptyMeaningConfiguredDefault) {
    const ScanRequest scan;

    EXPECT_FALSE(scan.yaw_range_rad.has_value());
    EXPECT_FALSE(scan.yaw_rate_rad_s.has_value());
    EXPECT_FALSE(scan.timeout_s.has_value());
}

TEST(SensorTest, MatchesMavlinkArgusSensor) {
    EXPECT_EQ(static_cast<int>(Sensor::Unknown), ARGUS_SENSOR_UNKNOWN);
    EXPECT_EQ(static_cast<int>(Sensor::Ir), ARGUS_SENSOR_IR);
    EXPECT_EQ(static_cast<int>(Sensor::Eo), ARGUS_SENSOR_EO);
}

TEST(FollowResultTest, DefaultsToRejectedAndNotAccepted) {
    const FollowResult result;

    EXPECT_EQ(result.outcome, Outcome::Rejected);
    EXPECT_FALSE(result.request_accepted);
}

TEST(VehicleStateTest, OptionalFieldsAreEmptyUntilReceived) {
    const VehicleState state;

    EXPECT_FALSE(state.local.has_value());
    EXPECT_FALSE(state.downward_distance_m.has_value());
    EXPECT_FALSE(state.global.has_value());
    EXPECT_FALSE(state.attitude.fc_time_monotonic_ns.has_value());
    EXPECT_EQ(state.mode, FlightMode::Unknown);
}

}  // namespace
}  // namespace argus::vehicle_api
