#include "argus/vehicle_api/types.hpp"

#include <gtest/gtest.h>

#include <cstdint>

#include "argus/mavlink.h"
// 인터페이스 헤더가 ZeroMQ·protobuf 없이 각각 컴파일되는지 함께 확인한다
#include "argus/vehicle_api/flight_mode_control.hpp"
#include "argus/vehicle_api/flight_mode_control_client.hpp"
#include "argus/vehicle_api/follow_control.hpp"
#include "argus/vehicle_api/follow_control_client.hpp"
#include "argus/vehicle_api/follow_status_events.hpp"
#include "argus/vehicle_api/follow_status_source.hpp"
#include "argus/vehicle_api/vehicle_telemetry.hpp"
#include "argus/vehicle_api/velocity_setpoint_sink.hpp"

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
    EXPECT_EQ(static_cast<std::int32_t>(Sensor::Unknown), ARGUS_SENSOR_UNKNOWN);
    EXPECT_EQ(static_cast<std::int32_t>(Sensor::Ir), ARGUS_SENSOR_IR);
    EXPECT_EQ(static_cast<std::int32_t>(Sensor::Eo), ARGUS_SENSOR_EO);
}

TEST(MissionStateTest, MatchesMavlinkArgusMissionState) {
    EXPECT_EQ(static_cast<std::int32_t>(MissionState::Idle), ARGUS_MISSION_STATE_IDLE);
    EXPECT_EQ(static_cast<std::int32_t>(MissionState::Preparing), ARGUS_MISSION_STATE_PREPARING);
    EXPECT_EQ(static_cast<std::int32_t>(MissionState::FollowActive),
              ARGUS_MISSION_STATE_FOLLOW_ACTIVE);
    EXPECT_EQ(static_cast<std::int32_t>(MissionState::HoldTransition),
              ARGUS_MISSION_STATE_HOLD_TRANSITION);
    EXPECT_EQ(static_cast<std::int32_t>(MissionState::HoldWait), ARGUS_MISSION_STATE_HOLD_WAIT);
    EXPECT_EQ(static_cast<std::int32_t>(MissionState::ScanActive), ARGUS_MISSION_STATE_SCAN_ACTIVE);
    EXPECT_EQ(static_cast<std::int32_t>(MissionState::Manual), ARGUS_MISSION_STATE_MANUAL);
    EXPECT_EQ(static_cast<std::int32_t>(MissionState::Fault), ARGUS_MISSION_STATE_FAULT);
}

TEST(TargetStateTest, MatchesMavlinkArgusTargetState) {
    EXPECT_EQ(static_cast<std::int32_t>(TargetState::None), ARGUS_TARGET_STATE_NONE);
    EXPECT_EQ(static_cast<std::int32_t>(TargetState::Confirmed), ARGUS_TARGET_STATE_CONFIRMED);
    EXPECT_EQ(static_cast<std::int32_t>(TargetState::Predicted), ARGUS_TARGET_STATE_PREDICTED);
    EXPECT_EQ(static_cast<std::int32_t>(TargetState::Lost), ARGUS_TARGET_STATE_LOST);
}

TEST(FollowStatusTest, DefaultsToUnknownStatesWithoutOptionalValues) {
    const FollowStatus status;

    EXPECT_EQ(status.mission_state, MissionState::Unknown);
    EXPECT_EQ(status.target_state, TargetState::Unknown);
    EXPECT_FALSE(status.selected.has_value());
    EXPECT_TRUE(status.command_results.empty());
    EXPECT_FALSE(status.scan.has_value());
}

TEST(RequestIdTest, ComparesEveryField) {
    RequestId lhs;
    lhs.command_id = CommandId{1};
    lhs.session_id = "s";
    lhs.seq = 2;
    lhs.fc_epoch = 3;
    RequestId rhs = lhs;
    EXPECT_EQ(lhs, rhs);

    rhs.fc_epoch = 4;
    EXPECT_NE(lhs, rhs);
    rhs = lhs;
    rhs.session_id = "t";
    EXPECT_NE(lhs, rhs);
}

TEST(FollowResultTest, DefaultsToRejectedAndNotAccepted) {
    const FollowResult result;

    EXPECT_EQ(result.outcome, FollowOutcome::Rejected);
    EXPECT_FALSE(result.request_accepted);
    EXPECT_EQ(result.id, RequestId{});
}

TEST(FlightModeReplyTest, DefaultsToNotAcceptedWithoutCommandId) {
    const FlightModeReply reply;

    EXPECT_FALSE(reply.accepted);
    EXPECT_FALSE(reply.id.has_value());
}

TEST(VehicleStateTest, EveryValueIsEmptyUntilReceived) {
    const VehicleState state;

    EXPECT_FALSE(state.attitude.has_value());
    EXPECT_FALSE(state.local.has_value());
    EXPECT_FALSE(state.downward_distance.has_value());
    EXPECT_FALSE(state.global.has_value());
    EXPECT_FALSE(state.status.has_value());
}

TEST(VehicleStateTest, EveryValueCarriesItsOwnReceiveTime) {
    Attitude attitude;
    attitude.received_monotonic_ns = 10;
    LocalPosition local;
    local.received_monotonic_ns = 20;
    DownwardDistance distance;
    distance.received_monotonic_ns = 30;
    FlightStatus status;
    status.received_monotonic_ns = 40;
    VehicleState state;
    state.attitude = attitude;
    state.local = local;
    state.downward_distance = distance;
    state.status = status;

    EXPECT_EQ(state.attitude.value_or(Attitude{}).received_monotonic_ns, 10);
    EXPECT_EQ(state.local.value_or(LocalPosition{}).received_monotonic_ns, 20);
    EXPECT_EQ(state.downward_distance.value_or(DownwardDistance{}).received_monotonic_ns, 30);
    EXPECT_EQ(state.status.value_or(FlightStatus{}).received_monotonic_ns, 40);
    EXPECT_EQ(state.status.value_or(FlightStatus{}).mode, FlightMode::Unknown);
}

}  // namespace
}  // namespace argus::vehicle_api
