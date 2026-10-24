#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_mode_control.hpp"
#include "argus/vehicle_api_testing/fake_vehicle_state.hpp"
#include "argus/vehicle_api_testing/fake_velocity_setpoint.hpp"

namespace argus::vehicle_api_testing {
namespace {

using vehicle_api::Attitude;
using vehicle_api::FlightMode;
using vehicle_api::ModeEvent;
using vehicle_api::VehicleState;
using vehicle_api::VelocitySetpoint;

constexpr std::int64_t kMs = 1'000'000;

Attitude attitude_received_at(std::int64_t received_monotonic_ns, double yaw_rad) {
    Attitude attitude;
    attitude.received_monotonic_ns = received_monotonic_ns;
    attitude.yaw_rad = yaw_rad;
    return attitude;
}

TEST(FakeVehicleStateTest, LatestIsEmptyUntilSet) {
    FakeVehicleState vehicle;
    EXPECT_FALSE(vehicle.latest().has_value());

    VehicleState state;
    state.mode = FlightMode::Hold;
    vehicle.set_latest(state);

    const auto latest = vehicle.latest();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest.value_or(VehicleState{}).mode, FlightMode::Hold);
}

TEST(FakeVehicleStateTest, AttitudeAtReturnsNearestWithinTolerance) {
    FakeVehicleState vehicle;
    vehicle.add_attitude(attitude_received_at(100 * kMs, 0.1));
    vehicle.add_attitude(attitude_received_at(130 * kMs, 0.2));

    const auto nearest = vehicle.attitude_at(125 * kMs);

    ASSERT_TRUE(nearest.has_value());
    EXPECT_DOUBLE_EQ(nearest.value_or(Attitude{}).yaw_rad, 0.2);
}

TEST(FakeVehicleStateTest, AttitudeAtIsEmptyBeyondTolerance) {
    FakeVehicleState vehicle;
    vehicle.add_attitude(attitude_received_at(100 * kMs, 0.1));

    EXPECT_TRUE(vehicle.attitude_at(150 * kMs).has_value());   // 차 50 ms: 허용
    EXPECT_FALSE(vehicle.attitude_at(151 * kMs).has_value());  // 차 51 ms: 없음
}

TEST(FakeVehicleStateTest, EmitModeEventCallsEveryHandler) {
    FakeVehicleState vehicle;
    std::vector<FlightMode> first;
    std::vector<FlightMode> second;
    vehicle.on_mode_event([&first](const ModeEvent& event) { first.push_back(event.mode); });
    vehicle.on_mode_event([&second](const ModeEvent& event) { second.push_back(event.mode); });

    ModeEvent event;
    event.mode = FlightMode::Offboard;
    vehicle.emit_mode_event(event);

    EXPECT_EQ(first, std::vector<FlightMode>{FlightMode::Offboard});
    EXPECT_EQ(second, std::vector<FlightMode>{FlightMode::Offboard});
}

TEST(FakeVelocitySetpointTest, RecordsSentSetpointsInOrder) {
    FakeVelocitySetpoint setpoint;
    VelocitySetpoint first;
    first.forward_m_s = 1.0;
    VelocitySetpoint second;
    second.yaw_rate_rad_s = 0.5;

    setpoint.send(first);
    setpoint.send(second);

    ASSERT_EQ(setpoint.sent().size(), 2U);
    EXPECT_DOUBLE_EQ(setpoint.sent()[0].forward_m_s, 1.0);
    EXPECT_DOUBLE_EQ(setpoint.sent()[1].yaw_rate_rad_s, 0.5);
}

TEST(FakeModeControlTest, AcceptsByDefaultAndRecordsRequests) {
    FakeModeControl mode;

    EXPECT_TRUE(mode.request_offboard(10 * kMs).accepted);
    EXPECT_TRUE(mode.request_heading(1.5, 20 * kMs).accepted);

    ASSERT_EQ(mode.requests().size(), 2U);
    EXPECT_EQ(mode.requests()[0].kind, ModeRequestKind::Offboard);
    EXPECT_FALSE(mode.requests()[0].heading_rad.has_value());
    EXPECT_EQ(mode.requests()[1].kind, ModeRequestKind::Heading);
    EXPECT_EQ(mode.requests()[1].heading_rad, 1.5);
    EXPECT_EQ(mode.requests()[1].valid_until_monotonic_ns, 20 * kMs);
}

TEST(FakeModeControlTest, ReturnsConfiguredReply) {
    FakeModeControl mode;
    mode.set_reply({false, std::string("not armed")});

    const auto reply = mode.request_hold(10 * kMs);

    EXPECT_FALSE(reply.accepted);
    EXPECT_EQ(reply.reason, "not armed");
}

}  // namespace
}  // namespace argus::vehicle_api_testing
