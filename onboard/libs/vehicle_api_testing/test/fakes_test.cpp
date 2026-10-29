#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_mode_control.hpp"
#include "argus/vehicle_api_testing/fake_vehicle_state.hpp"
#include "argus/vehicle_api_testing/fake_velocity_setpoint_sender.hpp"

namespace argus::vehicle_api_testing {
namespace {

using vehicle_api::Attitude;
using vehicle_api::DownwardDistance;
using vehicle_api::FlightMode;
using vehicle_api::FlightStatus;
using vehicle_api::LocalPosition;
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

    FlightStatus status;
    status.mode = FlightMode::Hold;
    VehicleState state;
    state.status = status;
    vehicle.set_latest(state);

    const auto latest = vehicle.latest();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest.value_or(VehicleState{}).status.value_or(FlightStatus{}).mode,
              FlightMode::Hold);
}

TEST(FakeVehicleStateTest, AttitudeAtReturnsNearestWithinTolerance) {
    FakeVehicleState vehicle;
    vehicle.set_now_monotonic_ns(140 * kMs);
    vehicle.add_attitude(attitude_received_at(100 * kMs, 0.1));
    vehicle.add_attitude(attitude_received_at(130 * kMs, 0.2));

    const auto nearest = vehicle.attitude_at(125 * kMs);

    ASSERT_TRUE(nearest.has_value());
    EXPECT_DOUBLE_EQ(nearest.value_or(Attitude{}).yaw_rad, 0.2);
}

TEST(FakeVehicleStateTest, AttitudeAtIsEmptyBeyondTolerance) {
    FakeVehicleState vehicle;
    vehicle.set_now_monotonic_ns(160 * kMs);
    vehicle.add_attitude(attitude_received_at(100 * kMs, 0.1));

    EXPECT_TRUE(vehicle.attitude_at(150 * kMs).has_value());   // 차 50 ms: 허용
    EXPECT_FALSE(vehicle.attitude_at(151 * kMs).has_value());  // 차 51 ms: 없음
}

TEST(FakeVehicleStateTest, AttitudeAtSkipsStaleSampleAndPicksFreshNearest) {
    // 프레임 0 ms, 표본 10 ms 와 45 ms, 조회 240 ms: 10 ms 표본은 가장 가깝지만 경과 230 ms 로
    // 무효이고, 45 ms 표본은 차 45 ms 와 경과 195 ms 로 유효하다
    FakeVehicleState vehicle;
    vehicle.set_now_monotonic_ns(240 * kMs);
    vehicle.add_attitude(attitude_received_at(10 * kMs, 0.1));
    vehicle.add_attitude(attitude_received_at(45 * kMs, 0.2));

    const auto picked = vehicle.attitude_at(0);

    ASSERT_TRUE(picked.has_value());
    EXPECT_DOUBLE_EQ(picked.value_or(Attitude{}).yaw_rad, 0.2);
}

TEST(FakeVehicleStateTest, AttitudeAtIsEmptyWhenEverySampleIsStale) {
    FakeVehicleState vehicle;
    vehicle.add_attitude(attitude_received_at(100 * kMs, 0.1));

    vehicle.set_now_monotonic_ns(300 * kMs);  // 경과 200 ms: 허용
    EXPECT_TRUE(vehicle.attitude_at(100 * kMs).has_value());
    vehicle.set_now_monotonic_ns(301 * kMs);  // 경과 201 ms: 없음
    EXPECT_FALSE(vehicle.attitude_at(100 * kMs).has_value());
}

TEST(FakeVehicleStateTest, LocalPositionAtAndDownwardDistanceAtFollowTheSameRule) {
    FakeVehicleState vehicle;
    vehicle.set_now_monotonic_ns(240 * kMs);
    LocalPosition stale;
    stale.received_monotonic_ns = 10 * kMs;
    stale.north_m = 1.0;
    LocalPosition fresh;
    fresh.received_monotonic_ns = 45 * kMs;
    fresh.north_m = 2.0;
    vehicle.add_local_position(stale);
    vehicle.add_local_position(fresh);
    DownwardDistance far;
    far.received_monotonic_ns = 100 * kMs;
    far.distance_m = 3.0;
    vehicle.add_downward_distance(far);

    const auto position = vehicle.local_position_at(0);
    ASSERT_TRUE(position.has_value());
    EXPECT_DOUBLE_EQ(position.value_or(LocalPosition{}).north_m, 2.0);

    EXPECT_FALSE(vehicle.downward_distance_at(0).has_value());  // 차 100 ms: 없음
    EXPECT_TRUE(vehicle.downward_distance_at(60 * kMs).has_value());
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

TEST(FakeVelocitySetpointSenderTest, RecordsSentSetpointsInOrder) {
    FakeVelocitySetpointSender setpoint;
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
