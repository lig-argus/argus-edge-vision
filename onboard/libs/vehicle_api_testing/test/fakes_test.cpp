#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_flight_mode_control.hpp"
#include "argus/vehicle_api_testing/fake_flight_mode_control_client.hpp"
#include "argus/vehicle_api_testing/fake_follow_control.hpp"
#include "argus/vehicle_api_testing/fake_follow_control_client.hpp"
#include "argus/vehicle_api_testing/fake_vehicle_telemetry.hpp"
#include "argus/vehicle_api_testing/fake_velocity_setpoint_sink.hpp"

namespace argus::vehicle_api_testing {
namespace {

using vehicle_api::Attitude;
using vehicle_api::CommandId;
using vehicle_api::DownwardDistance;
using vehicle_api::FlightMode;
using vehicle_api::FlightModeEvent;
using vehicle_api::FlightModeReply;
using vehicle_api::FlightStatus;
using vehicle_api::FollowOutcome;
using vehicle_api::FollowResult;
using vehicle_api::LocalPosition;
using vehicle_api::RequestHeader;
using vehicle_api::RequestId;
using vehicle_api::StopReason;
using vehicle_api::TrackTarget;
using vehicle_api::VehicleState;
using vehicle_api::VelocitySetpoint;

constexpr std::int64_t kMs = 1'000'000;

Attitude attitude_received_at(std::int64_t received_monotonic_ns, double yaw_rad) {
    Attitude attitude;
    attitude.received_monotonic_ns = received_monotonic_ns;
    attitude.yaw_rad = yaw_rad;
    return attitude;
}

RequestHeader header_with_id(std::uint64_t command_id) {
    RequestHeader header;
    header.id.command_id = CommandId{command_id};
    header.valid_until_monotonic_ns = 10 * kMs;
    return header;
}

FlightModeReply flight_mode_reply(CommandId command_id, bool accepted, const char* reason) {
    FlightModeReply reply;
    reply.id = RequestId{};
    reply.id->command_id = command_id;
    reply.accepted = accepted;
    reply.reason = reason;
    return reply;
}

std::optional<CommandId> command_id_of(const FlightModeReply& reply) {
    if (!reply.id) {
        return std::nullopt;
    }
    return reply.id->command_id;
}

std::optional<CommandId> command_id_of(const std::optional<FlightModeReply>& reply) {
    return reply ? command_id_of(*reply) : std::nullopt;
}

TEST(FakeVehicleTelemetryTest, LatestIsEmptyUntilSet) {
    FakeVehicleTelemetry vehicle;
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

TEST(FakeVehicleTelemetryTest, AttitudeAtReturnsNearestWithinTolerance) {
    FakeVehicleTelemetry vehicle;
    vehicle.set_now_monotonic_ns(140 * kMs);
    vehicle.add_attitude(attitude_received_at(100 * kMs, 0.1));
    vehicle.add_attitude(attitude_received_at(130 * kMs, 0.2));

    const auto nearest = vehicle.attitude_at(125 * kMs);

    ASSERT_TRUE(nearest.has_value());
    EXPECT_DOUBLE_EQ(nearest.value_or(Attitude{}).yaw_rad, 0.2);
}

TEST(FakeVehicleTelemetryTest, AttitudeAtIsEmptyBeyondTolerance) {
    FakeVehicleTelemetry vehicle;
    vehicle.set_now_monotonic_ns(160 * kMs);
    vehicle.add_attitude(attitude_received_at(100 * kMs, 0.1));

    EXPECT_TRUE(vehicle.attitude_at(150 * kMs).has_value());   // 차 50 ms: 허용
    EXPECT_FALSE(vehicle.attitude_at(151 * kMs).has_value());  // 차 51 ms: 없음
}

TEST(FakeVehicleTelemetryTest, AttitudeAtSkipsStaleSampleAndPicksFreshNearest) {
    // 프레임 0 ms, 표본 10 ms 와 45 ms, 조회 240 ms: 10 ms 표본은 가장 가깝지만 경과 230 ms 로
    // 무효이고, 45 ms 표본은 차 45 ms 와 경과 195 ms 로 유효하다
    FakeVehicleTelemetry vehicle;
    vehicle.set_now_monotonic_ns(240 * kMs);
    vehicle.add_attitude(attitude_received_at(10 * kMs, 0.1));
    vehicle.add_attitude(attitude_received_at(45 * kMs, 0.2));

    const auto picked = vehicle.attitude_at(0);

    ASSERT_TRUE(picked.has_value());
    EXPECT_DOUBLE_EQ(picked.value_or(Attitude{}).yaw_rad, 0.2);
}

TEST(FakeVehicleTelemetryTest, AttitudeAtIsEmptyWhenEverySampleIsStale) {
    FakeVehicleTelemetry vehicle;
    vehicle.add_attitude(attitude_received_at(100 * kMs, 0.1));

    vehicle.set_now_monotonic_ns(300 * kMs);  // 경과 200 ms: 허용
    EXPECT_TRUE(vehicle.attitude_at(100 * kMs).has_value());
    vehicle.set_now_monotonic_ns(301 * kMs);  // 경과 201 ms: 없음
    EXPECT_FALSE(vehicle.attitude_at(100 * kMs).has_value());
}

TEST(FakeVehicleTelemetryTest, LocalPositionAtAndDownwardDistanceAtFollowTheSameRule) {
    FakeVehicleTelemetry vehicle;
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

TEST(FakeVehicleTelemetryTest, EmitFlightModeEventCallsEverySubscribedHandler) {
    FakeVehicleTelemetry vehicle;
    std::vector<FlightMode> first;
    std::vector<FlightMode> second;
    const core::Subscription first_subscription = vehicle.subscribe_flight_mode_events(
        [&first](const FlightModeEvent& event) { first.push_back(event.mode); });
    core::Subscription second_subscription = vehicle.subscribe_flight_mode_events(
        [&second](const FlightModeEvent& event) { second.push_back(event.mode); });

    FlightModeEvent event;
    event.mode = FlightMode::Offboard;
    vehicle.simulate_flight_mode_event(event);

    second_subscription.reset();  // 등록을 풀면 더 받지 않는다
    event.mode = FlightMode::Hold;
    vehicle.simulate_flight_mode_event(event);

    EXPECT_EQ(first, (std::vector<FlightMode>{FlightMode::Offboard, FlightMode::Hold}));
    EXPECT_EQ(second, std::vector<FlightMode>{FlightMode::Offboard});
}

TEST(FakeVelocitySetpointSinkTest, RecordsSentSetpointsInOrder) {
    FakeVelocitySetpointSink sink;
    VelocitySetpoint first;
    first.forward_m_s = 1.0;
    VelocitySetpoint second;
    second.yaw_rate_rad_s = 0.5;

    sink.send(first);
    sink.send(second);

    ASSERT_EQ(sink.sent().size(), 2U);
    EXPECT_DOUBLE_EQ(sink.sent()[0].forward_m_s, 1.0);
    EXPECT_DOUBLE_EQ(sink.sent()[1].yaw_rate_rad_s, 0.5);
}

TEST(FakeFlightModeControlTest, AcceptsByDefaultAndRecordsRequestsWithCommandId) {
    FakeFlightModeControl mode;

    const FlightModeReply offboard = mode.request_offboard(header_with_id(7));
    const FlightModeReply heading = mode.request_heading(header_with_id(8), 1.5);

    EXPECT_TRUE(offboard.accepted);
    EXPECT_EQ(command_id_of(offboard), CommandId{7});
    EXPECT_TRUE(heading.accepted);
    EXPECT_EQ(command_id_of(heading), CommandId{8});

    ASSERT_EQ(mode.requests().size(), 2U);
    EXPECT_EQ(mode.requests()[0].kind, FlightModeRequestKind::Offboard);
    EXPECT_FALSE(mode.requests()[0].heading_rad.has_value());
    EXPECT_EQ(mode.requests()[1].kind, FlightModeRequestKind::Heading);
    EXPECT_EQ(mode.requests()[1].heading_rad, 1.5);
    EXPECT_EQ(mode.requests()[1].header.valid_until_monotonic_ns, 10 * kMs);
}

TEST(FakeFlightModeControlTest, ReturnsConfiguredReply) {
    FakeFlightModeControl mode;
    mode.set_reply({std::nullopt, false, std::string("not armed")});

    const auto reply = mode.request_hold(header_with_id(9));

    EXPECT_FALSE(reply.accepted);
    EXPECT_EQ(reply.reason, "not armed");
    EXPECT_EQ(command_id_of(reply), CommandId{9});
}

TEST(FakeFollowControlTest, InProgressByDefaultAndRecordsCommands) {
    FakeFollowControl follow;
    TrackTarget target;
    target.track_id = 3;

    const FollowResult started = follow.track_start(header_with_id(1), target);
    const FollowResult stopped = follow.stop(header_with_id(2), StopReason::Operator);

    EXPECT_EQ(started.outcome, FollowOutcome::InProgress);
    EXPECT_TRUE(started.request_accepted);
    EXPECT_EQ(started.id.command_id, CommandId{1});
    EXPECT_EQ(stopped.id.command_id, CommandId{2});

    ASSERT_EQ(follow.commands().size(), 2U);
    EXPECT_EQ(follow.commands()[0].kind, FollowCommandKind::TrackStart);
    EXPECT_EQ(follow.commands()[0].target.value_or(TrackTarget{}).track_id, 3U);
    EXPECT_EQ(follow.commands()[1].kind, FollowCommandKind::Stop);
    EXPECT_EQ(follow.commands()[1].reason, StopReason::Operator);
}

TEST(FakeFollowControlTest, ReturnsConfiguredResult) {
    FakeFollowControl follow;
    FollowResult rejected;
    rejected.outcome = FollowOutcome::Rejected;
    rejected.reason = "no such track";
    follow.set_result(rejected);

    const FollowResult result = follow.scan_start(header_with_id(4), {});

    EXPECT_EQ(result.outcome, FollowOutcome::Rejected);
    EXPECT_FALSE(result.request_accepted);
    EXPECT_EQ(result.reason, "no such track");
    EXPECT_EQ(result.id.command_id, CommandId{4});
}

class FakeClientTest : public ::testing::Test {
protected:
    void process() {
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }
    void process_until(std::chrono::milliseconds wait) {
        poller_.process_events(wait, std::chrono::milliseconds{5});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
};

TEST_F(FakeClientTest, FollowControlClientDeliversReplyInsideProcessEvents) {
    FakeFollowControlClient client(poller_, clock_);
    std::vector<std::optional<FollowResult>> results;
    const core::Subscription call = client.track_start(
        header_with_id(5), TrackTarget{}, std::chrono::milliseconds{500},
        [&results](std::optional<FollowResult> result) { results.push_back(result); });

    ASSERT_EQ(client.commands().size(), 1U);
    EXPECT_EQ(client.pending_count(), 1U);
    FollowResult in_progress;
    in_progress.id.command_id = CommandId{5};
    in_progress.outcome = FollowOutcome::InProgress;
    in_progress.request_accepted = true;
    EXPECT_TRUE(client.simulate_reply(in_progress));
    EXPECT_TRUE(results.empty());  // process_events() 전에는 전달되지 않는다

    process();
    ASSERT_EQ(results.size(), 1U);
    ASSERT_TRUE(results[0].has_value());
    EXPECT_EQ(results[0]->outcome, FollowOutcome::InProgress);
    EXPECT_EQ(client.pending_count(), 0U);
    EXPECT_FALSE(client.simulate_reply({}));  // 기다리는 명령이 없다
}

TEST_F(FakeClientTest, FollowControlClientTimesOutWithNullopt) {
    FakeFollowControlClient client(poller_, clock_);
    std::vector<std::optional<FollowResult>> results;
    const core::Subscription call =
        client.stop(header_with_id(6), StopReason::LinkLoss, std::chrono::milliseconds{500},
                    [&results](std::optional<FollowResult> result) { results.push_back(result); });

    process_until(std::chrono::milliseconds{499});
    EXPECT_TRUE(results.empty());

    process_until(std::chrono::milliseconds{1});
    ASSERT_EQ(results.size(), 1U);
    EXPECT_FALSE(results[0].has_value());

    EXPECT_TRUE(client.simulate_reply({}) == false);  // 시간 초과 뒤에는 기다리는 명령이 없다
}

TEST_F(FakeClientTest, ReleasedSubscriptionIsNeverCalled) {
    FakeFlightModeControlClient client(poller_, clock_);
    int calls = 0;
    core::Subscription call =
        client.request_offboard(header_with_id(7), std::chrono::milliseconds{100},
                                [&calls](const std::optional<FlightModeReply>&) { ++calls; });

    EXPECT_TRUE(client.simulate_reply(flight_mode_reply(CommandId{7}, true, "")));
    call.reset();
    process_until(std::chrono::milliseconds{200});

    EXPECT_EQ(calls, 0);
    ASSERT_EQ(client.requests().size(), 1U);
    EXPECT_EQ(client.requests()[0].kind, FlightModeRequestKind::Offboard);
}

TEST_F(FakeClientTest, FlightModeControlClientRepliesOldestRequestFirst) {
    FakeFlightModeControlClient client(poller_, clock_);
    std::vector<std::optional<CommandId>> replied;
    const auto record = [&replied](std::optional<FlightModeReply> reply) {
        replied.push_back(command_id_of(reply));
    };
    const core::Subscription hold =
        client.request_hold(header_with_id(1), std::chrono::milliseconds{500}, record);
    const core::Subscription heading =
        client.request_heading(header_with_id(2), 0.5, std::chrono::milliseconds{500}, record);

    EXPECT_TRUE(client.simulate_reply(flight_mode_reply(CommandId{1}, true, "")));
    EXPECT_TRUE(client.simulate_reply(flight_mode_reply(CommandId{2}, false, "busy")));
    process();

    ASSERT_EQ(replied.size(), 2U);
    EXPECT_EQ(replied[0], CommandId{1});
    EXPECT_EQ(replied[1], CommandId{2});
    EXPECT_EQ(client.requests()[1].heading_rad, 0.5);
}

}  // namespace
}  // namespace argus::vehicle_api_testing
