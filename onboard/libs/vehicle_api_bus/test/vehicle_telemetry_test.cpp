#include "argus/vehicle_api/vehicle_telemetry.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "argus/bus/message.hpp"
#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/bus_vehicle_telemetry_publisher.hpp"
#include "argus/vehicle_api_bus/bus_vehicle_telemetry_subscriber.hpp"
#include "argus_msg/fc_attitude.pb.h"

namespace argus::vehicle_api_bus {
namespace {

using vehicle_api::Attitude;
using vehicle_api::FlightMode;
using vehicle_api::FlightModeEvent;
using vehicle_api::VehicleState;

constexpr std::int64_t kMs = 1'000'000;

Attitude attitude_received_at(std::int64_t received_ns, double yaw_rad = 0.0) {
    Attitude attitude;
    attitude.yaw_rad = yaw_rad;
    attitude.received_monotonic_ns = received_ns;
    return attitude;
}

class VehicleTelemetryTest : public ::testing::Test {
protected:
    void SetUp() override {
        publisher_ = BusVehicleTelemetryPublisher::create(bus_);
        subscriber_ = BusVehicleTelemetrySubscriber::create(bus_, clock_);
        ASSERT_NE(publisher_, nullptr);
        ASSERT_NE(subscriber_, nullptr);
    }

    void process() { poller_.process_events(std::chrono::milliseconds{0}, kBudget); }

    // 시각을 at_ns 로 옮기고, 그 시각에 받은 자세를 발행해 구독자에게 전달한다
    void deliver_attitude_at(std::int64_t at_ns, double yaw_rad = 0.0) {
        clock_.advance_ns(at_ns - clock_.now_monotonic_ns());
        ASSERT_TRUE(publisher_->publish_attitude(attitude_received_at(at_ns, yaw_rad)));
        process();
    }

    static constexpr std::chrono::milliseconds kBudget{5};

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
    std::unique_ptr<BusVehicleTelemetryPublisher> publisher_;
    std::unique_ptr<BusVehicleTelemetrySubscriber> subscriber_;
};

TEST_F(VehicleTelemetryTest, LatestIsEmptyBeforeAnyMessage) {
    EXPECT_EQ(subscriber_->latest(), std::nullopt);
}

TEST_F(VehicleTelemetryTest, SubscriberReceivesTheAttitudeThePublisherSent) {
    Attitude sent;
    sent.roll_rad = 0.1;
    sent.pitch_rad = -0.2;
    sent.yaw_rad = 1.5;
    sent.roll_rate_rad_s = 0.01;
    sent.pitch_rate_rad_s = 0.02;
    sent.yaw_rate_rad_s = 0.03;
    sent.fc_time_boot_ms = 123'456;
    sent.received_monotonic_ns = 5 * kMs;
    sent.fc_time_monotonic_ns = 4 * kMs;
    sent.timesync_valid = true;
    sent.fc_epoch = 7;

    ASSERT_TRUE(publisher_->publish_attitude(sent));
    process();

    const std::optional<VehicleState> latest = subscriber_->latest();
    ASSERT_TRUE(latest.has_value());
    const std::optional<Attitude> received_attitude = latest.value_or(VehicleState{}).attitude;
    ASSERT_TRUE(received_attitude.has_value());
    const Attitude received = received_attitude.value_or(Attitude{});
    EXPECT_DOUBLE_EQ(received.roll_rad, 0.1);
    EXPECT_DOUBLE_EQ(received.pitch_rad, -0.2);
    EXPECT_DOUBLE_EQ(received.yaw_rad, 1.5);
    EXPECT_DOUBLE_EQ(received.roll_rate_rad_s, 0.01);
    EXPECT_DOUBLE_EQ(received.pitch_rate_rad_s, 0.02);
    EXPECT_DOUBLE_EQ(received.yaw_rate_rad_s, 0.03);
    EXPECT_EQ(received.fc_time_boot_ms, 123'456U);
    EXPECT_EQ(received.received_monotonic_ns, 5 * kMs);
    EXPECT_EQ(received.fc_time_monotonic_ns, std::optional<std::int64_t>{4 * kMs});
    EXPECT_TRUE(received.timesync_valid);
    EXPECT_EQ(received.fc_epoch, 7U);
}

TEST_F(VehicleTelemetryTest, AttitudeWithoutFcTimeArrivesWithoutFcTime) {
    ASSERT_TRUE(publisher_->publish_attitude(attitude_received_at(1 * kMs)));
    process();

    const std::optional<VehicleState> latest = subscriber_->latest();
    ASSERT_TRUE(latest.has_value());
    const std::optional<Attitude> received = latest.value_or(VehicleState{}).attitude;
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received.value_or(Attitude{}).fc_time_monotonic_ns, std::nullopt);
}

TEST_F(VehicleTelemetryTest, SubscriberReceivesTheStateThePublisherSent) {
    VehicleState sent;
    sent.local = vehicle_api::LocalPosition{1.0, 2.0, -3.0, 0.1, 0.2, -0.3, 10 * kMs, 7};
    sent.downward_distance = vehicle_api::DownwardDistance{12.5, 11 * kMs, 7};
    sent.global = vehicle_api::GlobalPosition{37.5, 127.0, 80.0, 12 * kMs, 7};
    sent.status = vehicle_api::FlightStatus{FlightMode::Hold, true, true, true, 13 * kMs, 7};

    ASSERT_TRUE(publisher_->publish_state(sent));
    process();

    const std::optional<VehicleState> latest = subscriber_->latest();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest.value_or(VehicleState{}).attitude,
              std::nullopt);  // fc/state 는 자세를 싣지 않는다
    const std::optional<vehicle_api::LocalPosition> local = latest.value_or(VehicleState{}).local;
    const std::optional<vehicle_api::DownwardDistance> distance =
        latest.value_or(VehicleState{}).downward_distance;
    const std::optional<vehicle_api::GlobalPosition> global =
        latest.value_or(VehicleState{}).global;
    const std::optional<vehicle_api::FlightStatus> status = latest.value_or(VehicleState{}).status;
    ASSERT_TRUE(local.has_value());
    ASSERT_TRUE(distance.has_value());
    ASSERT_TRUE(global.has_value());
    ASSERT_TRUE(status.has_value());
    EXPECT_DOUBLE_EQ(local.value_or(vehicle_api::LocalPosition{}).north_m, 1.0);
    EXPECT_DOUBLE_EQ(local.value_or(vehicle_api::LocalPosition{}).east_m, 2.0);
    EXPECT_DOUBLE_EQ(local.value_or(vehicle_api::LocalPosition{}).down_m, -3.0);
    EXPECT_DOUBLE_EQ(local.value_or(vehicle_api::LocalPosition{}).vel_north_m_s, 0.1);
    EXPECT_DOUBLE_EQ(local.value_or(vehicle_api::LocalPosition{}).vel_east_m_s, 0.2);
    EXPECT_DOUBLE_EQ(local.value_or(vehicle_api::LocalPosition{}).vel_down_m_s, -0.3);
    EXPECT_EQ(local.value_or(vehicle_api::LocalPosition{}).received_monotonic_ns, 10 * kMs);
    EXPECT_EQ(local.value_or(vehicle_api::LocalPosition{}).fc_epoch, 7U);
    EXPECT_DOUBLE_EQ(distance.value_or(vehicle_api::DownwardDistance{}).distance_m, 12.5);
    EXPECT_EQ(distance.value_or(vehicle_api::DownwardDistance{}).received_monotonic_ns, 11 * kMs);
    EXPECT_DOUBLE_EQ(global.value_or(vehicle_api::GlobalPosition{}).lat_deg, 37.5);
    EXPECT_DOUBLE_EQ(global.value_or(vehicle_api::GlobalPosition{}).lon_deg, 127.0);
    EXPECT_DOUBLE_EQ(global.value_or(vehicle_api::GlobalPosition{}).alt_m, 80.0);
    EXPECT_EQ(status.value_or(vehicle_api::FlightStatus{}).mode, FlightMode::Hold);
    EXPECT_TRUE(status.value_or(vehicle_api::FlightStatus{}).armed);
    EXPECT_TRUE(status.value_or(vehicle_api::FlightStatus{}).in_air);
    EXPECT_TRUE(status.value_or(vehicle_api::FlightStatus{}).offboard_hover);
    EXPECT_EQ(status.value_or(vehicle_api::FlightStatus{}).received_monotonic_ns, 13 * kMs);
}

TEST_F(VehicleTelemetryTest, ValueMissingFromStateMessageKeepsThePreviousValue) {
    VehicleState first;
    first.downward_distance = vehicle_api::DownwardDistance{12.5, 1 * kMs, 7};
    VehicleState second;  // 하향 거리가 없는 메시지
    second.status = vehicle_api::FlightStatus{FlightMode::Offboard, true, true, false, 2 * kMs, 7};

    ASSERT_TRUE(publisher_->publish_state(first));
    process();
    ASSERT_TRUE(publisher_->publish_state(second));
    process();

    const std::optional<VehicleState> latest = subscriber_->latest();
    ASSERT_TRUE(latest.has_value());
    const std::optional<vehicle_api::DownwardDistance> distance =
        latest.value_or(VehicleState{}).downward_distance;
    const std::optional<vehicle_api::FlightStatus> status = latest.value_or(VehicleState{}).status;
    ASSERT_TRUE(distance.has_value());
    ASSERT_TRUE(status.has_value());
    EXPECT_DOUBLE_EQ(distance.value_or(vehicle_api::DownwardDistance{}).distance_m, 12.5);
    EXPECT_EQ(status.value_or(vehicle_api::FlightStatus{}).mode, FlightMode::Offboard);
    EXPECT_FALSE(status.value_or(vehicle_api::FlightStatus{}).offboard_hover);
}

TEST_F(VehicleTelemetryTest, AttitudeAtPicksTheValueReceivedClosestToTheRequestedTime) {
    deliver_attitude_at(100 * kMs, 1.0);
    deliver_attitude_at(110 * kMs, 2.0);
    deliver_attitude_at(120 * kMs, 3.0);

    const std::optional<Attitude> before = subscriber_->attitude_at(106 * kMs);  // 110 에 가깝다
    const std::optional<Attitude> after = subscriber_->attitude_at(113 * kMs);   // 110 에 가깝다

    ASSERT_TRUE(before.has_value());
    ASSERT_TRUE(after.has_value());
    EXPECT_DOUBLE_EQ(before.value_or(Attitude{}).yaw_rad, 2.0);
    EXPECT_DOUBLE_EQ(after.value_or(Attitude{}).yaw_rad, 2.0);
}

TEST_F(VehicleTelemetryTest, AttitudeAtReturnsNothingWhenClosestValueIsOutsideTolerance) {
    deliver_attitude_at(100 * kMs);

    EXPECT_TRUE(subscriber_->attitude_at(150 * kMs));   // 차 50 ms: 허용
    EXPECT_FALSE(subscriber_->attitude_at(151 * kMs));  // 차 51 ms: 허용 범위 밖
}

TEST_F(VehicleTelemetryTest, AttitudeAtIgnoresValuesOlderThanMaxAgeAtQueryTime) {
    deliver_attitude_at(100 * kMs);

    clock_.advance_ns(200 * kMs);  // 조회 시점 300 ms: 받은 지 200 ms
    EXPECT_TRUE(subscriber_->attitude_at(100 * kMs));
    clock_.advance_ns(1 * kMs);  // 조회 시점 301 ms: 받은 지 201 ms
    EXPECT_FALSE(subscriber_->attitude_at(100 * kMs));
}

TEST_F(VehicleTelemetryTest, LocalPositionAndDownwardDistanceFollowTheSameSelectionRule) {
    VehicleState state;
    state.local = vehicle_api::LocalPosition{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100 * kMs, 7};
    state.downward_distance = vehicle_api::DownwardDistance{12.5, 100 * kMs, 7};
    clock_.advance_ns(100 * kMs);
    ASSERT_TRUE(publisher_->publish_state(state));
    process();

    EXPECT_TRUE(subscriber_->local_position_at(120 * kMs));
    EXPECT_TRUE(subscriber_->downward_distance_at(120 * kMs));
    EXPECT_FALSE(subscriber_->local_position_at(151 * kMs));
    EXPECT_FALSE(subscriber_->downward_distance_at(151 * kMs));
}

TEST_F(VehicleTelemetryTest, FlightModeEventReachesEveryHandlerUntilReleased) {
    std::vector<FlightModeEvent> first_received;
    int second_received = 0;
    const core::Subscription first = subscriber_->subscribe_flight_mode_events(
        [&](const FlightModeEvent& event) { first_received.push_back(event); });
    core::Subscription second = subscriber_->subscribe_flight_mode_events(
        [&](const FlightModeEvent&) { ++second_received; });
    FlightModeEvent sent;
    sent.mode = FlightMode::Position;
    sent.request_command_id = vehicle_api::CommandId{42};
    sent.exit_cause = vehicle_api::OffboardExitCause::PilotSwitch;
    sent.received_monotonic_ns = 9 * kMs;

    ASSERT_TRUE(publisher_->publish_mode_event(sent));
    process();
    second.reset();
    ASSERT_TRUE(publisher_->publish_mode_event(FlightModeEvent{}));
    process();

    ASSERT_EQ(first_received.size(), 2U);
    EXPECT_EQ(first_received[0].mode, FlightMode::Position);
    EXPECT_EQ(first_received[0].request_command_id,
              std::optional<vehicle_api::CommandId>{vehicle_api::CommandId{42}});
    EXPECT_EQ(first_received[0].exit_cause, vehicle_api::OffboardExitCause::PilotSwitch);
    EXPECT_EQ(first_received[0].received_monotonic_ns, 9 * kMs);
    EXPECT_EQ(first_received[1].request_command_id, std::nullopt);  // 요청 없이 바뀐 경우
    EXPECT_EQ(second_received, 1);
}

TEST_F(VehicleTelemetryTest, SubscribeFlightModeEventsRejectsEmptyHandler) {
    const core::Subscription subscription = subscriber_->subscribe_flight_mode_events(nullptr);

    EXPECT_FALSE(subscription.active());
}

TEST_F(VehicleTelemetryTest, DropsMessagesThatCannotBeParsed) {
    int mode_events = 0;
    const core::Subscription subscription =
        subscriber_->subscribe_flight_mode_events([&](const FlightModeEvent&) { ++mode_events; });
    // 필드 번호 1, wire type length-delimited(2), 길이 5 인데 내용이 없음
    const bus::Bytes broken{0x0A, 0x05};

    bus_.simulate_message_received({std::string(protocol::kFcAttitude), broken, {}});
    bus_.simulate_message_received({std::string(protocol::kFcState), broken, {}});
    bus_.simulate_message_received({std::string(protocol::kFcModeEvent), broken, {}});
    process();

    EXPECT_EQ(subscriber_->latest(), std::nullopt);
    EXPECT_EQ(mode_events, 0);
}

TEST_F(VehicleTelemetryTest, CountsLostMessagesPerTopic) {
    ASSERT_TRUE(publisher_->publish_attitude(attitude_received_at(1 * kMs)));  // seq 1
    process();
    msg::FcAttitude later;
    later.set_seq(5);  // seq 2, 3, 4 가 도착하지 않음
    later.set_received_monotonic_ns(2 * kMs);
    bus::Bytes body(later.ByteSizeLong());
    EXPECT_TRUE(later.SerializeToArray(body.data(), static_cast<int>(body.size())));

    bus_.simulate_message_received({std::string(protocol::kFcAttitude), body, {}});
    process();

    EXPECT_EQ(subscriber_->lost_attitude_count(), 3U);
    EXPECT_EQ(subscriber_->lost_state_count(), 0U);
    EXPECT_EQ(subscriber_->lost_mode_event_count(), 0U);
}

}  // namespace
}  // namespace argus::vehicle_api_bus
