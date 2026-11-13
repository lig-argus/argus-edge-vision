#include "argus/onboard/fc_telemetry_receiver.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/onboard/fc_link_tracker.hpp"
#include "argus/onboard/fc_time_converter.hpp"
#include "argus/onboard/health_state.hpp"
#include "argus/vehicle_api/types.hpp"
#include "fake_mavlink_connection.hpp"

namespace argus::onboard {
namespace {

constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr std::int64_t kStartNs = 5'000 * kNsPerMs;
constexpr std::uint8_t kFcSysid = 1;
constexpr std::uint8_t kFcCompid = 1;
constexpr std::uint32_t kOffboardCustomMode = 6U << 16U;
constexpr std::uint32_t kHoldCustomMode = (4U << 16U) | (3U << 24U);

// offset_ns 가 있는 경우 FC 시각에 그 값을 더하여 반환한다
class FakeFcTimeConverter : public IFcTimeConverter {
public:
    std::optional<std::int64_t> to_monotonic_ns(std::uint32_t fc_time_boot_ms) const override {
        if (!offset_ns.has_value()) {
            return std::nullopt;
        }
        return static_cast<std::int64_t>(fc_time_boot_ms) * kNsPerMs + offset_ns.value_or(0);
    }

    std::optional<std::int64_t> offset_ns;
};

class FcTelemetryReceiverTest : public ::testing::Test {
protected:
    void SetUp() override {
        attitude_subscription_ = receiver_.subscribe_attitude(
            [this](const vehicle_api::Attitude& attitude) { attitudes_.push_back(attitude); });
        state_subscription_ = receiver_.subscribe_state(
            [this](const vehicle_api::VehicleState& state) { states_.push_back(state); });
    }

    void deliver(const mavlink_message_t& message) {
        connection_.simulate_message_received(message);
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    void attitude(std::uint32_t time_boot_ms, std::uint8_t sysid = kFcSysid,
                  std::uint8_t compid = kFcCompid) {
        mavlink_message_t message{};
        mavlink_msg_attitude_pack(sysid, compid, &message, time_boot_ms, 0.1F, -0.2F, 1.5F, 0.01F,
                                  0.02F, 0.03F);
        deliver(message);
    }

    void local_position(std::uint32_t time_boot_ms) {
        mavlink_message_t message{};
        mavlink_msg_local_position_ned_pack(kFcSysid, kFcCompid, &message, time_boot_ms, 1.0F, 2.0F,
                                            -3.0F, 0.5F, -0.5F, 0.25F);
        deliver(message);
    }

    void global_position(std::uint32_t time_boot_ms) {
        mavlink_message_t message{};
        mavlink_msg_global_position_int_pack(kFcSysid, kFcCompid, &message, time_boot_ms,
                                             373'000'000, 1'270'000'000, 123'456, 0, 0, 0, 0, 0);
        deliver(message);
    }

    void distance_sensor(std::uint32_t time_boot_ms, std::uint16_t distance_cm,
                         std::uint8_t orientation) {
        mavlink_message_t message{};
        const std::array<float, 4> quaternion{};
        mavlink_msg_distance_sensor_pack(kFcSysid, kFcCompid, &message, time_boot_ms, 10, 4000,
                                         distance_cm, MAV_DISTANCE_SENSOR_LASER, 0, orientation, 0,
                                         0.0F, 0.0F, quaternion.data(), 0);
        deliver(message);
    }

    void heartbeat(std::uint32_t custom_mode, std::uint8_t base_mode, std::uint8_t sysid = kFcSysid,
                   std::uint8_t compid = kFcCompid) {
        mavlink_message_t message{};
        mavlink_msg_heartbeat_pack(sysid, compid, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                                   base_mode, custom_mode, MAV_STATE_ACTIVE);
        deliver(message);
    }

    core_testing::FakeClock clock_{kStartNs};
    core_testing::FakePoller poller_{clock_};
    FakeMavlinkConnection connection_{poller_};
    FakeFcTimeConverter time_converter_;
    HealthState health_;
    FcLinkTracker link_{clock_, health_};
    FcTelemetryReceiver receiver_{connection_, link_, time_converter_};
    std::vector<vehicle_api::Attitude> attitudes_;
    std::vector<vehicle_api::VehicleState> states_;
    core::Subscription attitude_subscription_;
    core::Subscription state_subscription_;
};

TEST_F(FcTelemetryReceiverTest, HasNoValuesBeforeAnyFcMessage) {
    EXPECT_FALSE(receiver_.latest().attitude.has_value());
    EXPECT_FALSE(receiver_.latest().status.has_value());
    EXPECT_FALSE(link_.connected());
}

TEST_F(FcTelemetryReceiverTest, ConvertsAttitudeAndRecordsReceiveTime) {
    attitude(1000);

    ASSERT_EQ(attitudes_.size(), 1U);
    const vehicle_api::Attitude& value = attitudes_.back();
    EXPECT_FLOAT_EQ(static_cast<float>(value.roll_rad), 0.1F);
    EXPECT_FLOAT_EQ(static_cast<float>(value.pitch_rad), -0.2F);
    EXPECT_FLOAT_EQ(static_cast<float>(value.yaw_rad), 1.5F);
    EXPECT_FLOAT_EQ(static_cast<float>(value.roll_rate_rad_s), 0.01F);
    EXPECT_FLOAT_EQ(static_cast<float>(value.pitch_rate_rad_s), 0.02F);
    EXPECT_FLOAT_EQ(static_cast<float>(value.yaw_rate_rad_s), 0.03F);
    EXPECT_EQ(value.fc_time_boot_ms, 1000U);
    EXPECT_EQ(value.received_monotonic_ns, kStartNs);
    EXPECT_EQ(value.fc_epoch, 1U);
    EXPECT_TRUE(states_.empty());  // 자세는 state handler 로 전달하지 않음
    ASSERT_TRUE(receiver_.latest().attitude.has_value());
}

TEST_F(FcTelemetryReceiverTest, AttitudeHasNoFcTimeConversionWhileTimeSyncIsInvalid) {
    attitude(1000);

    ASSERT_EQ(attitudes_.size(), 1U);
    EXPECT_FALSE(attitudes_.back().timesync_valid);
    EXPECT_FALSE(attitudes_.back().fc_time_monotonic_ns.has_value());
}

TEST_F(FcTelemetryReceiverTest, AttitudeCarriesConvertedFcTimeWhileTimeSyncIsValid) {
    time_converter_.offset_ns = 7 * kNsPerMs;
    attitude(1000);

    ASSERT_EQ(attitudes_.size(), 1U);
    EXPECT_TRUE(attitudes_.back().timesync_valid);
    EXPECT_EQ(attitudes_.back().fc_time_monotonic_ns.value_or(0), 1007 * kNsPerMs);
}

TEST_F(FcTelemetryReceiverTest, ConvertsLocalPosition) {
    local_position(1000);

    ASSERT_EQ(states_.size(), 1U);
    ASSERT_TRUE(states_.back().local.has_value());
    const vehicle_api::LocalPosition local =
        states_.back().local.value_or(vehicle_api::LocalPosition{});
    EXPECT_DOUBLE_EQ(local.north_m, 1.0);
    EXPECT_DOUBLE_EQ(local.east_m, 2.0);
    EXPECT_DOUBLE_EQ(local.down_m, -3.0);
    EXPECT_DOUBLE_EQ(local.vel_north_m_s, 0.5);
    EXPECT_DOUBLE_EQ(local.vel_east_m_s, -0.5);
    EXPECT_DOUBLE_EQ(local.vel_down_m_s, 0.25);
    EXPECT_EQ(local.received_monotonic_ns, kStartNs);
    EXPECT_EQ(local.fc_epoch, 1U);
}

TEST_F(FcTelemetryReceiverTest, ConvertsGlobalPositionUnits) {
    global_position(1000);

    ASSERT_EQ(states_.size(), 1U);
    ASSERT_TRUE(states_.back().global.has_value());
    const vehicle_api::GlobalPosition global =
        states_.back().global.value_or(vehicle_api::GlobalPosition{});
    EXPECT_DOUBLE_EQ(global.lat_deg, 37.3);
    EXPECT_DOUBLE_EQ(global.lon_deg, 127.0);
    EXPECT_DOUBLE_EQ(global.alt_m, 123.456);
    EXPECT_EQ(global.received_monotonic_ns, kStartNs);
}

TEST_F(FcTelemetryReceiverTest, ConvertsDownwardDistanceFromCentimeters) {
    distance_sensor(1000, 250, MAV_SENSOR_ROTATION_PITCH_270);

    ASSERT_EQ(states_.size(), 1U);
    ASSERT_TRUE(states_.back().downward_distance.has_value());
    const vehicle_api::DownwardDistance distance =
        states_.back().downward_distance.value_or(vehicle_api::DownwardDistance{});
    EXPECT_DOUBLE_EQ(distance.distance_m, 2.5);
    EXPECT_EQ(distance.received_monotonic_ns, kStartNs);
}

TEST_F(FcTelemetryReceiverTest, IgnoresDistanceOfSensorNotFacingDown) {
    distance_sensor(1000, 250, MAV_SENSOR_ROTATION_NONE);

    EXPECT_TRUE(states_.empty());
    EXPECT_FALSE(receiver_.latest().downward_distance.has_value());
    EXPECT_TRUE(link_.connected());  // 연결 판단에는 포함
}

TEST_F(FcTelemetryReceiverTest, FcTimeOfOtherDistanceSensorIsNotUsedForRebootDetection) {
    distance_sensor(50'000, 250, MAV_SENSOR_ROTATION_PITCH_270);
    distance_sensor(49'000, 300, MAV_SENSOR_ROTATION_NONE);  // 다른 센서의 time_boot_ms 가 더 작음
    distance_sensor(50'030, 251, MAV_SENSOR_ROTATION_PITCH_270);

    EXPECT_EQ(link_.fc_epoch(), 1U);
    ASSERT_TRUE(receiver_.latest().downward_distance.has_value());
    EXPECT_DOUBLE_EQ(
        receiver_.latest().downward_distance.value_or(vehicle_api::DownwardDistance{}).distance_m,
        2.51);
}

TEST_F(FcTelemetryReceiverTest, ConvertsHeartbeatModeAndArmedState) {
    heartbeat(kOffboardCustomMode, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED);

    ASSERT_EQ(states_.size(), 1U);
    ASSERT_TRUE(states_.back().status.has_value());
    vehicle_api::FlightStatus status = states_.back().status.value_or(vehicle_api::FlightStatus{});
    EXPECT_EQ(status.mode, vehicle_api::FlightMode::Offboard);
    EXPECT_TRUE(status.armed);
    EXPECT_EQ(status.received_monotonic_ns, kStartNs);

    heartbeat(kHoldCustomMode, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    status = states_.back().status.value_or(vehicle_api::FlightStatus{});
    EXPECT_EQ(status.mode, vehicle_api::FlightMode::Hold);
    EXPECT_FALSE(status.armed);
}

TEST_F(FcTelemetryReceiverTest, StateCarriesLatestValueOfEachItem) {
    local_position(1000);
    clock_.advance_ns(10 * kNsPerMs);
    heartbeat(kOffboardCustomMode, 0);

    ASSERT_EQ(states_.size(), 2U);
    const vehicle_api::VehicleState& state = states_.back();
    ASSERT_TRUE(state.local.has_value());
    ASSERT_TRUE(state.status.has_value());
    // 값마다 그 FC 메시지를 받은 시각을 가진다
    EXPECT_EQ(state.local.value_or(vehicle_api::LocalPosition{}).received_monotonic_ns, kStartNs);
    EXPECT_EQ(state.status.value_or(vehicle_api::FlightStatus{}).received_monotonic_ns,
              kStartNs + 10 * kNsPerMs);
    EXPECT_FALSE(state.global.has_value());
    EXPECT_FALSE(state.downward_distance.has_value());
}

TEST_F(FcTelemetryReceiverTest, IgnoresMessagesNotFromFc) {
    attitude(1000, 250, 190);                                // 통제기
    attitude(1000, kFcSysid, MAV_COMP_ID_ONBOARD_COMPUTER);  // 같은 기체의 다른 컴포넌트
    heartbeat(kOffboardCustomMode, 0, 250, 190);

    EXPECT_TRUE(attitudes_.empty());
    EXPECT_TRUE(states_.empty());
    EXPECT_FALSE(link_.connected());
    EXPECT_EQ(link_.fc_epoch(), 0U);
}

TEST_F(FcTelemetryReceiverTest, EveryFcMessageIsReportedToLinkTracker) {
    attitude(1000);
    EXPECT_TRUE(link_.connected());
    EXPECT_EQ(link_.fc_epoch(), 1U);

    // 각 메시지가 두절 시간 안에 도착하므로 연결이 유지된다
    clock_.advance_ns(kFcLinkTimeoutNs);
    local_position(1100);
    clock_.advance_ns(kFcLinkTimeoutNs);
    global_position(1200);
    clock_.advance_ns(kFcLinkTimeoutNs);
    distance_sensor(1300, 100, MAV_SENSOR_ROTATION_PITCH_270);
    clock_.advance_ns(kFcLinkTimeoutNs);
    heartbeat(kOffboardCustomMode, 0);
    clock_.advance_ns(kFcLinkTimeoutNs);
    link_.check_timeout();

    EXPECT_TRUE(link_.connected());
    EXPECT_EQ(link_.fc_epoch(), 1U);
}

TEST_F(FcTelemetryReceiverTest, MessageAfterLinkLossCarriesNewEpochAndOldValuesAreCleared) {
    local_position(1000);
    attitude(1000);
    clock_.advance_ns(kFcLinkTimeoutNs + 1);
    link_.check_timeout();

    attitude(4000);

    EXPECT_EQ(attitudes_.back().fc_epoch, 2U);
    EXPECT_FALSE(receiver_.latest().local.has_value());
    ASSERT_TRUE(receiver_.latest().attitude.has_value());
}

TEST_F(FcTelemetryReceiverTest, SmallerFcTimeOfSameMessageCarriesNewEpochAndOldValuesAreCleared) {
    attitude(50'000);
    local_position(50'000);
    clock_.advance_ns(100 * kNsPerMs);

    attitude(300);

    EXPECT_EQ(attitudes_.back().fc_epoch, 2U);
    EXPECT_FALSE(receiver_.latest().local.has_value());

    // 재부팅 후 다른 종류의 첫 메시지는 fc_epoch 를 다시 증가시키지 않는다
    local_position(310);
    ASSERT_TRUE(states_.back().local.has_value());
    EXPECT_EQ(states_.back().local.value_or(vehicle_api::LocalPosition{}).fc_epoch, 2U);
}

TEST_F(FcTelemetryReceiverTest, CallsEveryRegisteredHandler) {
    int second_attitude_count = 0;
    int second_state_count = 0;
    const core::Subscription attitude_subscription = receiver_.subscribe_attitude(
        [&second_attitude_count](const vehicle_api::Attitude&) { ++second_attitude_count; });
    const core::Subscription state_subscription = receiver_.subscribe_state(
        [&second_state_count](const vehicle_api::VehicleState&) { ++second_state_count; });

    attitude(1000);
    heartbeat(kOffboardCustomMode, 0);

    EXPECT_EQ(attitudes_.size(), 1U);
    EXPECT_EQ(states_.size(), 1U);
    EXPECT_EQ(second_attitude_count, 1);
    EXPECT_EQ(second_state_count, 1);
}

TEST_F(FcTelemetryReceiverTest, DoesNotCallHandlerAfterSubscriptionIsReleased) {
    attitude_subscription_.reset();
    state_subscription_.reset();

    attitude(1000);
    heartbeat(kOffboardCustomMode, 0);

    EXPECT_TRUE(attitudes_.empty());
    EXPECT_TRUE(states_.empty());
    EXPECT_TRUE(receiver_.latest().attitude.has_value());
    EXPECT_TRUE(receiver_.latest().status.has_value());
}

}  // namespace
}  // namespace argus::onboard
