#include "argus/mavlink_gateway/fc_rc_status_receiver.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/fc_link_tracker.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "fake_mavlink_connection.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr std::int64_t kStartNs = 5'000'000'000;
constexpr std::uint16_t kUnused = UINT16_MAX;

class FcRcStatusReceiverTest : public ::testing::Test {
protected:
    void deliver(const mavlink_message_t& message) {
        connection_.simulate_message_received(message);
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    void rc_channels(std::uint8_t count, std::uint8_t sysid = kVehicleSysid,
                     std::uint8_t compid = kFcCompid) {
        mavlink_message_t message{};
        mavlink_msg_rc_channels_pack(sysid, compid, &message, 1000, count, 1100, 1200, 1300, 1400,
                                     1500, 1600, 1700, 1800, kUnused, kUnused, kUnused, kUnused,
                                     kUnused, kUnused, kUnused, kUnused, kUnused, 1999, 200);
        deliver(message);
    }

    void sys_status(std::uint32_t present, std::uint32_t health) {
        mavlink_message_t message{};
        mavlink_msg_sys_status_pack(kVehicleSysid, kFcCompid, &message, present, present, health, 0,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        deliver(message);
    }

    // FC 연결 판단은 ATTITUDE 를 받은 FcTelemetryReceiver 가 FcLinkTracker 에 알린다
    void attitude(std::uint32_t time_boot_ms) {
        static_cast<void>(link_.on_fc_message(FcTimedMessage::Attitude, time_boot_ms));
    }

    core_testing::FakeClock clock_{kStartNs};
    core_testing::FakePoller poller_{clock_};
    FakeMavlinkConnection connection_{poller_};
    HealthState health_;
    FcLinkTracker link_{clock_, health_};
    FcRcStatusReceiver receiver_{connection_, link_};
};

TEST_F(FcRcStatusReceiverTest, HasNoValuesBeforeMessages) {
    EXPECT_FALSE(receiver_.rc_channels().has_value());
    EXPECT_FALSE(receiver_.sensor_health().has_value());
}

TEST_F(FcRcStatusReceiverTest, KeepsChannelValuesWithReceiveTime) {
    attitude(1000);
    clock_.advance_ns(10'000'000);
    rc_channels(8);

    ASSERT_TRUE(receiver_.rc_channels().has_value());
    const RcChannels channels = receiver_.rc_channels().value_or(RcChannels{});
    EXPECT_EQ(channels.channel_count, 8U);
    EXPECT_EQ(channels.raw_us[0], 1100U);
    EXPECT_EQ(channels.raw_us[7], 1800U);
    EXPECT_EQ(channels.raw_us[8], kUnused);
    EXPECT_EQ(channels.raw_us[17], 1999U);
    EXPECT_EQ(channels.received_monotonic_ns, kStartNs + 10'000'000);
    EXPECT_EQ(channels.fc_epoch, 1U);
}

TEST_F(FcRcStatusReceiverTest, IgnoresMessagesNotFromFc) {
    rc_channels(8, kVehicleSysid, kOnboardCommandCompid);

    EXPECT_FALSE(receiver_.rc_channels().has_value());
}

TEST_F(FcRcStatusReceiverTest, ReadsReceiverAndEstimateBits) {
    const std::uint32_t all = MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                              MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_AHRS;
    sys_status(all, all);
    ASSERT_TRUE(receiver_.sensor_health().has_value());
    FcSensorHealth health = receiver_.sensor_health().value_or(FcSensorHealth{});
    EXPECT_TRUE(health.rc_receiver_ok);
    EXPECT_TRUE(health.local_position_ok);
    EXPECT_TRUE(health.attitude_ok);

    // RC 수신기와 위치 추정은 present 비트도 있어야 하고, 자세 추정은 health 비트만 본다
    sys_status(0, all);
    ASSERT_TRUE(receiver_.sensor_health().has_value());
    health = receiver_.sensor_health().value_or(FcSensorHealth{});
    EXPECT_FALSE(health.rc_receiver_ok);
    EXPECT_FALSE(health.local_position_ok);
    EXPECT_TRUE(health.attitude_ok);

    sys_status(all, MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
    ASSERT_TRUE(receiver_.sensor_health().has_value());
    health = receiver_.sensor_health().value_or(FcSensorHealth{});
    EXPECT_TRUE(health.rc_receiver_ok);
    EXPECT_FALSE(health.local_position_ok);
    EXPECT_FALSE(health.attitude_ok);
}

TEST_F(FcRcStatusReceiverTest, DropsValuesWhenFcEpochChanges) {
    attitude(1000);
    rc_channels(8);
    sys_status(0, 0);

    clock_.advance_ns(kFcLinkTimeoutNs + 1);
    attitude(2000);  // 두절 뒤 다시 받음 → fc_epoch 증가

    EXPECT_FALSE(receiver_.rc_channels().has_value());
    EXPECT_FALSE(receiver_.sensor_health().has_value());
}

}  // namespace
}  // namespace argus::mavlink_gateway
