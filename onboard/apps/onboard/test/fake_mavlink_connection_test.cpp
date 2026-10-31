#include "fake_mavlink_connection.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
// 인터페이스 헤더가 ZeroMQ, UDP 없이 컴파일되는지 함께 확인한다
#include "argus/onboard/mavlink_command_handler.hpp"

namespace argus::onboard {
namespace {

constexpr std::uint8_t kFcSysid = 1;
constexpr std::uint8_t kGcsSysid = 250;

mavlink_message_t heartbeat_from(std::uint8_t sysid) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(sysid, 1, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 0, 0,
                               MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t attitude_from_fc() {
    mavlink_message_t message{};
    mavlink_msg_attitude_pack(kFcSysid, 1, &message, 0, 0.1F, 0.0F, 1.5F, 0.0F, 0.0F, 0.0F);
    return message;
}

class FakeMavlinkConnectionTest : public ::testing::Test {
protected:
    void process() {
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    FakeMavlinkConnection connection_{poller_};
};

TEST_F(FakeMavlinkConnectionTest, DeliversOnlyMatchingMsgidInsideProcessEvents) {
    std::vector<std::uint32_t> received;
    const core::Subscription subscription = connection_.subscribe(
        MAVLINK_MSG_ID_ATTITUDE, [&](const mavlink_message_t& m) { received.push_back(m.msgid); });

    connection_.simulate_message_received(heartbeat_from(kFcSysid));
    connection_.simulate_message_received(attitude_from_fc());
    EXPECT_TRUE(received.empty());

    process();
    EXPECT_EQ(received, (std::vector<std::uint32_t>{MAVLINK_MSG_ID_ATTITUDE}));
}

TEST_F(FakeMavlinkConnectionTest, EveryHandlerOfSameMsgidReceives) {
    std::vector<std::uint8_t> telemetry_sysids;
    std::vector<std::uint8_t> gcs_link_sysids;
    const core::Subscription telemetry = connection_.subscribe(
        MAVLINK_MSG_ID_HEARTBEAT,
        [&](const mavlink_message_t& m) { telemetry_sysids.push_back(m.sysid); });
    const core::Subscription gcs_link = connection_.subscribe(
        MAVLINK_MSG_ID_HEARTBEAT,
        [&](const mavlink_message_t& m) { gcs_link_sysids.push_back(m.sysid); });

    connection_.simulate_message_received(heartbeat_from(kFcSysid));
    connection_.simulate_message_received(heartbeat_from(kGcsSysid));
    process();

    EXPECT_EQ(telemetry_sysids, (std::vector<std::uint8_t>{kFcSysid, kGcsSysid}));
    EXPECT_EQ(gcs_link_sysids, (std::vector<std::uint8_t>{kFcSysid, kGcsSysid}));
}

TEST_F(FakeMavlinkConnectionTest, ReleasedSubscriptionStopsDelivery) {
    int received = 0;
    core::Subscription subscription = connection_.subscribe(
        MAVLINK_MSG_ID_HEARTBEAT, [&](const mavlink_message_t&) { ++received; });

    subscription.reset();
    connection_.simulate_message_received(heartbeat_from(kFcSysid));
    process();

    EXPECT_EQ(received, 0);
}

TEST_F(FakeMavlinkConnectionTest, RecordsSentMessagesAndResult) {
    EXPECT_TRUE(connection_.send(heartbeat_from(191)));
    connection_.set_send_result(false);
    EXPECT_FALSE(connection_.send(attitude_from_fc()));

    ASSERT_EQ(connection_.sent().size(), 2U);
    EXPECT_EQ(connection_.sent()[0].msgid, MAVLINK_MSG_ID_HEARTBEAT);
    EXPECT_EQ(connection_.sent()[1].msgid, MAVLINK_MSG_ID_ATTITUDE);
}

}  // namespace
}  // namespace argus::onboard
