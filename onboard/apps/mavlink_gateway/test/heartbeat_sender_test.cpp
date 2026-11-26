#include "argus/mavlink_gateway/heartbeat_sender.hpp"

#include <gtest/gtest.h>

#include <cstdint>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "fake_gcs_link.hpp"
#include "fake_mavlink_connection.hpp"

namespace argus::mavlink_gateway {
namespace {

mavlink_heartbeat_t heartbeat_of(const mavlink_message_t& message) {
    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    return heartbeat;
}

class HeartbeatSenderTest : public ::testing::Test {
protected:
    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    FakeGcsLink link_;
    FakeMavlinkConnection connection_{poller_};
    HealthState health_;
    HeartbeatSender sender_{link_, connection_, health_};
};

TEST_F(HeartbeatSenderTest, SendsToAllGcsAndFcFromCompid195) {
    sender_.tick();

    ASSERT_EQ(link_.sent().size(), 1U);
    EXPECT_FALSE(link_.sent()[0].to.has_value());
    ASSERT_EQ(connection_.sent().size(), 1U);
    for (const mavlink_message_t& message : {link_.sent()[0].message, connection_.sent()[0]}) {
        EXPECT_EQ(message.msgid, static_cast<std::uint32_t>(MAVLINK_MSG_ID_HEARTBEAT));
        EXPECT_EQ(message.sysid, kVehicleSysid);
        EXPECT_EQ(message.compid, kOnboardCommandCompid);
        const mavlink_heartbeat_t heartbeat = heartbeat_of(message);
        EXPECT_EQ(heartbeat.type, MAV_TYPE_ONBOARD_CONTROLLER);
        EXPECT_EQ(heartbeat.autopilot, MAV_AUTOPILOT_INVALID);
        EXPECT_EQ(heartbeat.base_mode, 0);
        EXPECT_EQ(heartbeat.custom_mode, 0U);
    }
}

TEST_F(HeartbeatSenderTest, SystemStatusFollowsFcConnection) {
    sender_.tick();
    EXPECT_EQ(heartbeat_of(link_.sent().back().message).system_status, MAV_STATE_CRITICAL);

    health_.set(ARGUS_HEALTH_FC_CONNECTED, true);
    sender_.tick();
    EXPECT_EQ(heartbeat_of(link_.sent().back().message).system_status, MAV_STATE_ACTIVE);

    health_.set(ARGUS_HEALTH_GCS_LINK, false);  // 통제기 링크는 영향을 주지 않는다
    sender_.tick();
    EXPECT_EQ(heartbeat_of(link_.sent().back().message).system_status, MAV_STATE_ACTIVE);
}

TEST_F(HeartbeatSenderTest, EachTickSendsOneHeartbeat) {
    sender_.tick();
    sender_.tick();

    EXPECT_EQ(link_.sent().size(), 2U);
    EXPECT_EQ(connection_.sent().size(), 2U);
}

}  // namespace
}  // namespace argus::mavlink_gateway
