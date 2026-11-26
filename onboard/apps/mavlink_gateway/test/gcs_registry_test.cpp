#include "argus/mavlink_gateway/gcs_registry.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/mavlink.h"
#include "fake_gcs_link.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway {
namespace {

using test_messages::command_long_from;
using test_messages::gcs_peer;
using test_messages::heartbeat_from;

class GcsRegistryTest : public ::testing::Test {
protected:
    GcsRegistryTest() {
        commands_ = registry_.subscribe(
            MAVLINK_MSG_ID_COMMAND_LONG,
            [this](const mavlink_message_t& /*message*/, const GcsPeer& sender) {
                command_senders_.push_back(sender.sysid);
            });
    }

    void receive_command(const GcsPeer& sender) {
        link_.simulate_message_received(
            command_long_from(sender, MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL, 1.0F), sender);
    }

    core_testing::FakeClock clock_;
    FakeGcsLink link_;
    GcsRegistry registry_{link_, clock_};
    std::vector<std::uint8_t> command_senders_;
    core::Subscription commands_;
};

TEST_F(GcsRegistryTest, IgnoresMessagesUntilGcsHeartbeat) {
    const GcsPeer gcs = gcs_peer(250);

    receive_command(gcs);

    EXPECT_TRUE(command_senders_.empty());
    EXPECT_EQ(registry_.last_heartbeat_ns(250), std::nullopt);
}

TEST_F(GcsRegistryTest, RecognizesGcsHeartbeatAndRecordsTimeAndAddress) {
    const GcsPeer gcs = gcs_peer(251, "10.0.0.5", 40000);
    clock_.advance_ns(7'000'000);

    link_.simulate_message_received(heartbeat_from(gcs), gcs);
    receive_command(gcs);

    EXPECT_EQ(registry_.last_heartbeat_ns(251), std::optional<std::int64_t>{7'000'000});
    ASSERT_TRUE(registry_.peer(251).has_value());
    EXPECT_TRUE(registry_.peer(251).value_or(GcsPeer{}).same_address(gcs));
    EXPECT_EQ(command_senders_, std::vector<std::uint8_t>{251});
}

TEST_F(GcsRegistryTest, DoesNotRecognizeHeartbeatOfOtherType) {
    const GcsPeer gcs = gcs_peer(250);

    link_.simulate_message_received(heartbeat_from(gcs, MAV_TYPE_QUADROTOR), gcs);
    receive_command(gcs);

    EXPECT_EQ(registry_.last_heartbeat_ns(250), std::nullopt);
    EXPECT_TRUE(command_senders_.empty());
}

TEST_F(GcsRegistryTest, DoesNotRecognizeSysidOutsideGcsRange) {
    for (const std::uint8_t sysid : {249, 254, 255}) {
        const GcsPeer sender = gcs_peer(sysid);
        link_.simulate_message_received(heartbeat_from(sender), sender);
        receive_command(sender);
        EXPECT_EQ(registry_.last_heartbeat_ns(sysid), std::nullopt) << +sysid;
    }
    EXPECT_TRUE(command_senders_.empty());
}

TEST_F(GcsRegistryTest, DoesNotRecognizeOtherCompid) {
    const GcsPeer other = gcs_peer(250, "192.168.0.10", 14550, 191);

    link_.simulate_message_received(heartbeat_from(other), other);

    EXPECT_EQ(registry_.last_heartbeat_ns(250), std::nullopt);
}

TEST_F(GcsRegistryTest, DropsOtherCompidOfRecognizedSysid) {
    const GcsPeer gcs = gcs_peer(250);
    link_.simulate_message_received(heartbeat_from(gcs), gcs);

    receive_command(gcs_peer(250, "192.168.0.10", 14550, 191));

    EXPECT_TRUE(command_senders_.empty());
}

TEST_F(GcsRegistryTest, WarnsBothAddressesWhenSysidArrivesFromAnotherAddress) {
    const GcsPeer first = gcs_peer(252, "192.168.0.10", 14550);
    const GcsPeer second = gcs_peer(252, "192.168.0.11", 14550);
    link_.simulate_message_received(heartbeat_from(first), first);
    link_.simulate_message_received(heartbeat_from(first), first);
    EXPECT_TRUE(link_.sent().empty());  // 같은 주소에서 다시 받으면 경고하지 않는다

    link_.simulate_message_received(heartbeat_from(second), second);

    ASSERT_EQ(link_.sent().size(), 2U);
    for (const FakeGcsLink::Sent& sent : link_.sent()) {
        EXPECT_EQ(sent.message.msgid, MAVLINK_MSG_ID_STATUSTEXT);
        EXPECT_EQ(test_messages::statustext_of(sent.message), "통제기 sysid 252 중복");
    }
    EXPECT_TRUE(link_.sent()[0].to.value_or(GcsPeer{}).same_address(first));
    EXPECT_TRUE(link_.sent()[1].to.value_or(GcsPeer{}).same_address(second));
    EXPECT_TRUE(registry_.peer(252).value_or(GcsPeer{}).same_address(second));
}

TEST_F(GcsRegistryTest, HeartbeatSubscribersSeeUpdatedRecord) {
    const GcsPeer gcs = gcs_peer(253);
    clock_.advance_ns(5'000'000);
    std::optional<std::int64_t> seen;
    const core::Subscription subscription = registry_.subscribe(
        MAVLINK_MSG_ID_HEARTBEAT,
        [this, &seen](const mavlink_message_t& /*message*/, const GcsPeer& sender) {
            seen = registry_.last_heartbeat_ns(sender.sysid);
        });

    link_.simulate_message_received(heartbeat_from(gcs), gcs);

    EXPECT_EQ(seen, std::optional<std::int64_t>{5'000'000});
}

}  // namespace
}  // namespace argus::mavlink_gateway
