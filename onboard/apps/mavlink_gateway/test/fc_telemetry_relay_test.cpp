#include "argus/mavlink_gateway/fc_telemetry_relay.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "fake_gcs_link.hpp"
#include "fake_mavlink_connection.hpp"

namespace argus::mavlink_gateway {
namespace {

mavlink_message_t attitude_from(std::uint8_t sysid, std::uint8_t compid) {
    mavlink_message_t message{};
    mavlink_msg_attitude_pack(sysid, compid, &message, 1000, 0.1F, 0.2F, 0.3F, 0, 0, 0);
    return message;
}

mavlink_message_t timesync_from_fc() {
    mavlink_message_t message{};
    mavlink_msg_timesync_pack(kVehicleSysid, kFcCompid, &message, 1, 2, kVehicleSysid,
                              kFcObserverCompid);
    return message;
}

class FcTelemetryRelayTest : public ::testing::Test {
protected:
    void receive(const mavlink_message_t& message) {
        connection_.simulate_message_received(message);
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    FakeMavlinkConnection connection_{poller_};
    FakeGcsLink link_;
    FcTelemetryRelay relay_{connection_, link_};
};

TEST_F(FcTelemetryRelayTest, FcTelemetryIsBroadcastUnchanged) {
    const mavlink_message_t attitude = attitude_from(kVehicleSysid, kFcCompid);
    receive(attitude);

    ASSERT_EQ(link_.sent().size(), 1U);
    EXPECT_FALSE(link_.sent()[0].to.has_value());
    EXPECT_EQ(link_.sent()[0].message.compid, kFcCompid);
    EXPECT_EQ(link_.sent()[0].message.checksum, attitude.checksum);
}

TEST_F(FcTelemetryRelayTest, MessagesOutsideListAreNotSent) {
    receive(timesync_from_fc());

    EXPECT_TRUE(link_.sent().empty());
}

TEST_F(FcTelemetryRelayTest, MessagesFromOtherNodesAreNotSent) {
    receive(attitude_from(kVehicleSysid, MAV_COMP_ID_CAMERA));
    receive(attitude_from(255, kFcCompid));

    EXPECT_TRUE(link_.sent().empty());
}

}  // namespace
}  // namespace argus::mavlink_gateway
