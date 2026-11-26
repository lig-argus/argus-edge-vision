#include "argus/mavlink_gateway/gcs_fc_forwarder.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/command_activity.hpp"
#include "argus/mavlink_gateway/gcs_registry.hpp"
#include "argus/mavlink_gateway/landing_lock_state.hpp"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "argus/mavlink_gateway/rc_override_state.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_follow_status_source.hpp"
#include "fake_fc_state_source.hpp"
#include "fake_gcs_link.hpp"
#include "fake_mavlink_connection.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway {
namespace {

using test_messages::ack_of;
using test_messages::gcs_peer;
using test_messages::heartbeat_from;
using test_messages::kNan;
using test_messages::statustext_of;

constexpr std::int64_t kSecond = 1'000'000'000;

class FakeAuthority : public IControlAuthority {
public:
    std::optional<std::uint8_t> controller() const override { return owner; }
    bool is_controller(std::uint8_t sysid) const override { return owner == sysid; }

    std::optional<std::uint8_t> owner;
};

class FakeFcCommandStatus : public IFcCommandStatus {
public:
    bool sending_setpoints() const override { return sending; }
    bool entering() const override { return false; }
    std::optional<std::int64_t> last_setpoint_timeout_ns() const override { return std::nullopt; }

    bool sending = false;
};

class FakeStopRequester : public IStopRequester {
public:
    bool stop_from_onboard() override {
        ++calls;
        return result;
    }

    std::int32_t calls = 0;
    bool result = true;
};

// sysid 통제기가 FC(1, 1)에 보낸 COMMAND_LONG
mavlink_message_t fc_command_from(const GcsPeer& peer, std::uint16_t command, float param1 = 0.0F,
                                  float param2 = 0.0F, float param3 = 0.0F,
                                  std::uint8_t target_component = kFcCompid) {
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(peer.sysid, peer.compid, &message, kVehicleSysid,
                                  target_component, command, 0, param1, param2, param3, kNan, kNan,
                                  kNan, kNan);
    return message;
}

// FC 가 보낸 COMMAND_ACK
mavlink_message_t fc_ack(std::uint16_t command, MAV_RESULT result, std::uint8_t target_system,
                         std::uint8_t target_component) {
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(kVehicleSysid, kFcCompid, &message, command,
                                 static_cast<std::uint8_t>(result), 0, 0, target_system,
                                 target_component);
    return message;
}

class GcsFcForwarderTest : public ::testing::Test {
protected:
    GcsFcForwarderTest() {
        clock_.advance_ns(100 * kSecond);
        fc_state_.set_status(vehicle_api::FlightMode::Position, clock_.now_monotonic_ns());
        receive(heartbeat_from(controller_), controller_);
        receive(heartbeat_from(other_), other_);
        authority_.owner = controller_.sysid;
        events_subscription_ = forwarder_.subscribe(
            [this](const ForwardedCommandEvent& event) { events_.push_back(event); });
        link_.clear_sent();
    }

    void receive(const mavlink_message_t& message, const GcsPeer& sender) {
        link_.simulate_message_received(message, sender);
    }

    void receive_from_fc(const mavlink_message_t& message) {
        connection_.simulate_message_received(message);
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    std::vector<mavlink_message_t> forwarded() const { return connection_.sent(); }

    const GcsPeer controller_ = gcs_peer(250, "192.168.0.10");
    const GcsPeer other_ = gcs_peer(251, "192.168.0.11");

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    FakeGcsLink link_;
    GcsRegistry registry_{link_, clock_};
    FakeMavlinkConnection connection_{poller_};
    FakeAuthority authority_;
    FakeFcStateSource fc_state_;
    FakeFcCommandStatus command_status_;
    CommandActivity activity_;
    vehicle_api_testing::FakeFollowStatusSource follow_status_;
    InactiveLandingLock landing_lock_;
    InactiveRcOverride rc_override_;
    FakeStopRequester stop_requester_;
    GcsFcForwarder forwarder_{GcsFcForwarderInputs{
        registry_, link_, registry_, connection_, clock_, authority_, fc_state_, command_status_,
        activity_, follow_status_, landing_lock_, rc_override_, stop_requester_}};
    std::vector<ForwardedCommandEvent> events_;
    core::Subscription events_subscription_;
};

TEST_F(GcsFcForwarderTest, ControllerCommandIsForwardedUnchanged) {
    const mavlink_message_t command = fc_command_from(controller_, MAV_CMD_NAV_TAKEOFF);
    receive(command, controller_);

    ASSERT_EQ(forwarded().size(), 1U);
    EXPECT_EQ(forwarded()[0].sysid, controller_.sysid);
    EXPECT_EQ(forwarded()[0].compid, kGcsCompid);
    EXPECT_EQ(forwarded()[0].msgid, static_cast<std::uint32_t>(MAVLINK_MSG_ID_COMMAND_LONG));
    EXPECT_EQ(forwarded()[0].checksum, command.checksum);
    EXPECT_TRUE(link_.sent().empty());
}

TEST_F(GcsFcForwarderTest, OnlyControllerHeartbeatIsForwarded) {
    receive(heartbeat_from(other_), other_);
    EXPECT_TRUE(forwarded().empty());

    receive(heartbeat_from(controller_), controller_);
    ASSERT_EQ(forwarded().size(), 1U);
    EXPECT_EQ(forwarded()[0].sysid, controller_.sysid);
}

TEST_F(GcsFcForwarderTest, RejectionIsSentOnlyToSenderFromCompid195) {
    receive(fc_command_from(other_, MAV_CMD_NAV_TAKEOFF), other_);

    EXPECT_TRUE(forwarded().empty());
    ASSERT_EQ(link_.sent().size(), 2U);
    for (const FakeGcsLink::Sent& sent : link_.sent()) {
        ASSERT_TRUE(sent.to.has_value());
        EXPECT_TRUE(sent.to.value_or(GcsPeer{}).same_address(other_));
        EXPECT_EQ(sent.message.sysid, kVehicleSysid);
        EXPECT_EQ(sent.message.compid, kOnboardCommandCompid);
    }
    const mavlink_command_ack_t ack = ack_of(link_.sent()[0].message);
    EXPECT_EQ(ack.command, MAV_CMD_NAV_TAKEOFF);
    EXPECT_EQ(ack.result, MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(ack.target_system, other_.sysid);
    EXPECT_EQ(ack.target_component, other_.compid);
    EXPECT_EQ(statustext_of(link_.sent()[1].message), "제어권 없음");
}

TEST_F(GcsFcForwarderTest, CommandsNotForFcAreIgnored) {
    receive(fc_command_from(controller_, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, kOnboardCommandCompid),
            controller_);
    receive(fc_command_from(controller_, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0), controller_);

    EXPECT_TRUE(forwarded().empty());
    EXPECT_TRUE(link_.sent().empty());
}

TEST_F(GcsFcForwarderTest, HoldWhileHoveringStopsOnboardAndAnswers) {
    command_status_.sending = true;
    receive(fc_command_from(controller_, MAV_CMD_DO_SET_MODE,
                            static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 4, 3),
            controller_);

    EXPECT_EQ(stop_requester_.calls, 1);
    EXPECT_TRUE(forwarded().empty());
    ASSERT_EQ(link_.sent().size(), 1U);
    EXPECT_EQ(ack_of(link_.sent()[0].message).result, MAV_RESULT_ACCEPTED);
}

TEST_F(GcsFcForwarderTest, HoldThatCannotStopIsRejected) {
    command_status_.sending = true;
    stop_requester_.result = false;
    receive(fc_command_from(controller_, MAV_CMD_DO_SET_MODE,
                            static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 4, 3),
            controller_);

    ASSERT_EQ(link_.sent().size(), 2U);
    EXPECT_EQ(ack_of(link_.sent()[0].message).result, MAV_RESULT_TEMPORARILY_REJECTED);
}

TEST_F(GcsFcForwarderTest, StaleFcStatusIsUnknown) {
    fc_state_.set_status(vehicle_api::FlightMode::Position, clock_.now_monotonic_ns(), false);
    clock_.advance_ns(3 * kSecond + 1);
    receive(fc_command_from(controller_, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN), controller_);

    EXPECT_TRUE(forwarded().empty());
    ASSERT_EQ(link_.sent().size(), 2U);
    EXPECT_EQ(statustext_of(link_.sent()[1].message), "FC 상태 모름");
}

TEST_F(GcsFcForwarderTest, FcAckIsSentOnlyToTargetGcs) {
    receive_from_fc(fc_ack(MAV_CMD_NAV_TAKEOFF, MAV_RESULT_ACCEPTED, other_.sysid, kGcsCompid));

    ASSERT_EQ(link_.sent().size(), 1U);
    ASSERT_TRUE(link_.sent()[0].to.has_value());
    EXPECT_TRUE(link_.sent()[0].to.value_or(GcsPeer{}).same_address(other_));
    EXPECT_EQ(link_.sent()[0].message.compid, kFcCompid);
}

TEST_F(GcsFcForwarderTest, OnboardAckAndUnknownTargetsAreNotSent) {
    receive_from_fc(
        fc_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED, kVehicleSysid, kOnboardCommandCompid));
    receive_from_fc(fc_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED, 252, kGcsCompid));

    EXPECT_TRUE(link_.sent().empty());
    EXPECT_TRUE(events_.empty());
}

TEST_F(GcsFcForwarderTest, EventsAreSentForForwardedCommandAndAck) {
    receive(fc_command_from(controller_, MAV_CMD_DO_SET_MODE,
                            static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 3, 0),
            controller_);
    receive_from_fc(
        fc_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED, controller_.sysid, kGcsCompid));

    ASSERT_EQ(events_.size(), 2U);
    EXPECT_EQ(events_[0].stage, ForwardStage::Forwarded);
    EXPECT_EQ(events_[0].command, MAV_CMD_DO_SET_MODE);
    EXPECT_EQ(events_[0].requested_mode, vehicle_api::FlightMode::Position);
    EXPECT_EQ(events_[0].result, std::nullopt);
    EXPECT_EQ(events_[0].sender.sysid, controller_.sysid);
    EXPECT_EQ(events_[1].stage, ForwardStage::Answered);
    EXPECT_EQ(events_[1].result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(events_[1].requested_mode, std::nullopt);
}

TEST_F(GcsFcForwarderTest, NoEventForRejectedOrFailedSend) {
    receive(fc_command_from(other_, MAV_CMD_NAV_LAND), other_);
    connection_.set_send_result(false);
    receive(fc_command_from(controller_, MAV_CMD_NAV_LAND), controller_);

    EXPECT_TRUE(events_.empty());
}

TEST_F(GcsFcForwarderTest, NonCommandHeartbeatProducesNoEvent) {
    receive(heartbeat_from(controller_), controller_);

    EXPECT_EQ(forwarded().size(), 1U);
    EXPECT_TRUE(events_.empty());
}

}  // namespace
}  // namespace argus::mavlink_gateway
