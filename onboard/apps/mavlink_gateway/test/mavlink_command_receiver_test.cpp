#include "argus/mavlink_gateway/mavlink_command_receiver.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/command_activity.hpp"
#include "argus/mavlink_gateway/control_authority.hpp"
#include "argus/mavlink_gateway/gcs_registry.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "fake_gcs_link.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway {
namespace {

using test_messages::ack_of;
using test_messages::command_int_from;
using test_messages::command_long_from;
using test_messages::gcs_peer;
using test_messages::heartbeat_from;
using test_messages::kNan;
using test_messages::statustext_of;

constexpr CommandRules kStartRules{true, true, CommandKind::Start};
constexpr CommandRules kStopRules{true, true, CommandKind::Stop};
constexpr CommandRules kOtherRules{true, false, CommandKind::Other};

class FakeControlAuthority : public IControlAuthority {
public:
    std::optional<std::uint8_t> controller() const override { return owner; }
    bool is_controller(std::uint8_t sysid) const override { return owner == sysid; }

    std::optional<std::uint8_t> owner;
};

// 받은 요청과 responder 를 보관한다. 테스트가 responder 로 결과를 보낸다.
// reply 가 있으면 받자마자 그 결과를 보낸다
class RecordingHandler : public IMavlinkCommandHandler {
public:
    explicit RecordingHandler(std::uint16_t command) : command_(command) {}

    std::uint16_t command() const override { return command_; }
    void handle(const CommandRequest& request, CommandResponder responder) override {
        requests.push_back(request);
        if (activity != nullptr) {
            activity_during_handle.push_back(activity->command_in_progress());
        }
        if (reply.has_value()) {
            responder.send(*reply);
            return;
        }
        responders.push_back(std::move(responder));
    }

    std::vector<CommandRequest> requests;
    std::vector<CommandResponder> responders;
    std::optional<CommandAck> reply;
    const CommandActivity* activity = nullptr;
    std::vector<bool> activity_during_handle;

private:
    std::uint16_t command_;
};

class MavlinkCommandReceiverTest : public ::testing::Test {
protected:
    MavlinkCommandReceiverTest() {
        authority_.owner = 250;
        link_.simulate_message_received(heartbeat_from(owner_), owner_);
        link_.simulate_message_received(heartbeat_from(other_), other_);
        receiver_->add_handler(track_start_, kStartRules);
        receiver_->add_handler(scan_start_, kStartRules);
        receiver_->add_handler(track_stop_, kStopRules);
        receiver_->add_handler(request_control_, kOtherRules);
    }

    void send(const GcsPeer& sender, std::uint16_t command, float number) {
        link_.simulate_message_received(command_long_from(sender, command, number), sender);
    }

    // 보낸 COMMAND_ACK 만 순서대로
    std::vector<mavlink_command_ack_t> acks() const {
        std::vector<mavlink_command_ack_t> result;
        for (const FakeGcsLink::Sent& sent : link_.sent()) {
            if (sent.message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                result.push_back(ack_of(sent.message));
            }
        }
        return result;
    }

    std::vector<std::string> statustexts() const {
        std::vector<std::string> result;
        for (const FakeGcsLink::Sent& sent : link_.sent()) {
            if (sent.message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                result.push_back(statustext_of(sent.message));
            }
        }
        return result;
    }

    std::optional<MAV_RESULT> last_result() const {
        const std::vector<mavlink_command_ack_t> all = acks();
        if (all.empty()) {
            return std::nullopt;
        }
        return static_cast<MAV_RESULT>(all.back().result);
    }

    const GcsPeer owner_ = gcs_peer(250, "192.168.0.10", 40000);
    const GcsPeer other_ = gcs_peer(251, "192.168.0.11", 40001);

    core_testing::FakeClock clock_;
    FakeGcsLink link_;
    GcsRegistry registry_{link_, clock_};
    FakeControlAuthority authority_;
    CommandActivity activity_;
    RecordingHandler track_start_{MAV_CMD_ARGUS_TRACK_START};
    RecordingHandler scan_start_{MAV_CMD_ARGUS_SCAN_START};
    RecordingHandler track_stop_{MAV_CMD_ARGUS_TRACK_STOP};
    RecordingHandler request_control_{MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL};
    std::unique_ptr<MavlinkCommandReceiver> receiver_ =
        std::make_unique<MavlinkCommandReceiver>(registry_, link_, clock_, authority_, activity_);
};

TEST_F(MavlinkCommandReceiverTest, RejectsSecondHandlerForSameCommand) {
    RecordingHandler duplicate{MAV_CMD_ARGUS_TRACK_START};

    EXPECT_FALSE(receiver_->add_handler(duplicate, kStartRules));
}

TEST_F(MavlinkCommandReceiverTest, DropsCommandNotAddressedToOnboardCommandCompid) {
    for (const std::uint8_t component : {std::uint8_t{0}, std::uint8_t{1}}) {
        link_.simulate_message_received(
            command_long_from(owner_, MAV_CMD_ARGUS_TRACK_START, 1.0F, kNan, kNan, 1, component),
            owner_);
    }
    link_.simulate_message_received(
        command_long_from(owner_, MAV_CMD_ARGUS_TRACK_START, 1.0F, kNan, kNan, 2), owner_);

    EXPECT_TRUE(link_.sent().empty());
    EXPECT_TRUE(track_start_.requests.empty());
}

TEST_F(MavlinkCommandReceiverTest, AnswersUnsupportedForUnregisteredCommand) {
    send(owner_, MAV_CMD_ARGUS_END_RC_OVERRIDE, 1.0F);

    EXPECT_EQ(last_result(), std::optional<MAV_RESULT>{MAV_RESULT_UNSUPPORTED});
}

TEST_F(MavlinkCommandReceiverTest, DeniesInvalidCommandNumber) {
    for (const float number : {kNan, -1.0F, 1.5F, 16'777'216.0F}) {
        send(owner_, MAV_CMD_ARGUS_TRACK_START, number);
        EXPECT_EQ(last_result(), std::optional<MAV_RESULT>{MAV_RESULT_DENIED}) << number;
    }
    EXPECT_TRUE(track_start_.requests.empty());
    EXPECT_EQ(statustexts().size(), 4U);
}

TEST_F(MavlinkCommandReceiverTest, PassesCommandToHandlerAndAnswersOnlyTheSender) {
    track_start_.reply = CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, {}};

    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);

    ASSERT_EQ(track_start_.requests.size(), 1U);
    EXPECT_EQ(track_start_.requests[0].sender.sysid, 250);
    ASSERT_EQ(link_.sent().size(), 1U);
    EXPECT_TRUE(link_.sent()[0].to.value_or(GcsPeer{}).same_address(owner_));
    const mavlink_command_ack_t ack = ack_of(link_.sent()[0].message);
    EXPECT_EQ(ack.command, MAV_CMD_ARGUS_TRACK_START);
    EXPECT_EQ(ack.result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(ack.target_system, 250);
    EXPECT_EQ(ack.target_component, kGcsCompid);
    EXPECT_EQ(link_.sent()[0].message.sysid, kVehicleSysid);
    EXPECT_EQ(link_.sent()[0].message.compid, kOnboardCommandCompid);
}

TEST_F(MavlinkCommandReceiverTest, RetransmissionResendsFinalResultWithoutCallingHandler) {
    track_start_.reply = CommandAck{MAV_RESULT_DENIED, kProgressUnknown, "no track"};
    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);

    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);

    EXPECT_EQ(track_start_.requests.size(), 1U);
    ASSERT_EQ(acks().size(), 2U);
    EXPECT_EQ(acks()[1].result, MAV_RESULT_DENIED);
    EXPECT_EQ(statustexts(), (std::vector<std::string>{"no track", "no track"}));
}

TEST_F(MavlinkCommandReceiverTest, RetransmissionWhileInProgressResendsInProgress) {
    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);
    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);  // handler 가 아직 답하지 않음
    ASSERT_EQ(track_start_.responders.size(), 1U);
    track_start_.responders[0].send(CommandAck{MAV_RESULT_IN_PROGRESS, 40, {}});

    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);

    ASSERT_EQ(acks().size(), 3U);
    EXPECT_EQ(acks()[0].result, MAV_RESULT_IN_PROGRESS);
    EXPECT_EQ(acks()[0].progress, kProgressUnknown);
    EXPECT_EQ(acks()[2].result, MAV_RESULT_IN_PROGRESS);
    EXPECT_EQ(acks()[2].progress, 40);
    EXPECT_EQ(track_start_.requests.size(), 1U);
}

TEST_F(MavlinkCommandReceiverTest, DropsOlderNumberAndTreatsWrappedNumberAsNew) {
    track_start_.reply = CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, {}};
    send(owner_, MAV_CMD_ARGUS_TRACK_START, 16'777'215.0F);

    send(owner_, MAV_CMD_ARGUS_TRACK_START, 16'777'214.0F);  // 늦게 도착한 재전송
    EXPECT_EQ(acks().size(), 1U);

    send(owner_, MAV_CMD_ARGUS_TRACK_START, 0.0F);  // 16,777,215 다음 번호
    EXPECT_EQ(track_start_.requests.size(), 2U);
    EXPECT_EQ(acks().size(), 2U);
}

TEST_F(MavlinkCommandReceiverTest, NumbersAreKeptPerSenderAndCommand) {
    track_start_.reply = CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, {}};
    request_control_.reply = CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, {}};
    authority_.owner = std::nullopt;
    send(owner_, MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL, 5.0F);
    send(other_, MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL, 5.0F);

    EXPECT_EQ(request_control_.requests.size(), 2U);
}

TEST_F(MavlinkCommandReceiverTest, RejectsCommandFromGcsWithoutControl) {
    send(other_, MAV_CMD_ARGUS_TRACK_START, 7.0F);

    EXPECT_TRUE(track_start_.requests.empty());
    EXPECT_EQ(last_result(), std::optional<MAV_RESULT>{MAV_RESULT_TEMPORARILY_REJECTED});
    EXPECT_EQ(statustexts(), std::vector<std::string>{"제어권 없음"});
    EXPECT_TRUE(link_.sent().back().to.value_or(GcsPeer{}).same_address(other_));
}

TEST_F(MavlinkCommandReceiverTest, CommandWithoutControlRuleReachesHandler) {
    send(other_, MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL, 7.0F);

    EXPECT_EQ(request_control_.requests.size(), 1U);
}

TEST_F(MavlinkCommandReceiverTest, RejectsSameOrOtherStartCommandWhileStartIsInProgress) {
    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);

    send(owner_, MAV_CMD_ARGUS_TRACK_START, 8.0F);
    EXPECT_EQ(last_result(), std::optional<MAV_RESULT>{MAV_RESULT_TEMPORARILY_REJECTED});
    send(owner_, MAV_CMD_ARGUS_SCAN_START, 9.0F);
    EXPECT_EQ(last_result(), std::optional<MAV_RESULT>{MAV_RESULT_TEMPORARILY_REJECTED});

    EXPECT_EQ(track_start_.requests.size(), 1U);
    EXPECT_TRUE(scan_start_.requests.empty());
}

TEST_F(MavlinkCommandReceiverTest, OtherCommandIsHandledWhileStartIsInProgress) {
    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);

    send(owner_, MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL, 8.0F);

    EXPECT_EQ(request_control_.requests.size(), 1U);
}

TEST_F(MavlinkCommandReceiverTest, StopCancelsStartInProgressThenReachesHandler) {
    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);

    send(owner_, MAV_CMD_ARGUS_TRACK_STOP, 8.0F);

    ASSERT_EQ(acks().size(), 1U);
    EXPECT_EQ(acks()[0].command, MAV_CMD_ARGUS_TRACK_START);
    EXPECT_EQ(acks()[0].result, MAV_RESULT_CANCELLED);
    EXPECT_EQ(track_stop_.requests.size(), 1U);

    // 취소된 뒤 handler 가 보낸 결과는 보내지 않는다
    track_start_.responders[0].send(CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, {}});
    EXPECT_EQ(acks().size(), 1U);
}

TEST_F(MavlinkCommandReceiverTest, StopFromGcsWithoutControlDoesNotCancel) {
    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);

    send(other_, MAV_CMD_ARGUS_TRACK_STOP, 8.0F);

    ASSERT_EQ(acks().size(), 1U);
    EXPECT_EQ(acks()[0].command, MAV_CMD_ARGUS_TRACK_STOP);
    EXPECT_EQ(acks()[0].result, MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_TRUE(track_stop_.requests.empty());
}

TEST_F(MavlinkCommandReceiverTest, TickResendsLastInProgressOfRunningCommands) {
    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);
    track_start_.responders[0].send(CommandAck{MAV_RESULT_IN_PROGRESS, 20, {}});

    receiver_->tick();
    ASSERT_EQ(acks().size(), 2U);
    EXPECT_EQ(acks()[1].result, MAV_RESULT_IN_PROGRESS);
    EXPECT_EQ(acks()[1].progress, 20);

    track_start_.responders[0].send(CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, {}});
    receiver_->tick();
    EXPECT_EQ(acks().size(), 3U);
}

TEST_F(MavlinkCommandReceiverTest, ForgetsNumberSixtySecondsAfterLastCommand) {
    track_start_.reply = CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, {}};
    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);
    clock_.advance_ns(kCommandRecordLifetimeNs + 1);
    receiver_->tick();

    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);

    EXPECT_EQ(track_start_.requests.size(), 2U);
}

TEST_F(MavlinkCommandReceiverTest, ActivityShowsUnfinishedCommandOutsideItsOwnHandler) {
    track_start_.activity = &activity_;
    request_control_.activity = &activity_;
    request_control_.reply = CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, {}};

    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);
    EXPECT_TRUE(activity_.command_in_progress());
    send(owner_, MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL, 8.0F);
    track_start_.responders[0].send(CommandAck{MAV_RESULT_FAILED, kProgressUnknown, "x"});

    EXPECT_FALSE(activity_.command_in_progress());
    EXPECT_EQ(track_start_.activity_during_handle, std::vector<bool>{false});
    EXPECT_EQ(request_control_.activity_during_handle, std::vector<bool>{true});
}

TEST_F(MavlinkCommandReceiverTest, CommandIntGetsCommandLongOnlyOrUnsupported) {
    link_.simulate_message_received(command_int_from(owner_, MAV_CMD_ARGUS_TRACK_START), owner_);
    EXPECT_EQ(last_result(), std::optional<MAV_RESULT>{MAV_RESULT_COMMAND_LONG_ONLY});

    link_.simulate_message_received(command_int_from(owner_, MAV_CMD_DO_SET_MODE), owner_);
    EXPECT_EQ(last_result(), std::optional<MAV_RESULT>{MAV_RESULT_UNSUPPORTED});
    EXPECT_TRUE(track_start_.requests.empty());
}

TEST_F(MavlinkCommandReceiverTest, ResponderOutlivingReceiverSendsNothing) {
    send(owner_, MAV_CMD_ARGUS_TRACK_START, 7.0F);
    CommandResponder responder = std::move(track_start_.responders[0]);
    link_.clear_sent();

    receiver_.reset();
    responder.send(CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, {}});

    EXPECT_TRUE(link_.sent().empty());
}

}  // namespace
}  // namespace argus::mavlink_gateway
