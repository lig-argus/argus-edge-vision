#include "argus/mavlink_gateway/request_message_handler.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/command_activity.hpp"
#include "argus/mavlink_gateway/control_authority.hpp"
#include "argus/mavlink_gateway/gcs_registry.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/mavlink_gateway/mavlink_command_receiver.hpp"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "fake_gcs_link.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway {
namespace {

using test_messages::ack_of;
using test_messages::gcs_peer;
using test_messages::heartbeat_from;
using test_messages::kNan;

// 요청한 메시지 대신 보내는 표시용 메시지 (HEARTBEAT)
mavlink_message_t marker_message() {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack_chan(kVehicleSysid, kOnboardCommandCompid, kCommandChannel, &message,
                                    MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0,
                                    MAV_STATE_ACTIVE);
    return message;
}

RequestedMessages accepted_with_marker() {
    RequestedMessages built;
    built.result = MAV_RESULT_ACCEPTED;
    built.messages.push_back(marker_message());
    return built;
}

class RequestMessageHandlerTest : public ::testing::Test {
protected:
    // param1 의 요청을 처리하고, 응답(COMMAND_ACK 는 "ack:<결과>", 보낸 메시지는 "msg:<번호>")을
    // 기록한다. COMMAND_ACK 를 보낼 때 이미 보낸 메시지가 없어야 한다 (COMMAND_ACK 가 먼저)
    std::vector<std::string> handle(float param1) {
        CommandRequest request;
        request.sender = sender_;
        request.command.command = MAV_CMD_REQUEST_MESSAGE;
        request.command.param1 = param1;
        handler_.handle(request, CommandResponder([this](const CommandAck& ack) {
                            EXPECT_TRUE(link_.sent().empty());
                            log_.push_back("ack:" + std::to_string(ack.result));
                            reasons_.push_back(ack.reason);
                        }));
        for (const FakeGcsLink::Sent& sent : link_.sent()) {
            EXPECT_TRUE(sent.to.has_value() && sent.to->same_address(sender_));
            log_.push_back("msg:" + std::to_string(sent.message.msgid));
        }
        link_.clear_sent();
        return log_;
    }

    const GcsPeer sender_ = gcs_peer(251, "192.168.0.11", 40001);
    FakeGcsLink link_;
    RequestMessageHandler handler_{link_};
    std::vector<std::string> log_;
    std::vector<std::string> reasons_;
};

TEST_F(RequestMessageHandlerTest, HandlesRequestMessageWithoutNumberAndControlRules) {
    EXPECT_EQ(handler_.command(), MAV_CMD_REQUEST_MESSAGE);
    EXPECT_FALSE(RequestMessageHandler::kRules.numbered);
    EXPECT_FALSE(RequestMessageHandler::kRules.requires_control);
    EXPECT_EQ(RequestMessageHandler::kRules.kind, CommandKind::Other);
}

TEST_F(RequestMessageHandlerTest, SendsAckBeforeBuiltMessagesToRequester) {
    ASSERT_TRUE(
        handler_.add_message(269, [](const CommandRequest&) { return accepted_with_marker(); }));

    const std::vector<std::string> log = handle(269.0F);

    EXPECT_EQ(log, (std::vector<std::string>{"ack:" + std::to_string(MAV_RESULT_ACCEPTED),
                                             "msg:" + std::to_string(MAVLINK_MSG_ID_HEARTBEAT)}));
}

TEST_F(RequestMessageHandlerTest, SendsBuilderRejectionWithoutMessages) {
    ASSERT_TRUE(handler_.add_message(269, [](const CommandRequest&) {
        RequestedMessages built = accepted_with_marker();
        built.result = MAV_RESULT_DENIED;
        built.reason = "no video stream info";
        return built;
    }));

    const std::vector<std::string> log = handle(269.0F);

    EXPECT_EQ(log, std::vector<std::string>{"ack:" + std::to_string(MAV_RESULT_DENIED)});
    EXPECT_EQ(reasons_, std::vector<std::string>{"no video stream info"});
}

TEST_F(RequestMessageHandlerTest, DeniesUnregisteredOrInvalidMessageId) {
    ASSERT_TRUE(
        handler_.add_message(269, [](const CommandRequest&) { return accepted_with_marker(); }));

    for (const float param1 : {270.0F, 269.5F, -1.0F, kNan, 16'777'216.0F}) {
        log_.clear();
        reasons_.clear();
        EXPECT_EQ(handle(param1),
                  std::vector<std::string>{"ack:" + std::to_string(MAV_RESULT_DENIED)})
            << param1;
        EXPECT_FALSE(reasons_.front().empty());
    }
}

TEST_F(RequestMessageHandlerTest, RejectsDuplicateOrEmptyBuilder) {
    const auto builder = [](const CommandRequest&) { return accepted_with_marker(); };
    EXPECT_TRUE(handler_.add_message(269, builder));
    EXPECT_FALSE(handler_.add_message(269, builder));
    EXPECT_FALSE(handler_.add_message(270, nullptr));
}

class AlwaysOwnerAuthority : public IControlAuthority {
public:
    std::optional<std::uint8_t> controller() const override { return 250; }
    bool is_controller(std::uint8_t sysid) const override { return sysid == 250; }
};

// MavlinkCommandReceiver 에 kRules 로 등록했을 때: 제어권이 없는 통제기의 요청도 처리하고, param7
// 을 명령 번호로 보지 않는다
TEST(RequestMessageHandlerReceiverTest, GcsWithoutControlGetsAnswerRegardlessOfParam7) {
    core_testing::FakeClock clock;
    FakeGcsLink link;
    GcsRegistry registry{link, clock};
    AlwaysOwnerAuthority authority;
    CommandActivity activity;
    MavlinkCommandReceiver receiver{registry, link, clock, authority, activity};
    RequestMessageHandler handler{link};
    ASSERT_TRUE(
        handler.add_message(269, [](const CommandRequest&) { return accepted_with_marker(); }));
    ASSERT_TRUE(receiver.add_handler(handler, RequestMessageHandler::kRules));
    const GcsPeer other = gcs_peer(251, "192.168.0.11", 40001);
    link.simulate_message_received(heartbeat_from(other), other);

    // param7 0 과 같은 값을 두 번 보내도 재전송으로 처리하지 않고 매번 답한다
    for (std::int32_t i = 0; i < 2; ++i) {
        link.clear_sent();
        mavlink_message_t request{};
        mavlink_msg_command_long_pack(other.sysid, other.compid, &request, kVehicleSysid,
                                      kOnboardCommandCompid, MAV_CMD_REQUEST_MESSAGE, 0, 269.0F,
                                      0.0F, kNan, kNan, kNan, kNan, 0.0F);
        link.simulate_message_received(request, other);

        ASSERT_EQ(link.sent().size(), 2U);
        EXPECT_EQ(link.sent()[0].message.msgid, MAVLINK_MSG_ID_COMMAND_ACK);
        EXPECT_EQ(ack_of(link.sent()[0].message).result, MAV_RESULT_ACCEPTED);
        EXPECT_EQ(link.sent()[1].message.msgid, MAVLINK_MSG_ID_HEARTBEAT);
    }
}

}  // namespace
}  // namespace argus::mavlink_gateway
