#include "argus/mavlink_gateway/request_operator_control_handler.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/control_requests.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway {
namespace {

using test_messages::gcs_peer;
using test_messages::kNan;

class FakeControlRequests : public IControlRequests {
public:
    MAV_RESULT request(const GcsPeer& sender, ControlAction action) override {
        calls.emplace_back(sender.sysid, action);
        return result;
    }

    std::vector<std::pair<std::uint8_t, ControlAction>> calls;
    MAV_RESULT result = MAV_RESULT_ACCEPTED;
};

class RequestOperatorControlHandlerTest : public ::testing::Test {
protected:
    // 보낸 통제기 sysid 250 의 45010 을 처리하고 보낸 결과들을 반환한다
    std::vector<CommandAck> handle(float param1, float param4) {
        CommandRequest request;
        request.sender = gcs_peer(250);
        request.command.command = MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL;
        request.command.param1 = param1;
        request.command.param4 = param4;
        std::vector<CommandAck> sent;
        handler_.handle(request,
                        CommandResponder([&sent](const CommandAck& ack) { sent.push_back(ack); }));
        return sent;
    }

    FakeControlRequests requests_;
    RequestOperatorControlHandler handler_{requests_};
};

TEST_F(RequestOperatorControlHandlerTest, HandlesRequestOperatorControlCommand) {
    EXPECT_EQ(handler_.command(), MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL);
    EXPECT_TRUE(RequestOperatorControlHandler::kRules.numbered);
    EXPECT_FALSE(RequestOperatorControlHandler::kRules.requires_control);
}

TEST_F(RequestOperatorControlHandlerTest, PassesRequestAndReleaseWithResult) {
    requests_.result = MAV_RESULT_TEMPORARILY_REJECTED;

    const std::vector<CommandAck> request = handle(1.0F, 0.0F);
    const std::vector<CommandAck> release = handle(0.0F, 250.0F);

    ASSERT_EQ(request.size(), 1U);
    EXPECT_EQ(request[0].result, MAV_RESULT_TEMPORARILY_REJECTED);
    ASSERT_EQ(release.size(), 1U);
    EXPECT_EQ(requests_.calls, (std::vector<std::pair<std::uint8_t, ControlAction>>{
                                   {250, ControlAction::Request}, {250, ControlAction::Release}}));
}

TEST_F(RequestOperatorControlHandlerTest, DeniesUnknownAction) {
    for (const float param1 : {2.0F, kNan, 0.5F}) {
        const std::vector<CommandAck> sent = handle(param1, 0.0F);
        ASSERT_EQ(sent.size(), 1U);
        EXPECT_EQ(sent[0].result, MAV_RESULT_DENIED);
        EXPECT_FALSE(sent[0].reason.empty());
    }
    EXPECT_TRUE(requests_.calls.empty());
}

TEST_F(RequestOperatorControlHandlerTest, DeniesRequestNamingAnotherGcs) {
    for (const float param4 : {251.0F, kNan}) {
        const std::vector<CommandAck> sent = handle(1.0F, param4);
        ASSERT_EQ(sent.size(), 1U);
        EXPECT_EQ(sent[0].result, MAV_RESULT_DENIED);
    }
    EXPECT_TRUE(requests_.calls.empty());
}

}  // namespace
}  // namespace argus::mavlink_gateway
