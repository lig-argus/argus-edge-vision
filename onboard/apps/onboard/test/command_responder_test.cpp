#include <gtest/gtest.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "argus/mavlink.h"
#include "argus/onboard/mavlink_command_handler.hpp"

namespace argus::onboard {
namespace {

class CommandResponderTest : public ::testing::Test {
protected:
    CommandResponder make_responder() {
        return CommandResponder([this](const CommandAck& ack) { sent_.push_back(ack); });
    }

    std::vector<CommandAck> sent_;
};

TEST_F(CommandResponderTest, SendsInProgressRepeatedlyThenFinalOnce) {
    CommandResponder responder = make_responder();

    responder.send({MAV_RESULT_IN_PROGRESS, 0, ""});
    responder.send({MAV_RESULT_IN_PROGRESS, 50, ""});
    EXPECT_FALSE(responder.completed());

    responder.send({MAV_RESULT_ACCEPTED, kProgressUnknown, ""});
    EXPECT_TRUE(responder.completed());

    responder.send({MAV_RESULT_FAILED, kProgressUnknown, "ignored"});  // 최종 뒤는 무시
    responder.send({MAV_RESULT_IN_PROGRESS, 70, ""});

    ASSERT_EQ(sent_.size(), 3U);
    EXPECT_EQ(sent_[0].result, MAV_RESULT_IN_PROGRESS);
    EXPECT_EQ(sent_[0].progress, 0);
    EXPECT_EQ(sent_[1].progress, 50);
    EXPECT_EQ(sent_[2].result, MAV_RESULT_ACCEPTED);
}

TEST_F(CommandResponderTest, DeniedIsFinalWithReason) {
    {
        CommandResponder responder = make_responder();
        responder.send({MAV_RESULT_DENIED, kProgressUnknown, "not the controlling GCS"});
        EXPECT_TRUE(responder.completed());
    }
    ASSERT_EQ(sent_.size(), 1U);  // 소멸자가 FAILED 를 더 보내지 않는다
    EXPECT_EQ(sent_[0].result, MAV_RESULT_DENIED);
    EXPECT_EQ(sent_[0].reason, "not the controlling GCS");
}

TEST_F(CommandResponderTest, DestroyedWithoutFinalSendsFailed) {
    {
        CommandResponder responder = make_responder();
        responder.send({MAV_RESULT_IN_PROGRESS, 10, ""});
    }
    ASSERT_EQ(sent_.size(), 2U);
    EXPECT_EQ(sent_[1].result, MAV_RESULT_FAILED);
    EXPECT_EQ(sent_[1].progress, kProgressUnknown);
    EXPECT_FALSE(sent_[1].reason.empty());
}

TEST_F(CommandResponderTest, DestroyedWithoutAnySendAlsoSendsFailed) {
    {
        const CommandResponder responder = make_responder();
    }
    ASSERT_EQ(sent_.size(), 1U);
    EXPECT_EQ(sent_[0].result, MAV_RESULT_FAILED);
}

TEST_F(CommandResponderTest, MovedFromResponderSendsNothing) {
    std::optional<CommandResponder> moved_to;
    {
        CommandResponder responder = make_responder();
        moved_to.emplace(std::move(responder));
    }  // 원본 소멸: 빈 responder 라 FAILED 를 보내지 않는다
    EXPECT_TRUE(sent_.empty());

    moved_to->send({MAV_RESULT_ACCEPTED, kProgressUnknown, ""});
    ASSERT_EQ(sent_.size(), 1U);
    EXPECT_EQ(sent_[0].result, MAV_RESULT_ACCEPTED);
}

TEST_F(CommandResponderTest, MoveAssignmentFinishesPreviousCommand) {
    CommandResponder first = make_responder();
    first.send({MAV_RESULT_IN_PROGRESS, 20, ""});

    first = make_responder();  // 아직 안 끝난 첫 명령은 FAILED 로 끝낸다
    ASSERT_EQ(sent_.size(), 2U);
    EXPECT_EQ(sent_[1].result, MAV_RESULT_FAILED);

    first.send({MAV_RESULT_ACCEPTED, kProgressUnknown, ""});
    ASSERT_EQ(sent_.size(), 3U);
    EXPECT_EQ(sent_[2].result, MAV_RESULT_ACCEPTED);
}

TEST_F(CommandResponderTest, EmptyResponderDoesNothing) {
    {
        CommandResponder empty;
        empty.send({MAV_RESULT_ACCEPTED, kProgressUnknown, ""});
        EXPECT_FALSE(empty.completed());
    }
    EXPECT_TRUE(sent_.empty());
}

TEST_F(CommandResponderTest, ResponderSharedByCallbacksFailsWhenLastCallbackIsDropped) {
    // handler 의 사용 방식: std::function 콜백은 복사 가능해야 하므로 responder 를 shared_ptr 로
    // 싸서 추종 제어 응답 콜백과 follow/status 콜백이 함께 든다
    std::function<void()> on_reply;
    std::function<void()> on_status;
    {
        auto shared = std::make_shared<CommandResponder>(make_responder());
        on_reply = [shared] { shared->send({MAV_RESULT_IN_PROGRESS, 0, ""}); };
        on_status = [shared] { shared->send({MAV_RESULT_IN_PROGRESS, 40, ""}); };
    }
    EXPECT_TRUE(sent_.empty());

    on_reply();
    on_status();
    ASSERT_EQ(sent_.size(), 2U);
    EXPECT_EQ(sent_[1].progress, 40);

    on_reply = nullptr;  // 하나가 남아 있으면 아직 보내지 않는다
    EXPECT_EQ(sent_.size(), 2U);

    on_status = nullptr;  // 마지막 콜백이 사라지면 최종 미송신 → FAILED
    ASSERT_EQ(sent_.size(), 3U);
    EXPECT_EQ(sent_[2].result, MAV_RESULT_FAILED);
}

}  // namespace
}  // namespace argus::onboard
