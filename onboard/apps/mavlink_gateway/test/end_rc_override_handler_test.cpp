#include "argus/mavlink_gateway/end_rc_override_handler.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/fc_hover_entry.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/mavlink_gateway/rc_override_return.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::FlightMode;

class FakeRcOverrideReturn : public IRcOverrideReturn {
public:
    ReturnCheck check_return() const override { return check; }
    bool return_still_allowed() const override { return still_allowed; }
    void end_override() override { ++ended; }

    ReturnCheck check{ReturnBlock::None, FlightMode::Position};
    bool still_allowed = true;
    std::int32_t ended = 0;
};

class FakeHoverEntry : public IFcHoverEntry {
public:
    std::string_view request_hover_entry(StillAllowed still_allowed, Done done) override {
        ++requests;
        if (!denial.empty()) {
            return denial;
        }
        allowed = std::move(still_allowed);
        finish = std::move(done);
        return {};
    }

    std::string_view denial;
    std::int32_t requests = 0;
    StillAllowed allowed;
    Done finish;
};

class EndRcOverrideHandlerTest : public ::testing::Test {
protected:
    void handle() {
        handler_.handle(CommandRequest{},
                        CommandResponder([this](const CommandAck& ack) { acks_.push_back(ack); }));
    }

    FakeRcOverrideReturn rc_return_;
    FakeHoverEntry hover_entry_;
    EndRcOverrideHandler handler_{rc_return_, hover_entry_};
    std::vector<CommandAck> acks_;
};

TEST_F(EndRcOverrideHandlerTest, IsAllowedWhileLockedAndNeedsControl) {
    EXPECT_TRUE(EndRcOverrideHandler::kRules.allowed_while_locked);
    EXPECT_TRUE(EndRcOverrideHandler::kRules.requires_control);
    EXPECT_TRUE(EndRcOverrideHandler::kRules.numbered);
    EXPECT_EQ(handler_.command(), MAV_CMD_ARGUS_END_RC_OVERRIDE);
}

TEST_F(EndRcOverrideHandlerTest, UnmetConditionIsTemporarilyRejectedWithReason) {
    rc_return_.check.block = ReturnBlock::Sticks;

    handle();

    ASSERT_EQ(acks_.size(), 1U);
    EXPECT_EQ(acks_[0].result, MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(acks_[0].reason, std::string(to_text(ReturnBlock::Sticks)));
    EXPECT_EQ(hover_entry_.requests, 0);
    EXPECT_EQ(rc_return_.ended, 0);
}

TEST_F(EndRcOverrideHandlerTest, HoldIsAcceptedWithoutFcCommand) {
    rc_return_.check.fc_mode = FlightMode::Hold;

    handle();

    ASSERT_EQ(acks_.size(), 1U);
    EXPECT_EQ(acks_[0].result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(rc_return_.ended, 1);
    EXPECT_EQ(hover_entry_.requests, 0);
}

TEST_F(EndRcOverrideHandlerTest, PositionReportsProgressThenAcceptsWhenHovering) {
    handle();

    ASSERT_EQ(acks_.size(), 1U);
    EXPECT_EQ(acks_[0].result, MAV_RESULT_IN_PROGRESS);
    ASSERT_TRUE(hover_entry_.allowed);
    EXPECT_TRUE(hover_entry_.allowed());
    rc_return_.still_allowed = false;
    EXPECT_FALSE(hover_entry_.allowed());
    EXPECT_EQ(rc_return_.ended, 0);

    hover_entry_.finish(true);

    ASSERT_EQ(acks_.size(), 2U);
    EXPECT_EQ(acks_[1].result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(rc_return_.ended, 1);
}

TEST_F(EndRcOverrideHandlerTest, FailedEntryKeepsOverride) {
    handle();

    hover_entry_.finish(false);

    ASSERT_EQ(acks_.size(), 2U);
    EXPECT_EQ(acks_[1].result, MAV_RESULT_FAILED);
    EXPECT_FALSE(acks_[1].reason.empty());
    EXPECT_EQ(rc_return_.ended, 0);
}

TEST_F(EndRcOverrideHandlerTest, EntryThatCannotStartIsTemporarilyRejected) {
    hover_entry_.denial = "pose not fresh";

    handle();

    ASSERT_EQ(acks_.size(), 1U);
    EXPECT_EQ(acks_[0].result, MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(rc_return_.ended, 0);
}

}  // namespace
}  // namespace argus::mavlink_gateway
