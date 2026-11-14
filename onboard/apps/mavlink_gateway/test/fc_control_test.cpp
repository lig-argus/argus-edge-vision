// FcControl 의 입력 수집과 실행 테스트: MAVLink 메시지, FC 의 응답 처리, 모드 변경의 전달.
// 전환의 판단은 fc_command_policy_test, 주기에 걸친 동작은 fc_command_scenario_test 가 확인한다
#include "argus/mavlink_gateway/fc_control.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/fc_command_state.hpp"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "argus/vehicle_api/types.hpp"
#include "fake_fc_state_source.hpp"
#include "fake_mavlink_connection.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::FlightMode;
using vehicle_api::OffboardExitCause;

constexpr std::int64_t kStartNs = 10'000'000'000;
constexpr std::int64_t kTickNs = 50'000'000;
constexpr std::uint16_t kPositionAndYawMask = 2552;
constexpr std::uint16_t kVelocityAndYawRateMask = 1479;

class FcControlTest : public ::testing::Test {
protected:
    void SetUp() override {
        subscription_ = control_.subscribe_mode_events(
            [this](const vehicle_api::FlightModeEvent& event) { events_.push_back(event); });
        set_fc(FlightMode::Hold);
    }

    // FC 의 비행 상태, 자세, local 위치를 지금 받은 것으로 넣는다
    void set_fc(FlightMode mode) {
        const std::int64_t now_ns = clock_.now_monotonic_ns();
        fc_state_.set_status(mode, now_ns);
        vehicle_api::LocalPosition local;
        local.north_m = 5.0;
        local.east_m = 6.0;
        local.down_m = -7.0;
        local.received_monotonic_ns = now_ns;
        fc_state_.state.local = local;
        vehicle_api::Attitude attitude;
        attitude.yaw_rad = 0.25;
        attitude.received_monotonic_ns = now_ns;
        fc_state_.state.attitude = attitude;
    }

    void publish(double forward, std::int64_t valid_ns = 200'000'000) {
        vehicle_api::VelocitySetpoint setpoint;
        setpoint.forward_m_s = forward;
        setpoint.right_m_s = 0.5;
        setpoint.down_m_s = -0.25;
        setpoint.yaw_rate_rad_s = 0.125;
        setpoint.valid_until_monotonic_ns = clock_.now_monotonic_ns() + valid_ns;
        control_.send(setpoint);
    }

    // 한 주기: FC 의 값은 mode 로 매번 새로 받고, 추종 제어가 발행한 경우 forward 를 보낸다
    void step(FlightMode mode, bool follow_publishes = true, double forward = 1.0) {
        clock_.advance_ns(kTickNs);
        set_fc(mode);
        if (follow_publishes) {
            publish(forward);
        }
        control_.tick();
    }

    void steps(std::int32_t count, FlightMode mode, bool follow_publishes = true) {
        for (std::int32_t index = 0; index < count; ++index) {
            step(mode, follow_publishes);
        }
    }

    vehicle_api::RequestHeader header(std::uint64_t command_id,
                                      const std::string& session = "follow") const {
        vehicle_api::RequestHeader value;
        value.id.command_id = vehicle_api::CommandId{command_id};
        value.id.session_id = session;
        value.id.fc_epoch = fc_state_.epoch;
        value.valid_until_monotonic_ns = clock_.now_monotonic_ns() + 1'000'000'000;
        return value;
    }

    void receive_ack(std::uint16_t command, std::uint8_t result, std::uint8_t target_component,
                     std::uint8_t sender_sysid = kVehicleSysid,
                     std::uint8_t sender_compid = kFcCompid) {
        mavlink_message_t message{};
        mavlink_msg_command_ack_pack(sender_sysid, sender_compid, &message, command, result, 0, 0,
                                     kVehicleSysid, target_component);
        connection_.simulate_message_received(message);
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    // setpoint 를 1.1초 동안 발행한 후 진입을 요청한 상태
    void request_entry(std::uint64_t command_id = 1) {
        steps(22, FlightMode::Hold);
        ASSERT_TRUE(control_.request_offboard(header(command_id)).accepted);
    }

    void start_following() {
        request_entry();
        receive_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED, kOnboardCommandCompid);
        step(FlightMode::Offboard);
        ASSERT_TRUE(holds<Following>());
    }

    // 진입이 확인되는 주기까지 진행한다 (상태는 확인하지 않음)
    void start_following_without_assert() {
        steps(22, FlightMode::Hold);
        control_.request_offboard(header(1));
        receive_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED, kOnboardCommandCompid);
        step(FlightMode::Offboard);
    }

    void start_hovering() {
        start_following();
        ASSERT_TRUE(control_.request_hold(header(2)).accepted);
        step(FlightMode::Offboard, false);
        ASSERT_TRUE(control_.offboard_hover());
    }

    template <typename State>
    bool holds() const {
        return std::holds_alternative<State>(control_.state());
    }

    std::vector<mavlink_message_t> sent(std::uint32_t msgid) const {
        std::vector<mavlink_message_t> messages;
        for (const mavlink_message_t& message : connection_.sent()) {
            if (message.msgid == msgid) {
                messages.push_back(message);
            }
        }
        return messages;
    }

    mavlink_set_position_target_local_ned_t last_setpoint() const {
        mavlink_set_position_target_local_ned_t target{};
        const std::vector<mavlink_message_t> messages =
            sent(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
        if (!messages.empty()) {
            mavlink_msg_set_position_target_local_ned_decode(&messages.back(), &target);
        }
        return target;
    }

    core_testing::FakeClock clock_{kStartNs};
    core_testing::FakePoller poller_{clock_};
    FakeMavlinkConnection connection_{poller_};
    FakeFcStateSource fc_state_;
    FcControl control_{connection_, fc_state_, clock_};
    std::vector<vehicle_api::FlightModeEvent> events_;
    core::Subscription subscription_;
};

TEST_F(FcControlTest, StartsIdleAndSendsNothing) {
    steps(5, FlightMode::Hold, false);

    EXPECT_TRUE(holds<Idle>());
    EXPECT_TRUE(connection_.sent().empty());
    EXPECT_FALSE(control_.offboard_hover());
}

TEST_F(FcControlTest, EntryRequestSendsDoSetModeForPx4Offboard) {
    request_entry();

    const std::vector<mavlink_message_t> commands = sent(MAVLINK_MSG_ID_COMMAND_LONG);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_EQ(commands[0].sysid, kVehicleSysid);
    EXPECT_EQ(commands[0].compid, kOnboardCommandCompid);
    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&commands[0], &command);
    EXPECT_EQ(command.command, MAV_CMD_DO_SET_MODE);
    EXPECT_EQ(command.target_system, kVehicleSysid);
    EXPECT_EQ(command.target_component, kFcCompid);
    EXPECT_FLOAT_EQ(command.param1, static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
    EXPECT_FLOAT_EQ(command.param2, 6.0F);  // PX4 의 Offboard (icd.md 부록 A)
    EXPECT_TRUE(holds<Entering>());
}

TEST_F(FcControlTest, EntryRequestNeedsOneSecondOfSetpointStream) {
    steps(20, FlightMode::Hold);  // 첫 송신부터 0.95초
    EXPECT_EQ(control_.request_offboard(header(1)).reason, "prestream not ready");

    steps(1, FlightMode::Hold);
    EXPECT_TRUE(control_.request_offboard(header(2)).accepted);
}

TEST_F(FcControlTest, GapInSetpointStreamRestartsPrestream) {
    steps(22, FlightMode::Hold);
    steps(6, FlightMode::Hold, false);  // 유효 시간(0.2초)이 지나 송신이 멈춤

    steps(10, FlightMode::Hold);
    EXPECT_EQ(control_.request_offboard(header(1)).reason, "prestream not ready");
}

TEST_F(FcControlTest, FailedEntryCommandIsReportedAndCancelsEntry) {
    steps(22, FlightMode::Hold);
    connection_.set_send_result(false);

    const vehicle_api::FlightModeReply reply = control_.request_offboard(header(1));

    EXPECT_FALSE(reply.accepted);
    EXPECT_EQ(reply.reason, "send failed");
    EXPECT_TRUE(holds<CancellingEntry>());
}

TEST_F(FcControlTest, ReplyCarriesRequestId) {
    const vehicle_api::RequestHeader request = header(42);

    const vehicle_api::FlightModeReply reply = control_.request_hold(request);

    EXPECT_TRUE(reply.id.value_or(vehicle_api::RequestId{}) == request.id);
    EXPECT_TRUE(reply.accepted);
    EXPECT_TRUE(reply.reason.empty());
}

TEST_F(FcControlTest, SendsVelocitySetpointInBodyFrame) {
    step(FlightMode::Hold, true, 1.5);

    const std::vector<mavlink_message_t> setpoints =
        sent(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
    ASSERT_EQ(setpoints.size(), 1U);
    EXPECT_EQ(setpoints[0].compid, kOnboardCommandCompid);
    const mavlink_set_position_target_local_ned_t target = last_setpoint();
    EXPECT_EQ(target.coordinate_frame, MAV_FRAME_BODY_NED);
    EXPECT_EQ(target.type_mask, kVelocityAndYawRateMask);
    EXPECT_FLOAT_EQ(target.vx, 1.5F);
    EXPECT_FLOAT_EQ(target.vy, 0.5F);
    EXPECT_FLOAT_EQ(target.vz, -0.25F);
    EXPECT_FLOAT_EQ(target.yaw_rate, 0.125F);
}

TEST_F(FcControlTest, HoverSendsPositionOfMomentItStartedInLocalFrame) {
    start_hovering();

    const mavlink_set_position_target_local_ned_t target = last_setpoint();
    EXPECT_EQ(target.coordinate_frame, MAV_FRAME_LOCAL_NED);
    EXPECT_EQ(target.type_mask, kPositionAndYawMask);
    EXPECT_FLOAT_EQ(target.x, 5.0F);
    EXPECT_FLOAT_EQ(target.y, 6.0F);
    EXPECT_FLOAT_EQ(target.z, -7.0F);
    EXPECT_FLOAT_EQ(target.yaw, 0.25F);
}

TEST_F(FcControlTest, PoseExactlyAtAgeLimitIsUsedForHoverTarget) {
    start_following();
    ASSERT_TRUE(control_.request_hold(header(2)).accepted);

    // 자세와 위치를 받은 후 정확히 0.2초: 목표로 쓴다
    clock_.advance_ns(FcControlConfig{}.pose_max_age_ns);
    control_.tick();
    EXPECT_TRUE(control_.offboard_hover());
}

TEST_F(FcControlTest, HoverWithStalePoseSendsZeroVelocityAndIsNotReportedAsHover) {
    start_following();
    ASSERT_TRUE(control_.request_hold(header(2)).accepted);

    clock_.advance_ns(FcControlConfig{}.pose_max_age_ns + 1);
    control_.tick();

    EXPECT_TRUE(holds<Hovering>());
    EXPECT_FALSE(control_.offboard_hover());
    EXPECT_EQ(last_setpoint().type_mask, kVelocityAndYawRateMask);
    EXPECT_FLOAT_EQ(last_setpoint().vx, 0.0F);
}

TEST_F(FcControlTest, OneSetpointIsUsedForAtMostOneSecond) {
    publish(1.0, 10'000'000'000);
    for (std::int32_t index = 0; index < 20; ++index) {
        clock_.advance_ns(kTickNs);
        set_fc(FlightMode::Hold);
        control_.tick();
    }
    EXPECT_EQ(sent(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED).size(), 20U);

    clock_.advance_ns(kTickNs);
    set_fc(FlightMode::Hold);
    control_.tick();
    EXPECT_EQ(sent(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED).size(), 20U);
}

TEST_F(FcControlTest, UsesAckWithoutTargetAndIgnoresAckOfOtherCommandOrComponent) {
    request_entry();

    receive_ack(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED, kOnboardCommandCompid);
    receive_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED, MAV_COMP_ID_MISSIONPLANNER);
    EXPECT_FALSE(std::get<Entering>(control_.state()).acknowledged);

    receive_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED, 0);
    EXPECT_TRUE(std::get<Entering>(control_.state()).acknowledged);
}

TEST_F(FcControlTest, IgnoresAckNotSentByFc) {
    request_entry();

    receive_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_DENIED, kOnboardCommandCompid, 250, 190);

    EXPECT_TRUE(holds<Entering>());
}

TEST_F(FcControlTest, InProgressAckKeepsEntryPending) {
    request_entry();

    receive_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_IN_PROGRESS, kOnboardCommandCompid);

    ASSERT_TRUE(holds<Entering>());
    EXPECT_FALSE(std::get<Entering>(control_.state()).acknowledged);
}

TEST_F(FcControlTest, RejectingAckCancelsEntry) {
    request_entry();

    receive_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_DENIED, kOnboardCommandCompid);

    EXPECT_TRUE(holds<CancellingEntry>());
}

TEST_F(FcControlTest, FirstStatusIsReportedAndSameModeIsNotRepeated) {
    steps(3, FlightMode::Hold, false);

    ASSERT_EQ(events_.size(), 1U);
    EXPECT_EQ(events_[0].mode, FlightMode::Hold);
    EXPECT_EQ(events_[0].exit_cause, OffboardExitCause::None);
    EXPECT_FALSE(events_[0].request_command_id.has_value());
}

TEST_F(FcControlTest, ModeChangeOutsideOffboardHasNoExitCauseAndDoesNotLock) {
    step(FlightMode::Hold, false);
    step(FlightMode::Position, false);

    ASSERT_EQ(events_.size(), 2U);
    EXPECT_EQ(events_[1].mode, FlightMode::Position);
    EXPECT_EQ(events_[1].exit_cause, OffboardExitCause::None);
    EXPECT_TRUE(holds<Idle>());
}

TEST_F(FcControlTest, ReportsRequestedOffboardWithCommandIdAndStatusTime) {
    request_entry(9);
    receive_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED, kOnboardCommandCompid);
    step(FlightMode::Offboard);

    ASSERT_FALSE(events_.empty());
    EXPECT_EQ(events_.back().mode, FlightMode::Offboard);
    EXPECT_EQ(events_.back().request_command_id.value_or(vehicle_api::CommandId{}).value, 9U);
    EXPECT_EQ(events_.back().received_monotonic_ns, clock_.now_monotonic_ns());
}

TEST_F(FcControlTest, RejectedEntryWhileFcIsInOffboardIsReportedOnceWithoutCommandId) {
    request_entry();
    step(FlightMode::Offboard);  // FC 의 응답보다 모드 변경을 먼저 확인: 전달을 미룸
    const std::size_t before = events_.size();

    receive_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_FAILED, kOnboardCommandCompid);
    steps(3, FlightMode::Offboard, false);

    ASSERT_EQ(events_.size(), before + 1);
    EXPECT_EQ(events_.back().mode, FlightMode::Offboard);
    EXPECT_FALSE(events_.back().request_command_id.has_value());
    EXPECT_TRUE(holds<Hovering>());
}

TEST_F(FcControlTest, RequestAcceptedWhileHoveringIsReportedWithItsCommandId) {
    start_hovering();
    const std::size_t before = events_.size();

    ASSERT_TRUE(control_.request_offboard(header(31)).accepted);

    ASSERT_EQ(events_.size(), before + 1);
    EXPECT_EQ(events_.back().mode, FlightMode::Offboard);
    EXPECT_EQ(events_.back().request_command_id.value_or(vehicle_api::CommandId{}).value, 31U);
    EXPECT_EQ(sent(MAVLINK_MSG_ID_COMMAND_LONG).size(), 1U);  // 진입 명령은 처음 한 번뿐
}

TEST_F(FcControlTest, ExitFromOffboardIsReportedWithCauseByMode) {
    struct Row {
        FlightMode mode;
        OffboardExitCause cause;
    };
    const std::vector<Row> rows = {
        {FlightMode::Position, OffboardExitCause::PilotSwitch},
        {FlightMode::Altitude, OffboardExitCause::PilotSwitch},
        {FlightMode::Hold, OffboardExitCause::Failsafe},
        {FlightMode::Return, OffboardExitCause::Failsafe},
        {FlightMode::Land, OffboardExitCause::Failsafe},
        {FlightMode::Mission, OffboardExitCause::Unknown},
    };
    for (const Row& row : rows) {
        core_testing::FakeClock clock{kStartNs};
        core_testing::FakePoller poller{clock};
        FakeMavlinkConnection connection{poller};
        FakeFcStateSource fc_state;
        FcControl control{connection, fc_state, clock};
        std::vector<vehicle_api::FlightModeEvent> events;
        const core::Subscription subscription = control.subscribe_mode_events(
            [&events](const vehicle_api::FlightModeEvent& event) { events.push_back(event); });

        // mavlink_gateway 가 다시 시작했을 때 FC 가 Offboard 인 경우부터 시작한다
        fc_state.set_status(FlightMode::Offboard, clock.now_monotonic_ns());
        control.tick();
        clock.advance_ns(kTickNs);
        fc_state.set_status(row.mode, clock.now_monotonic_ns());
        control.tick();

        ASSERT_EQ(events.size(), 2U);
        EXPECT_EQ(events[1].mode, row.mode);
        EXPECT_EQ(events[1].exit_cause, row.cause);
        EXPECT_TRUE(std::holds_alternative<Locked>(control.state()));
    }
}

TEST_F(FcControlTest, RequestsUseFcStatusReceivedAfterLastTick) {
    start_following();

    // 주기 작업보다 먼저 도착한 요청도 방금 받은 비행 상태로 판단한다
    clock_.advance_ns(kTickNs);
    set_fc(FlightMode::Position);

    EXPECT_EQ(control_.request_offboard(header(5)).reason, "external mode change");
    EXPECT_FALSE(control_.stop_from_onboard());
    EXPECT_TRUE(holds<Locked>());
}

TEST_F(FcControlTest, RequestWithNewFcEpochIsAcceptedBeforeNextTick) {
    start_hovering();
    ++fc_state_.epoch;
    set_fc(FlightMode::Offboard);

    // 새 fc_epoch 의 요청을, fc_epoch 변경을 처리하는 주기 작업보다 먼저 받은 경우
    EXPECT_TRUE(control_.request_offboard(header(6)).accepted);
    step(FlightMode::Offboard, false);
    EXPECT_TRUE(holds<Following>());
}

TEST_F(FcControlTest, SessionThatNeverPublishesAfterAcceptanceDuringHoverLosesAuthority) {
    start_hovering();
    ASSERT_TRUE(control_.request_offboard(header(3)).accepted);

    // 확인 시간(3초) 동안 setpoint 가 없으면 송신권을 폐기하여 다른 세션이 요청할 수 있게 한다
    steps(61, FlightMode::Offboard, false);

    EXPECT_TRUE(holds<Hovering>());
    EXPECT_TRUE(control_.offboard_hover());
    EXPECT_FLOAT_EQ(last_setpoint().x, 5.0F);
}

TEST_F(FcControlTest, LockIsReleasedOnlyByExplicitRequest) {
    start_following();
    step(FlightMode::Position);
    ASSERT_TRUE(holds<Locked>());

    steps(5, FlightMode::Hold);
    EXPECT_TRUE(holds<Locked>());

    EXPECT_TRUE(control_.release_lock());
    EXPECT_TRUE(holds<Idle>());
    EXPECT_TRUE(control_.release_lock());  // 잠겨 있지 않은 경우에도 true
}

TEST_F(FcControlTest, LockIsNotReleasedWhileFcStatusIsStale) {
    start_following();
    step(FlightMode::Position);
    ASSERT_TRUE(holds<Locked>());

    clock_.advance_ns(FcControlConfig{}.policy.status_max_age_ns + 1);

    EXPECT_FALSE(control_.release_lock());
    EXPECT_TRUE(holds<Locked>());
}

TEST_F(FcControlTest, ModeEventHandlerMayRequestFromInsideCallback) {
    bool requested = false;
    bool accepted = true;
    const core::Subscription subscription =
        control_.subscribe_mode_events([&](const vehicle_api::FlightModeEvent& event) {
            if (event.mode == FlightMode::Offboard && !requested) {
                requested = true;
                accepted = control_.request_hold(header(50)).accepted;
            }
        });

    start_following_without_assert();

    EXPECT_TRUE(requested);
    EXPECT_TRUE(accepted);
    EXPECT_TRUE(holds<Hovering>());
}

TEST_F(FcControlTest, DoesNotCallHandlerAfterSubscriptionIsReleased) {
    subscription_ = core::Subscription{};

    steps(2, FlightMode::Hold, false);

    EXPECT_TRUE(events_.empty());
}

}  // namespace
}  // namespace argus::mavlink_gateway
