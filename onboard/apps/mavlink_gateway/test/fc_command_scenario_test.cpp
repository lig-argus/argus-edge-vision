// FC 명령 송신을 실제 구현으로 연결한 시나리오 테스트.
// Fake 는 FC 와의 연결(FakeMavlinkConnection), FC 상태(FakeFcStateSource), 시각(FakeClock)뿐이다.
// FC 의 실제 모드(fc_mode_)와 mavlink_gateway 가 받은 비행 상태를 구분한다: 비행 상태는 HEARTBEAT
// 주기 (1 Hz)로만 도착하므로 mavlink_gateway 는 FC 의 모드 변경을 최대 1초 늦게 안다
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/vehicle_api/types.hpp"
#include "fake_fc_state_source.hpp"
#include "fake_mavlink_connection.hpp"
#include "fc_command_rig.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::FlightMode;

constexpr std::int64_t kStartNs = 10'000'000'000;
constexpr std::int64_t kTickNs = 50'000'000;        // 20 Hz
constexpr std::int32_t kHeartbeatPeriodTicks = 20;  // 1 Hz
constexpr std::int64_t kSetpointValidNs = 200'000'000;
constexpr std::int64_t kPoseMaxAgeNs = 200'000'000;
constexpr std::int64_t kEntryConfirmNs = 3'000'000'000;
constexpr std::uint16_t kPositionAndYawMask = 2552;
constexpr std::uint16_t kVelocityAndYawRateMask = 1479;

class FcCommandScenarioTest : public ::testing::Test {
protected:
    void SetUp() override {
        subscription_ = rig_.subscribe_mode_events(
            [this](const vehicle_api::FlightModeEvent& event) { events_.push_back(event); });
        deliver_heartbeat();
        deliver_pose();
    }

    // FC 의 실제 모드를 바꾼다. mavlink_gateway 는 다음 HEARTBEAT 에서 알게 된다
    void set_fc_mode(FlightMode mode) {
        fc_mode_ = mode;
        if (mode == FlightMode::Offboard && clock_.now_monotonic_ns() <= entry_window_end_ns_) {
            fc_offboard_by_onboard_ = true;
        }
        if (mode != FlightMode::Offboard) {
            fc_offboard_by_onboard_ = false;
        }
    }

    void deliver_heartbeat() {
        fc_state_.set_status(fc_mode_, clock_.now_monotonic_ns());
        if (fc_mode_ == FlightMode::Offboard) {
            onboard_saw_offboard_ = true;
            fc_offboard_by_onboard_ = true;
        } else if (onboard_saw_offboard_) {
            // mavlink_gateway 는 FC 를 Offboard 에서 벗어나게 하지 않으므로 외부의 변경이다
            must_stay_silent_ = true;
        }
    }

    void deliver_pose() {
        const std::int64_t now_ns = clock_.now_monotonic_ns();
        vehicle_api::LocalPosition local;
        local.north_m = 5.0;
        local.received_monotonic_ns = now_ns;
        local.fc_epoch = fc_state_.epoch;
        fc_state_.state.local = local;
        vehicle_api::Attitude attitude;
        attitude.received_monotonic_ns = now_ns;
        attitude.fc_epoch = fc_state_.epoch;
        fc_state_.state.attitude = attitude;
    }

    void publish(double forward, std::int64_t valid_ns = kSetpointValidNs) {
        vehicle_api::VelocitySetpoint setpoint;
        setpoint.forward_m_s = forward;
        setpoint.valid_until_monotonic_ns = clock_.now_monotonic_ns() + valid_ns;
        rig_.publish(setpoint);
    }

    // 한 주기(50 ms)를 진행한다: FC 로부터의 수신 → 추종 제어의 발행 → mavlink_gateway 의 주기
    // 작업. follow_publishes 인 경우 추종 제어가 전진 속도 forward 의 setpoint 를 발행한 것으로
    // 처리한다. 반환값: 이 주기에 HEARTBEAT 가 도착했는지
    bool step(bool follow_publishes, double forward = 1.0) {
        clock_.advance_ns(kTickNs);
        ++tick_index_;
        const bool heartbeat = !fc_silent_ && !heartbeat_lost_ &&
                               tick_index_ % kHeartbeatPeriodTicks == heartbeat_phase_;
        if (heartbeat) {
            deliver_heartbeat();
        }
        if (!fc_silent_ && pose_valid_) {
            deliver_pose();
        }
        if (follow_publishes) {
            publish(forward);
        }
        const std::size_t sent_before = connection_.sent().size();
        const std::int32_t commands_before = mode_command_count();
        rig_.tick();
        if (mode_command_count() > commands_before) {
            entry_window_end_ns_ = clock_.now_monotonic_ns() + kEntryConfirmNs;
        }
        check_invariants(setpoints_since(sent_before));
        return heartbeat;
    }

    // 모든 주기에 확인하는 규칙
    // - FC 가 Offboard 일 수 있는 동안(진입 명령을 보낸 후 확인 시간까지, 또는 mavlink_gateway 가
    // 진입시킨
    //   FC 가 실제로 Offboard 인 동안) 주기마다 setpoint 가 정확히 하나 나간다
    // - FC 가 요청 없이 Offboard 를 벗어난 것을 안 후에는 해제 전까지 아무것도 나가지 않는다
    void check_invariants(std::int32_t setpoints_this_tick) {
        if (!check_invariants_) {
            return;
        }
        if (must_stay_silent_) {
            EXPECT_EQ(setpoints_this_tick, 0) << "Locked 에서 송신, 주기 " << tick_index_;
            return;
        }
        const bool may_be_offboard = clock_.now_monotonic_ns() <= entry_window_end_ns_ ||
                                     (fc_mode_ == FlightMode::Offboard && fc_offboard_by_onboard_);
        if (may_be_offboard) {
            EXPECT_EQ(setpoints_this_tick, 1) << "주기 " << tick_index_;
        }
    }

    void steps(std::int32_t count, bool follow_publishes, double forward = 1.0) {
        for (std::int32_t index = 0; index < count; ++index) {
            step(follow_publishes, forward);
        }
    }

    // 다음 HEARTBEAT 가 도착하는 주기까지 진행한다
    void run_until_heartbeat(bool follow_publishes, double forward = 1.0) {
        for (std::int32_t index = 0; index < kHeartbeatPeriodTicks; ++index) {
            if (step(follow_publishes, forward)) {
                return;
            }
        }
        FAIL() << "HEARTBEAT 가 도착하지 않음";
    }

    // FC 와의 연결이 다시 된 것으로 처리한다: fc_epoch 가 증가하고 이전 fc_epoch 의 값은 삭제된다.
    // 새 fc_epoch 의 값은 fc_silent_ 를 false 로 바꾼 후의 주기부터 도착한다
    void reconnect_fc() {
        ++fc_state_.epoch;
        fc_state_.state = {};
        fc_silent_ = true;
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

    vehicle_api::FlightModeReply request_offboard(const vehicle_api::RequestHeader& request) {
        const std::int32_t commands_before = mode_command_count();
        const vehicle_api::FlightModeReply reply = rig_.request_offboard(request);
        if (mode_command_count() > commands_before) {
            entry_window_end_ns_ = clock_.now_monotonic_ns() + kEntryConfirmNs;
        }
        return reply;
    }

    void release_lock() {
        ASSERT_TRUE(rig_.release_lock());
        must_stay_silent_ = false;
        onboard_saw_offboard_ = false;
    }

    // FC 가 모드 전환 명령에 응답한 것으로 처리한다
    void fc_answers_mode_command(std::uint8_t result = MAV_RESULT_ACCEPTED) {
        mavlink_message_t message{};
        mavlink_msg_command_ack_pack(1, 1, &message, MAV_CMD_DO_SET_MODE, result, 0, 0, 1,
                                     MAV_COMP_ID_PATHPLANNER);
        connection_.simulate_message_received(message);
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    std::int32_t setpoints_since(std::size_t first) const {
        std::int32_t count = 0;
        for (std::size_t index = first; index < connection_.sent().size(); ++index) {
            count += connection_.sent()[index].msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED
                         ? 1
                         : 0;
        }
        return count;
    }

    // 이 시점까지 FC 로 송신한 메시지 수를 기억하고, 이후 송신한 setpoint 를 확인하는 데 쓴다
    void mark() { mark_ = connection_.sent().size(); }
    std::int32_t setpoints_since_mark() const { return setpoints_since(mark_); }

    // 마지막으로 송신한 setpoint. 없는 경우 type_mask 0
    mavlink_set_position_target_local_ned_t last_setpoint() const {
        mavlink_set_position_target_local_ned_t target{};
        for (auto it = connection_.sent().rbegin(); it != connection_.sent().rend(); ++it) {
            if (it->msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) {
                mavlink_msg_set_position_target_local_ned_decode(&*it, &target);
                break;
            }
        }
        return target;
    }

    bool last_is_zero_velocity() const {
        const mavlink_set_position_target_local_ned_t target = last_setpoint();
        return target.type_mask == kVelocityAndYawRateMask && target.vx == 0.0F &&
               target.vy == 0.0F && target.vz == 0.0F && target.yaw_rate == 0.0F;
    }

    std::int32_t mode_command_count() const {
        std::int32_t count = 0;
        for (const mavlink_message_t& message : connection_.sent()) {
            count += message.msgid == MAVLINK_MSG_ID_COMMAND_LONG ? 1 : 0;
        }
        return count;
    }

    std::int32_t offboard_event_count() const {
        std::int32_t count = 0;
        for (const vehicle_api::FlightModeEvent& event : events_) {
            count += event.mode == FlightMode::Offboard ? 1 : 0;
        }
        return count;
    }

    // 추종 제어가 1.1초 동안 setpoint 를 발행한 후 Offboard 진입을 요청한다
    void request_entry(std::uint64_t command_id = 1) {
        steps(22, true);
        ASSERT_TRUE(request_offboard(header(command_id)).accepted);
    }

    // FC 가 Offboard 로 바뀌고 mavlink_gateway 가 그것을 확인하여 추종 중인 상태까지 진행한다
    void start_following(std::uint64_t command_id = 1) {
        request_entry(command_id);
        fc_answers_mode_command();
        set_fc_mode(FlightMode::Offboard);
        run_until_heartbeat(true);
        step(true);
        ASSERT_EQ(rig_.owner(), "follow");
        ASSERT_EQ(last_setpoint().type_mask, kVelocityAndYawRateMask);
    }

    // 추종 중에 정지를 요청하여 제자리 유지 중인 상태까지 진행한다
    void start_hovering() {
        start_following();
        ASSERT_TRUE(rig_.request_hold(header(2)).accepted);
        steps(2, false);
        ASSERT_TRUE(rig_.holding_position());
    }

    core_testing::FakeClock clock_{kStartNs};
    core_testing::FakePoller poller_{clock_};
    FakeMavlinkConnection connection_{poller_};
    FakeFcStateSource fc_state_;
    FcCommandRig rig_{connection_, fc_state_, clock_};

    std::vector<vehicle_api::FlightModeEvent> events_;
    core::Subscription subscription_;

    FlightMode fc_mode_ = FlightMode::Hold;  // FC 의 실제 모드
    std::int32_t heartbeat_phase_ = 0;       // HEARTBEAT 가 도착하는 주기 (20주기 중 몇 번째)
    bool pose_valid_ = true;
    bool fc_silent_ = false;       // FC 로부터 아무것도 도착하지 않음
    bool heartbeat_lost_ = false;  // HEARTBEAT 만 도착하지 않음
    bool check_invariants_ = true;

    std::int32_t tick_index_ = 0;
    std::size_t mark_ = 0;
    std::int64_t entry_window_end_ns_ = 0;  // 마지막 진입 명령의 확인 시간이 끝나는 시각
    bool fc_offboard_by_onboard_ = false;   // FC 가 mavlink_gateway 의 진입 명령 후 Offboard 가 됨
    bool onboard_saw_offboard_ =
        false;  // Offboard 인 HEARTBEAT 가 mavlink_gateway 에 도착한 적 있음
    bool must_stay_silent_ = false;
};

TEST_F(FcCommandScenarioTest, SetpointsReachFcBeforeOffboardSoThatEntryIsAccepted) {
    // Offboard 진입 전: 송신권이 없어도 setpoint 가 FC 로 송신된다
    steps(10, true);
    EXPECT_EQ(setpoints_since_mark(), 10);
    EXPECT_TRUE(rig_.owner().empty());

    // 0.5초만 송신한 시점의 진입 요청은 거부된다
    EXPECT_EQ(request_offboard(header(1)).reason, "prestream not ready");
    EXPECT_EQ(mode_command_count(), 0);

    // 1초 이상 송신한 후에는 수락되고 모드 전환 명령이 송신된다
    steps(12, true);
    EXPECT_TRUE(request_offboard(header(2)).accepted);
    EXPECT_EQ(mode_command_count(), 1);
}

TEST_F(FcCommandScenarioTest, FollowThenStopThenFollowAgain) {
    start_following();
    EXPECT_FLOAT_EQ(last_setpoint().vx, 1.0F);

    // 정지: 위치 setpoint 로 바뀌고, 추종 제어가 계속 발행하는 속도 setpoint 는 FC 로 가지 않는다
    ASSERT_TRUE(rig_.request_hold(header(2)).accepted);
    mark();
    steps(5, true, 3.0);
    EXPECT_TRUE(rig_.holding_position());
    EXPECT_EQ(setpoints_since_mark(), 5);
    EXPECT_EQ(last_setpoint().type_mask, kPositionAndYawMask);
    EXPECT_FLOAT_EQ(last_setpoint().x, 5.0F);

    // 다시 추종: FC 가 이미 Offboard 이므로 모드 전환 명령 없이 수락되고 속도 setpoint 로 바뀐다
    const std::int32_t commands_before = mode_command_count();
    ASSERT_TRUE(request_offboard(header(3)).accepted);
    steps(2, true, 2.0);
    EXPECT_EQ(mode_command_count(), commands_before);
    EXPECT_FALSE(rig_.stopping());
    EXPECT_EQ(last_setpoint().type_mask, kVelocityAndYawRateMask);
    EXPECT_FLOAT_EQ(last_setpoint().vx, 2.0F);
}

// HEARTBEAT 가 도착하는 주기(20주기 중 몇 번째)와, FC 의 응답이 HEARTBEAT 보다 먼저인지
struct EntryTiming {
    std::int32_t heartbeat_phase;
    bool ack_first;
};

class FcCommandEntryTimingTest : public FcCommandScenarioTest,
                                 public ::testing::WithParamInterface<EntryTiming> {};

TEST_P(FcCommandEntryTimingTest, SetpointsFlowEveryTickThroughEntryAndStop) {
    // 진입, 추종, 정지를 거치는 동안 주기마다 setpoint 가 하나씩 나간다 (step() 의 규칙 확인)
    heartbeat_phase_ = GetParam().heartbeat_phase;
    request_entry();
    if (GetParam().ack_first) {
        fc_answers_mode_command();
    }
    set_fc_mode(FlightMode::Offboard);
    run_until_heartbeat(true);
    if (!GetParam().ack_first) {
        fc_answers_mode_command();
    }
    steps(2, true);
    EXPECT_EQ(rig_.owner(), "follow");
    EXPECT_EQ(last_setpoint().type_mask, kVelocityAndYawRateMask);
    EXPECT_EQ(offboard_event_count(), 1);

    ASSERT_TRUE(rig_.request_hold(header(2)).accepted);
    steps(3, false);
    EXPECT_TRUE(rig_.holding_position());
}

INSTANTIATE_TEST_SUITE_P(HeartbeatPhases, FcCommandEntryTimingTest,
                         ::testing::Values(EntryTiming{0, true}, EntryTiming{0, false},
                                           EntryTiming{2, true}, EntryTiming{3, false},
                                           EntryTiming{7, true}, EntryTiming{11, false},
                                           EntryTiming{19, true}, EntryTiming{19, false}));

TEST_F(FcCommandScenarioTest, FollowControlStopsPublishingWhileFollowing) {
    start_following();

    // 추종 제어가 발행을 멈춘 경우: 유효 시간(0.2초) 동안 마지막 값을 송신한 후 위치 유지로 바뀐다.
    // 그 사이 setpoint 가 송신되지 않는 주기가 없다 (step() 의 규칙)
    steps(8, false);
    EXPECT_TRUE(rig_.holding_position());
    EXPECT_TRUE(rig_.owner().empty());
    EXPECT_EQ(last_setpoint().type_mask, kPositionAndYawMask);
}

TEST_F(FcCommandScenarioTest, StopWithoutValidPoseSendsZeroVelocityInsteadOfLastSetpoint) {
    start_following();
    pose_valid_ = false;
    steps(static_cast<std::int32_t>(kPoseMaxAgeNs / kTickNs) + 1, true);

    ASSERT_TRUE(rig_.request_hold(header(2)).accepted);
    steps(5, true, 4.0);

    // 추종 제어가 계속 발행하여도 마지막 속도로 비행하지 않는다
    EXPECT_TRUE(rig_.stopping());
    EXPECT_FALSE(rig_.holding_position());
    EXPECT_TRUE(last_is_zero_velocity());

    // 위치 값이 다시 들어오면 위치 유지로 바뀐다
    pose_valid_ = true;
    step(true, 4.0);
    EXPECT_TRUE(rig_.holding_position());
}

TEST_F(FcCommandScenarioTest, StopRightAfterOffboardRequestKeepsSetpointsFlowing) {
    request_entry();
    // FC 가 Offboard 로 바뀌기 전에 정지 요청
    ASSERT_TRUE(rig_.request_hold(header(2)).accepted);

    // FC 가 Offboard 로 바뀌는 전후로 주기마다 속도 0 이 송신된다.
    // 추종 제어는 정지 요청 후에도 한동안 발행을 계속한다고 가정한다
    for (std::int32_t index = 0; index < 4; ++index) {
        step(true, 3.0);
        EXPECT_TRUE(last_is_zero_velocity());
    }
    set_fc_mode(FlightMode::Offboard);
    run_until_heartbeat(true, 3.0);
    step(true, 3.0);
    EXPECT_TRUE(rig_.holding_position());
    EXPECT_TRUE(rig_.owner().empty());
}

TEST_F(FcCommandScenarioTest, PilotTakeoverStopsAllSendingUntilCleared) {
    start_following();

    set_fc_mode(FlightMode::Position);
    run_until_heartbeat(true);
    mark();
    steps(30, true);  // 추종 제어가 계속 발행하고, 그 뒤 발행을 멈춘 경우까지
    steps(30, false);

    EXPECT_EQ(setpoints_since_mark(), 0);
    EXPECT_TRUE(rig_.locked());
    EXPECT_EQ(request_offboard(header(5)).reason, "external mode change");
    EXPECT_FALSE(rig_.stop_from_onboard());

    // 해제 후에도 FC 로 아무것도 송신되지 않는다
    release_lock();
    mark();
    steps(5, false);
    EXPECT_EQ(setpoints_since_mark(), 0);
    EXPECT_FALSE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, PilotTakeoverIsNoticedAtNextHeartbeat) {
    start_following();

    // FC 가 Offboard 를 벗어난 후 다음 HEARTBEAT 까지(최대 1초) mavlink_gateway 는 알지 못하고 속도
    // setpoint 를 계속 보낸다. PX4 는 Offboard 가 아닌 동안 받은 setpoint 를 비행에 쓰지 않는다
    set_fc_mode(FlightMode::Position);
    mark();
    std::int32_t ticks = 0;
    while (!rig_.locked() && ticks < kHeartbeatPeriodTicks) {
        step(true);
        ++ticks;
    }
    EXPECT_TRUE(rig_.locked());
    EXPECT_EQ(setpoints_since_mark(), ticks - 1);
    ASSERT_FALSE(events_.empty());
    EXPECT_EQ(events_.back().exit_cause, vehicle_api::OffboardExitCause::PilotSwitch);
}

TEST_F(FcCommandScenarioTest, SecondSessionCannotTakeOverWhileFirstOwns) {
    start_following();

    EXPECT_EQ(request_offboard(header(9, "other")).reason, "another writer owns FC");
    EXPECT_EQ(rig_.request_hold(header(10, "other")).reason, "another writer owns FC");
    EXPECT_EQ(rig_.owner(), "follow");
}

TEST_F(FcCommandScenarioTest, HoverKeepsSendingWhileFcLinkIsReportedLost) {
    start_hovering();

    // 정지 setpoint 를 멈추면 PX4 가 failsafe 로 넘어가므로, 연결 두절로 판단한 동안에도 송신한다
    fc_state_.connected = false;
    fc_silent_ = true;
    mark();
    steps(60, false);
    EXPECT_EQ(setpoints_since_mark(), 60);
    EXPECT_EQ(last_setpoint().type_mask, kPositionAndYawMask);
}

TEST_F(FcCommandScenarioTest, FollowingStopsAndHoverTakesOverWhenFcLinkIsReportedLost) {
    start_following();

    // 추종 제어의 속도 setpoint 는 연결 두절로 판단한 동안 송신하지 않고, 제자리 유지로 바뀐다
    fc_state_.connected = false;
    steps(8, true, 3.0);

    EXPECT_TRUE(rig_.stopping());
    EXPECT_TRUE(rig_.owner().empty());
    EXPECT_EQ(last_setpoint().type_mask, kPositionAndYawMask);
}

TEST_F(FcCommandScenarioTest, FollowingStopsWhenOnlyHeartbeatIsLostForTwoSeconds) {
    start_following();

    // 자세와 위치는 도착하지만 HEARTBEAT 만 2초 넘게 도착하지 않는 경우 (icd.md 6.2)
    heartbeat_lost_ = true;
    steps(45, true, 3.0);

    EXPECT_TRUE(rig_.holding_position());
    EXPECT_TRUE(rig_.owner().empty());

    // 다시 도착한 HEARTBEAT 가 Offboard 이면 제자리 유지를 이어 간다
    heartbeat_lost_ = false;
    run_until_heartbeat(true, 3.0);
    step(true, 3.0);
    EXPECT_TRUE(rig_.holding_position());
    EXPECT_FALSE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, NoGapAtOffboardEntryWhenFollowControlPublishesSlowly) {
    // 추종 제어가 5 Hz 로 발행하고 유효 시간이 0.5초인 경우
    for (std::int32_t index = 0; index < 24; ++index) {
        if (index % 4 == 0) {
            publish(1.0, 500'000'000);
        }
        step(false);
    }
    ASSERT_TRUE(request_offboard(header(1)).accepted);
    fc_answers_mode_command();
    set_fc_mode(FlightMode::Offboard);

    // 진입 확인 전후로 주기마다 속도 setpoint 가 송신된다
    for (std::int32_t index = 0; index < 30; ++index) {
        if (index % 4 == 3) {
            publish(1.0, 500'000'000);
        }
        step(false);
        EXPECT_EQ(last_setpoint().type_mask, kVelocityAndYawRateMask) << "주기 " << index;
        EXPECT_FLOAT_EQ(last_setpoint().vx, 1.0F) << "주기 " << index;
    }
    EXPECT_EQ(rig_.owner(), "follow");
    EXPECT_FALSE(rig_.stopping());
}

TEST_F(FcCommandScenarioTest, FcEpochChangeWhileFcStaysInOffboardKeepsSetpointsFlowing) {
    start_following();
    const vehicle_api::RequestHeader old_request = header(4);

    // FC 로부터의 수신만 끊겼다 회복된 경우. FC 는 계속 Offboard 이다.
    // 새 fc_epoch 의 비행 상태를 받기 전: 송신권은 폐기되고, 추종 제어의 속도는 나가지 않는다
    reconnect_fc();
    for (std::int32_t index = 0; index < 10; ++index) {
        step(true, 3.0);
        EXPECT_NE(last_setpoint().vx, 3.0F) << "주기 " << index;
    }
    EXPECT_TRUE(rig_.owner().empty());
    EXPECT_TRUE(rig_.stopping());
    EXPECT_FALSE(rig_.locked());
    EXPECT_EQ(rig_.request_hold(old_request).reason, "fc_epoch mismatch");

    // 새 fc_epoch 의 첫 비행 상태가 Offboard: 제자리 유지를 이어 가고 Locked 로 전환하지 않는다.
    // 추종 제어가 계속 발행하는 속도는 다시 진입을 요청하기 전에는 송신되지 않는다
    fc_silent_ = false;
    run_until_heartbeat(true, 3.0);
    steps(3, true, 3.0);
    EXPECT_TRUE(rig_.holding_position());
    EXPECT_EQ(last_setpoint().type_mask, kPositionAndYawMask);
    EXPECT_FALSE(rig_.locked());
    EXPECT_TRUE(rig_.owner().empty());

    // 새 fc_epoch 의 요청으로 다시 추종할 수 있다
    ASSERT_TRUE(request_offboard(header(5)).accepted);
    steps(2, true, 2.0);
    EXPECT_FALSE(rig_.stopping());
    EXPECT_FLOAT_EQ(last_setpoint().vx, 2.0F);
}

TEST_F(FcCommandScenarioTest, FcEpochChangeEndsSendingWhenFcIsNoLongerInOffboard) {
    start_following();

    reconnect_fc();
    steps(10, true, 3.0);
    ASSERT_TRUE(rig_.stopping());
    ASSERT_FALSE(rig_.locked());

    // 새 fc_epoch 의 첫 비행 상태가 Offboard 가 아님 (FC 재부팅, 두절 중의 failsafe 등)
    fc_silent_ = false;
    set_fc_mode(FlightMode::Hold);
    run_until_heartbeat(true, 3.0);
    mark();
    steps(30, true, 3.0);
    steps(30, false);

    EXPECT_EQ(setpoints_since_mark(), 0);
    EXPECT_FALSE(rig_.stopping());
    EXPECT_TRUE(rig_.locked());
    EXPECT_EQ(events_.back().exit_cause, vehicle_api::OffboardExitCause::Failsafe);
    EXPECT_EQ(request_offboard(header(5)).reason, "external mode change");
}

TEST_F(FcCommandScenarioTest, FcEpochChangeDuringHoverKeepsPositionTarget) {
    start_hovering();

    reconnect_fc();
    for (std::int32_t index = 0; index < 10; ++index) {
        step(false);
        EXPECT_EQ(last_setpoint().type_mask, kPositionAndYawMask) << "주기 " << index;
    }
    fc_silent_ = false;
    run_until_heartbeat(false);
    steps(3, false);

    EXPECT_TRUE(rig_.holding_position());
    EXPECT_FLOAT_EQ(last_setpoint().x, 5.0F);
    EXPECT_FALSE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, StopDuringPendingEntryThenFcEpochChangeIsNotLocked) {
    request_entry();
    ASSERT_TRUE(rig_.request_hold(header(2)).accepted);
    step(false);
    ASSERT_TRUE(last_is_zero_velocity());

    // 속도 0 송신은 새 fc_epoch 의 비행 상태를 받을 때까지 계속된다
    reconnect_fc();
    mark();
    steps(4, false);
    EXPECT_EQ(setpoints_since_mark(), 4);

    // FC 는 진입하지 않았음: 확인 시간 후의 비행 상태로 확인한 때에 끝내며, Locked 로 전환하지
    // 않는다
    fc_silent_ = false;
    steps(80, false);
    EXPECT_FALSE(rig_.stopping());
    EXPECT_FALSE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, HoldRequestDeniedDuringPendingEntryStillSendsZeroVelocity) {
    request_entry();

    // 송신권을 가진 세션의 정지 요청이 유효 시간 초과로 거부되어도 진입을 취소하고 속도 0 을
    // 보낸다. FC 가 Offboard 로 바뀌는 시점에 setpoint 가 도착하고 있어야 한다
    vehicle_api::RequestHeader expired = header(2);
    expired.valid_until_monotonic_ns = clock_.now_monotonic_ns() - 1;
    EXPECT_FALSE(rig_.request_hold(expired).accepted);

    set_fc_mode(FlightMode::Offboard);
    steps(6, false);  // 추종 제어는 정지를 요청했으므로 발행하지 않는다 (step() 의 규칙 확인)
    EXPECT_TRUE(last_is_zero_velocity());
    EXPECT_TRUE(rig_.owner().empty());
    run_until_heartbeat(false);
    step(false);
    EXPECT_TRUE(rig_.holding_position());
}

TEST_F(FcCommandScenarioTest, FcEpochChangeDuringPendingEntryIsNotLockedByHeartbeatBeforeDeadline) {
    request_entry();

    // 진입 명령을 보낸 직후 FC 연결이 다시 됨. 새 fc_epoch 의 첫 비행 상태는 FC 가 아직 전환하지
    // 않은 때의 값일 수 있으므로 그것만으로 Locked 로 전환하지 않는다
    reconnect_fc();
    step(false);
    fc_silent_ = false;
    run_until_heartbeat(false);
    EXPECT_FALSE(rig_.locked());
    EXPECT_TRUE(last_is_zero_velocity());

    // 확인 시간 안에 FC 가 Offboard 로 바뀐 경우 제자리 유지로 이어진다
    set_fc_mode(FlightMode::Offboard);
    run_until_heartbeat(false);
    step(false);
    EXPECT_TRUE(rig_.holding_position());
    EXPECT_FALSE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, EntryConfirmTimeoutSendsZeroVelocityUntilFcModeIsKnown) {
    request_entry();
    // FC 가 응답하지 않고, 확인 시간이 지난 직후 Offboard 로 바뀌는 경우
    steps(static_cast<std::int32_t>(kEntryConfirmNs / kTickNs) + 1, true, 3.0);
    EXPECT_TRUE(rig_.owner().empty());
    EXPECT_TRUE(last_is_zero_velocity());

    // 확인 시간이 지난 후에도, FC 가 Offboard 가 아님을 확인하기 전에는 주기마다 속도 0 을 보낸다
    mark();
    steps(5, true, 3.0);
    EXPECT_EQ(setpoints_since_mark(), 5);
    EXPECT_TRUE(last_is_zero_velocity());
}

TEST_F(FcCommandScenarioTest, RejectedEntryCommandSendsZeroVelocityUntilFcModeIsKnown) {
    request_entry();
    fc_answers_mode_command(MAV_RESULT_TEMPORARILY_REJECTED);

    mark();
    steps(5, true, 3.0);
    EXPECT_TRUE(rig_.owner().empty());
    EXPECT_EQ(setpoints_since_mark(), 5);
    EXPECT_TRUE(last_is_zero_velocity());

    // 확인 시간 후의 비행 상태가 Offboard 가 아니면 끝내고, Locked 로 전환하지 않는다
    steps(80, false);
    EXPECT_FALSE(rig_.stopping());
    EXPECT_FALSE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, LostHeartbeatAfterEntryCommandFallsBackToZeroVelocity) {
    request_entry();
    fc_answers_mode_command();
    // FC 는 Offboard 로 바뀌었지만 HEARTBEAT 만 도착하지 않는다 (자세와 위치는 수신)
    heartbeat_lost_ = true;
    set_fc_mode(FlightMode::Offboard);

    // 확인 시간(3초)이 지난 후까지: 비행 상태가 오래된 후에는 추종 제어의 속도 대신 속도 0 이
    // 송신된다. setpoint 가 송신되지 않는 주기는 없다 (step() 의 규칙)
    steps(80, true, 3.0);
    EXPECT_TRUE(rig_.stopping());
    EXPECT_TRUE(rig_.owner().empty());
    EXPECT_TRUE(last_is_zero_velocity());

    // HEARTBEAT 가 다시 도착하여 Offboard 임을 확인한 경우 위치 유지로 바뀐다
    heartbeat_lost_ = false;
    run_until_heartbeat(true, 3.0);
    step(true, 3.0);
    EXPECT_TRUE(rig_.holding_position());
    EXPECT_FALSE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, LostHeartbeatAfterEntryCommandEndsWhenFcTurnsOutNotInOffboard) {
    request_entry();
    heartbeat_lost_ = true;
    steps(80, true, 3.0);
    ASSERT_TRUE(rig_.stopping());

    // FC 는 진입하지 않았음: 최신 비행 상태로 확인한 때에 속도 0 송신을 끝낸다
    heartbeat_lost_ = false;
    run_until_heartbeat(false);
    step(false);
    EXPECT_FALSE(rig_.stopping());
    mark();
    steps(3, false);
    EXPECT_EQ(setpoints_since_mark(), 0);
    EXPECT_FALSE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, HoldRequestRightAfterPilotTakeoverSendsNothing) {
    start_following();

    // 조종사가 넘겨받은 것을 알리는 HEARTBEAT 가 도착한 후, 주기 작업보다 먼저 정지 요청이 도착
    set_fc_mode(FlightMode::Position);
    clock_.advance_ns(kTickNs);
    deliver_heartbeat();
    mark();
    const vehicle_api::FlightModeReply reply = rig_.request_hold(header(2));

    EXPECT_FALSE(reply.accepted);
    EXPECT_EQ(reply.reason, "external mode change");
    steps(5, true);
    EXPECT_EQ(setpoints_since_mark(), 0);
    EXPECT_TRUE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, OffboardEntryIsReportedOnceWhenModeChangeIsSeenBeforeAck) {
    request_entry(7);
    set_fc_mode(FlightMode::Offboard);
    run_until_heartbeat(true);
    step(true);
    EXPECT_EQ(offboard_event_count(), 0);

    fc_answers_mode_command();
    steps(3, true);

    ASSERT_EQ(offboard_event_count(), 1);
    EXPECT_EQ(events_.back().request_command_id.value_or(vehicle_api::CommandId{}).value, 7U);
    EXPECT_EQ(rig_.owner(), "follow");
}

TEST_F(FcCommandScenarioTest, PilotTakeoverWhileHoveringSendsNothing) {
    start_hovering();

    set_fc_mode(FlightMode::Position);
    run_until_heartbeat(true);
    mark();
    steps(10, true);

    EXPECT_EQ(setpoints_since_mark(), 0);
    EXPECT_TRUE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, NanSetpointsWhileFollowingLeadToHoverWithoutGap) {
    start_following();

    // 추종 제어가 NaN 을 계속 발행하는 경우: 마지막 정상 값의 유효 시간 후 위치 유지로 바뀐다
    for (std::int32_t index = 0; index < 10; ++index) {
        step(true, std::nan(""));
        EXPECT_FALSE(std::isnan(last_setpoint().vx));
    }
    EXPECT_TRUE(rig_.holding_position());
    EXPECT_TRUE(rig_.owner().empty());
}

TEST_F(FcCommandScenarioTest, OwnerThatStopsPublishingDuringEntryIsReplacedByOnboard) {
    request_entry();
    fc_answers_mode_command();
    set_fc_mode(FlightMode::Offboard);

    // 진입 요청 후 추종 제어가 setpoint 를 하나도 발행하지 않는 경우
    run_until_heartbeat(false);
    steps(2, false);

    EXPECT_TRUE(rig_.holding_position());
    EXPECT_TRUE(rig_.owner().empty());
}

TEST_F(FcCommandScenarioTest, RestartFromHoverWithoutSetpointsKeepsHovering) {
    start_hovering();

    // 다시 추종을 요청했지만 추종 제어가 setpoint 를 발행하지 않는 경우: 위치 유지가 계속된다
    ASSERT_TRUE(request_offboard(header(3)).accepted);
    for (std::int32_t index = 0; index < 6; ++index) {
        step(false);
        EXPECT_EQ(last_setpoint().type_mask, kPositionAndYawMask);
        EXPECT_FLOAT_EQ(last_setpoint().x, 5.0F);
    }
}

TEST_F(FcCommandScenarioTest, OnboardStartingWhileFcIsAlreadyInOffboardBeginsHover) {
    // mavlink_gateway 프로세스가 비행 중 다시 시작한 경우: 처음 받은 HEARTBEAT 의 모드가 Offboard
    set_fc_mode(FlightMode::Offboard);
    run_until_heartbeat(false);
    mark();
    steps(3, false);

    EXPECT_TRUE(rig_.holding_position());
    EXPECT_EQ(setpoints_since_mark(), 3);
}

TEST_F(FcCommandScenarioTest, FailsafeLandStopsAllSending) {
    start_following();

    set_fc_mode(FlightMode::Land);
    run_until_heartbeat(true);
    mark();
    steps(10, true);

    EXPECT_EQ(setpoints_since_mark(), 0);
    EXPECT_TRUE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, StopDuringPendingEntryThatNeverHappensEndsZeroVelocityStream) {
    request_entry();
    ASSERT_TRUE(rig_.request_hold(header(2)).accepted);

    // FC 가 끝내 Offboard 로 바뀌지 않는 경우: 확인 시간(3초) 후의 HEARTBEAT 로 확인하고 끝낸다
    steps(81, false);
    EXPECT_FALSE(rig_.stopping());
    mark();
    steps(3, false);
    EXPECT_EQ(setpoints_since_mark(), 0);
    EXPECT_FALSE(rig_.locked());
}

TEST_F(FcCommandScenarioTest, SetpointLeftAfterStopIsNotSentWithAuthorityOfAnotherSession) {
    start_following();
    ASSERT_TRUE(rig_.request_hold(header(2)).accepted);
    // 정지 직후 도착한 값 (유효 시간 1초). 이 시점에는 송신권을 가진 세션이 없다
    publish(4.0, 1'000'000'000);

    // 다른 세션이 송신권을 획득한다 (FC 가 이미 Offboard 이므로 바로 수락)
    ASSERT_TRUE(request_offboard(header(3, "other")).accepted);

    // 남은 값은 새 송신권으로 송신되지 않고, 그 값의 유효 시간이 지나도 송신권은 유지된다
    for (std::int32_t index = 0; index < 30; ++index) {
        step(false);
        EXPECT_EQ(last_setpoint().type_mask, kPositionAndYawMask) << "주기 " << index;
    }
    EXPECT_EQ(rig_.owner(), "other");

    // 새 setpoint 가 오면 속도를 송신한다
    step(true, 2.0);
    EXPECT_FLOAT_EQ(last_setpoint().vx, 2.0F);
    EXPECT_FALSE(rig_.stopping());
}

TEST_F(FcCommandScenarioTest, FailedVelocitySendAfterHoverKeepsAuthority) {
    start_hovering();
    ASSERT_TRUE(request_offboard(header(3)).accepted);

    // 제자리 유지에서 속도로 넘어가는 첫 송신이 실패한 경우
    check_invariants_ = false;
    connection_.set_send_result(false);
    step(true, 2.0);
    EXPECT_EQ(rig_.owner(), "follow");

    connection_.set_send_result(true);
    step(true, 2.0);
    EXPECT_EQ(rig_.owner(), "follow");
    EXPECT_FLOAT_EQ(last_setpoint().vx, 2.0F);
}

TEST_F(FcCommandScenarioTest, StopFromOnboardWhileFollowingStartsHover) {
    start_following();

    // 통제기의 정지 명령이나 통제기 두절로 mavlink_gateway 가 스스로 정지하는 경우
    EXPECT_TRUE(rig_.stop_from_onboard());
    steps(3, true, 3.0);

    EXPECT_TRUE(rig_.holding_position());
    EXPECT_TRUE(rig_.owner().empty());
    EXPECT_EQ(last_setpoint().type_mask, kPositionAndYawMask);
}

}  // namespace
}  // namespace argus::mavlink_gateway
