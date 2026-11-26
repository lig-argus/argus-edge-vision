// fc_command_policy 의 전환표 테스트. 행 하나가 (상태, 입력) 하나와 기대하는 결과이다
#include "argus/mavlink_gateway/fc_command_policy.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "argus/mavlink_gateway/fc_command_state.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::FlightMode;

constexpr std::int64_t kNowNs = 100'000'000'000;
constexpr std::int64_t kDeadlineNs = kNowNs + 1'000'000'000;  // 아직 지나지 않은 진입 기한
constexpr std::uint64_t kEpoch = 3;
const FcCommandPolicyConfig kConfig;

// 상태의 번호. FcCommandState 의 순서와 같다
constexpr std::size_t kIdle = 0;
constexpr std::size_t kEntering = 1;
constexpr std::size_t kFollowing = 2;
constexpr std::size_t kCancellingEntry = 3;
constexpr std::size_t kHovering = 4;
constexpr std::size_t kLocked = 5;
constexpr std::size_t kLanding = 6;
constexpr std::size_t kHoverEntering = 7;

const char* name_of(std::size_t index) {
    static const std::array<const char*, 8> kNames = {
        "Idle",     "Entering", "Following", "CancellingEntry",
        "Hovering", "Locked",   "Landing",   "HoverEntering"};
    return kNames.at(index);
}

using Change = std::function<void(FcObservation&)>;

// 기본 입력: FC 연결됨, 0.1초 전에 받은 비행 상태(mode), 위치 최신, 유효한 setpoint 를 방금 받음,
// setpoint 를 1.5초 동안 송신 중
FcObservation observed(FlightMode mode, const Change& change = nullptr) {
    FcObservation observation;
    observation.now_ns = kNowNs;
    observation.fc_epoch = kEpoch;
    observation.fc_connected = true;
    vehicle_api::FlightStatus status;
    status.mode = mode;
    status.received_monotonic_ns = kNowNs - 100'000'000;
    status.fc_epoch = kEpoch;
    observation.status = status;
    observation.fresh_pose = HoverTarget{1.0, 2.0, -3.0, 0.5};
    observation.setpoint_received_ns = kNowNs - 10'000'000;
    observation.streaming_for_ns = 1'500'000'000;
    if (change) {
        change(observation);
    }
    return observation;
}

void no_status(FcObservation& observation) { observation.status.reset(); }
// 비행 상태를 받은 시각을 바꾼다
void set_status_received(FcObservation& observation, std::int64_t received_ns) {
    vehicle_api::FlightStatus status = observation.status.value_or(vehicle_api::FlightStatus{});
    status.received_monotonic_ns = received_ns;
    observation.status = status;
}

void stale_status(FcObservation& observation) {
    set_status_received(observation, kNowNs - kConfig.status_max_age_ns - 1);
}
void disconnected(FcObservation& observation) { observation.fc_connected = false; }
void new_epoch(FcObservation& observation) {
    observation.fc_epoch = kEpoch + 1;
    observation.status.reset();
}
void no_setpoint(FcObservation& observation) { observation.setpoint_received_ns.reset(); }
void no_pose(FcObservation& observation) { observation.fresh_pose.reset(); }
void after_deadline(FcObservation& observation) {
    observation.now_ns = kDeadlineNs + 1;
    observation.setpoint_received_ns = observation.now_ns;
}
void status_after_deadline(FcObservation& observation) {
    observation.now_ns = kDeadlineNs + 200'000'000;
    set_status_received(observation, kDeadlineNs + 100'000'000);
}

Entering entering(bool acknowledged = false) {
    Entering state;
    state.authority_session = "follow";
    state.command_id = vehicle_api::CommandId{7};
    state.fc_epoch = kEpoch;
    state.deadline_ns = kDeadlineNs;
    state.acknowledged = acknowledged;
    return state;
}

Following following() {
    Following state;
    state.authority_session = "follow";
    state.fc_epoch = kEpoch;
    return state;
}

// 제자리 유지 중에 since_ns 에 수락되어 첫 setpoint 를 기다리는 상태
Following awaiting(std::int64_t since_ns) {
    Following state = following();
    state.values_after_ns = since_ns;
    state.awaiting_first_value = true;
    state.hover_target = HoverTarget{9.0, 8.0, -7.0, 0.1};
    return state;
}

vehicle_api::RequestHeader request(const std::string& session = "follow",
                                   std::uint64_t command_id = 11) {
    vehicle_api::RequestHeader header;
    header.id.session_id = session;
    header.id.command_id = vehicle_api::CommandId{command_id};
    header.id.fc_epoch = kEpoch;
    header.valid_until_monotonic_ns = kNowNs + 500'000'000;
    return header;
}

// ---- FC 의 모드 판단

TEST(FcModeBeliefTest, FollowsFreshStatusOfCurrentFcEpoch) {
    EXPECT_EQ(fc_mode_belief(observed(FlightMode::Offboard), kConfig), FcModeBelief::Offboard);
    EXPECT_EQ(fc_mode_belief(observed(FlightMode::Position), kConfig), FcModeBelief::NotOffboard);
}

TEST(FcModeBeliefTest, IsPendingWhileNoStatusOfCurrentFcEpochHasArrived) {
    EXPECT_EQ(fc_mode_belief(observed(FlightMode::Offboard, no_status), kConfig),
              FcModeBelief::Pending);
    const auto status_of_earlier_epoch = [](FcObservation& observation) {
        observation.fc_epoch = kEpoch + 1;
    };
    EXPECT_EQ(fc_mode_belief(observed(FlightMode::Offboard, status_of_earlier_epoch), kConfig),
              FcModeBelief::Pending);
}

TEST(FcModeBeliefTest, IsNotReceivedWhenDisconnectedOrStatusIsOlderThanLimit) {
    EXPECT_EQ(fc_mode_belief(observed(FlightMode::Offboard, disconnected), kConfig),
              FcModeBelief::NotReceived);
    EXPECT_EQ(fc_mode_belief(observed(FlightMode::Offboard, stale_status), kConfig),
              FcModeBelief::NotReceived);
    const auto at_age_limit = [](FcObservation& observation) {
        set_status_received(observation, kNowNs - kConfig.status_max_age_ns);
    };
    EXPECT_EQ(fc_mode_belief(observed(FlightMode::Offboard, at_age_limit), kConfig),
              FcModeBelief::Offboard);
}

TEST(FcModeBeliefTest, NotOffboardIsConclusiveOnlyAfterGivenTime) {
    const FcObservation observation = observed(FlightMode::Hold);
    const std::int64_t received_ns =
        observation.status.value_or(vehicle_api::FlightStatus{}).received_monotonic_ns;

    EXPECT_EQ(fc_mode_belief(observation, kConfig, received_ns), FcModeBelief::Pending);
    EXPECT_EQ(fc_mode_belief(observation, kConfig, received_ns - 1), FcModeBelief::NotOffboard);
    // Offboard 는 받은 시각과 관계없이 Offboard 이다
    EXPECT_EQ(fc_mode_belief(observed(FlightMode::Offboard), kConfig, kNowNs + 1),
              FcModeBelief::Offboard);
}

// ---- 요청 없이 일어나는 전환

struct AdvanceRow {
    const char* name;
    FcCommandState state;
    FcObservation observation;
    std::size_t expected;
    bool entered = false;       // 진입 완료를 알림
    bool exited = false;        // Offboard 이탈을 알림
    bool entry_failed = false;  // 진입 실패를 기록
    bool timed_out = false;     // setpoint 유효 시간 경과로 정지함을 알림
};

std::vector<AdvanceRow> advance_rows() {
    const FcObservation offboard = observed(FlightMode::Offboard);
    const FcObservation hold = observed(FlightMode::Hold);
    return {
        // Idle
        {"Idle: FC 가 Offboard 가 아님", Idle{}, hold, kIdle},
        {"Idle: FC 가 Offboard (mavlink_gateway 재시작, 외부의 변경)", Idle{}, offboard, kHovering},
        {"Idle: 비행 상태 없음", Idle{}, observed(FlightMode::Hold, no_status), kIdle},
        {"Idle: 연결 두절", Idle{}, observed(FlightMode::Offboard, disconnected), kIdle},
        {"Idle: fc_epoch 변경", Idle{}, observed(FlightMode::Hold, new_epoch), kIdle},
        // Entering
        {"Entering: FC 가 아직 전환하지 않음", entering(true), hold, kEntering},
        {"Entering: Offboard 이나 FC 의 응답 전", entering(false), offboard, kEntering},
        {"Entering: 응답과 Offboard 를 모두 확인", entering(true), offboard, kFollowing, true},
        {"Entering: 비행 상태가 오래됨", entering(true), observed(FlightMode::Hold, stale_status),
         kCancellingEntry, false, false, true},
        {"Entering: 연결 두절", entering(true), observed(FlightMode::Hold, disconnected),
         kCancellingEntry, false, false, true},
        {"Entering: 기한 초과, Offboard 가 아님", entering(true),
         observed(FlightMode::Hold, after_deadline), kCancellingEntry, false, false, true},
        {"Entering: 기한 초과, 응답 없이 Offboard", entering(false),
         observed(FlightMode::Offboard, after_deadline), kHovering, false, false, true},
        {"Entering: 유효한 setpoint 없음", entering(true), observed(FlightMode::Hold, no_setpoint),
         kCancellingEntry, false, false, true},
        {"Entering: fc_epoch 변경", entering(true), observed(FlightMode::Hold, new_epoch),
         kCancellingEntry, false, false, true},
        // Following
        {"Following: 추종 중", following(), offboard, kFollowing},
        {"Following: FC 가 Offboard 를 벗어남", following(), observed(FlightMode::Position),
         kLocked, false, true},
        {"Following: 비행 상태가 오래됨", following(), observed(FlightMode::Offboard, stale_status),
         kHovering},
        {"Following: 연결 두절", following(), observed(FlightMode::Offboard, disconnected),
         kHovering},
        {"Following: setpoint 의 유효 시간 초과", following(),
         observed(FlightMode::Offboard, no_setpoint), kHovering, false, false, false, true},
        {"Following: fc_epoch 변경", following(), observed(FlightMode::Offboard, new_epoch),
         kHovering},
        {"Following: 첫 setpoint 를 기다림", awaiting(kNowNs - 1'000'000'000),
         observed(FlightMode::Offboard, no_setpoint), kFollowing},
        {"Following: 수락 전에 받은 setpoint 는 쓰지 않음", awaiting(kNowNs - 10'000'000), offboard,
         kFollowing},
        {"Following: 첫 setpoint 를 확인 시간 넘게 기다림",
         awaiting(kNowNs - kConfig.entry_confirm_ns - 1),
         observed(FlightMode::Offboard, no_setpoint), kHovering},
        {"Following: 기다리던 중 FC 가 Offboard 를 벗어남", awaiting(kNowNs - 1'000'000'000),
         observed(FlightMode::Land), kLocked, false, true},
        // CancellingEntry
        {"CancellingEntry: 기한 전의 Offboard 아님", CancellingEntry{kDeadlineNs}, hold,
         kCancellingEntry},
        {"CancellingEntry: Offboard 확인", CancellingEntry{kDeadlineNs}, offboard, kHovering},
        {"CancellingEntry: 기한 후에 받은 Offboard 아님", CancellingEntry{kDeadlineNs},
         observed(FlightMode::Hold, status_after_deadline), kIdle},
        {"CancellingEntry: 기한이 지났으나 비행 상태는 기한 전의 값", CancellingEntry{kDeadlineNs},
         observed(FlightMode::Hold, after_deadline), kCancellingEntry},
        {"CancellingEntry: 연결 두절", CancellingEntry{kDeadlineNs},
         observed(FlightMode::Hold, disconnected), kCancellingEntry},
        {"CancellingEntry: fc_epoch 변경", CancellingEntry{kDeadlineNs},
         observed(FlightMode::Hold, new_epoch), kCancellingEntry},
        // Hovering
        {"Hovering: 제자리 유지 중", Hovering{}, offboard, kHovering},
        {"Hovering: FC 가 Offboard 를 벗어남", Hovering{}, observed(FlightMode::Return), kLocked,
         false, true},
        {"Hovering: 비행 상태가 오래됨", Hovering{}, observed(FlightMode::Hold, stale_status),
         kHovering},
        {"Hovering: 연결 두절", Hovering{}, observed(FlightMode::Hold, disconnected), kHovering},
        {"Hovering: fc_epoch 변경", Hovering{}, observed(FlightMode::Offboard, new_epoch),
         kHovering},
        // Locked
        {"Locked: FC 가 다시 Offboard", Locked{}, offboard, kLocked},
        {"Locked: fc_epoch 변경", Locked{}, observed(FlightMode::Hold, new_epoch), kLocked},
    };
}

TEST(FcCommandAdvanceTest, FollowsTransitionTable) {
    for (const AdvanceRow& row : advance_rows()) {
        SCOPED_TRACE(row.name);
        const FcCommandStep step = advance(row.state, row.observation, kConfig);

        EXPECT_STREQ(name_of(step.state.index()), name_of(row.expected));
        EXPECT_EQ(step.effects.entered_command_id.has_value(), row.entered);
        EXPECT_EQ(step.effects.exited_to.has_value(), row.exited);
        EXPECT_EQ(step.effects.entry_failure != nullptr, row.entry_failed);
        EXPECT_EQ(step.effects.setpoint_timed_out, row.timed_out);
        EXPECT_FALSE(step.effects.send_entry_command);
    }
}

TEST(FcCommandAdvanceTest, ResultDoesNotChangeWhenAppliedAgain) {
    for (const AdvanceRow& row : advance_rows()) {
        SCOPED_TRACE(row.name);
        const FcCommandStep first = advance(row.state, row.observation, kConfig);
        const FcCommandStep second = advance(first.state, row.observation, kConfig);

        EXPECT_EQ(second.state.index(), first.state.index());
        EXPECT_FALSE(second.effects.entered_command_id.has_value());
        EXPECT_FALSE(second.effects.exited_to.has_value());
        EXPECT_EQ(second.effects.entry_failure, nullptr);
    }
}

TEST(FcCommandAdvanceTest, EnteredFollowingUsesSetpointsReceivedBeforeEntry) {
    const FcCommandStep step = advance(entering(true), observed(FlightMode::Offboard), kConfig);

    const auto* state = std::get_if<Following>(&step.state);
    ASSERT_NE(state, nullptr);
    EXPECT_EQ(state->authority_session, "follow");
    EXPECT_EQ(state->values_after_ns, 0);
    EXPECT_FALSE(state->awaiting_first_value);
    EXPECT_EQ(step.effects.entered_command_id.value_or(vehicle_api::CommandId{}).value, 7U);
}

TEST(FcCommandAdvanceTest, CancelledEntryKeepsDeadlineOfThatEntry) {
    const FcCommandStep step =
        advance(entering(), observed(FlightMode::Hold, no_setpoint), kConfig);

    const auto* state = std::get_if<CancellingEntry>(&step.state);
    ASSERT_NE(state, nullptr);
    EXPECT_EQ(state->deadline_ns, kDeadlineNs);
}

TEST(FcCommandAdvanceTest, ExitReportsModeFcChangedTo) {
    const FcCommandStep step = advance(following(), observed(FlightMode::Land), kConfig);

    EXPECT_EQ(step.effects.exited_to.value_or(FlightMode::Unknown), FlightMode::Land);
}

TEST(FcCommandAdvanceTest, HoverTargetIsFixedAtFirstFreshPose) {
    const FcCommandStep without_pose =
        advance(Hovering{}, observed(FlightMode::Offboard, no_pose), kConfig);
    EXPECT_FALSE(std::get<Hovering>(without_pose.state).target.has_value());

    const FcCommandStep fixed = advance(Hovering{}, observed(FlightMode::Offboard), kConfig);
    EXPECT_DOUBLE_EQ(std::get<Hovering>(fixed.state).target.value_or(HoverTarget{}).north_m, 1.0);

    const auto moved = [](FcObservation& observation) {
        observation.fresh_pose = HoverTarget{50.0, 2.0, -3.0, 0.5};
    };
    const FcCommandStep later =
        advance(fixed.state, observed(FlightMode::Offboard, moved), kConfig);
    EXPECT_DOUBLE_EQ(std::get<Hovering>(later.state).target.value_or(HoverTarget{}).north_m, 1.0);
}

TEST(FcCommandAdvanceTest, StopWhileAwaitingFirstValueKeepsHoverTarget) {
    const FcCommandStep step = advance(awaiting(kNowNs - 1'000'000'000),
                                       observed(FlightMode::Offboard, disconnected), kConfig);

    const auto* state = std::get_if<Hovering>(&step.state);
    ASSERT_NE(state, nullptr);
    EXPECT_DOUBLE_EQ(state->target.value_or(HoverTarget{}).north_m, 9.0);
}

TEST(FcCommandAdvanceTest, ValueReceivedAtOrBeforeAcceptanceDoesNotEndWaiting) {
    // 수락한 시각에 이미 받아 둔 값은 정지 전의 추종에서 남은 값일 수 있다
    const auto value_at_acceptance = [](FcObservation& observation) {
        observation.setpoint_received_ns = kNowNs - 10'000'000;
    };
    const FcCommandStep step =
        advance(awaiting(kNowNs - 10'000'000), observed(FlightMode::Offboard, value_at_acceptance),
                kConfig);

    const auto* state = std::get_if<Following>(&step.state);
    ASSERT_NE(state, nullptr);
    EXPECT_TRUE(state->awaiting_first_value);
    EXPECT_EQ(
        select_setpoint(step.state, observed(FlightMode::Offboard, value_at_acceptance), kConfig)
            .kind,
        SetpointKind::Position);
}

TEST(FcCommandAdvanceTest, FirstValueAfterAcceptanceEndsWaiting) {
    const auto new_value = [](FcObservation& observation) {
        observation.setpoint_received_ns = kNowNs;
    };
    const FcCommandStep step =
        advance(awaiting(kNowNs - 10'000'000), observed(FlightMode::Offboard, new_value), kConfig);

    const auto* state = std::get_if<Following>(&step.state);
    ASSERT_NE(state, nullptr);
    EXPECT_FALSE(state->awaiting_first_value);
    EXPECT_FALSE(state->hover_target.has_value());
}

// 모든 상태와 입력 조합에서 지켜지는 규칙
TEST(FcCommandPolicyTest, RulesHoldForEveryStateAndObservation) {
    const std::vector<FcCommandState> states = {Idle{},
                                                entering(false),
                                                entering(true),
                                                following(),
                                                awaiting(kNowNs - 10'000'000),
                                                CancellingEntry{kDeadlineNs},
                                                Hovering{},
                                                Hovering{HoverTarget{}},
                                                Locked{},
                                                Landing{},
                                                Landing{HoverTarget{}, true, std::nullopt},
                                                Landing{std::nullopt, false, kDeadlineNs}};
    const std::vector<Change> changes = {nullptr,      no_status,      stale_status,
                                         disconnected, new_epoch,      no_setpoint,
                                         no_pose,      after_deadline, status_after_deadline};
    for (const FcCommandState& state : states) {
        for (const FlightMode mode : {FlightMode::Offboard, FlightMode::Hold}) {
            for (std::size_t index = 0; index < changes.size(); ++index) {
                SCOPED_TRACE(std::string(name_of(state.index())) + ", 입력 " +
                             std::to_string(index) +
                             (mode == FlightMode::Offboard ? ", Offboard" : ", Hold"));
                const FcObservation observation = observed(mode, changes[index]);
                const FcCommandStep step = advance(state, observation, kConfig);
                const SetpointKind kind = select_setpoint(step.state, observation, kConfig).kind;

                // Locked 는 해제 요청 없이 끝나지 않고, 아무것도 보내지 않는다
                if (state.index() == kLocked) {
                    EXPECT_EQ(step.state.index(), kLocked);
                }
                if (step.state.index() == kLocked) {
                    EXPECT_EQ(kind, SetpointKind::None);
                }
                // FC 가 Offboard 일 수 있는 상태는 반드시 하나를 보낸다
                if (step.state.index() != kIdle && step.state.index() != kLocked) {
                    EXPECT_NE(kind, SetpointKind::None);
                }
                // 추종 제어의 값은 유효한 값이 있을 때만 보낸다
                if (kind == SetpointKind::Velocity) {
                    EXPECT_TRUE(observation.setpoint_received_ns.has_value());
                }
                // 요청 없이 진입을 시작하지 않고, 추종은 진입을 거쳐서만 시작한다
                if (state.index() != kEntering) {
                    EXPECT_NE(step.state.index(), kEntering);
                }
                if (state.index() != kEntering && state.index() != kFollowing) {
                    EXPECT_NE(step.state.index(), kFollowing);
                }
                // Locked 로의 전환은 FC 가 Offboard 임을 확인한 적이 있는 상태에서만
                if (step.state.index() == kLocked && state.index() != kLocked) {
                    EXPECT_TRUE(state.index() == kFollowing || state.index() == kHovering);
                }
            }
        }
    }
}

// ---- Offboard 진입 요청

struct RequestRow {
    const char* name;
    FcCommandState state;
    FcObservation observation;
    vehicle_api::RequestHeader header;
    FcCommandDenial denial;
    std::size_t expected;
};

vehicle_api::RequestHeader expired_request() {
    vehicle_api::RequestHeader header = request();
    header.valid_until_monotonic_ns = kNowNs - 1;
    return header;
}

vehicle_api::RequestHeader request_of_other_epoch() {
    vehicle_api::RequestHeader header = request();
    header.id.fc_epoch = kEpoch - 1;
    return header;
}

TEST(FcCommandRequestTest, OffboardRequestFollowsTable) {
    const FcObservation hold = observed(FlightMode::Hold);
    const FcObservation offboard = observed(FlightMode::Offboard);
    const auto short_stream = [](FcObservation& observation) {
        observation.streaming_for_ns = kConfig.prestream_ns - 1;
    };
    const std::vector<RequestRow> rows = {
        {"Idle: 수락", Idle{}, hold, request(), FcCommandDenial::None, kEntering},
        {"Idle: 세션 없음", Idle{}, hold, request(""), FcCommandDenial::EmptySession, kIdle},
        {"Idle: fc_epoch 불일치", Idle{}, hold, request_of_other_epoch(),
         FcCommandDenial::FcEpochMismatch, kIdle},
        {"Idle: 연결 두절", Idle{}, observed(FlightMode::Hold, disconnected), request(),
         FcCommandDenial::FcDisconnected, kIdle},
        {"Idle: 비행 상태가 오래됨", Idle{}, observed(FlightMode::Hold, stale_status), request(),
         FcCommandDenial::FcStatusStale, kIdle},
        {"Idle: 비행 상태 없음", Idle{}, observed(FlightMode::Hold, no_status), request(),
         FcCommandDenial::FcStatusStale, kIdle},
        {"Idle: 유효 시간 초과", Idle{}, hold, expired_request(), FcCommandDenial::Expired, kIdle},
        {"Idle: setpoint 송신이 1초 미만", Idle{}, observed(FlightMode::Hold, short_stream),
         request(), FcCommandDenial::PrestreamNotReady, kIdle},
        {"Entering: 같은 세션의 새 요청", entering(true), hold, request(), FcCommandDenial::None,
         kEntering},
        {"Entering: 다른 세션", entering(), hold, request("other"), FcCommandDenial::NoAuthority,
         kEntering},
        {"Entering: 유효 시간 초과", entering(), hold, expired_request(), FcCommandDenial::Expired,
         kEntering},
        {"Following: 같은 세션", following(), offboard, request(), FcCommandDenial::None,
         kFollowing},
        {"Following: 다른 세션", following(), offboard, request("other"),
         FcCommandDenial::NoAuthority, kFollowing},
        {"Following: fc_epoch 불일치", following(), offboard, request_of_other_epoch(),
         FcCommandDenial::FcEpochMismatch, kFollowing},
        {"CancellingEntry: 거부", CancellingEntry{kDeadlineNs}, hold, request(),
         FcCommandDenial::EntryCancelled, kCancellingEntry},
        {"Hovering: 진입 명령 없이 수락", Hovering{}, offboard, request(), FcCommandDenial::None,
         kFollowing},
        {"Hovering: 다른 세션도 수락", Hovering{}, offboard, request("other"),
         FcCommandDenial::None, kFollowing},
        {"Hovering: 세션 없음", Hovering{}, offboard, request(""), FcCommandDenial::EmptySession,
         kHovering},
        {"Hovering: fc_epoch 불일치", Hovering{}, offboard, request_of_other_epoch(),
         FcCommandDenial::FcEpochMismatch, kHovering},
        {"Hovering: 비행 상태가 오래됨", Hovering{}, observed(FlightMode::Offboard, stale_status),
         request(), FcCommandDenial::FcStatusStale, kHovering},
        {"Hovering: 새 fc_epoch 의 비행 상태 전", Hovering{},
         observed(FlightMode::Offboard, no_status), request(), FcCommandDenial::FcStatusStale,
         kHovering},
        {"Hovering: 유효 시간 초과", Hovering{}, offboard, expired_request(),
         FcCommandDenial::Expired, kHovering},
        {"Locked: 거부", Locked{}, hold, request(), FcCommandDenial::ExternalModeChange, kLocked},
    };
    for (const RequestRow& row : rows) {
        SCOPED_TRACE(row.name);
        const FcRequestStep step =
            on_offboard_request(row.state, row.observation, row.header, kConfig);

        EXPECT_EQ(to_string(step.denial), to_string(row.denial));
        EXPECT_STREQ(name_of(step.state.index()), name_of(row.expected));
        // 진입 명령은 Idle 과 Entering 에서 수락한 경우에만 보낸다
        const bool expects_command =
            row.denial == FcCommandDenial::None && row.expected == kEntering;
        EXPECT_EQ(step.effects.send_entry_command, expects_command);
        // 이미 Offboard 인 FC 에서 수락한 요청은 바로 완료로 알린다
        const bool expects_entered =
            row.denial == FcCommandDenial::None && row.expected == kFollowing;
        EXPECT_EQ(step.effects.entered_command_id.has_value(), expects_entered);
    }
}

TEST(FcCommandRequestTest, AcceptedEntryRecordsSessionCommandAndDeadline) {
    const FcRequestStep step =
        on_offboard_request(Idle{}, observed(FlightMode::Hold), request("follow", 21), kConfig);

    const auto* state = std::get_if<Entering>(&step.state);
    ASSERT_NE(state, nullptr);
    EXPECT_EQ(state->authority_session, "follow");
    EXPECT_EQ(state->command_id.value, 21U);
    EXPECT_EQ(state->fc_epoch, kEpoch);
    EXPECT_EQ(state->deadline_ns, kNowNs + kConfig.entry_confirm_ns);
    EXPECT_FALSE(state->acknowledged);
}

TEST(FcCommandRequestTest, RepeatedEntryRequestReplacesPendingOne) {
    const FcRequestStep step = on_offboard_request(entering(true), observed(FlightMode::Hold),
                                                   request("follow", 22), kConfig);

    const auto* state = std::get_if<Entering>(&step.state);
    ASSERT_NE(state, nullptr);
    EXPECT_EQ(state->command_id.value, 22U);
    EXPECT_FALSE(state->acknowledged);
}

TEST(FcCommandRequestTest, RequestAcceptedWhileHoveringWaitsForNewSetpointAndKeepsTarget) {
    const Hovering hovering{HoverTarget{4.0, 5.0, -6.0, 0.2}};
    const FcRequestStep step =
        on_offboard_request(hovering, observed(FlightMode::Offboard), request("other"), kConfig);

    const auto* state = std::get_if<Following>(&step.state);
    ASSERT_NE(state, nullptr);
    EXPECT_EQ(state->authority_session, "other");
    EXPECT_EQ(state->values_after_ns, kNowNs);
    EXPECT_TRUE(state->awaiting_first_value);
    EXPECT_DOUBLE_EQ(state->hover_target.value_or(HoverTarget{}).north_m, 4.0);
    EXPECT_EQ(step.effects.entered_command_id.value_or(vehicle_api::CommandId{}).value, 11U);
}

// ---- 정지 요청

TEST(FcCommandRequestTest, HoldRequestFollowsTable) {
    const FcObservation hold = observed(FlightMode::Hold);
    const FcObservation offboard = observed(FlightMode::Offboard);
    const std::vector<RequestRow> rows = {
        {"Idle: 수락, 보낼 것 없음", Idle{}, hold, request(), FcCommandDenial::None, kIdle},
        {"Idle: fc_epoch 불일치", Idle{}, hold, request_of_other_epoch(),
         FcCommandDenial::FcEpochMismatch, kIdle},
        {"Entering: 같은 세션", entering(), hold, request(), FcCommandDenial::None,
         kCancellingEntry},
        {"Entering: 같은 세션의 유효 시간이 지난 요청도 정지", entering(), hold, expired_request(),
         FcCommandDenial::Expired, kCancellingEntry},
        {"Entering: 다른 세션", entering(), hold, request("other"), FcCommandDenial::NoAuthority,
         kEntering},
        {"Following: 같은 세션", following(), offboard, request(), FcCommandDenial::None,
         kHovering},
        {"Following: 같은 세션의 fc_epoch 가 다른 요청도 정지", following(), offboard,
         request_of_other_epoch(), FcCommandDenial::FcEpochMismatch, kHovering},
        {"Following: 다른 세션", following(), offboard, request("other"),
         FcCommandDenial::NoAuthority, kFollowing},
        {"CancellingEntry: 수락", CancellingEntry{kDeadlineNs}, hold, request(),
         FcCommandDenial::None, kCancellingEntry},
        {"Hovering: 수락", Hovering{}, offboard, request(), FcCommandDenial::None, kHovering},
        {"Hovering: 유효 시간 초과", Hovering{}, offboard, expired_request(),
         FcCommandDenial::Expired, kHovering},
        {"Locked: 거부", Locked{}, hold, request(), FcCommandDenial::ExternalModeChange, kLocked},
    };
    for (const RequestRow& row : rows) {
        SCOPED_TRACE(row.name);
        const FcRequestStep step = on_hold_request(row.state, row.observation, row.header);

        EXPECT_EQ(to_string(step.denial), to_string(row.denial));
        EXPECT_STREQ(name_of(step.state.index()), name_of(row.expected));
        EXPECT_FALSE(step.effects.send_entry_command);
    }
}

TEST(FcCommandRequestTest, OnboardStopNeedsNoSessionAndIsRefusedOnlyWhenLocked) {
    struct Row {
        FcCommandState state;
        std::size_t expected;
        bool accepted;
    };
    const std::vector<Row> rows = {
        {Idle{}, kIdle, true},          {entering(), kCancellingEntry, true},
        {following(), kHovering, true}, {CancellingEntry{kDeadlineNs}, kCancellingEntry, true},
        {Hovering{}, kHovering, true},  {Locked{}, kLocked, false},
    };
    for (const Row& row : rows) {
        SCOPED_TRACE(name_of(row.state.index()));
        const FcRequestStep step = on_onboard_stop(row.state);

        EXPECT_STREQ(name_of(step.state.index()), name_of(row.expected));
        EXPECT_EQ(step.denial == FcCommandDenial::None, row.accepted);
    }
}

// ---- FC 의 응답, 송신 실패, Locked 해제

TEST(FcCommandRequestTest, AcceptedAnswerMarksEntryAcknowledged) {
    const FcCommandStep step = on_entry_command_answered(entering(false), true);

    ASSERT_NE(std::get_if<Entering>(&step.state), nullptr);
    EXPECT_TRUE(std::get<Entering>(step.state).acknowledged);
}

TEST(FcCommandRequestTest, RejectedAnswerOrSendFailureCancelsEntry) {
    const FcCommandStep rejected = on_entry_command_answered(entering(), false);
    EXPECT_EQ(rejected.state.index(), kCancellingEntry);
    EXPECT_NE(rejected.effects.entry_failure, nullptr);

    const FcCommandStep failed = on_entry_command_send_failed(entering());
    EXPECT_EQ(failed.state.index(), kCancellingEntry);
    EXPECT_EQ(std::get<CancellingEntry>(failed.state).deadline_ns, kDeadlineNs);
    EXPECT_NE(failed.effects.entry_failure, nullptr);
}

TEST(FcCommandRequestTest, AnswerOutsideEnteringChangesNothing) {
    for (const FcCommandState& state : std::vector<FcCommandState>{
             Idle{}, following(), CancellingEntry{kDeadlineNs}, Hovering{}, Locked{}}) {
        SCOPED_TRACE(name_of(state.index()));
        EXPECT_EQ(on_entry_command_answered(state, false).state.index(), state.index());
        EXPECT_EQ(on_entry_command_send_failed(state).state.index(), state.index());
    }
}

TEST(FcCommandRequestTest, LockIsReleasedOnlyWithFreshFcStatus) {
    EXPECT_EQ(on_lock_release(Locked{}, observed(FlightMode::Hold), kConfig).state.index(), kIdle);
    EXPECT_EQ(on_lock_release(Locked{}, observed(FlightMode::Offboard), kConfig).state.index(),
              kIdle);

    const FcRequestStep stale =
        on_lock_release(Locked{}, observed(FlightMode::Hold, stale_status), kConfig);
    EXPECT_EQ(stale.state.index(), kLocked);
    EXPECT_EQ(stale.denial, FcCommandDenial::FcStatusStale);
    const FcRequestStep lost =
        on_lock_release(Locked{}, observed(FlightMode::Hold, disconnected), kConfig);
    EXPECT_EQ(lost.state.index(), kLocked);
    EXPECT_EQ(lost.denial, FcCommandDenial::FcDisconnected);

    // Locked 가 아닌 상태는 그대로 둔다
    const FcRequestStep not_locked =
        on_lock_release(following(), observed(FlightMode::Offboard), kConfig);
    EXPECT_EQ(not_locked.state.index(), kFollowing);
    EXPECT_EQ(not_locked.denial, FcCommandDenial::None);
}

// ---- 주기마다 보낼 setpoint

TEST(FcCommandSelectTest, FollowsSelectionTable) {
    const FcObservation hold = observed(FlightMode::Hold);
    const FcObservation offboard = observed(FlightMode::Offboard);
    struct Row {
        const char* name;
        FcCommandState state;
        FcObservation observation;
        SetpointKind expected;
    };
    const std::vector<Row> rows = {
        {"Idle: 진입 전의 미리 송신", Idle{}, hold, SetpointKind::Velocity},
        {"Idle: setpoint 없음", Idle{}, observed(FlightMode::Hold, no_setpoint),
         SetpointKind::None},
        {"Idle: 비행 상태가 오래됨", Idle{}, observed(FlightMode::Hold, stale_status),
         SetpointKind::None},
        {"Idle: 비행 상태 없음", Idle{}, observed(FlightMode::Hold, no_status), SetpointKind::None},
        {"Idle: 연결 두절", Idle{}, observed(FlightMode::Hold, disconnected), SetpointKind::None},
        {"Entering: 세션의 값", entering(), hold, SetpointKind::Velocity},
        {"Entering: 값 없음", entering(), observed(FlightMode::Hold, no_setpoint),
         SetpointKind::ZeroVelocity},
        {"Following: 세션의 값", following(), offboard, SetpointKind::Velocity},
        {"Following: 값 없음", following(), observed(FlightMode::Offboard, no_setpoint),
         SetpointKind::ZeroVelocity},
        {"Following: 첫 값을 기다리는 동안 제자리 유지", awaiting(kNowNs - 10'000'000), offboard,
         SetpointKind::Position},
        {"CancellingEntry: 속도 0", CancellingEntry{kDeadlineNs}, hold, SetpointKind::ZeroVelocity},
        {"Hovering: 목표 없음", Hovering{}, offboard, SetpointKind::ZeroVelocity},
        {"Hovering: 목표 있음", Hovering{HoverTarget{}}, offboard, SetpointKind::Position},
        {"Locked: 보내지 않음", Locked{}, offboard, SetpointKind::None},
    };
    for (const Row& row : rows) {
        SCOPED_TRACE(row.name);
        EXPECT_EQ(select_setpoint(row.state, row.observation, kConfig).kind, row.expected);
    }
}

TEST(FcCommandSelectTest, PositionSetpointCarriesHoverTarget) {
    const SetpointChoice hovering = select_setpoint(Hovering{HoverTarget{4.0, 5.0, -6.0, 0.2}},
                                                    observed(FlightMode::Offboard), kConfig);
    EXPECT_DOUBLE_EQ(hovering.target.east_m, 5.0);

    const SetpointChoice waiting =
        select_setpoint(awaiting(kNowNs - 10'000'000), observed(FlightMode::Offboard), kConfig);
    EXPECT_DOUBLE_EQ(waiting.target.north_m, 9.0);
}

TEST(FcCommandPolicyTest, AuthorityBelongsToSessionOfEnteringAndFollowing) {
    ASSERT_NE(authority_session(entering()), nullptr);
    EXPECT_EQ(*authority_session(following()), "follow");
    for (const FcCommandState& state :
         std::vector<FcCommandState>{Idle{}, CancellingEntry{kDeadlineNs}, Hovering{}, Locked{}}) {
        EXPECT_EQ(authority_session(state), nullptr);
    }
}

TEST(FcCommandPolicyTest, HasReasonTextForEveryDenial) {
    for (std::int32_t value = 1; value <= static_cast<std::int32_t>(FcCommandDenial::PoseStale);
         ++value) {
        EXPECT_FALSE(to_string(static_cast<FcCommandDenial>(value)).empty()) << value;
    }
    EXPECT_TRUE(to_string(FcCommandDenial::None).empty());
}

// ---- 착륙 명령 전달 (icd 5.3)

HoverEntering hover_entering(std::optional<std::int64_t> deadline_ns = std::nullopt,
                             bool acknowledged = false) {
    HoverEntering state;
    state.target = HoverTarget{6.0, 7.0, -8.0, 0.3};
    state.fc_epoch = kEpoch;
    state.started_ns = kNowNs - 100'000'000;
    state.start_mode = FlightMode::Position;
    state.deadline_ns = deadline_ns;
    state.acknowledged = acknowledged;
    return state;
}

TEST(FcCommandLandingTest, ReleaseDropsAuthorityAndKeepsHoldingWhileFcMayBeOffboard) {
    struct Row {
        const char* name;
        FcCommandState state;
        std::size_t expected;
        bool was_offboard;
        std::optional<std::int64_t> entry_deadline_ns;
    };
    const std::vector<Row> rows = {
        {"Idle", Idle{}, kIdle, false, std::nullopt},
        {"Entering", entering(true), kLanding, false, kDeadlineNs},
        {"Following", following(), kLanding, true, std::nullopt},
        {"CancellingEntry", CancellingEntry{kDeadlineNs}, kLanding, false, kDeadlineNs},
        {"Hovering", Hovering{HoverTarget{}}, kLanding, true, std::nullopt},
        {"Locked", Locked{}, kLocked, false, std::nullopt},
        {"Landing", Landing{}, kLanding, false, std::nullopt},
        {"HoverEntering", hover_entering(kDeadlineNs), kLanding, false, kDeadlineNs},
    };
    for (const Row& row : rows) {
        SCOPED_TRACE(row.name);
        const FcCommandStep step = on_landing_release(row.state);

        EXPECT_STREQ(name_of(step.state.index()), name_of(row.expected));
        EXPECT_EQ(authority_session(step.state), nullptr);
        if (const auto* landing = std::get_if<Landing>(&step.state)) {
            EXPECT_EQ(landing->was_offboard, row.was_offboard);
            EXPECT_EQ(landing->entry_deadline_ns, row.entry_deadline_ns);
        }
    }
}

TEST(FcCommandLandingTest, ReleaseKeepsHoverTargetAndEndsHoverEntry) {
    const FcCommandStep from_hovering =
        on_landing_release(Hovering{HoverTarget{4.0, 5.0, -6.0, 0.2}});
    EXPECT_DOUBLE_EQ(std::get<Landing>(from_hovering.state).target.value_or(HoverTarget{}).east_m,
                     5.0);

    const FcCommandStep from_entry = on_landing_release(hover_entering());
    EXPECT_EQ(from_entry.effects.hover_entered, std::optional<bool>(false));
}

TEST(FcCommandLandingTest, EndsWithoutLockWhenFcLeavesOffboard) {
    const FcCommandStep step =
        advance(Landing{HoverTarget{}, true, std::nullopt}, observed(FlightMode::Land), kConfig);

    EXPECT_EQ(step.state.index(), kIdle);
    EXPECT_EQ(step.effects.exited_to.value_or(FlightMode::Unknown), FlightMode::Land);
    EXPECT_TRUE(step.effects.exit_requested);
}

TEST(FcCommandLandingTest, ReportsNoExitWhenFcWasNeverOffboard) {
    const FcCommandStep step = advance(Landing{}, observed(FlightMode::Land), kConfig);

    EXPECT_EQ(step.state.index(), kIdle);
    EXPECT_FALSE(step.effects.exited_to.has_value());
}

TEST(FcCommandLandingTest, KeepsHoldingWhileOffboardOrUnknown) {
    for (const Change& change :
         std::vector<Change>{nullptr, stale_status, disconnected, new_epoch}) {
        const FcCommandStep step =
            advance(Landing{}, observed(FlightMode::Offboard, change), kConfig);
        EXPECT_EQ(step.state.index(), kLanding);
        EXPECT_NE(select_setpoint(step.state, observed(FlightMode::Offboard, change), kConfig).kind,
                  SetpointKind::None);
    }
    const FcCommandStep offboard = advance(Landing{}, observed(FlightMode::Offboard), kConfig);
    EXPECT_TRUE(std::get<Landing>(offboard.state).was_offboard);
    EXPECT_TRUE(std::get<Landing>(offboard.state).target.has_value());
}

TEST(FcCommandLandingTest, UsesOnlyStatusAfterCancelledEntryDeadline) {
    const Landing landing{std::nullopt, false, kDeadlineNs};
    EXPECT_EQ(advance(landing, observed(FlightMode::Land, after_deadline), kConfig).state.index(),
              kLanding);
    EXPECT_EQ(
        advance(landing, observed(FlightMode::Land, status_after_deadline), kConfig).state.index(),
        kIdle);
}

TEST(FcCommandLandingTest, RefusesOffboardRequest) {
    const FcRequestStep step =
        on_offboard_request(Landing{}, observed(FlightMode::Offboard), request(), kConfig);
    EXPECT_EQ(step.denial, FcCommandDenial::Landing);
    EXPECT_EQ(step.state.index(), kLanding);
}

// ---- 복귀 명령의 제자리 유지 진입 (icd 5.4)

TEST(FcCommandHoverEntryTest, StartsFromIdleOrLockedWithFreshPose) {
    for (const FcCommandState& state : std::vector<FcCommandState>{Idle{}, Locked{}}) {
        const FcRequestStep step =
            on_hover_entry_request(state, observed(FlightMode::Position), kConfig);
        ASSERT_EQ(step.denial, FcCommandDenial::None);
        const auto& entering = std::get<HoverEntering>(step.state);
        EXPECT_DOUBLE_EQ(entering.target.north_m, 1.0);
        EXPECT_EQ(entering.start_mode, FlightMode::Position);
        EXPECT_EQ(entering.started_ns, kNowNs);
        EXPECT_FALSE(entering.deadline_ns.has_value());
    }
}

TEST(FcCommandHoverEntryTest, RefusesWhenBusyUnknownOrPoseStale) {
    EXPECT_EQ(on_hover_entry_request(Hovering{}, observed(FlightMode::Position), kConfig).denial,
              FcCommandDenial::Busy);
    EXPECT_EQ(on_hover_entry_request(Idle{}, observed(FlightMode::Offboard), kConfig).denial,
              FcCommandDenial::Busy);
    EXPECT_EQ(on_hover_entry_request(Idle{}, observed(FlightMode::Position, stale_status), kConfig)
                  .denial,
              FcCommandDenial::FcStatusStale);
    EXPECT_EQ(
        on_hover_entry_request(Idle{}, observed(FlightMode::Position, no_pose), kConfig).denial,
        FcCommandDenial::PoseStale);
}

TEST(FcCommandHoverEntryTest, SendsPositionTargetThenEntryCommandAfterPrestream) {
    const HoverEntering entering = hover_entering();
    const SetpointChoice choice =
        select_setpoint(entering, observed(FlightMode::Position), kConfig);
    EXPECT_EQ(choice.kind, SetpointKind::Position);
    EXPECT_DOUBLE_EQ(choice.target.east_m, 7.0);

    const FcCommandStep waiting = advance(entering, observed(FlightMode::Position), kConfig);
    EXPECT_FALSE(waiting.effects.send_entry_command);

    const auto after_prestream = [](FcObservation& observation) {
        observation.now_ns = kNowNs - 100'000'000 + kConfig.prestream_ns;
    };
    const FcCommandStep sending =
        advance(entering, observed(FlightMode::Position, after_prestream), kConfig);
    EXPECT_TRUE(sending.effects.send_entry_command);
    EXPECT_EQ(std::get<HoverEntering>(sending.state).deadline_ns,
              std::optional<std::int64_t>(kNowNs - 100'000'000 + kConfig.prestream_ns +
                                          kConfig.entry_confirm_ns));
}

TEST(FcCommandHoverEntryTest, GivesUpBeforeEntryCommandWhenModeChanges) {
    const FcCommandStep step = advance(hover_entering(), observed(FlightMode::Altitude), kConfig);
    EXPECT_EQ(step.state.index(), kIdle);
    EXPECT_EQ(step.effects.hover_entered, std::optional<bool>(false));

    const FcCommandStep aborted = on_hover_entry_abort(hover_entering());
    EXPECT_EQ(aborted.state.index(), kIdle);
    EXPECT_EQ(aborted.effects.hover_entered, std::optional<bool>(false));
    // 진입 명령을 보낸 뒤에는 포기하지 않는다
    EXPECT_EQ(on_hover_entry_abort(hover_entering(kDeadlineNs)).state.index(), kHoverEntering);
}

TEST(FcCommandHoverEntryTest, EntersHoveringWhenAcknowledgedAndOffboard) {
    const FcCommandStep acknowledged = on_entry_command_answered(hover_entering(kDeadlineNs), true);
    const FcCommandStep step = advance(acknowledged.state, observed(FlightMode::Offboard), kConfig);

    EXPECT_EQ(step.state.index(), kHovering);
    EXPECT_DOUBLE_EQ(std::get<Hovering>(step.state).target.value_or(HoverTarget{}).north_m, 6.0);
    EXPECT_EQ(step.effects.hover_entered, std::optional<bool>(true));
}

TEST(FcCommandHoverEntryTest, FailsOnRejectionTimeoutSendFailureOrStop) {
    const FcCommandStep rejected = on_entry_command_answered(hover_entering(kDeadlineNs), false);
    EXPECT_EQ(rejected.state.index(), kCancellingEntry);
    EXPECT_EQ(rejected.effects.hover_entered, std::optional<bool>(false));

    const FcCommandStep timed_out = advance(
        hover_entering(kDeadlineNs, true), observed(FlightMode::Position, after_deadline), kConfig);
    EXPECT_EQ(timed_out.state.index(), kCancellingEntry);
    EXPECT_EQ(timed_out.effects.hover_entered, std::optional<bool>(false));

    const FcCommandStep unsent = on_entry_command_send_failed(hover_entering(kDeadlineNs));
    EXPECT_EQ(unsent.state.index(), kIdle);
    EXPECT_EQ(unsent.effects.hover_entered, std::optional<bool>(false));

    const FcRequestStep stopped = on_onboard_stop(hover_entering(kDeadlineNs));
    EXPECT_EQ(stopped.state.index(), kCancellingEntry);
    EXPECT_EQ(stopped.effects.hover_entered, std::optional<bool>(false));
}

TEST(FcCommandHoverEntryTest, RefusesOffboardRequestDuringEntry) {
    EXPECT_EQ(
        on_offboard_request(hover_entering(), observed(FlightMode::Position), request(), kConfig)
            .denial,
        FcCommandDenial::HoverEntry);
}

}  // namespace
}  // namespace argus::mavlink_gateway
