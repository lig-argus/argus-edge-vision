#include "argus/mavlink_gateway/fc_command_policy.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include "argus/mavlink_gateway/fc_command_state.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::FlightMode;
using vehicle_api::RequestHeader;

// advance() 가 전환을 이어서 적용하는 횟수의 상한. 전환은 되돌아가지 않으므로 상태의 수보다
// 많이 이어지지 않는다
constexpr std::int32_t kMaxChainedTransitions = 6;

FcCommandStep to(FcCommandState state) { return FcCommandStep{std::move(state), {}}; }

FcCommandStep entry_failed(FcCommandState state, const char* cause) {
    FcCommandStep step = to(std::move(state));
    step.effects.entry_failure = cause;
    return step;
}

FcRequestStep accept(FcCommandState state) {
    return FcRequestStep{std::move(state), {}, FcCommandDenial::None};
}

FcRequestStep deny(const FcCommandState& state, FcCommandDenial denial) {
    return FcRequestStep{state, {}, denial};
}

// FC 의 모드를 알 수 없는 경우의 거부 원인
FcCommandDenial unknown_mode_denial(const FcObservation& observation) {
    return observation.fc_connected ? FcCommandDenial::FcStatusStale
                                    : FcCommandDenial::FcDisconnected;
}

// 요청의 fc_epoch 와 유효 시간
FcCommandDenial validity_denial(const RequestHeader& header, const FcObservation& observation) {
    if (header.id.fc_epoch != observation.fc_epoch) {
        return FcCommandDenial::FcEpochMismatch;
    }
    if (observation.now_ns > header.valid_until_monotonic_ns) {
        return FcCommandDenial::Expired;
    }
    return FcCommandDenial::None;
}

bool has_setpoint_after(const FcObservation& observation, std::int64_t after_ns) {
    return observation.setpoint_received_ns.has_value() &&
           *observation.setpoint_received_ns > after_ns;
}

// 세션 session 이 제어하며, values_after_ns 보다 늦게 받은 setpoint 만 쓰는 추종 상태
Following following_by(const std::string& session, std::uint64_t fc_epoch,
                       std::int64_t values_after_ns) {
    Following following;
    following.authority_session = session;
    following.fc_epoch = fc_epoch;
    following.values_after_ns = values_after_ns;
    return following;
}

FcCommandState stopped(const Entering& entering) { return CancellingEntry{entering.deadline_ns}; }

// 첫 setpoint 를 기다리던 중이면 그동안 보내던 목표를 그대로 이어 간다
FcCommandState stopped(const Following& following) {
    return Hovering{following.awaiting_first_value ? following.hover_target : std::nullopt};
}

// 요청 없이 일어나는 전환 하나. 바뀌지 않는 경우 nullopt
class Advance {
public:
    Advance(const FcObservation& observation, const FcCommandPolicyConfig& config)
        : observation_(observation), config_(config) {}

    std::optional<FcCommandStep> operator()(const Idle& /*idle*/) const {
        // mavlink_gateway 가 다시 시작했거나 외부가 FC 를 Offboard 로 바꾼 경우
        if (belief() == FcModeBelief::Offboard) {
            return to(Hovering{});
        }
        return std::nullopt;
    }

    std::optional<FcCommandStep> operator()(const Entering& entering) const {
        if (entering.fc_epoch != observation_.fc_epoch) {
            return entry_failed(stopped(entering), "FC 연결이 다시 됨");
        }
        const FcModeBelief mode = belief();
        if (mode == FcModeBelief::Offboard && entering.acknowledged) {
            FcCommandStep step = to(following_by(entering.authority_session, entering.fc_epoch, 0));
            step.effects.entered_command_id = entering.command_id;
            return step;
        }
        if (mode == FcModeBelief::NotReceived) {
            return entry_failed(stopped(entering), "FC 의 비행 상태를 받지 못함");
        }
        if (observation_.now_ns > entering.deadline_ns) {
            // FC 의 응답만 없이 Offboard 가 된 경우 mavlink_gateway 가 제자리 유지를 맡는다
            return entry_failed(
                mode == FcModeBelief::Offboard ? FcCommandState{Hovering{}} : stopped(entering),
                "확인 시간 초과");
        }
        if (!observation_.setpoint_received_ns.has_value()) {
            return entry_failed(stopped(entering), "유효한 setpoint 가 없음");
        }
        return std::nullopt;
    }

    std::optional<FcCommandStep> operator()(const Following& following) const {
        if (following.fc_epoch != observation_.fc_epoch) {
            return to(stopped(following));
        }
        const FcModeBelief mode = belief();
        if (mode == FcModeBelief::NotOffboard) {
            return locked();
        }
        if (mode == FcModeBelief::NotReceived) {
            return to(stopped(following));
        }
        const bool has_value = has_setpoint_after(observation_, following.values_after_ns);
        if (following.awaiting_first_value) {
            return has_value ? started(following) : give_up_if_overdue(following);
        }
        if (has_value) {
            return std::nullopt;
        }
        // setpoint 의 유효 시간이 지남 (icd.md 6.4 ⓐ)
        FcCommandStep step = to(stopped(following));
        step.effects.setpoint_timed_out = true;
        return step;
    }

    std::optional<FcCommandStep> operator()(const CancellingEntry& cancelling) const {
        switch (fc_mode_belief(observation_, config_, cancelling.deadline_ns)) {
            case FcModeBelief::Offboard:
                return to(Hovering{});
            case FcModeBelief::NotOffboard:
                // 진입이 이루어지지 않은 것이므로 Locked 로 전환하지 않는다
                return to(Idle{});
            case FcModeBelief::Pending:
            case FcModeBelief::NotReceived:
                return std::nullopt;
        }
        return std::nullopt;
    }

    std::optional<FcCommandStep> operator()(const Hovering& hovering) const {
        if (belief() == FcModeBelief::NotOffboard) {
            return locked();
        }
        if (!hovering.target.has_value() && observation_.fresh_pose.has_value()) {
            return to(Hovering{observation_.fresh_pose});
        }
        return std::nullopt;
    }

    std::optional<FcCommandStep> operator()(const Locked& /*locked*/) const { return std::nullopt; }

private:
    FcModeBelief belief() const { return fc_mode_belief(observation_, config_); }

    static std::optional<FcCommandStep> started(const Following& following) {
        return to(following_by(following.authority_session, following.fc_epoch,
                               following.values_after_ns));
    }

    // 제자리 유지 중에 송신권을 얻은 세션이 확인 시간 동안 setpoint 를 보내지 않은 경우 송신권을
    // 폐기한다 (폐기하지 않으면 다른 세션의 요청이 계속 거부된다)
    std::optional<FcCommandStep> give_up_if_overdue(const Following& following) const {
        const bool overdue =
            observation_.now_ns - following.values_after_ns > config_.entry_confirm_ns;
        return overdue ? std::optional<FcCommandStep>(to(stopped(following))) : std::nullopt;
    }

    // mavlink_gateway 는 FC 를 Offboard 에서 벗어나게 하는 명령을 보내지 않으므로, FC 가 Offboard
    // 를 벗어난 것은 조종사나 통제기의 모드 전환 또는 FC 의 failsafe 이다
    FcCommandStep locked() const {
        FcCommandStep step = to(Locked{});
        if (observation_.status.has_value()) {
            step.effects.exited_to = observation_.status->mode;
        }
        return step;
    }

    const FcObservation& observation_;
    const FcCommandPolicyConfig& config_;
};

void merge(FcCommandEffects& into, const FcCommandEffects& from) {
    into.send_entry_command = into.send_entry_command || from.send_entry_command;
    if (from.entered_command_id.has_value()) {
        into.entered_command_id = from.entered_command_id;
    }
    if (from.exited_to.has_value()) {
        into.exited_to = from.exited_to;
    }
    if (from.entry_failure != nullptr) {
        into.entry_failure = from.entry_failure;
    }
    into.setpoint_timed_out = into.setpoint_timed_out || from.setpoint_timed_out;
}

class OffboardRequest {
public:
    OffboardRequest(const FcObservation& observation, const RequestHeader& header,
                    const FcCommandPolicyConfig& config)
        : observation_(observation), header_(header), config_(config) {}

    FcRequestStep operator()(const Idle& idle) const {
        if (const FcCommandDenial denial = validity_before_mode();
            denial != FcCommandDenial::None) {
            return deny(idle, denial);
        }
        if (fc_mode_belief(observation_, config_) != FcModeBelief::NotOffboard) {
            return deny(idle, unknown_mode_denial(observation_));
        }
        if (observation_.now_ns > header_.valid_until_monotonic_ns) {
            return deny(idle, FcCommandDenial::Expired);
        }
        if (observation_.streaming_for_ns < config_.prestream_ns) {
            return deny(idle, FcCommandDenial::PrestreamNotReady);
        }
        return send_entry_command();
    }

    FcRequestStep operator()(const Entering& entering) const {
        if (const FcCommandDenial denial = owner_denial(entering.authority_session);
            denial != FcCommandDenial::None) {
            return deny(entering, denial);
        }
        // 확인을 기다리던 요청은 새 요청으로 대체한다
        return send_entry_command();
    }

    FcRequestStep operator()(const Following& following) const {
        if (const FcCommandDenial denial = owner_denial(following.authority_session);
            denial != FcCommandDenial::None) {
            return deny(following, denial);
        }
        return entered(following);
    }

    FcRequestStep operator()(const CancellingEntry& cancelling) const {
        return deny(cancelling, FcCommandDenial::EntryCancelled);
    }

    // FC 가 이미 Offboard 이므로 진입 명령 없이 수락한다
    FcRequestStep operator()(const Hovering& hovering) const {
        if (const FcCommandDenial denial = validity_before_mode();
            denial != FcCommandDenial::None) {
            return deny(hovering, denial);
        }
        if (fc_mode_belief(observation_, config_) != FcModeBelief::Offboard) {
            return deny(hovering, unknown_mode_denial(observation_));
        }
        if (observation_.now_ns > header_.valid_until_monotonic_ns) {
            return deny(hovering, FcCommandDenial::Expired);
        }
        Following following =
            following_by(header_.id.session_id, observation_.fc_epoch, observation_.now_ns);
        following.awaiting_first_value = true;
        following.hover_target = hovering.target;
        return entered(following);
    }

    FcRequestStep operator()(const Locked& locked) const {
        return deny(locked, FcCommandDenial::ExternalModeChange);
    }

private:
    // FC 의 모드보다 먼저 확인하는 조건
    FcCommandDenial validity_before_mode() const {
        if (header_.id.session_id.empty()) {
            return FcCommandDenial::EmptySession;
        }
        return header_.id.fc_epoch == observation_.fc_epoch ? FcCommandDenial::None
                                                            : FcCommandDenial::FcEpochMismatch;
    }

    // 송신권을 가진 세션이 있는 상태에서 확인하는 조건
    FcCommandDenial owner_denial(const std::string& owner) const {
        if (header_.id.session_id.empty()) {
            return FcCommandDenial::EmptySession;
        }
        if (header_.id.session_id != owner) {
            return FcCommandDenial::NoAuthority;
        }
        return validity_denial(header_, observation_);
    }

    FcRequestStep send_entry_command() const {
        Entering entering;
        entering.authority_session = header_.id.session_id;
        entering.command_id = header_.id.command_id;
        entering.fc_epoch = observation_.fc_epoch;
        entering.deadline_ns = observation_.now_ns + config_.entry_confirm_ns;
        FcRequestStep step = accept(entering);
        step.effects.send_entry_command = true;
        return step;
    }

    FcRequestStep entered(const Following& following) const {
        FcRequestStep step = accept(following);
        step.effects.entered_command_id = header_.id.command_id;
        return step;
    }

    const FcObservation& observation_;
    const RequestHeader& header_;
    const FcCommandPolicyConfig& config_;
};

// 송신권을 가진 세션이 있는 상태(Entering, Following)의 정지 요청
template <typename State>
FcRequestStep hold_by_session(const State& state, const FcObservation& observation,
                              const RequestHeader& header) {
    if (header.id.session_id != state.authority_session) {
        return deny(state, FcCommandDenial::NoAuthority);
    }
    FcRequestStep step = accept(stopped(state));
    step.denial = validity_denial(header, observation);
    return step;
}

}  // namespace

std::string_view to_string(FcCommandDenial denial) {
    switch (denial) {
        case FcCommandDenial::None:
            return "";
        case FcCommandDenial::EmptySession:
            return "empty session_id";
        case FcCommandDenial::NoAuthority:
            return "another writer owns FC";
        case FcCommandDenial::ExternalModeChange:
            return "external mode change";
        case FcCommandDenial::FcEpochMismatch:
            return "fc_epoch mismatch";
        case FcCommandDenial::FcDisconnected:
            return "fc disconnected";
        case FcCommandDenial::FcStatusStale:
            return "fc status stale";
        case FcCommandDenial::Expired:
            return "expired";
        case FcCommandDenial::PrestreamNotReady:
            return "prestream not ready";
        case FcCommandDenial::EntryCancelled:
            return "entry cancelled";
        case FcCommandDenial::SendFailed:
            return "send failed";
    }
    return "";
}

FcModeBelief fc_mode_belief(const FcObservation& observation, const FcCommandPolicyConfig& config,
                            std::optional<std::int64_t> not_offboard_after_ns) {
    if (!observation.fc_connected) {
        return FcModeBelief::NotReceived;
    }
    if (!observation.status.has_value() || observation.status->fc_epoch != observation.fc_epoch) {
        return FcModeBelief::Pending;
    }
    const vehicle_api::FlightStatus& status = *observation.status;
    if (observation.now_ns - status.received_monotonic_ns > config.status_max_age_ns) {
        return FcModeBelief::NotReceived;
    }
    if (status.mode == FlightMode::Offboard) {
        return FcModeBelief::Offboard;
    }
    const bool conclusive =
        !not_offboard_after_ns.has_value() || status.received_monotonic_ns > *not_offboard_after_ns;
    return conclusive ? FcModeBelief::NotOffboard : FcModeBelief::Pending;
}

FcCommandStep advance(const FcCommandState& state, const FcObservation& observation,
                      const FcCommandPolicyConfig& config) {
    FcCommandStep result = to(state);
    for (std::int32_t count = 0; count < kMaxChainedTransitions; ++count) {
        const std::optional<FcCommandStep> next =
            std::visit(Advance{observation, config}, result.state);
        if (!next.has_value()) {
            break;
        }
        result.state = next->state;
        merge(result.effects, next->effects);
    }
    return result;
}

FcRequestStep on_offboard_request(const FcCommandState& state, const FcObservation& observation,
                                  const RequestHeader& header,
                                  const FcCommandPolicyConfig& config) {
    return std::visit(OffboardRequest{observation, header, config}, state);
}

FcRequestStep on_hold_request(const FcCommandState& state, const FcObservation& observation,
                              const RequestHeader& header) {
    if (std::holds_alternative<Locked>(state)) {
        return deny(state, FcCommandDenial::ExternalModeChange);
    }
    if (const auto* entering = std::get_if<Entering>(&state)) {
        return hold_by_session(*entering, observation, header);
    }
    if (const auto* following = std::get_if<Following>(&state)) {
        return hold_by_session(*following, observation, header);
    }
    // 정지시킬 대상이 없거나 이미 정지 중
    return deny(state, validity_denial(header, observation));
}

FcRequestStep on_onboard_stop(const FcCommandState& state) {
    if (std::holds_alternative<Locked>(state)) {
        return deny(state, FcCommandDenial::ExternalModeChange);
    }
    if (const auto* entering = std::get_if<Entering>(&state)) {
        return accept(stopped(*entering));
    }
    if (const auto* following = std::get_if<Following>(&state)) {
        return accept(stopped(*following));
    }
    return accept(state);
}

FcCommandStep on_entry_command_answered(const FcCommandState& state, bool accepted) {
    const auto* entering = std::get_if<Entering>(&state);
    if (entering == nullptr) {
        return to(state);
    }
    if (!accepted) {
        return entry_failed(stopped(*entering), "FC 가 거부");
    }
    Entering acknowledged = *entering;
    acknowledged.acknowledged = true;
    return to(acknowledged);
}

FcCommandStep on_entry_command_send_failed(const FcCommandState& state) {
    const auto* entering = std::get_if<Entering>(&state);
    if (entering == nullptr) {
        return to(state);
    }
    return entry_failed(stopped(*entering), "진입 명령 송신 실패");
}

FcRequestStep on_lock_release(const FcCommandState& state, const FcObservation& observation,
                              const FcCommandPolicyConfig& config) {
    if (!std::holds_alternative<Locked>(state)) {
        return accept(state);
    }
    const FcModeBelief mode = fc_mode_belief(observation, config);
    if (mode == FcModeBelief::Pending || mode == FcModeBelief::NotReceived) {
        return deny(state, unknown_mode_denial(observation));
    }
    return accept(Idle{});
}

SetpointChoice select_setpoint(const FcCommandState& state, const FcObservation& observation,
                               const FcCommandPolicyConfig& config) {
    const auto hold = [](const std::optional<HoverTarget>& target) {
        return target.has_value() ? SetpointChoice{SetpointKind::Position, *target}
                                  : SetpointChoice{SetpointKind::ZeroVelocity, {}};
    };
    const auto session_value = [&](std::int64_t after_ns) {
        return has_setpoint_after(observation, after_ns)
                   ? SetpointChoice{SetpointKind::Velocity, {}}
                   : SetpointChoice{SetpointKind::ZeroVelocity, {}};
    };
    if (std::holds_alternative<Idle>(state)) {
        // Offboard 진입 전에 미리 보내는 setpoint. FC 가 Offboard 가 아님을 아는 동안에만 보낸다
        const bool prestream = fc_mode_belief(observation, config) == FcModeBelief::NotOffboard &&
                               observation.setpoint_received_ns.has_value();
        return SetpointChoice{prestream ? SetpointKind::Velocity : SetpointKind::None, {}};
    }
    if (std::holds_alternative<Entering>(state)) {
        return session_value(0);
    }
    if (const auto* following = std::get_if<Following>(&state)) {
        return following->awaiting_first_value ? hold(following->hover_target)
                                               : session_value(following->values_after_ns);
    }
    if (std::holds_alternative<CancellingEntry>(state)) {
        return SetpointChoice{SetpointKind::ZeroVelocity, {}};
    }
    if (const auto* hovering = std::get_if<Hovering>(&state)) {
        return hold(hovering->target);
    }
    return SetpointChoice{};
}

const std::string* authority_session(const FcCommandState& state) {
    if (const auto* entering = std::get_if<Entering>(&state)) {
        return &entering->authority_session;
    }
    if (const auto* following = std::get_if<Following>(&state)) {
        return &following->authority_session;
    }
    return nullptr;
}

}  // namespace argus::mavlink_gateway
