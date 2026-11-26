#include "argus/mavlink_gateway/fc_control.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "argus/mavlink_gateway/setpoint_validation.hpp"
#include "argus/vehicle_api/types.hpp"
#include "log.hpp"
#include "mode_command_encoding.hpp"
#include "setpoint_encoding.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::FlightMode;
using vehicle_api::FlightModeReply;
using vehicle_api::OffboardExitCause;
using vehicle_api::RequestHeader;

FlightModeReply reply_to(const RequestHeader& header, FcCommandDenial denial) {
    FlightModeReply reply;
    reply.id = header.id;
    reply.accepted = denial == FcCommandDenial::None;
    reply.reason = std::string(to_string(denial));
    return reply;
}

// 조종사가 조종기로 직접 조종하는 모드
bool is_pilot_mode(FlightMode mode) {
    return mode == FlightMode::Manual || mode == FlightMode::Stabilized ||
           mode == FlightMode::Altitude || mode == FlightMode::Position;
}

// FC 가 failsafe 로 선택하는 모드. 조종사나 통제기가 같은 모드로 바꾼 경우와 구분할 수 없다
bool is_failsafe_mode(FlightMode mode) {
    return mode == FlightMode::Hold || mode == FlightMode::Return || mode == FlightMode::Land;
}

OffboardExitCause exit_cause_of(FlightMode mode) {
    if (is_pilot_mode(mode)) {
        return OffboardExitCause::PilotSwitch;
    }
    return is_failsafe_mode(mode) ? OffboardExitCause::Failsafe : OffboardExitCause::Unknown;
}

}  // namespace

FcControl::FcControl(IMavlinkConnection& connection, const IFcStateSource& fc_state,
                     const core::IClock& clock, FcControlConfig config)
    : connection_(connection),
      fc_state_(fc_state),
      clock_(clock),
      config_(config),
      inbox_(config.setpoint_max_valid_ns),
      stream_(config.stream_max_gap_ns) {
    ack_subscription_ =
        connection_.subscribe(MAVLINK_MSG_ID_COMMAND_ACK, [this](const mavlink_message_t& message) {
            if (is_from_fc(message)) {
                on_command_ack(message);
            }
        });
}

FlightModeReply FcControl::request_offboard(const RequestHeader& header) {
    const FcObservation observation = observe();
    sync(observation);
    const FcRequestStep step = on_offboard_request(state_, observation, header, config_.policy);
    apply(step.state, step.effects, observation);
    if (step.effects.send_entry_command && !send_entry_command(observation)) {
        return reply_to(header, FcCommandDenial::SendFailed);
    }
    return reply_to(header, step.denial);
}

FlightModeReply FcControl::request_hold(const RequestHeader& header) {
    const FcObservation observation = observe();
    sync(observation);
    const FcRequestStep step = on_hold_request(state_, observation, header);
    apply(step.state, step.effects, observation);
    return reply_to(header, step.denial);
}

void FcControl::send(const vehicle_api::VelocitySetpoint& setpoint) {
    inbox_.put(setpoint, clock_.now_monotonic_ns());
}

bool FcControl::stop_from_onboard() {
    const FcObservation observation = observe();
    sync(observation);
    const FcRequestStep step = on_onboard_stop(state_);
    apply(step.state, step.effects, observation);
    return step.denial == FcCommandDenial::None;
}

bool FcControl::release_lock() {
    const FcObservation observation = observe();
    sync(observation);
    const FcRequestStep step = on_lock_release(state_, observation, config_.policy);
    apply(step.state, step.effects, observation);
    return step.denial == FcCommandDenial::None;
}

bool FcControl::sending_setpoints() const { return last_setpoint_kind_ != SetpointKind::None; }

bool FcControl::entering() const { return std::holds_alternative<Entering>(state_); }

core::Subscription FcControl::subscribe_mode_events(ModeEventHandler handler) {
    return mode_event_handlers_.subscribe(std::move(handler));
}

void FcControl::tick() {
    const FcObservation observation = observe();
    sync(observation);
    send_setpoint(select_setpoint(state_, observation, config_.policy), observation.now_ns);
}

FcObservation FcControl::observe() const {
    const vehicle_api::VehicleState vehicle = fc_state_.latest();
    FcObservation observation;
    observation.now_ns = clock_.now_monotonic_ns();
    observation.fc_epoch = fc_state_.fc_epoch();
    observation.fc_connected = fc_state_.fc_connected();
    observation.status = vehicle.status;
    observation.fresh_pose = fresh_pose(vehicle, observation.now_ns);
    observation.setpoint_received_ns = inbox_.valid_received_ns(observation.now_ns);
    observation.streaming_for_ns = stream_.streaming_for_ns(observation.now_ns);
    return observation;
}

std::optional<HoverTarget> FcControl::fresh_pose(const vehicle_api::VehicleState& vehicle,
                                                 std::int64_t now_ns) const {
    if (!vehicle.attitude.has_value() || !vehicle.local.has_value()) {
        return std::nullopt;
    }
    const vehicle_api::Attitude& attitude = *vehicle.attitude;
    const vehicle_api::LocalPosition& local = *vehicle.local;
    const auto fresh = [&](std::int64_t received_ns) {
        return now_ns - received_ns <= config_.pose_max_age_ns;
    };
    if (!fresh(attitude.received_monotonic_ns) || !fresh(local.received_monotonic_ns)) {
        return std::nullopt;
    }
    if (!fits_float(local.north_m) || !fits_float(local.east_m) || !fits_float(local.down_m) ||
        !fits_float(attitude.yaw_rad)) {
        return std::nullopt;
    }
    return HoverTarget{local.north_m, local.east_m, local.down_m, attitude.yaw_rad};
}

void FcControl::sync(const FcObservation& observation) {
    const FcCommandStep step = advance(state_, observation, config_.policy);
    apply(step.state, step.effects, observation);
    report_mode_change(observation);
}

void FcControl::apply(const FcCommandState& state, const FcCommandEffects& effects,
                      const FcObservation& observation) {
    state_ = state;
    if (effects.entry_failure != nullptr) {
        detail::log_error(std::string("Offboard 진입 실패 (") + effects.entry_failure + ")");
    }
    if (effects.setpoint_timed_out) {
        last_setpoint_timeout_ns_ = observation.now_ns;
    }
    const std::int64_t received_ns = observation.status.has_value()
                                         ? observation.status->received_monotonic_ns
                                         : observation.now_ns;
    if (effects.entered_command_id.has_value()) {
        emit(FlightMode::Offboard, received_ns, effects.entered_command_id,
             OffboardExitCause::None);
    }
    if (effects.exited_to.has_value()) {
        emit(*effects.exited_to, received_ns, std::nullopt, exit_cause_of(*effects.exited_to));
    }
}

bool FcControl::send_entry_command(const FcObservation& observation) {
    const std::optional<mavlink_message_t> command = detail::encode_offboard_command();
    if (command.has_value() && connection_.send(*command)) {
        return true;
    }
    const FcCommandStep step = on_entry_command_send_failed(state_);
    apply(step.state, step.effects, observation);
    return false;
}

void FcControl::send_setpoint(const SetpointChoice& choice, std::int64_t now_ns) {
    // 송신에 실패한 경우에도 상태를 바꾸지 않는다. 다음 주기에 다시 송신한다
    offboard_hover_ =
        choice.kind == SetpointKind::Position && std::holds_alternative<Hovering>(state_);
    last_setpoint_kind_ = choice.kind;
    switch (choice.kind) {
        case SetpointKind::None:
            stream_.reset();
            return;
        case SetpointKind::Velocity:
            if (connection_.send(detail::encode_velocity_setpoint(inbox_.setpoint()))) {
                stream_.record_sent(now_ns);
            }
            return;
        case SetpointKind::ZeroVelocity:
            stream_.reset();
            connection_.send(detail::encode_velocity_setpoint(vehicle_api::VelocitySetpoint{}));
            return;
        case SetpointKind::Position:
            stream_.reset();
            connection_.send(
                detail::encode_position_setpoint(choice.target.north_m, choice.target.east_m,
                                                 choice.target.down_m, choice.target.yaw_rad));
            return;
    }
}

void FcControl::report_mode_change(const FcObservation& observation) {
    if (!observation.status.has_value() || observation.status->fc_epoch != observation.fc_epoch) {
        return;
    }
    const FlightMode mode = observation.status->mode;
    const bool already_reported = reported_mode_.has_value() && *reported_mode_ == mode &&
                                  reported_fc_epoch_ == observation.fc_epoch;
    // 진입의 확인을 기다리는 중에 본 Offboard 로의 변경은 그 요청의 결과가 정해진 때에 전달한다
    const bool deferred = mode == FlightMode::Offboard && std::holds_alternative<Entering>(state_);
    if (already_reported || deferred) {
        return;
    }
    emit(mode, observation.status->received_monotonic_ns, std::nullopt, OffboardExitCause::None);
}

void FcControl::emit(FlightMode mode, std::int64_t received_ns,
                     std::optional<vehicle_api::CommandId> command_id,
                     OffboardExitCause exit_cause) {
    reported_mode_ = mode;
    reported_fc_epoch_ = fc_state_.fc_epoch();
    vehicle_api::FlightModeEvent event;
    event.mode = mode;
    event.received_monotonic_ns = received_ns;
    event.request_command_id = command_id;
    event.exit_cause = exit_cause;
    mode_event_handlers_.notify(event);
}

void FcControl::on_command_ack(const mavlink_message_t& message) {
    const std::optional<bool> accepted = detail::decode_mode_command_ack(message);
    if (!accepted.has_value()) {
        return;
    }
    const FcCommandStep step = on_entry_command_answered(state_, *accepted);
    apply(step.state, step.effects, observe());
}

}  // namespace argus::mavlink_gateway
