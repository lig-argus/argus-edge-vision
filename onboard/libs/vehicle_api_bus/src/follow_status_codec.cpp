#include "follow_status_codec.hpp"

#include <optional>
#include <utility>

#include "argus_msg/follow_status.pb.h"
#include "codec_common.hpp"
#include "follow_command_codec.hpp"

namespace argus::vehicle_api_bus {
namespace {

msg::MissionState mission_state_to_message(vehicle_api::MissionState state) {
    using vehicle_api::MissionState;
    switch (state) {
        case MissionState::Idle:
            return msg::MISSION_STATE_IDLE;
        case MissionState::Preparing:
            return msg::MISSION_STATE_PREPARING;
        case MissionState::FollowActive:
            return msg::MISSION_STATE_FOLLOW_ACTIVE;
        case MissionState::HoldTransition:
            return msg::MISSION_STATE_HOLD_TRANSITION;
        case MissionState::HoldWait:
            return msg::MISSION_STATE_HOLD_WAIT;
        case MissionState::ScanActive:
            return msg::MISSION_STATE_SCAN_ACTIVE;
        case MissionState::Manual:
            return msg::MISSION_STATE_MANUAL;
        case MissionState::Fault:
            return msg::MISSION_STATE_FAULT;
        case MissionState::Unknown:
            return msg::MISSION_STATE_UNSPECIFIED;
    }
    return msg::MISSION_STATE_UNSPECIFIED;
}

vehicle_api::MissionState mission_state_from_message(msg::MissionState state) {
    using vehicle_api::MissionState;
    switch (state) {
        case msg::MISSION_STATE_IDLE:
            return MissionState::Idle;
        case msg::MISSION_STATE_PREPARING:
            return MissionState::Preparing;
        case msg::MISSION_STATE_FOLLOW_ACTIVE:
            return MissionState::FollowActive;
        case msg::MISSION_STATE_HOLD_TRANSITION:
            return MissionState::HoldTransition;
        case msg::MISSION_STATE_HOLD_WAIT:
            return MissionState::HoldWait;
        case msg::MISSION_STATE_SCAN_ACTIVE:
            return MissionState::ScanActive;
        case msg::MISSION_STATE_MANUAL:
            return MissionState::Manual;
        case msg::MISSION_STATE_FAULT:
            return MissionState::Fault;
        default:
            return MissionState::Unknown;
    }
}

msg::TargetState target_state_to_message(vehicle_api::TargetState state) {
    using vehicle_api::TargetState;
    switch (state) {
        case TargetState::None:
            return msg::TARGET_STATE_NONE;
        case TargetState::Confirmed:
            return msg::TARGET_STATE_CONFIRMED;
        case TargetState::Predicted:
            return msg::TARGET_STATE_PREDICTED;
        case TargetState::Lost:
            return msg::TARGET_STATE_LOST;
        case TargetState::Unknown:
            return msg::TARGET_STATE_UNSPECIFIED;
    }
    return msg::TARGET_STATE_UNSPECIFIED;
}

vehicle_api::TargetState target_state_from_message(msg::TargetState state) {
    using vehicle_api::TargetState;
    switch (state) {
        case msg::TARGET_STATE_NONE:
            return TargetState::None;
        case msg::TARGET_STATE_CONFIRMED:
            return TargetState::Confirmed;
        case msg::TARGET_STATE_PREDICTED:
            return TargetState::Predicted;
        case msg::TARGET_STATE_LOST:
            return TargetState::Lost;
        default:
            return TargetState::Unknown;
    }
}

msg::FollowAction action_to_message(vehicle_api::FollowAction action) {
    using vehicle_api::FollowAction;
    switch (action) {
        case FollowAction::TrackStart:
            return msg::FOLLOW_ACTION_TRACK_START;
        case FollowAction::Stop:
            return msg::FOLLOW_ACTION_STOP;
        case FollowAction::Search:
            return msg::FOLLOW_ACTION_SEARCH;
        case FollowAction::Unknown:
            return msg::FOLLOW_ACTION_UNSPECIFIED;
    }
    return msg::FOLLOW_ACTION_UNSPECIFIED;
}

vehicle_api::FollowAction action_from_message(msg::FollowAction action) {
    using vehicle_api::FollowAction;
    switch (action) {
        case msg::FOLLOW_ACTION_TRACK_START:
            return FollowAction::TrackStart;
        case msg::FOLLOW_ACTION_STOP:
            return FollowAction::Stop;
        case msg::FOLLOW_ACTION_SEARCH:
            return FollowAction::Search;
        default:
            return FollowAction::Unknown;
    }
}

void target_to_message(const vehicle_api::TrackTarget& target, msg::FollowStatus::Target& message) {
    message.set_generation(target.generation);
    message.set_track_id(target.track_id);
    message.set_class_id(target.class_id);
    message.set_label(target.label);
    message.set_sensor(sensor_to_message(target.sensor));
}

vehicle_api::TrackTarget target_from_message(const msg::FollowStatus::Target& message) {
    vehicle_api::TrackTarget target;
    target.generation = message.generation();
    target.track_id = message.track_id();
    target.class_id = message.class_id();
    target.label = message.label();
    target.sensor = sensor_from_message(message.sensor());
    return target;
}

void command_result_to_message(const vehicle_api::FollowCommandResult& result,
                               msg::FollowStatus::CommandResult& message) {
    message.set_command_id(result.command_id.value);
    message.set_action(action_to_message(result.action));
    message.set_outcome(outcome_to_message(result.outcome));
    message.set_request_accepted(result.request_accepted);
    message.set_reason(result.reason);
    if (result.target) {
        target_to_message(*result.target, *message.mutable_target());
    }
}

// outcome 이 정해진 값이 아니면 nullopt
std::optional<vehicle_api::FollowCommandResult> command_result_from_message(
    const msg::FollowStatus::CommandResult& message) {
    const std::optional<vehicle_api::FollowOutcome> outcome =
        outcome_from_message(message.outcome());
    if (!outcome) {
        return std::nullopt;
    }
    vehicle_api::FollowCommandResult result;
    result.command_id = vehicle_api::CommandId{message.command_id()};
    result.action = action_from_message(message.action());
    result.outcome = *outcome;
    result.request_accepted = message.request_accepted();
    result.reason = message.reason();
    if (message.has_target()) {
        result.target = target_from_message(message.target());
    }
    return result;
}

void scan_to_message(const vehicle_api::ScanProgress& scan, msg::FollowStatus::Scan& message) {
    message.set_yaw_rate_rad_s(scan.yaw_rate_rad_s);
    message.set_sweep_rad(scan.sweep_rad);
    message.set_timeout_s(scan.timeout_s);
    message.set_progress_deg(scan.progress_deg);
    if (scan.remaining_s) {
        message.set_remaining_s(*scan.remaining_s);
    }
}

vehicle_api::ScanProgress scan_from_message(const msg::FollowStatus::Scan& message) {
    vehicle_api::ScanProgress scan;
    scan.yaw_rate_rad_s = message.yaw_rate_rad_s();
    scan.sweep_rad = message.sweep_rad();
    scan.timeout_s = message.timeout_s();
    scan.progress_deg = message.progress_deg();
    if (message.has_remaining_s()) {
        scan.remaining_s = message.remaining_s();
    }
    return scan;
}

}  // namespace

bus_zmq::Bytes encode_follow_status(const vehicle_api::FollowStatus& status, std::uint64_t seq) {
    msg::FollowStatus message;
    message.set_seq(seq);
    message.set_session_id(status.session_id);
    message.set_controller_session(status.controller_session);
    message.set_client_session(status.client_session);
    message.set_onboard_session(status.onboard_session);
    message.set_uid(status.uid);
    message.set_fc_epoch(status.fc_epoch);
    message.set_received_monotonic_ns(status.received_monotonic_ns);
    message.set_mission_state(mission_state_to_message(status.mission_state));
    message.set_target_state(target_state_to_message(status.target_state));
    if (status.selected) {
        target_to_message(*status.selected, *message.mutable_selected());
    }
    for (const vehicle_api::FollowCommandResult& result : status.command_results) {
        command_result_to_message(result, *message.add_command_results());
    }
    message.set_command_epoch(status.command_epoch);
    message.set_command_sent(status.command_sent);
    if (status.scan) {
        scan_to_message(*status.scan, *message.mutable_scan());
    }
    return to_bytes(message);
}

std::optional<Decoded<vehicle_api::FollowStatus>> decode_follow_status(const bus_zmq::Bytes& body) {
    msg::FollowStatus message;
    if (!parse(body, message)) {
        return std::nullopt;
    }
    Decoded<vehicle_api::FollowStatus> decoded;
    decoded.seq = message.seq();
    vehicle_api::FollowStatus& status = decoded.value;
    status.session_id = message.session_id();
    status.controller_session = message.controller_session();
    status.client_session = message.client_session();
    status.onboard_session = message.onboard_session();
    status.uid = message.uid();
    status.fc_epoch = message.fc_epoch();
    status.received_monotonic_ns = message.received_monotonic_ns();
    status.mission_state = mission_state_from_message(message.mission_state());
    status.target_state = target_state_from_message(message.target_state());
    if (message.has_selected()) {
        status.selected = target_from_message(message.selected());
    }
    for (const msg::FollowStatus::CommandResult& entry : message.command_results()) {
        if (std::optional<vehicle_api::FollowCommandResult> result =
                command_result_from_message(entry)) {
            status.command_results.push_back(std::move(*result));
        }
    }
    status.command_epoch = message.command_epoch();
    status.command_sent = message.command_sent();
    if (message.has_scan()) {
        status.scan = scan_from_message(message.scan());
    }
    return decoded;
}

}  // namespace argus::vehicle_api_bus
