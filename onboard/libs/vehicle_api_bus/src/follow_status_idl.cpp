#include "follow_status_idl.hpp"

#include <optional>
#include <utility>

#include "follow_command_idl.hpp"

namespace argus::vehicle_api_bus {
namespace {

idl::MissionState mission_state_to_idl(vehicle_api::MissionState state) {
    using vehicle_api::MissionState;
    switch (state) {
        case MissionState::Idle:
            return idl::MissionState::MISSION_STATE_IDLE;
        case MissionState::Preparing:
            return idl::MissionState::MISSION_STATE_PREPARING;
        case MissionState::FollowActive:
            return idl::MissionState::MISSION_STATE_FOLLOW_ACTIVE;
        case MissionState::HoldTransition:
            return idl::MissionState::MISSION_STATE_HOLD_TRANSITION;
        case MissionState::HoldWait:
            return idl::MissionState::MISSION_STATE_HOLD_WAIT;
        case MissionState::ScanActive:
            return idl::MissionState::MISSION_STATE_SCAN_ACTIVE;
        case MissionState::Manual:
            return idl::MissionState::MISSION_STATE_MANUAL;
        case MissionState::Fault:
            return idl::MissionState::MISSION_STATE_FAULT;
        case MissionState::Unknown:
            return idl::MissionState::MISSION_STATE_UNSPECIFIED;
    }
    return idl::MissionState::MISSION_STATE_UNSPECIFIED;
}

vehicle_api::MissionState mission_state_from_idl(idl::MissionState state) {
    using vehicle_api::MissionState;
    switch (state) {
        case idl::MissionState::MISSION_STATE_IDLE:
            return MissionState::Idle;
        case idl::MissionState::MISSION_STATE_PREPARING:
            return MissionState::Preparing;
        case idl::MissionState::MISSION_STATE_FOLLOW_ACTIVE:
            return MissionState::FollowActive;
        case idl::MissionState::MISSION_STATE_HOLD_TRANSITION:
            return MissionState::HoldTransition;
        case idl::MissionState::MISSION_STATE_HOLD_WAIT:
            return MissionState::HoldWait;
        case idl::MissionState::MISSION_STATE_SCAN_ACTIVE:
            return MissionState::ScanActive;
        case idl::MissionState::MISSION_STATE_MANUAL:
            return MissionState::Manual;
        case idl::MissionState::MISSION_STATE_FAULT:
            return MissionState::Fault;
        case idl::MissionState::MISSION_STATE_UNSPECIFIED:
            return MissionState::Unknown;
    }
    return MissionState::Unknown;
}

idl::TargetState target_state_to_idl(vehicle_api::TargetState state) {
    using vehicle_api::TargetState;
    switch (state) {
        case TargetState::None:
            return idl::TargetState::TARGET_STATE_NONE;
        case TargetState::Confirmed:
            return idl::TargetState::TARGET_STATE_CONFIRMED;
        case TargetState::Predicted:
            return idl::TargetState::TARGET_STATE_PREDICTED;
        case TargetState::Lost:
            return idl::TargetState::TARGET_STATE_LOST;
        case TargetState::Unknown:
            return idl::TargetState::TARGET_STATE_UNSPECIFIED;
    }
    return idl::TargetState::TARGET_STATE_UNSPECIFIED;
}

vehicle_api::TargetState target_state_from_idl(idl::TargetState state) {
    using vehicle_api::TargetState;
    switch (state) {
        case idl::TargetState::TARGET_STATE_NONE:
            return TargetState::None;
        case idl::TargetState::TARGET_STATE_CONFIRMED:
            return TargetState::Confirmed;
        case idl::TargetState::TARGET_STATE_PREDICTED:
            return TargetState::Predicted;
        case idl::TargetState::TARGET_STATE_LOST:
            return TargetState::Lost;
        case idl::TargetState::TARGET_STATE_UNSPECIFIED:
            return TargetState::Unknown;
    }
    return TargetState::Unknown;
}

idl::FollowAction action_to_idl(vehicle_api::FollowAction action) {
    using vehicle_api::FollowAction;
    switch (action) {
        case FollowAction::TrackStart:
            return idl::FollowAction::FOLLOW_ACTION_TRACK_START;
        case FollowAction::Stop:
            return idl::FollowAction::FOLLOW_ACTION_STOP;
        case FollowAction::Search:
            return idl::FollowAction::FOLLOW_ACTION_SEARCH;
        case FollowAction::Unknown:
            return idl::FollowAction::FOLLOW_ACTION_UNSPECIFIED;
    }
    return idl::FollowAction::FOLLOW_ACTION_UNSPECIFIED;
}

vehicle_api::FollowAction action_from_idl(idl::FollowAction action) {
    using vehicle_api::FollowAction;
    switch (action) {
        case idl::FollowAction::FOLLOW_ACTION_TRACK_START:
            return FollowAction::TrackStart;
        case idl::FollowAction::FOLLOW_ACTION_STOP:
            return FollowAction::Stop;
        case idl::FollowAction::FOLLOW_ACTION_SEARCH:
            return FollowAction::Search;
        case idl::FollowAction::FOLLOW_ACTION_UNSPECIFIED:
            return FollowAction::Unknown;
    }
    return FollowAction::Unknown;
}

idl::FollowStatus_Target target_to_idl(const vehicle_api::TrackTarget& target) {
    idl::FollowStatus_Target sample;
    sample.generation(target.generation);
    sample.track_id(target.track_id);
    sample.class_id(target.class_id);
    sample.label(target.label);
    sample.sensor(sensor_to_idl(target.sensor));
    return sample;
}

vehicle_api::TrackTarget target_from_idl(const idl::FollowStatus_Target& sample) {
    vehicle_api::TrackTarget target;
    target.generation = sample.generation();
    target.track_id = sample.track_id();
    target.class_id = sample.class_id();
    target.label = sample.label();
    target.sensor = sensor_from_idl(sample.sensor());
    return target;
}

idl::FollowStatus_CommandResult command_result_to_idl(
    const vehicle_api::FollowCommandResult& result) {
    idl::FollowStatus_CommandResult sample;
    sample.command_id(result.command_id.value);
    sample.action(action_to_idl(result.action));
    sample.outcome(outcome_to_idl(result.outcome));
    sample.request_accepted(result.request_accepted);
    sample.reason(result.reason);
    if (result.target) {
        sample.target(target_to_idl(*result.target));
    }
    return sample;
}

// outcome 이 정해진 값이 아니면 nullopt
std::optional<vehicle_api::FollowCommandResult> command_result_from_idl(
    const idl::FollowStatus_CommandResult& sample) {
    const std::optional<vehicle_api::FollowOutcome> outcome = outcome_from_idl(sample.outcome());
    if (!outcome) {
        return std::nullopt;
    }
    vehicle_api::FollowCommandResult result;
    result.command_id = vehicle_api::CommandId{sample.command_id()};
    result.action = action_from_idl(sample.action());
    result.outcome = *outcome;
    result.request_accepted = sample.request_accepted();
    result.reason = sample.reason();
    if (sample.target().has_value()) {
        result.target = target_from_idl(*sample.target());
    }
    return result;
}

idl::FollowStatus_Scan scan_to_idl(const vehicle_api::ScanProgress& scan) {
    idl::FollowStatus_Scan sample;
    sample.yaw_rate_rad_s(scan.yaw_rate_rad_s);
    sample.sweep_rad(scan.sweep_rad);
    sample.timeout_s(scan.timeout_s);
    sample.progress_deg(scan.progress_deg);
    if (scan.remaining_s) {
        sample.remaining_s(*scan.remaining_s);
    }
    return sample;
}

vehicle_api::ScanProgress scan_from_idl(const idl::FollowStatus_Scan& sample) {
    vehicle_api::ScanProgress scan;
    scan.yaw_rate_rad_s = sample.yaw_rate_rad_s();
    scan.sweep_rad = sample.sweep_rad();
    scan.timeout_s = sample.timeout_s();
    scan.progress_deg = sample.progress_deg();
    if (sample.remaining_s().has_value()) {
        scan.remaining_s = *sample.remaining_s();
    }
    return scan;
}

}  // namespace

void to_idl_follow_status(const vehicle_api::FollowStatus& status, idl::FollowStatus& sample) {
    sample.session_id(status.session_id);
    sample.controller_session(status.controller_session);
    sample.client_session(status.client_session);
    sample.onboard_session(status.onboard_session);
    sample.uid(status.uid);
    sample.fc_epoch(status.fc_epoch);
    sample.received_monotonic_ns(status.received_monotonic_ns);
    sample.mission_state(mission_state_to_idl(status.mission_state));
    sample.target_state(target_state_to_idl(status.target_state));
    sample.selected().reset();
    if (status.selected) {
        sample.selected(target_to_idl(*status.selected));
    }
    sample.command_results().clear();
    for (const vehicle_api::FollowCommandResult& result : status.command_results) {
        sample.command_results().push_back(command_result_to_idl(result));
    }
    sample.command_epoch(status.command_epoch);
    sample.command_sent(status.command_sent);
    sample.scan().reset();
    if (status.scan) {
        sample.scan(scan_to_idl(*status.scan));
    }
}

bool from_idl_follow_status(const idl::FollowStatus& sample, vehicle_api::FollowStatus& status) {
    status = vehicle_api::FollowStatus{};
    status.session_id = sample.session_id();
    status.controller_session = sample.controller_session();
    status.client_session = sample.client_session();
    status.onboard_session = sample.onboard_session();
    status.uid = sample.uid();
    status.fc_epoch = sample.fc_epoch();
    status.received_monotonic_ns = sample.received_monotonic_ns();
    status.mission_state = mission_state_from_idl(sample.mission_state());
    status.target_state = target_state_from_idl(sample.target_state());
    if (sample.selected().has_value()) {
        status.selected = target_from_idl(*sample.selected());
    }
    for (const idl::FollowStatus_CommandResult& entry : sample.command_results()) {
        if (std::optional<vehicle_api::FollowCommandResult> result =
                command_result_from_idl(entry)) {
            status.command_results.push_back(std::move(*result));
        }
    }
    status.command_epoch = sample.command_epoch();
    status.command_sent = sample.command_sent();
    if (sample.scan().has_value()) {
        status.scan = scan_from_idl(*sample.scan());
    }
    return true;
}

}  // namespace argus::vehicle_api_bus
