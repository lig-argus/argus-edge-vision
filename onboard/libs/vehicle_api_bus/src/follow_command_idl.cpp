#include "follow_command_idl.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <variant>

#include "idl_common.hpp"

namespace argus::vehicle_api_bus {
namespace {

idl::Sensor to_idl(vehicle_api::Sensor sensor) {
    switch (sensor) {
        case vehicle_api::Sensor::Unknown:
            return idl::Sensor::SENSOR_UNKNOWN;
        case vehicle_api::Sensor::Ir:
            return idl::Sensor::SENSOR_IR;
        case vehicle_api::Sensor::Eo:
            return idl::Sensor::SENSOR_EO;
    }
    return idl::Sensor::SENSOR_UNKNOWN;
}

// 이 코드에 정의되지 않은 값(표본을 만든 프로그램이 더 새 버전인 경우)은 Unknown 으로 바꾼다
vehicle_api::Sensor from_idl(idl::Sensor sensor) {
    switch (sensor) {
        case idl::Sensor::SENSOR_UNKNOWN:
            return vehicle_api::Sensor::Unknown;
        case idl::Sensor::SENSOR_IR:
            return vehicle_api::Sensor::Ir;
        case idl::Sensor::SENSOR_EO:
            return vehicle_api::Sensor::Eo;
    }
    return vehicle_api::Sensor::Unknown;
}

idl::StopReason to_idl(vehicle_api::StopReason reason) {
    switch (reason) {
        case vehicle_api::StopReason::Operator:
            return idl::StopReason::STOP_REASON_OPERATOR;
        case vehicle_api::StopReason::LinkLoss:
            return idl::StopReason::STOP_REASON_LINK_LOSS;
        case vehicle_api::StopReason::Onboard:
            return idl::StopReason::STOP_REASON_ONBOARD;
    }
    return idl::StopReason::STOP_REASON_UNSPECIFIED;
}

std::optional<vehicle_api::StopReason> from_idl(idl::StopReason reason) {
    switch (reason) {
        case idl::StopReason::STOP_REASON_OPERATOR:
            return vehicle_api::StopReason::Operator;
        case idl::StopReason::STOP_REASON_LINK_LOSS:
            return vehicle_api::StopReason::LinkLoss;
        case idl::StopReason::STOP_REASON_ONBOARD:
            return vehicle_api::StopReason::Onboard;
        case idl::StopReason::STOP_REASON_UNSPECIFIED:
            return std::nullopt;
    }
    return std::nullopt;
}

idl::FollowOutcome to_idl(vehicle_api::FollowOutcome outcome) {
    switch (outcome) {
        case vehicle_api::FollowOutcome::InProgress:
            return idl::FollowOutcome::FOLLOW_OUTCOME_IN_PROGRESS;
        case vehicle_api::FollowOutcome::Succeeded:
            return idl::FollowOutcome::FOLLOW_OUTCOME_SUCCEEDED;
        case vehicle_api::FollowOutcome::Rejected:
            return idl::FollowOutcome::FOLLOW_OUTCOME_REJECTED;
        case vehicle_api::FollowOutcome::Failed:
            return idl::FollowOutcome::FOLLOW_OUTCOME_FAILED;
    }
    return idl::FollowOutcome::FOLLOW_OUTCOME_UNSPECIFIED;
}

std::optional<vehicle_api::FollowOutcome> from_idl(idl::FollowOutcome outcome) {
    switch (outcome) {
        case idl::FollowOutcome::FOLLOW_OUTCOME_IN_PROGRESS:
            return vehicle_api::FollowOutcome::InProgress;
        case idl::FollowOutcome::FOLLOW_OUTCOME_SUCCEEDED:
            return vehicle_api::FollowOutcome::Succeeded;
        case idl::FollowOutcome::FOLLOW_OUTCOME_REJECTED:
            return vehicle_api::FollowOutcome::Rejected;
        case idl::FollowOutcome::FOLLOW_OUTCOME_FAILED:
            return vehicle_api::FollowOutcome::Failed;
        case idl::FollowOutcome::FOLLOW_OUTCOME_UNSPECIFIED:
            return std::nullopt;
    }
    return std::nullopt;
}

double nan_if_empty(const std::optional<double>& value) {
    return value.value_or(std::numeric_limits<double>::quiet_NaN());
}

std::optional<double> empty_if_nan(double value) {
    return std::isnan(value) ? std::nullopt : std::optional<double>{value};
}

// std::variant 의 타입마다 표본의 action(union)을 채운다
struct ActionToIdl {
    idl::FollowCommand_Action& action;

    void operator()(const vehicle_api::TrackTarget& target) const {
        idl::FollowCommand_TrackStart track_start;
        track_start.sensor(to_idl(target.sensor));
        track_start.generation(target.generation);
        track_start.class_id(target.class_id);
        track_start.track_id(target.track_id);
        track_start.label(target.label);
        action.track_start(track_start);
    }
    void operator()(vehicle_api::StopReason reason) const {
        idl::FollowCommand_Stop stop;
        stop.reason(to_idl(reason));
        action.stop(stop);
    }
    void operator()(const vehicle_api::ScanRequest& scan) const {
        idl::FollowCommand_Search search;
        search.yaw_range_rad(nan_if_empty(scan.yaw_range_rad));
        search.yaw_rate_rad_s(nan_if_empty(scan.yaw_rate_rad_s));
        search.timeout_s(nan_if_empty(scan.timeout_s));
        action.search(search);
    }
};

}  // namespace

void to_idl_follow_command(const FollowCommandRequest& request, idl::FollowCommand& sample) {
    to_idl(request.header, sample.header());
    std::visit(ActionToIdl{sample.action()}, request.action);
}

bool from_idl_follow_command(const idl::FollowCommand& sample, FollowCommandRequest& request) {
    const idl::FollowCommand_Action& action = sample.action();
    switch (action._d()) {
        case idl::FollowCommand_ActionCase::FOLLOW_COMMAND_ACTION_CASE_TRACK_START: {
            vehicle_api::TrackTarget target;
            target.sensor = from_idl(action.track_start().sensor());
            target.generation = action.track_start().generation();
            target.class_id = action.track_start().class_id();
            target.track_id = action.track_start().track_id();
            target.label = action.track_start().label();
            request.action = target;
            break;
        }
        case idl::FollowCommand_ActionCase::FOLLOW_COMMAND_ACTION_CASE_STOP: {
            const std::optional<vehicle_api::StopReason> reason = from_idl(action.stop().reason());
            if (!reason) {
                return false;
            }
            request.action = *reason;
            break;
        }
        case idl::FollowCommand_ActionCase::FOLLOW_COMMAND_ACTION_CASE_SEARCH: {
            vehicle_api::ScanRequest scan;
            scan.yaw_range_rad = empty_if_nan(action.search().yaw_range_rad());
            scan.yaw_rate_rad_s = empty_if_nan(action.search().yaw_rate_rad_s());
            scan.timeout_s = empty_if_nan(action.search().timeout_s());
            request.action = scan;
            break;
        }
        default:
            // 이 코드에 정의되지 않은 판별 값
            return false;
    }
    request.header = from_idl(sample.header());
    return true;
}

void to_idl_follow_result(const vehicle_api::FollowResult& result, idl::FollowResult& sample) {
    to_idl(result.id, sample.id());
    sample.outcome(to_idl(result.outcome));
    sample.request_accepted(result.request_accepted);
    sample.reason(result.reason);
}

bool from_idl_follow_result(const idl::FollowResult& sample, vehicle_api::FollowResult& result) {
    const std::optional<vehicle_api::FollowOutcome> outcome = from_idl(sample.outcome());
    if (!outcome) {
        return false;
    }
    result.id = from_idl(sample.id());
    result.outcome = *outcome;
    result.request_accepted = sample.request_accepted();
    result.reason = sample.reason();
    return true;
}

}  // namespace argus::vehicle_api_bus
