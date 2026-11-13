#include "follow_command_codec.hpp"

#include <cmath>
#include <limits>

#include "argus_msg/follow_command.pb.h"
#include "codec_common.hpp"

namespace argus::vehicle_api_bus {
namespace {

msg::Sensor to_message(vehicle_api::Sensor sensor) {
    switch (sensor) {
        case vehicle_api::Sensor::Unknown:
            return msg::SENSOR_UNKNOWN;
        case vehicle_api::Sensor::Ir:
            return msg::SENSOR_IR;
        case vehicle_api::Sensor::Eo:
            return msg::SENSOR_EO;
    }
    return msg::SENSOR_UNKNOWN;
}

// 이 코드에 정의되지 않은 값(메시지를 만든 프로그램이 더 새 버전인 경우)은 Unknown 으로 바꾼다
vehicle_api::Sensor from_message(msg::Sensor sensor) {
    switch (sensor) {
        case msg::SENSOR_IR:
            return vehicle_api::Sensor::Ir;
        case msg::SENSOR_EO:
            return vehicle_api::Sensor::Eo;
        default:
            return vehicle_api::Sensor::Unknown;
    }
}

msg::StopReason to_message(vehicle_api::StopReason reason) {
    switch (reason) {
        case vehicle_api::StopReason::Operator:
            return msg::STOP_REASON_OPERATOR;
        case vehicle_api::StopReason::LinkLoss:
            return msg::STOP_REASON_LINK_LOSS;
        case vehicle_api::StopReason::Onboard:
            return msg::STOP_REASON_ONBOARD;
    }
    return msg::STOP_REASON_UNSPECIFIED;
}

std::optional<vehicle_api::StopReason> from_message(msg::StopReason reason) {
    switch (reason) {
        case msg::STOP_REASON_OPERATOR:
            return vehicle_api::StopReason::Operator;
        case msg::STOP_REASON_LINK_LOSS:
            return vehicle_api::StopReason::LinkLoss;
        case msg::STOP_REASON_ONBOARD:
            return vehicle_api::StopReason::Onboard;
        default:
            return std::nullopt;
    }
}

msg::FollowOutcome to_message(vehicle_api::FollowOutcome outcome) {
    switch (outcome) {
        case vehicle_api::FollowOutcome::InProgress:
            return msg::FOLLOW_OUTCOME_IN_PROGRESS;
        case vehicle_api::FollowOutcome::Succeeded:
            return msg::FOLLOW_OUTCOME_SUCCEEDED;
        case vehicle_api::FollowOutcome::Rejected:
            return msg::FOLLOW_OUTCOME_REJECTED;
        case vehicle_api::FollowOutcome::Failed:
            return msg::FOLLOW_OUTCOME_FAILED;
    }
    return msg::FOLLOW_OUTCOME_UNSPECIFIED;
}

std::optional<vehicle_api::FollowOutcome> from_message(msg::FollowOutcome outcome) {
    switch (outcome) {
        case msg::FOLLOW_OUTCOME_IN_PROGRESS:
            return vehicle_api::FollowOutcome::InProgress;
        case msg::FOLLOW_OUTCOME_SUCCEEDED:
            return vehicle_api::FollowOutcome::Succeeded;
        case msg::FOLLOW_OUTCOME_REJECTED:
            return vehicle_api::FollowOutcome::Rejected;
        case msg::FOLLOW_OUTCOME_FAILED:
            return vehicle_api::FollowOutcome::Failed;
        default:
            return std::nullopt;
    }
}

double nan_if_empty(const std::optional<double>& value) {
    return value.value_or(std::numeric_limits<double>::quiet_NaN());
}

std::optional<double> empty_if_nan(double value) {
    return std::isnan(value) ? std::nullopt : std::optional<double>{value};
}

// std::variant 의 타입마다 메시지의 action 을 채운다
struct ActionEncoder {
    msg::FollowCommand& message;

    void operator()(const vehicle_api::TrackTarget& target) const {
        msg::FollowCommand::TrackStart& track_start = *message.mutable_track_start();
        track_start.set_sensor(to_message(target.sensor));
        track_start.set_generation(target.generation);
        track_start.set_class_id(target.class_id);
        track_start.set_track_id(target.track_id);
        track_start.set_label(target.label);
    }
    void operator()(vehicle_api::StopReason reason) const {
        message.mutable_stop()->set_reason(to_message(reason));
    }
    void operator()(const vehicle_api::ScanRequest& scan) const {
        msg::FollowCommand::Search& search = *message.mutable_search();
        search.set_yaw_range_rad(nan_if_empty(scan.yaw_range_rad));
        search.set_yaw_rate_rad_s(nan_if_empty(scan.yaw_rate_rad_s));
        search.set_timeout_s(nan_if_empty(scan.timeout_s));
    }
};

}  // namespace

bus::Bytes encode_follow_command(const FollowCommandRequest& request) {
    msg::FollowCommand message;
    to_message(request.header, *message.mutable_header());
    std::visit(ActionEncoder{message}, request.action);
    return to_bytes(message);
}

std::optional<FollowCommandRequest> decode_follow_command(const bus::Bytes& body) {
    msg::FollowCommand message;
    if (!parse(body, message) || !message.has_header()) {
        return std::nullopt;
    }
    FollowCommandRequest request;
    request.header = from_message(message.header());
    switch (message.action_case()) {
        case msg::FollowCommand::kTrackStart: {
            vehicle_api::TrackTarget target;
            target.sensor = from_message(message.track_start().sensor());
            target.generation = message.track_start().generation();
            target.class_id = message.track_start().class_id();
            target.track_id = message.track_start().track_id();
            target.label = message.track_start().label();
            request.action = target;
            break;
        }
        case msg::FollowCommand::kStop: {
            const std::optional<vehicle_api::StopReason> reason =
                from_message(message.stop().reason());
            if (!reason) {
                return std::nullopt;
            }
            request.action = *reason;
            break;
        }
        case msg::FollowCommand::kSearch: {
            vehicle_api::ScanRequest scan;
            scan.yaw_range_rad = empty_if_nan(message.search().yaw_range_rad());
            scan.yaw_rate_rad_s = empty_if_nan(message.search().yaw_rate_rad_s());
            scan.timeout_s = empty_if_nan(message.search().timeout_s());
            request.action = scan;
            break;
        }
        case msg::FollowCommand::ACTION_NOT_SET:
            return std::nullopt;
    }
    return request;
}

bus::Bytes encode_follow_result(const vehicle_api::FollowResult& result) {
    msg::FollowResult message;
    to_message(result.id, *message.mutable_id());
    message.set_outcome(to_message(result.outcome));
    message.set_request_accepted(result.request_accepted);
    message.set_reason(result.reason);
    return to_bytes(message);
}

std::optional<vehicle_api::FollowResult> decode_follow_result(const bus::Bytes& body) {
    msg::FollowResult message;
    if (!parse(body, message) || !message.has_id()) {
        return std::nullopt;
    }
    const std::optional<vehicle_api::FollowOutcome> outcome = from_message(message.outcome());
    if (!outcome) {
        return std::nullopt;
    }
    vehicle_api::FollowResult result;
    result.id = from_message(message.id());
    result.outcome = *outcome;
    result.request_accepted = message.request_accepted();
    result.reason = message.reason();
    return result;
}

}  // namespace argus::vehicle_api_bus
