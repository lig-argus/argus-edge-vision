#include "argus/mavlink_gateway/gcs_forward_policy.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include "argus/mavlink_gateway/px4_flight_mode.hpp"

namespace argus::mavlink_gateway {
namespace {

// STATUSTEXT 로 보내는 거부 이유 (icd 4.1)
constexpr const char* kNotInControlText = "제어권 없음";
constexpr const char* kUnsupportedText = "지원하지 않는 명령";
constexpr const char* kForceArmText = "강제 시동 거부";
constexpr const char* kModeNotAllowedText = "허용하지 않는 모드";
constexpr const char* kLandingLockText = "착륙 잠금";
constexpr const char* kPilotText = "조종사 개입 중";
constexpr const char* kSetpointText = "setpoint 송신 중";
constexpr const char* kFcStatusUnknownText = "FC 상태 모름";
constexpr const char* kArmedText = "시동 켜짐";
constexpr const char* kFollowingText = "추종 중: 추종 정지 사용";

constexpr float kMaxModeValue = 255.0F;

ForwardDecision forward() {
    return ForwardDecision{ForwardAction::Forward, MAV_RESULT_ACCEPTED, {}};
}

ForwardDecision drop() { return ForwardDecision{ForwardAction::Drop, MAV_RESULT_DENIED, {}}; }

ForwardDecision reply(MAV_RESULT result, const char* reason) {
    return ForwardDecision{ForwardAction::Reply, result, reason};
}

ForwardDecision rejected(const char* reason) {
    return reply(MAV_RESULT_TEMPORARILY_REJECTED, reason);
}

ForwardDecision stop() { return ForwardDecision{ForwardAction::Stop, MAV_RESULT_ACCEPTED, {}}; }

std::optional<std::uint32_t> mode_value(float value) {
    if (!std::isfinite(value) || value < 0.0F || value > kMaxModeValue ||
        std::floor(value) != value) {
        return std::nullopt;
    }
    return static_cast<std::uint32_t>(value);
}

// 통제기가 DO_SET_MODE 로 바꿀 수 있는 모드 (icd 5.2). Hold 는 따로 처리한다
bool is_pilot_mode(vehicle_api::FlightMode mode) {
    return mode == vehicle_api::FlightMode::Position || mode == vehicle_api::FlightMode::Altitude ||
           mode == vehicle_api::FlightMode::Stabilized;
}

bool is_listed_command(std::uint16_t command) {
    switch (command) {
        case MAV_CMD_COMPONENT_ARM_DISARM:
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        case MAV_CMD_NAV_LAND:
        case MAV_CMD_DO_SET_MODE:
        case MAV_CMD_DO_SET_HOME:
        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            return true;
        default:
            return false;
    }
}

// 값 조건. 맞지 않으면 DENIED 의 이유, 맞으면 nullptr
const char* value_violation(const GcsMessageRequest& request) {
    if (!is_listed_command(request.command)) {
        return kUnsupportedText;
    }
    if (request.command == MAV_CMD_COMPONENT_ARM_DISARM && request.param2 != 0.0F) {
        return kForceArmText;  // 0 이 아닌 param2 는 강제 시동, 강제 시동 끔 (NaN 포함)
    }
    if (request.command == MAV_CMD_DO_SET_MODE) {
        const std::optional<vehicle_api::FlightMode> mode =
            set_mode_request(request.param1, request.param2, request.param3);
        if (!mode.has_value() ||
            (!is_pilot_mode(*mode) && *mode != vehicle_api::FlightMode::Hold)) {
            return kModeNotAllowedText;
        }
    }
    return nullptr;
}

// HOLD: mavlink_gateway 가 FC 를 Offboard 로 제어하지 않으면 FC 로 전달한다 (GPS 가 없으면 FC 가
// 거부한다). 제어 중이면 FC 로 보내지 않는다. 추종, 스캔, 진행 중인 명령이 있으면 정지 절차를
// TRACK_STOP 한 곳에 두기 위해 거부하고, 그 밖에는 mavlink_gateway 가 정지시킨다
ForwardDecision decide_hold(const ForwardObservation& observation) {
    if (!observation.sending_setpoints) {
        return forward();
    }
    if (observation.follow_in_progress || observation.command_in_progress) {
        return rejected(kFollowingText);
    }
    return stop();
}

// 시동이 꺼져 있어야 하는 명령 (지상 판단. in_air 를 알 수 없어 armed 로 판단한다)
ForwardDecision require_disarmed(const ForwardObservation& observation) {
    if (!observation.fc_status_known) {
        return rejected(kFcStatusUnknownText);
    }
    if (observation.armed) {
        return rejected(kArmedText);
    }
    return forward();
}

ForwardDecision decide_command(const ForwardObservation& observation,
                               const GcsMessageRequest& request) {
    if (!observation.from_controller) {
        return rejected(kNotInControlText);
    }
    if (request.command == MAV_CMD_NAV_LAND) {
        return forward();
    }
    if (const char* violation = value_violation(request); violation != nullptr) {
        return reply(MAV_RESULT_DENIED, violation);
    }
    if (observation.landing_locked) {
        return rejected(kLandingLockText);
    }
    if (observation.pilot_in_control) {
        return rejected(kPilotText);
    }
    switch (request.command) {
        case MAV_CMD_COMPONENT_ARM_DISARM:
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            return observation.sending_setpoints ? rejected(kSetpointText) : forward();
        case MAV_CMD_DO_SET_MODE: {
            const std::optional<vehicle_api::FlightMode> mode =
                set_mode_request(request.param1, request.param2, request.param3);
            return mode == vehicle_api::FlightMode::Hold ? decide_hold(observation) : forward();
        }
        case MAV_CMD_DO_SET_HOME:
        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            return require_disarmed(observation);
        default:
            return reply(MAV_RESULT_DENIED, kUnsupportedText);
    }
}

bool manual_control_allowed(const ForwardObservation& observation) {
    return observation.fc_status_known && is_pilot_mode(observation.fc_mode) &&
           !observation.sending_setpoints && !observation.landing_locked &&
           !observation.pilot_in_control;
}

}  // namespace

std::optional<vehicle_api::FlightMode> set_mode_request(float base_mode, float main_mode,
                                                        float sub_mode) {
    const std::optional<std::uint32_t> base = mode_value(base_mode);
    const std::optional<std::uint32_t> main = mode_value(main_mode);
    const std::optional<std::uint32_t> sub = mode_value(sub_mode);
    if (!base.has_value() || !main.has_value() || !sub.has_value() ||
        (*base & static_cast<std::uint32_t>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) == 0U) {
        return std::nullopt;
    }
    return flight_mode_from_custom_mode((*main << 16U) | (*sub << 24U));
}

ForwardDecision decide_forward(const ForwardObservation& observation,
                               const GcsMessageRequest& request) {
    switch (request.kind) {
        case GcsMessageKind::CommandInt:
            return reply(MAV_RESULT_COMMAND_LONG_ONLY, "");
        case GcsMessageKind::CommandLong:
            return decide_command(observation, request);
        case GcsMessageKind::Heartbeat:
        case GcsMessageKind::ParamRequestRead:
            return observation.from_controller ? forward() : drop();
        case GcsMessageKind::ParamSet:
            return observation.from_controller && !observation.landing_locked &&
                           observation.fc_status_known && !observation.armed
                       ? forward()
                       : drop();
        case GcsMessageKind::ManualControl:
            return observation.from_controller && manual_control_allowed(observation) ? forward()
                                                                                      : drop();
    }
    return drop();
}

}  // namespace argus::mavlink_gateway
