#include "argus/mavlink_gateway/argus_status_policy.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include "argus/mavlink_gateway/follow_activity.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr double kPi = 3.14159265358979323846;
constexpr std::int64_t kNsPerMs = 1'000'000;

bool status_lost(const StatusObservation& observation) {
    const std::int64_t since_ns = observation.follow.has_value()
                                      ? observation.follow->arrived_monotonic_ns
                                      : observation.started_ns;
    return observation.now_ns - since_ns > kFollowStatusTimeoutNs;
}

bool active(vehicle_api::MissionState state) {
    return state == vehicle_api::MissionState::FollowActive ||
           state == vehicle_api::MissionState::ScanActive;
}

// setpoint 유효 시간 경과 뒤에 받은 follow/status. 없으면 nullopt
std::optional<vehicle_api::FollowStatus> status_after_timeout(
    const StatusObservation& observation) {
    if (!observation.setpoint_timeout_ns.has_value() || !observation.follow.has_value() ||
        observation.follow->arrived_monotonic_ns <= *observation.setpoint_timeout_ns) {
        return std::nullopt;
    }
    return observation.follow->status;
}

std::uint8_t to_target_state(vehicle_api::TargetState state) {
    return state == vehicle_api::TargetState::Unknown ? ARGUS_TARGET_STATE_NONE
                                                      : static_cast<std::uint8_t>(state);
}

}  // namespace

FollowFault follow_fault(const StatusObservation& observation) {
    if (status_lost(observation)) {
        return FollowFault::StatusLost;
    }
    if (observation.setpoint_timeout_ns.has_value()) {
        const std::optional<vehicle_api::FollowStatus> after = status_after_timeout(observation);
        if (!after.has_value() || active(after->mission_state)) {
            return FollowFault::SetpointTimeout;
        }
    }
    if (observation.follow.has_value() &&
        observation.follow->status.mission_state == vehicle_api::MissionState::Unknown) {
        return FollowFault::UnknownState;
    }
    return FollowFault::None;
}

bool setpoint_timeout_cleared(const StatusObservation& observation) {
    const std::optional<vehicle_api::FollowStatus> after = status_after_timeout(observation);
    return after.has_value() && !active(after->mission_state);
}

bool was_following(const std::optional<vehicle_api::ReceivedFollowStatus>& follow) {
    return follow.has_value() && is_follow_mission(follow->status.mission_state);
}

mavlink_argus_status_t compose_argus_status(const StatusObservation& observation) {
    mavlink_argus_status_t status{};
    status.time_boot_ms =
        static_cast<std::uint32_t>((observation.now_ns - observation.started_ns) / kNsPerMs);
    status.cpu_temp =
        observation.cpu_celsius.has_value() ? static_cast<float>(*observation.cpu_celsius) : kNan;
    status.health = observation.health;
    status.control_sysid = observation.controller.value_or(0);
    status.mission_state = ARGUS_MISSION_STATE_IDLE;
    status.target_state = ARGUS_TARGET_STATE_NONE;
    status.active_sensor = ARGUS_SENSOR_UNKNOWN;
    status.scan_yaw_done_rad = kNan;
    status.scan_remaining_s = kNan;

    const FollowFault fault = follow_fault(observation);
    if (fault == FollowFault::StatusLost || !observation.follow.has_value()) {
        if (fault != FollowFault::None) {
            status.mission_state = ARGUS_MISSION_STATE_FAULT;
        }
        return status;
    }

    const vehicle_api::FollowStatus& follow = observation.follow->status;
    status.mission_state = fault == FollowFault::None
                               ? static_cast<std::uint8_t>(follow.mission_state)
                               : static_cast<std::uint8_t>(ARGUS_MISSION_STATE_FAULT);
    status.target_state = to_target_state(follow.target_state);
    if (follow.selected.has_value()) {
        status.target_id = follow.selected->track_id;
        status.target_generation = follow.selected->generation;
        if (follow.mission_state == vehicle_api::MissionState::Preparing ||
            follow.mission_state == vehicle_api::MissionState::FollowActive) {
            status.active_sensor = static_cast<std::uint8_t>(follow.selected->sensor);
        }
    }
    if (follow.mission_state == vehicle_api::MissionState::ScanActive && follow.scan.has_value()) {
        status.scan_yaw_done_rad = static_cast<float>(follow.scan->progress_deg * kPi / 180.0);
        status.scan_remaining_s = follow.scan->remaining_s.has_value()
                                      ? static_cast<float>(*follow.scan->remaining_s)
                                      : kNan;
    }
    return status;
}

}  // namespace argus::mavlink_gateway
